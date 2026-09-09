use super::nes_apu::Apu;
use crate::bus::Bus8;
use crate::cpu6502::Mos6502;
use crate::input::{DOWN, FACE_EAST, FACE_SOUTH, LEFT, RIGHT, SELECT, START, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 256;
const HEIGHT: u32 = 240;
const CPU_CLOCK: f64 = 1_789_773.0;
const FRAME_RATE: f64 = 60.0988;

fn ram_size_from_shift(shift: u8) -> usize {
    if shift == 0 {
        0
    } else {
        64usize << shift
    }
}

fn nes2_rom_size(lsb: u8, msb: u8, unit: usize) -> Result<usize, String> {
    if msb != 0x0f {
        return ((usize::from(msb) << 8) | usize::from(lsb))
            .checked_mul(unit)
            .ok_or_else(|| "NES 2.0 ROM size overflows host address space".to_string());
    }
    let exponent = u32::from(lsb >> 2);
    let multiplier = usize::from((lsb & 3) * 2 + 1);
    1usize
        .checked_shl(exponent)
        .and_then(|base| base.checked_mul(multiplier))
        .ok_or_else(|| "NES 2.0 exponent ROM size exceeds host address space".to_string())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mirroring {
    Horizontal,
    Vertical,
    SingleScreen0,
    SingleScreen1,
    FourScreen,
}

#[derive(Clone, Debug)]
struct Mmc1 {
    shift: u8,
    bits: u8,
    control: u8,
    chr0: u8,
    chr1: u8,
    prg: u8,
}
impl Default for Mmc1 {
    fn default() -> Self {
        Self {
            shift: 0,
            bits: 0,
            control: 0x0c,
            chr0: 0,
            chr1: 0,
            prg: 0,
        }
    }
}

#[derive(Clone, Debug)]
struct Mmc3 {
    bank_select: u8,
    regs: [u8; 8],
    mirror_horizontal: bool,
    prg_ram_protect: u8,
    irq_latch: u8,
    irq_counter: u8,
    irq_reload: bool,
    irq_enabled: bool,
    irq_pending: bool,
}
impl Default for Mmc3 {
    fn default() -> Self {
        Self {
            bank_select: 0,
            regs: [0; 8],
            mirror_horizontal: false,
            prg_ram_protect: 0x80,
            irq_latch: 0,
            irq_counter: 0,
            irq_reload: false,
            irq_enabled: false,
            irq_pending: false,
        }
    }
}

struct Cartridge {
    prg_rom: Vec<u8>,
    prg_ram: Vec<u8>,
    mapper: u16,
    submapper: u8,
    prg_bank: usize,
    base_mirroring: Mirroring,
    four_screen: bool,
    chr_banks_1k: usize,
    simple_reg: u8,
    battery_len: usize,
    mmc1: Mmc1,
    mmc3: Mmc3,
}

impl Cartridge {
    fn parse(rom: &[u8]) -> Result<(Self, Ppu), String> {
        if rom.len() < 16 || &rom[..4] != b"NES\x1a" {
            return Err("NES image is not an iNES/NES 2.0 file".into());
        }
        let flags6 = rom[6];
        let flags7 = rom[7];
        let nes2 = flags7 & 0x0c == 0x08;
        let prg_size = if nes2 {
            nes2_rom_size(rom[4], rom[9] & 0x0f, 16 * 1024)?
        } else {
            usize::from(rom[4]) * 16 * 1024
        };
        let chr_size = if nes2 {
            nes2_rom_size(rom[5], rom[9] >> 4, 8 * 1024)?
        } else {
            usize::from(rom[5]) * 8 * 1024
        };
        let submapper = if nes2 { rom[8] >> 4 } else { 0 };
        let mapper = u16::from(flags6 >> 4)
            | u16::from(flags7 & 0xf0)
            | if nes2 {
                u16::from(rom[8] & 0x0f) << 8
            } else {
                0
            };
        if !matches!(mapper, 0..=4 | 7 | 11 | 66) {
            return Err(format!(
                "NES mapper {mapper} is not implemented in OmniCore yet"
            ));
        }
        let trainer = if flags6 & 0x04 != 0 { 512 } else { 0 };
        let offset = 16 + trainer;
        match mapper {
            0 | 3 if prg_size != 16 * 1024 && prg_size != 32 * 1024 => {
                return Err(format!(
                    "mapper {mapper} requires 16 or 32 KiB PRG ROM, got {prg_size} bytes"
                ));
            }
            1 | 2 | 4 if prg_size < 32 * 1024 || !prg_size.is_multiple_of(16 * 1024) => {
                return Err(format!(
                    "mapper {mapper} requires 16 KiB PRG banks and at least 32 KiB, got {prg_size} bytes"
                ));
            }
            7 | 11 | 66 if prg_size < 32 * 1024 || !prg_size.is_multiple_of(32 * 1024) => {
                return Err(format!(
                    "mapper {mapper} requires 32 KiB PRG banks, got {prg_size} bytes"
                ));
            }
            _ => {}
        }
        if mapper == 3 && chr_size == 0 {
            return Err("CNROM requires CHR ROM".into());
        }
        if rom.len() < offset + prg_size + chr_size {
            return Err("NES image is truncated".into());
        }
        let prg_rom = rom[offset..offset + prg_size].to_vec();
        let chr_start = offset + prg_size;
        let chr_ram_size = if chr_size == 0 {
            if nes2 {
                (ram_size_from_shift(rom[11] & 0x0f) + ram_size_from_shift(rom[11] >> 4))
                    .max(8 * 1024)
            } else {
                8 * 1024
            }
        } else {
            0
        };
        let chr = if chr_size == 0 {
            vec![0; chr_ram_size]
        } else {
            rom[chr_start..chr_start + chr_size].to_vec()
        };
        let chr_banks_1k = ((if chr_size == 0 {
            chr_ram_size
        } else {
            chr_size
        }) / 0x400)
            .max(1);
        let four_screen = flags6 & 0x08 != 0;
        let mirroring = if four_screen {
            Mirroring::FourScreen
        } else if flags6 & 1 != 0 {
            Mirroring::Vertical
        } else {
            Mirroring::Horizontal
        };
        let mut ppu = Ppu::new(chr, chr_size == 0, mirroring);
        let (mut prg_ram_size, battery_len) = if nes2 {
            let volatile = ram_size_from_shift(rom[10] & 0x0f);
            let nonvolatile = ram_size_from_shift(rom[10] >> 4);
            let total = volatile.saturating_add(nonvolatile);
            let battery = if nonvolatile != 0 {
                nonvolatile
            } else if flags6 & 0x02 != 0 {
                total
            } else {
                0
            };
            (total, battery)
        } else {
            let total = usize::from(rom[8].max(1)) * 8 * 1024;
            (total, if flags6 & 0x02 != 0 { total } else { 0 })
        };
        if trainer != 0 {
            prg_ram_size = prg_ram_size.max(8 * 1024);
        }
        let mut prg_ram = vec![0; prg_ram_size];
        if trainer != 0 {
            prg_ram[0x1000..0x1200].copy_from_slice(&rom[16..16 + 512]);
        }
        let cartridge = Self {
            prg_rom,
            prg_ram,
            mapper,
            submapper,
            prg_bank: 0,
            base_mirroring: mirroring,
            four_screen,
            chr_banks_1k,
            simple_reg: 0,
            battery_len,
            mmc1: Mmc1::default(),
            mmc3: Mmc3::default(),
        };
        cartridge.sync_ppu(&mut ppu);
        Ok((cartridge, ppu))
    }

    fn read_prg(&self, address: u16) -> u8 {
        debug_assert!(address >= 0x8000);
        match self.mapper {
            1 => self.read_mmc1_prg(address),
            2 => {
                let banks = self.prg_rom.len() / 0x4000;
                let bank = if address < 0xc000 {
                    self.prg_bank % banks
                } else {
                    banks - 1
                };
                self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
            }
            4 => self.read_mmc3_prg(address),
            7 | 11 | 66 => {
                let banks = self.prg_rom.len() / 0x8000;
                let bank = match self.mapper {
                    7 => self.simple_reg as usize & 0x0f,
                    11 => self.simple_reg as usize & 0x03,
                    66 => (self.simple_reg as usize >> 4) & 0x03,
                    _ => unreachable!(),
                } % banks;
                self.prg_rom[bank * 0x8000 + (address as usize - 0x8000)]
            }
            _ => {
                let mut index = address as usize - 0x8000;
                if self.prg_rom.len() == 16 * 1024 {
                    index &= 0x3fff;
                }
                self.prg_rom[index]
            }
        }
    }

    fn read_mmc1_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x4000;
        let selected = self.mmc1.prg as usize % banks;
        let mode = (self.mmc1.control >> 2) & 3;
        let bank = match mode {
            0 | 1 => ((selected & !1) + usize::from(address >= 0xc000)) % banks,
            2 => {
                if address < 0xc000 {
                    0
                } else {
                    selected
                }
            }
            _ => {
                if address < 0xc000 {
                    selected
                } else {
                    banks - 1
                }
            }
        };
        self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
    }
    fn read_mmc3_prg(&self, address: u16) -> u8 {
        let banks = self.prg_rom.len() / 0x2000;
        let last = banks - 1;
        let second_last = banks - 2;
        let r6 = self.mmc3.regs[6] as usize % banks;
        let r7 = self.mmc3.regs[7] as usize % banks;
        let slot = (address as usize - 0x8000) / 0x2000;
        let prg_mode = self.mmc3.bank_select & 0x40 != 0;
        let bank = match (prg_mode, slot) {
            (false, 0) => r6,
            (false, 1) => r7,
            (false, 2) => second_last,
            (false, 3) => last,
            (true, 0) => second_last,
            (true, 1) => r7,
            (true, 2) => r6,
            (true, 3) => last,
            _ => unreachable!(),
        };
        self.prg_rom[bank * 0x2000 + (address as usize & 0x1fff)]
    }

    fn write_mapper(&mut self, address: u16, value: u8) {
        match self.mapper {
            1 => self.write_mmc1(address, value),
            2 => self.prg_bank = value as usize,
            3 => self.prg_bank = value as usize,
            4 => self.write_mmc3(address, value),
            7 | 11 | 66 => self.simple_reg = value,
            _ => {}
        }
    }
    fn write_mmc1(&mut self, address: u16, value: u8) {
        if value & 0x80 != 0 {
            self.mmc1.shift = 0;
            self.mmc1.bits = 0;
            self.mmc1.control |= 0x0c;
            return;
        }
        self.mmc1.shift |= (value & 1) << self.mmc1.bits;
        self.mmc1.bits += 1;
        if self.mmc1.bits < 5 {
            return;
        }
        let data = self.mmc1.shift & 0x1f;
        match address {
            0x8000..=0x9fff => self.mmc1.control = data,
            0xa000..=0xbfff => self.mmc1.chr0 = data,
            0xc000..=0xdfff => self.mmc1.chr1 = data,
            0xe000..=0xffff => self.mmc1.prg = data,
            _ => {}
        }
        self.mmc1.shift = 0;
        self.mmc1.bits = 0;
    }

    fn write_mmc3(&mut self, address: u16, value: u8) {
        match address & 0xe001 {
            0x8000 => self.mmc3.bank_select = value,
            0x8001 => self.mmc3.regs[(self.mmc3.bank_select & 7) as usize] = value,
            0xa000 if !self.four_screen => self.mmc3.mirror_horizontal = value & 1 != 0,
            0xa001 => self.mmc3.prg_ram_protect = value,
            0xc000 => self.mmc3.irq_latch = value,
            0xc001 => self.mmc3.irq_reload = true,
            0xe000 => {
                self.mmc3.irq_enabled = false;
                self.mmc3.irq_pending = false;
            }
            0xe001 => self.mmc3.irq_enabled = true,
            _ => {}
        }
    }

    fn sync_ppu(&self, ppu: &mut Ppu) {
        ppu.set_mirroring(self.mapper_mirroring());
        ppu.set_chr_map(self.chr_map());
    }

    fn mapper_mirroring(&self) -> Mirroring {
        if self.four_screen {
            return Mirroring::FourScreen;
        }
        match self.mapper {
            1 => match self.mmc1.control & 3 {
                0 => Mirroring::SingleScreen0,
                1 => Mirroring::SingleScreen1,
                2 => Mirroring::Vertical,
                _ => Mirroring::Horizontal,
            },
            4 => {
                if self.mmc3.mirror_horizontal {
                    Mirroring::Horizontal
                } else {
                    Mirroring::Vertical
                }
            }
            7 => {
                if self.simple_reg & 0x10 != 0 {
                    Mirroring::SingleScreen1
                } else {
                    Mirroring::SingleScreen0
                }
            }
            _ => self.base_mirroring,
        }
    }

    fn chr_map(&self) -> [usize; 8] {
        let count = self.chr_bank_count_1k();
        let mut map = [0usize; 8];
        match self.mapper {
            1 => {
                if self.mmc1.control & 0x10 == 0 {
                    let base = ((self.mmc1.chr0 & 0x1e) as usize * 4) % count;
                    for (i, slot) in map.iter_mut().enumerate() {
                        *slot = (base + i) % count;
                    }
                } else {
                    let lower = self.mmc1.chr0 as usize * 4;
                    let upper = self.mmc1.chr1 as usize * 4;
                    for i in 0..4 {
                        map[i] = (lower + i) % count;
                        map[i + 4] = (upper + i) % count;
                    }
                }
            }
            3 => {
                let base = (self.prg_bank * 8) % count;
                for (i, slot) in map.iter_mut().enumerate() {
                    *slot = (base + i) % count;
                }
            }
            11 | 66 => {
                let bank = if self.mapper == 11 {
                    self.simple_reg >> 4
                } else {
                    self.simple_reg & 3
                };
                let base = bank as usize * 8;
                for (i, slot) in map.iter_mut().enumerate() {
                    *slot = (base + i) % count;
                }
            }
            4 => {
                let r = self.mmc3.regs;
                let normal = [
                    r[0] & 0xfe,
                    r[0] | 1,
                    r[1] & 0xfe,
                    r[1] | 1,
                    r[2],
                    r[3],
                    r[4],
                    r[5],
                ];
                let inverted = [
                    r[2],
                    r[3],
                    r[4],
                    r[5],
                    r[0] & 0xfe,
                    r[0] | 1,
                    r[1] & 0xfe,
                    r[1] | 1,
                ];
                let selected = if self.mmc3.bank_select & 0x80 != 0 {
                    inverted
                } else {
                    normal
                };
                for (slot, bank) in map.iter_mut().zip(selected) {
                    *slot = bank as usize % count;
                }
            }
            _ => {
                for (i, slot) in map.iter_mut().enumerate() {
                    *slot = i % count;
                }
            }
        }
        map
    }

    fn chr_bank_count_1k(&self) -> usize {
        self.chr_banks_1k.max(1)
    }
    fn clock_scanline(&mut self) {
        if self.mapper != 4 {
            return;
        }
        if self.mmc3.irq_counter == 0 || self.mmc3.irq_reload {
            self.mmc3.irq_counter = self.mmc3.irq_latch;
            self.mmc3.irq_reload = false;
        } else {
            self.mmc3.irq_counter -= 1;
        }
        if self.mmc3.irq_counter == 0 && self.mmc3.irq_enabled {
            self.mmc3.irq_pending = true;
        }
    }

    fn irq_pending(&self) -> bool {
        self.mapper == 4 && self.mmc3.irq_pending
    }

    fn ram_readable(&self) -> bool {
        if self.prg_ram.is_empty() {
            return false;
        }
        match self.mapper {
            1 => self.mmc1.prg & 0x10 == 0,
            4 => self.mmc3.prg_ram_protect & 0x80 != 0,
            _ => true,
        }
    }
    fn ram_writable(&self) -> bool {
        self.ram_readable() && (self.mapper != 4 || self.mmc3.prg_ram_protect & 0x40 == 0)
    }
    fn read_ram(&self, address: u16) -> u8 {
        if !self.ram_readable() {
            return 0xff;
        }
        let index = (address as usize - 0x6000) % self.prg_ram.len();
        self.prg_ram[index]
    }
    fn write_ram(&mut self, address: u16, value: u8) {
        if !self.ram_writable() {
            return;
        }
        let index = (address as usize - 0x6000) % self.prg_ram.len();
        self.prg_ram[index] = value;
    }
}

struct Ppu {
    chr: Vec<u8>,
    chr_ram: bool,
    chr_map: [usize; 8],
    chr_bank_count: usize,
    mirroring: Mirroring,
    nametable: [u8; 4096],
    palette: [u8; 32],
    oam: [u8; 256],
    video: VideoBuffer,
    ctrl: u8,
    mask: u8,
    status: u8,
    oam_addr: u8,
    v: u16,
    t: u16,
    fine_x: u8,
    write_toggle: bool,
    data_buffer: u8,
    scroll_x: u8,
    scroll_y: u8,
    cycle: u16,
    scanline: u16,
    frame: u64,
    nmi_pending: bool,
}

impl Ppu {
    fn new(chr: Vec<u8>, chr_ram: bool, mirroring: Mirroring) -> Self {
        Self {
            chr_bank_count: (chr.len() / 0x0400).max(1),
            chr_map: [0, 1, 2, 3, 4, 5, 6, 7],
            chr,
            chr_ram,
            mirroring,
            nametable: [0; 4096],
            palette: [0; 32],
            oam: [0; 256],
            video: VideoBuffer::new(WIDTH, HEIGHT),
            ctrl: 0,
            mask: 0,
            status: 0,
            oam_addr: 0,
            v: 0,
            t: 0,
            fine_x: 0,
            write_toggle: false,
            data_buffer: 0,
            scroll_x: 0,
            scroll_y: 0,
            cycle: 0,
            scanline: 0,
            frame: 0,
            nmi_pending: false,
        }
    }
    fn reset(&mut self) {
        self.ctrl = 0;
        self.mask = 0;
        self.status = 0;
        self.oam_addr = 0;
        self.v = 0;
        self.t = 0;
        self.fine_x = 0;
        self.write_toggle = false;
        self.data_buffer = 0;
        self.scroll_x = 0;
        self.scroll_y = 0;
        self.cycle = 0;
        self.scanline = 0;
        self.frame = 0;
        self.nmi_pending = false;
        self.video.clear([0, 0, 0, 255]);
    }

    fn mirror_nametable(&self, address: u16) -> usize {
        let offset = (address as usize - 0x2000) & 0x0fff;
        let table = offset / 0x400;
        let inner = offset & 0x3ff;
        let physical = match self.mirroring {
            Mirroring::Vertical => table & 1,
            Mirroring::Horizontal => table >> 1,
            Mirroring::SingleScreen0 => 0,
            Mirroring::SingleScreen1 => 1,
            Mirroring::FourScreen => table,
        };
        physical * 0x400 + inner
    }

    fn palette_index(address: u16) -> usize {
        let mut index = (address as usize - 0x3f00) & 0x1f;
        if matches!(index, 0x10 | 0x14 | 0x18 | 0x1c) {
            index -= 0x10;
        }
        index
    }
    fn read_vram(&self, address: u16) -> u8 {
        let address = address & 0x3fff;
        match address {
            0x0000..=0x1fff => {
                let slot = address as usize / 0x400;
                let bank = self.chr_map[slot] % self.chr_bank_count;
                self.chr[bank * 0x400 + (address as usize & 0x3ff)]
            }
            0x2000..=0x3eff => {
                self.nametable[self.mirror_nametable(if address >= 0x3000 {
                    address - 0x1000
                } else {
                    address
                })]
            }
            0x3f00..=0x3fff => self.palette[Self::palette_index(address)],
            _ => unreachable!(),
        }
    }

    fn write_vram(&mut self, address: u16, value: u8) {
        let address = address & 0x3fff;
        match address {
            0x0000..=0x1fff if self.chr_ram => {
                let slot = address as usize / 0x400;
                let bank = self.chr_map[slot] % self.chr_bank_count;
                self.chr[bank * 0x400 + (address as usize & 0x3ff)] = value;
            }
            0x0000..=0x1fff => {}
            0x2000..=0x3eff => {
                let mirrored = if address >= 0x3000 {
                    address - 0x1000
                } else {
                    address
                };
                let index = self.mirror_nametable(mirrored);
                self.nametable[index] = value;
            }
            0x3f00..=0x3fff => {
                let index = Self::palette_index(address);
                self.palette[index] = value & 0x3f;
            }
            _ => unreachable!(),
        }
    }
    fn read_register(&mut self, register: u16) -> u8 {
        match register & 7 {
            2 => {
                let value = self.status;
                self.status &= !0x80;
                self.write_toggle = false;
                value
            }
            4 => self.oam[self.oam_addr as usize],
            7 => {
                let address = self.v & 0x3fff;
                let raw = self.read_vram(address);
                let value = if address < 0x3f00 {
                    let buffered = self.data_buffer;
                    self.data_buffer = raw;
                    buffered
                } else {
                    self.data_buffer = self.read_vram(address.wrapping_sub(0x1000));
                    raw
                };
                self.v = self
                    .v
                    .wrapping_add(if self.ctrl & 0x04 != 0 { 32 } else { 1 })
                    & 0x7fff;
                value
            }
            _ => 0,
        }
    }
    fn write_register(&mut self, register: u16, value: u8) {
        match register & 7 {
            0 => {
                let had_nmi = self.ctrl & 0x80 != 0;
                self.ctrl = value;
                self.t = (self.t & !0x0c00) | (((value as u16) & 3) << 10);
                if !had_nmi && value & 0x80 != 0 && self.status & 0x80 != 0 {
                    self.nmi_pending = true;
                }
            }
            1 => self.mask = value,
            3 => self.oam_addr = value,
            4 => {
                self.oam[self.oam_addr as usize] = value;
                self.oam_addr = self.oam_addr.wrapping_add(1);
            }
            5 => {
                if !self.write_toggle {
                    self.scroll_x = value;
                    self.fine_x = value & 7;
                    self.t = (self.t & !0x001f) | ((value as u16) >> 3);
                } else {
                    self.scroll_y = value;
                    self.t = (self.t & !0x73e0)
                        | (((value as u16 & 0xf8) << 2) & 0x03e0)
                        | (((value as u16) & 7) << 12);
                }
                self.write_toggle = !self.write_toggle;
            }
            6 => {
                if !self.write_toggle {
                    self.t = (self.t & 0x00ff) | (((value as u16) & 0x3f) << 8);
                } else {
                    self.t = (self.t & 0x7f00) | value as u16;
                    self.v = self.t;
                }
                self.write_toggle = !self.write_toggle;
            }
            7 => {
                let address = self.v & 0x3fff;
                self.write_vram(address, value);
                self.v = self
                    .v
                    .wrapping_add(if self.ctrl & 0x04 != 0 { 32 } else { 1 })
                    & 0x7fff;
            }
            _ => {}
        }
    }

    fn write_oam_dma(&mut self, data: &[u8; 256]) {
        for (offset, value) in data.iter().copied().enumerate() {
            self.oam[self.oam_addr.wrapping_add(offset as u8) as usize] = value;
        }
    }

    fn set_chr_map(&mut self, map: [usize; 8]) {
        for (slot, mapped) in map.into_iter().enumerate() {
            self.chr_map[slot] = mapped % self.chr_bank_count;
        }
    }
    fn set_mirroring(&mut self, mirroring: Mirroring) {
        self.mirroring = mirroring;
    }

    fn take_nmi(&mut self) -> bool {
        let pending = self.nmi_pending;
        self.nmi_pending = false;
        pending
    }
    fn tick(&mut self, ticks: u32) -> u32 {
        let mut visible_scanlines = 0;
        for _ in 0..ticks {
            self.cycle += 1;
            if self.scanline == 241 && self.cycle == 1 {
                self.status |= 0x80;
                self.render_frame();
                self.frame = self.frame.wrapping_add(1);
                if self.ctrl & 0x80 != 0 {
                    self.nmi_pending = true;
                }
            } else if self.scanline == 261 && self.cycle == 1 {
                self.status &= !0xe0;
            }
            if self.cycle >= 341 {
                self.cycle = 0;
                if self.scanline < 240 && self.mask & 0x18 != 0 {
                    visible_scanlines += 1;
                }
                self.scanline += 1;
                if self.scanline >= 262 {
                    self.scanline = 0;
                }
            }
        }
        visible_scanlines
    }

    fn render_frame(&mut self) {
        self.status &= !0x40;
        let universal = nes_color(self.palette[0] & 0x3f);
        self.video.clear(universal);
        if self.mask & 0x08 != 0 {
            self.render_background();
        }
        if self.mask & 0x10 != 0 {
            self.render_sprites();
        }
    }
    fn background_pixel(&self, x: u32, y: u32) -> ([u8; 4], bool) {
        let world_x = x as usize + self.scroll_x as usize;
        let world_y = y as usize + self.scroll_y as usize;
        let base_x = (self.ctrl & 1) as usize;
        let base_y = ((self.ctrl >> 1) & 1) as usize;
        let nt_x = (base_x + world_x / 256) & 1;
        let nt_y = (base_y + world_y / 240) & 1;
        let local_x = world_x % 256;
        let local_y = world_y % 240;
        let tile_x = local_x / 8;
        let tile_y = local_y / 8;
        let table = nt_y * 2 + nt_x;
        let base = 0x2000 + (table as u16) * 0x400;
        let tile = self.read_vram(base + (tile_y * 32 + tile_x) as u16);
        let attribute = self.read_vram(base + 0x3c0 + ((tile_y / 4) * 8 + tile_x / 4) as u16);
        let quadrant = ((tile_y & 2) << 1) | (tile_x & 2);
        let palette_select = (attribute >> quadrant) & 3;
        let pattern_base = if self.ctrl & 0x10 != 0 { 0x1000 } else { 0 };
        let row = (local_y & 7) as u16;
        let pattern = pattern_base + tile as u16 * 16 + row;
        let low = self.read_vram(pattern);
        let high = self.read_vram(pattern + 8);
        let bit = 7 - (local_x & 7);
        let color = ((low >> bit) & 1) | (((high >> bit) & 1) << 1);
        let palette_index = if color == 0 {
            0
        } else {
            palette_select * 4 + color
        } as u16;
        (
            nes_color(self.read_vram(0x3f00 + palette_index) & 0x3f),
            color != 0,
        )
    }

    fn render_background(&mut self) {
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                if x < 8 && self.mask & 0x02 == 0 {
                    continue;
                }
                let (rgba, _) = self.background_pixel(x, y);
                let offset = ((y * WIDTH + x) * 4) as usize;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
    }

    fn render_sprites(&mut self) {
        let sprite_height = if self.ctrl & 0x20 != 0 { 16 } else { 8 };
        for sprite_index in (0..64).rev() {
            let base = sprite_index * 4;
            let sprite_y = self.oam[base] as i32 + 1;
            let tile = self.oam[base + 1];
            let attributes = self.oam[base + 2];
            let sprite_x = self.oam[base + 3] as i32;
            for py in 0..sprite_height {
                let screen_y = sprite_y + py;
                if !(0..HEIGHT as i32).contains(&screen_y) {
                    continue;
                }
                let source_y = if attributes & 0x80 != 0 {
                    sprite_height - 1 - py
                } else {
                    py
                };
                let (pattern_base, tile_index, row) = if sprite_height == 16 {
                    let table = if tile & 1 != 0 { 0x1000 } else { 0 };
                    let pair = tile & 0xfe;
                    let which = if source_y >= 8 { 1 } else { 0 };
                    (table, pair.wrapping_add(which), (source_y & 7) as u16)
                } else {
                    (
                        if self.ctrl & 0x08 != 0 { 0x1000 } else { 0 },
                        tile,
                        source_y as u16,
                    )
                };
                let pattern = pattern_base + tile_index as u16 * 16 + row;
                let low = self.read_vram(pattern);
                let high = self.read_vram(pattern + 8);
                for px in 0..8 {
                    let screen_x = sprite_x + px;
                    if !(0..WIDTH as i32).contains(&screen_x) {
                        continue;
                    }
                    if screen_x < 8 && self.mask & 0x04 == 0 {
                        continue;
                    }
                    let source_x = if attributes & 0x40 != 0 { 7 - px } else { px };
                    let bit = 7 - source_x;
                    let color = ((low >> bit) & 1) | (((high >> bit) & 1) << 1);
                    if color == 0 {
                        continue;
                    }
                    let (_, background_opaque) =
                        self.background_pixel(screen_x as u32, screen_y as u32);
                    if sprite_index == 0 && background_opaque && screen_x < 255 {
                        self.status |= 0x40;
                    }
                    if attributes & 0x20 != 0 && background_opaque {
                        continue;
                    }
                    let palette = 0x3f10 + ((attributes & 3) as u16) * 4 + color as u16;
                    let rgba = nes_color(self.read_vram(palette) & 0x3f);
                    let offset = (((screen_y as u32) * WIDTH + screen_x as u32) * 4) as usize;
                    self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                }
            }
        }
    }
}

fn nes_color(index: u8) -> [u8; 4] {
    const PALETTE: [u32; 64] = [
        0x545454, 0x001e74, 0x081090, 0x300088, 0x440064, 0x5c0030, 0x540400, 0x3c1800, 0x202a00,
        0x083a00, 0x004000, 0x003c00, 0x00323c, 0x000000, 0x000000, 0x000000, 0x989698, 0x084cc4,
        0x3032ec, 0x5c1ee4, 0x8814b0, 0xa01464, 0x982220, 0x783c00, 0x545a00, 0x287200, 0x087c00,
        0x007628, 0x006678, 0x000000, 0x000000, 0x000000, 0xeceeec, 0x4c9aec, 0x787cec, 0xb062ec,
        0xe454ec, 0xec58b4, 0xec6a64, 0xd48820, 0xa0aa00, 0x74c400, 0x4cd020, 0x38cc6c, 0x38b4cc,
        0x3c3c3c, 0x000000, 0x000000, 0xeceeec, 0xa8ccec, 0xbcbcec, 0xd4b2ec, 0xecaeec, 0xecaed4,
        0xecb4b0, 0xe4c490, 0xccd278, 0xb4de78, 0xa8e290, 0x98e2b4, 0xa0d6e4, 0xa0a2a0, 0x000000,
        0x000000,
    ];
    let rgb = PALETTE[index as usize];
    [
        ((rgb >> 16) & 0xff) as u8,
        ((rgb >> 8) & 0xff) as u8,
        (rgb & 0xff) as u8,
        255,
    ]
}

#[derive(Default)]
struct Controller {
    live: u8,
    latched: u8,
    shift: u8,
    strobe: bool,
}

impl Controller {
    fn set_buttons(&mut self, mask: u64) {
        self.live = u8::from(mask & FACE_SOUTH != 0)
            | (u8::from(mask & FACE_EAST != 0) << 1)
            | (u8::from(mask & SELECT != 0) << 2)
            | (u8::from(mask & START != 0) << 3)
            | (u8::from(mask & UP != 0) << 4)
            | (u8::from(mask & DOWN != 0) << 5)
            | (u8::from(mask & LEFT != 0) << 6)
            | (u8::from(mask & RIGHT != 0) << 7);
        if self.strobe {
            self.latch();
        }
    }

    fn latch(&mut self) {
        self.latched = self.live;
        self.shift = self.latched;
    }
    fn write_strobe(&mut self, value: u8) {
        let next = value & 1 != 0;
        if self.strobe && !next {
            self.latch();
        }
        self.strobe = next;
        if next {
            self.latch();
        }
    }

    fn read(&mut self) -> u8 {
        if self.strobe {
            return (self.live & 1) | 0x40;
        }
        let value = (self.shift & 1) | 0x40;
        self.shift = (self.shift >> 1) | 0x80;
        value
    }
}

struct NesBus {
    ram: [u8; 2048],
    cartridge: Cartridge,
    ppu: Ppu,
    controllers: [Controller; 2],
    apu: Apu,
    dma_page: Option<u8>,
}

impl NesBus {
    fn new(cartridge: Cartridge, ppu: Ppu) -> Self {
        Self {
            ram: [0; 2048],
            cartridge,
            ppu,
            controllers: [Controller::default(), Controller::default()],
            apu: Apu::default(),
            dma_page: None,
        }
    }
    fn perform_dma(&mut self, page: u8) {
        let mut data = [0u8; 256];
        let base = (page as u16) << 8;
        for (offset, value) in data.iter_mut().enumerate() {
            *value = self.read8(base | offset as u16);
        }
        self.ppu.write_oam_dma(&data);
    }
}

impl Bus8 for NesBus {
    fn read8(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0x1fff => self.ram[(address as usize) & 0x07ff],
            0x2000..=0x3fff => self.ppu.read_register(0x2000 | (address & 7)),
            0x4015 => self.apu.read_status(),
            0x4016 => self.controllers[0].read(),
            0x4017 => self.controllers[1].read(),
            0x6000..=0x7fff => self.cartridge.read_ram(address),
            0x8000..=0xffff => self.cartridge.read_prg(address),
            _ => 0,
        }
    }

    fn write8(&mut self, address: u16, value: u8) {
        match address {
            0x0000..=0x1fff => self.ram[(address as usize) & 0x07ff] = value,
            0x2000..=0x3fff => self.ppu.write_register(0x2000 | (address & 7), value),
            0x4000..=0x4013 => self.apu.write_register(address, value),
            0x4014 => self.dma_page = Some(value),
            0x4015 => self.apu.write_register(address, value),
            0x4016 => {
                self.controllers[0].write_strobe(value);
                self.controllers[1].write_strobe(value);
            }
            0x4017 => self.apu.write_register(address, value),
            0x6000..=0x7fff => self.cartridge.write_ram(address, value),
            0x8000..=0xffff => {
                self.cartridge.write_mapper(address, value);
                self.cartridge.sync_ppu(&mut self.ppu);
            }
            _ => {}
        }
    }
}

pub struct NesMachine {
    cpu: Mos6502,
    bus: NesBus,
    audio: AudioBuffer,
    powered: bool,
}

impl NesMachine {
    pub fn from_rom(rom: &[u8]) -> Result<Self, String> {
        let (cartridge, ppu) = Cartridge::parse(rom)?;
        let mut bus = NesBus::new(cartridge, ppu);
        let mut cpu = Mos6502::default();
        cpu.reset(&mut bus);
        Ok(Self {
            cpu,
            bus,
            audio: AudioBuffer::new(48_000, 2),
            powered: true,
        })
    }

    fn tick_devices(&mut self, cpu_cycles: u32) {
        let scanlines = self.bus.ppu.tick(cpu_cycles * 3);
        self.bus.apu.tick_cpu_cycles(cpu_cycles);
        for _ in 0..scanlines {
            self.bus.cartridge.clock_scanline();
        }
        if let Some(address) = self.bus.apu.take_dmc_fetch_request() {
            let value = self.bus.read8(address);
            self.bus.apu.supply_dmc_byte(value);
            self.cpu.cycles = self.cpu.cycles.wrapping_add(4);
            let scanlines = self.bus.ppu.tick(12);
            self.bus.apu.tick_cpu_cycles(4);
            for _ in 0..scanlines {
                self.bus.cartridge.clock_scanline();
            }
        }
    }

    fn clock_cpu_instruction(&mut self) -> u32 {
        let cycles = self.cpu.step(&mut self.bus);
        if cycles == 0 {
            return 0;
        }
        self.tick_devices(cycles);
        if let Some(page) = self.bus.dma_page.take() {
            self.bus.perform_dma(page);
            let dma_cycles = 513 + (self.cpu.cycles as u32 & 1);
            self.cpu.cycles += dma_cycles as u64;
            self.tick_devices(dma_cycles);
        }
        if self.bus.ppu.take_nmi() {
            let before = self.cpu.cycles;
            self.cpu.nmi(&mut self.bus);
            let interrupt_cycles = (self.cpu.cycles - before) as u32;
            self.tick_devices(interrupt_cycles);
        }
        if self.bus.apu.irq_pending() || self.bus.cartridge.irq_pending() {
            let before = self.cpu.cycles;
            self.cpu.irq(&mut self.bus);
            let interrupt_cycles = (self.cpu.cycles - before) as u32;
            self.tick_devices(interrupt_cycles);
        }
        cycles
    }

    fn finish_audio_frame(&mut self) {
        self.bus.apu.drain_audio(&mut self.audio);
    }
}

impl Machine for NesMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Nes
    }

    fn reset(&mut self) {
        self.bus.ppu.reset();
        self.bus.apu.reset();
        self.cpu.reset(&mut self.bus);
        self.powered = true;
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.controllers[0].set_buttons(input.buttons[0]);
        self.bus.controllers[1].set_buttons(input.buttons[1]);
        let target = self.bus.ppu.frame.wrapping_add(1);
        let cycle_budget = (CPU_CLOCK / FRAME_RATE * 2.0).ceil() as u64;
        let cycle_deadline = self.cpu.cycles.saturating_add(cycle_budget);
        while self.bus.ppu.frame != target && self.cpu.cycles < cycle_deadline {
            if self.clock_cpu_instruction() == 0 {
                self.powered = false;
                break;
            }
        }
        self.finish_audio_frame();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }
    fn video(&self) -> &VideoBuffer {
        &self.bus.ppu.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }
    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Nes, 1);
        out.u16(self.cpu.pc);
        out.u8(self.cpu.sp);
        out.u8(self.cpu.a);
        out.u8(self.cpu.x);
        out.u8(self.cpu.y);
        out.u8(self.cpu.p);
        out.u64(self.cpu.cycles);
        out.u8(self.cpu.stopped as u8);
        out.blob(&self.bus.ram);
        out.u16(self.bus.cartridge.mapper);
        out.u8(self.bus.cartridge.submapper);
        out.u64(self.bus.cartridge.prg_bank as u64);
        save_mapper_state(&mut out, &self.bus.cartridge);
        out.blob(&self.bus.cartridge.prg_ram);
        out.u8(self.bus.ppu.chr_ram as u8);
        if self.bus.ppu.chr_ram {
            out.blob(&self.bus.ppu.chr);
        } else {
            out.blob(&[]);
        }
        out.blob(&self.bus.ppu.nametable);
        out.blob(&self.bus.ppu.palette);
        out.blob(&self.bus.ppu.oam);
        save_ppu_registers(&mut out, &self.bus.ppu);
        for controller in &self.bus.controllers {
            save_controller(&mut out, controller);
        }
        out.u8(self.bus.dma_page.unwrap_or(0));
        out.u8(self.bus.dma_page.is_some() as u8);
        self.bus.apu.save(&mut out);
        out.u8(self.powered as u8);
        out.blob(self.bus.ppu.video.pixels());
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Nes, 1)?;
        self.cpu.pc = input.u16()?;
        self.cpu.sp = input.u8()?;
        self.cpu.a = input.u8()?;
        self.cpu.x = input.u8()?;
        self.cpu.y = input.u8()?;
        self.cpu.p = input.u8()?;
        self.cpu.cycles = input.u64()?;
        self.cpu.stopped = input.u8()? != 0;
        read_exact_blob(&mut input, &mut self.bus.ram)?;
        let mapper = input.u16()?;
        if mapper != self.bus.cartridge.mapper {
            return Err("NES save state mapper differs from loaded cartridge".into());
        }
        let submapper = input.u8()?;
        if submapper != self.bus.cartridge.submapper {
            return Err("NES save state submapper differs from loaded cartridge".into());
        }
        self.bus.cartridge.prg_bank = input.u64()? as usize;
        load_mapper_state(&mut input, &mut self.bus.cartridge)?;
        read_vec_blob(&mut input, &mut self.bus.cartridge.prg_ram)?;
        let chr_ram = input.u8()? != 0;
        if chr_ram != self.bus.ppu.chr_ram {
            return Err("NES save state CHR memory type differs from loaded cartridge".into());
        }
        let chr = input.blob()?;
        if chr_ram {
            if chr.len() != self.bus.ppu.chr.len() {
                return Err("NES CHR RAM size differs".into());
            }
            self.bus.ppu.chr.copy_from_slice(chr);
        }
        self.bus.cartridge.sync_ppu(&mut self.bus.ppu);
        read_exact_blob(&mut input, &mut self.bus.ppu.nametable)?;
        read_exact_blob(&mut input, &mut self.bus.ppu.palette)?;
        read_exact_blob(&mut input, &mut self.bus.ppu.oam)?;
        load_ppu_registers(&mut input, &mut self.bus.ppu)?;
        for controller in &mut self.bus.controllers {
            load_controller(&mut input, controller)?;
        }
        let dma = input.u8()?;
        self.bus.dma_page = if input.u8()? != 0 { Some(dma) } else { None };
        self.bus.apu.load(&mut input)?;
        self.powered = input.u8()? != 0;
        let video = input.blob()?;
        if video.len() != self.bus.ppu.video.pixels().len() {
            return Err("NES framebuffer size differs".into());
        }
        self.bus.ppu.video.pixels_mut().copy_from_slice(video);
        self.audio.begin_frame();
        input.finish()
    }
    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            self.bus.cartridge.battery_len
        } else {
            0
        }
    }
    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        let len = self.persistent_len(kind, slot);
        if len == 0 {
            return Err("NES cartridge has no battery-backed storage".into());
        }
        if out.len() != len {
            return Err(format!(
                "persistent output has {} bytes; expected {len}",
                out.len()
            ));
        }
        out.copy_from_slice(&self.bus.cartridge.prg_ram[..len]);
        Ok(())
    }
    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        let len = self.persistent_len(kind, slot);
        if len == 0 {
            return Err("NES cartridge has no battery-backed storage".into());
        }
        if data.len() != len {
            return Err(format!(
                "persistent input has {} bytes; expected {len}",
                data.len()
            ));
        }
        self.bus.cartridge.prg_ram[..len].copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_nrom() -> Vec<u8> {
        let mut rom = vec![0u8; 16 + 16 * 1024 + 8 * 1024];
        rom[..4].copy_from_slice(b"NES\x1a");
        rom[4] = 1;
        rom[5] = 1;
        let prg = 16;
        let program = [
            0x78, 0xd8, 0xa2, 0xff, 0x9a, 0xa9, 0x80, 0x8d, 0x00, 0x20, 0xa9, 0x08, 0x8d, 0x01,
            0x20, 0xa9, 0x3f, 0x8d, 0x06, 0x20,
        ];
        rom[prg..prg + program.len()].copy_from_slice(&program);
        let tail = [
            0xa9, 0x00, 0x8d, 0x06, 0x20, 0xa9, 0x0f, 0x8d, 0x07, 0x20, 0xa9, 0x30, 0x8d, 0x07,
            0x20, 0x4c, 0x23, 0x80,
        ];
        let tail_start = prg + program.len();
        rom[tail_start..tail_start + tail.len()].copy_from_slice(&tail);
        let vectors = prg + 0x3ffa;
        rom[vectors..vectors + 6].copy_from_slice(&[0x00, 0x80, 0x00, 0x80, 0x00, 0x80]);
        let chr = prg + 16 * 1024;
        rom[chr..chr + 8].fill(0xff);
        rom
    }

    #[test]
    fn nrom_machine_executes_cpu_and_renders_a_frame() {
        let rom = synthetic_nrom();
        let mut machine = NesMachine::from_rom(&rom).unwrap();
        machine.run_frame(&InputState::default());
        assert!(machine.powered);
        assert!(machine.cpu.cycles > CPU_CLOCK as u64 / 100);
        assert_eq!(machine.video().width(), WIDTH);
        assert_eq!(machine.video().height(), HEIGHT);
        assert!(machine
            .video()
            .pixels()
            .as_chunks::<4>()
            .0
            .iter()
            .any(|pixel| pixel[0] > 100));
    }

    #[test]
    fn save_state_round_trip_is_deterministic() {
        let rom = synthetic_nrom();
        let mut first = NesMachine::from_rom(&rom).unwrap();
        first.run_frame(&InputState::default());
        first.bus.ram[17] = 0x5a;
        let state = first.save_state().unwrap();
        let mut second = NesMachine::from_rom(&rom).unwrap();
        second.load_state(&state).unwrap();
        assert_eq!(second.save_state().unwrap(), state);
        first.run_frame(&InputState::default());
        second.run_frame(&InputState::default());
        assert_eq!(first.cpu.cycles, second.cpu.cycles);
        assert_eq!(first.video().pixels(), second.video().pixels());
        assert_eq!(first.save_state().unwrap(), second.save_state().unwrap());
    }
}

fn save_mapper_state(out: &mut StateWriter, cart: &Cartridge) {
    out.u8(cart.simple_reg);
    out.u8(cart.mmc1.shift);
    out.u8(cart.mmc1.bits);
    out.u8(cart.mmc1.control);
    out.u8(cart.mmc1.chr0);
    out.u8(cart.mmc1.chr1);
    out.u8(cart.mmc1.prg);
    out.u8(cart.mmc3.bank_select);
    out.blob(&cart.mmc3.regs);
    out.u8(cart.mmc3.mirror_horizontal as u8);
    out.u8(cart.mmc3.prg_ram_protect);
    out.u8(cart.mmc3.irq_latch);
    out.u8(cart.mmc3.irq_counter);
    out.u8(cart.mmc3.irq_reload as u8);
    out.u8(cart.mmc3.irq_enabled as u8);
    out.u8(cart.mmc3.irq_pending as u8);
}

fn load_mapper_state(input: &mut StateReader<'_>, cart: &mut Cartridge) -> Result<(), String> {
    cart.simple_reg = input.u8()?;
    cart.mmc1.shift = input.u8()?;
    cart.mmc1.bits = input.u8()?;
    cart.mmc1.control = input.u8()?;
    cart.mmc1.chr0 = input.u8()?;
    cart.mmc1.chr1 = input.u8()?;
    cart.mmc1.prg = input.u8()?;
    cart.mmc3.bank_select = input.u8()?;
    let regs = input.blob()?;
    if regs.len() != 8 {
        return Err("NES MMC3 register state has wrong size".into());
    }
    cart.mmc3.regs.copy_from_slice(regs);
    cart.mmc3.mirror_horizontal = input.u8()? != 0;
    cart.mmc3.prg_ram_protect = input.u8()?;
    cart.mmc3.irq_latch = input.u8()?;
    cart.mmc3.irq_counter = input.u8()?;
    cart.mmc3.irq_reload = input.u8()? != 0;
    cart.mmc3.irq_enabled = input.u8()? != 0;
    cart.mmc3.irq_pending = input.u8()? != 0;
    Ok(())
}

fn read_exact_blob<const N: usize>(
    input: &mut StateReader<'_>,
    target: &mut [u8; N],
) -> Result<(), String> {
    let blob = input.blob()?;
    if blob.len() != N {
        return Err(format!(
            "save-state blob has {} bytes; expected {N}",
            blob.len()
        ));
    }
    target.copy_from_slice(blob);
    Ok(())
}
fn read_vec_blob(input: &mut StateReader<'_>, target: &mut [u8]) -> Result<(), String> {
    let blob = input.blob()?;
    if blob.len() != target.len() {
        return Err(format!(
            "save-state blob has {} bytes; expected {}",
            blob.len(),
            target.len()
        ));
    }
    target.copy_from_slice(blob);
    Ok(())
}
fn save_controller(out: &mut StateWriter, controller: &Controller) {
    out.u8(controller.live);
    out.u8(controller.latched);
    out.u8(controller.shift);
    out.u8(controller.strobe as u8);
}
fn load_controller(input: &mut StateReader<'_>, controller: &mut Controller) -> Result<(), String> {
    controller.live = input.u8()?;
    controller.latched = input.u8()?;
    controller.shift = input.u8()?;
    controller.strobe = input.u8()? != 0;
    Ok(())
}

fn save_ppu_registers(out: &mut StateWriter, ppu: &Ppu) {
    out.u8(ppu.ctrl);
    out.u8(ppu.mask);
    out.u8(ppu.status);
    out.u8(ppu.oam_addr);
    out.u16(ppu.v);
    out.u16(ppu.t);
    out.u8(ppu.fine_x);
    out.u8(ppu.write_toggle as u8);
    out.u8(ppu.data_buffer);
    out.u8(ppu.scroll_x);
    out.u8(ppu.scroll_y);
    out.u16(ppu.cycle);
    out.u16(ppu.scanline);
    out.u64(ppu.frame);
    out.u8(ppu.nmi_pending as u8);
}
fn load_ppu_registers(input: &mut StateReader<'_>, ppu: &mut Ppu) -> Result<(), String> {
    ppu.ctrl = input.u8()?;
    ppu.mask = input.u8()?;
    ppu.status = input.u8()?;
    ppu.oam_addr = input.u8()?;
    ppu.v = input.u16()?;
    ppu.t = input.u16()?;
    ppu.fine_x = input.u8()?;
    ppu.write_toggle = input.u8()? != 0;
    ppu.data_buffer = input.u8()?;
    ppu.scroll_x = input.u8()?;
    ppu.scroll_y = input.u8()?;
    ppu.cycle = input.u16()?;
    ppu.scanline = input.u16()?;
    ppu.frame = input.u64()?;
    ppu.nmi_pending = input.u8()? != 0;
    Ok(())
}

#[cfg(test)]
mod mapper_tests {
    use super::*;

    fn mapper_rom(mapper: u16, prg_banks: u8, chr_banks: u8) -> Vec<u8> {
        let mut rom = vec![0u8; 16 + prg_banks as usize * 0x4000 + chr_banks as usize * 0x2000];
        rom[..4].copy_from_slice(b"NES\x1a");
        rom[4] = prg_banks;
        rom[5] = chr_banks;
        rom[6] = ((mapper & 0x0f) as u8) << 4;
        rom[7] = (mapper & 0xf0) as u8;
        rom
    }

    #[test]
    fn uxrom_switches_lower_prg_and_fixes_last_bank() {
        let mut rom = mapper_rom(2, 4, 0);
        for bank in 0..4usize {
            rom[16 + bank * 0x4000] = bank as u8;
        }
        let (mut cart, _) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.read_prg(0x8000), 0);
        assert_eq!(cart.read_prg(0xc000), 3);
        cart.write_mapper(0x8000, 2);
        assert_eq!(cart.read_prg(0x8000), 2);
        assert_eq!(cart.read_prg(0xc000), 3);
    }
    #[test]
    fn cnrom_switches_chr_banks_inside_the_same_machine() {
        let mut rom = mapper_rom(3, 2, 2);
        let chr_start = 16 + 2 * 0x4000;
        rom[chr_start] = 0x11;
        rom[chr_start + 0x2000] = 0x77;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(ppu.read_vram(0), 0x11);
        cart.write_mapper(0x8000, 1);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0), 0x77);
    }
    fn mmc1_write(cart: &mut Cartridge, address: u16, value: u8) {
        for bit in 0..5 {
            cart.write_mapper(address, (value >> bit) & 1);
        }
    }

    #[test]
    fn mmc1_switches_prg_and_chr_banks() {
        let mut rom = mapper_rom(1, 4, 2);
        for bank in 0..4usize {
            rom[16 + bank * 0x4000] = (0x10 + bank) as u8;
        }
        let chr_start = 16 + 4 * 0x4000;
        for bank in 0..4usize {
            rom[chr_start + bank * 0x1000] = (0x40 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        mmc1_write(&mut cart, 0xe000, 2);
        assert_eq!(cart.read_prg(0x8000), 0x12);
        assert_eq!(cart.read_prg(0xc000), 0x13);
        mmc1_write(&mut cart, 0x8000, 0x1c);
        mmc1_write(&mut cart, 0xa000, 2);
        mmc1_write(&mut cart, 0xc000, 3);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x0000), 0x42);
        assert_eq!(ppu.read_vram(0x1000), 0x43);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen0);
    }
    #[test]
    fn mmc3_switches_prg_chr_and_raises_scanline_irq() {
        let mut rom = mapper_rom(4, 4, 2);
        for bank in 0..8usize {
            rom[16 + bank * 0x2000] = (0x20 + bank) as u8;
        }
        let chr_start = 16 + 4 * 0x4000;
        for bank in 0..16usize {
            rom[chr_start + bank * 0x400] = (0x60 + bank) as u8;
        }
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        cart.write_mapper(0x8000, 6);
        cart.write_mapper(0x8001, 3);
        assert_eq!(cart.read_prg(0x8000), 0x23);
        assert_eq!(cart.read_prg(0xe000), 0x27);
        cart.write_mapper(0x8000, 0x46);
        cart.write_mapper(0x8001, 2);
        assert_eq!(cart.read_prg(0x8000), 0x26);
        assert_eq!(cart.read_prg(0xc000), 0x22);
        cart.write_mapper(0x8000, 2);
        cart.write_mapper(0x8001, 5);
        cart.sync_ppu(&mut ppu);
        assert_eq!(ppu.read_vram(0x1000), 0x65);
        cart.write_mapper(0xc000, 2);
        cart.write_mapper(0xc001, 0);
        cart.write_mapper(0xe001, 0);
        cart.clock_scanline();
        cart.clock_scanline();
        cart.clock_scanline();
        assert!(cart.irq_pending());
        cart.write_mapper(0xe000, 0);
        assert!(!cart.irq_pending());
    }
    #[test]
    fn axrom_switches_32k_prg_and_one_screen_mirroring() {
        let mut rom = mapper_rom(7, 4, 0);
        rom[16] = 0x31;
        rom[16 + 0x8000] = 0x72;
        let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.read_prg(0x8000), 0x31);
        cart.write_mapper(0x8000, 0x11);
        cart.sync_ppu(&mut ppu);
        assert_eq!(cart.read_prg(0x8000), 0x72);
        assert_eq!(ppu.mirroring, Mirroring::SingleScreen1);
    }

    #[test]
    fn gxrom_and_color_dreams_switch_prg_and_chr_together() {
        for mapper in [11u16, 66u16] {
            let mut rom = mapper_rom(mapper, 4, 4);
            rom[16] = 0x10;
            rom[16 + 0x8000] = 0x20;
            let chr_start = 16 + 4 * 0x4000;
            rom[chr_start] = 0x40;
            rom[chr_start + 0x2000] = 0x50;
            rom[chr_start + 0x4000] = 0x60;
            let (mut cart, mut ppu) = Cartridge::parse(&rom).unwrap();
            let value = if mapper == 11 { 0x21 } else { 0x11 };
            cart.write_mapper(0x8000, value);
            cart.sync_ppu(&mut ppu);
            assert_eq!(cart.read_prg(0x8000), 0x20);
            assert_eq!(ppu.read_vram(0), if mapper == 11 { 0x60 } else { 0x50 });
        }
    }
    #[test]
    fn nes2_header_decodes_submapper_and_nonvolatile_ram() {
        let mut rom = vec![0u8; 16 + 32 * 1024 + 8 * 1024];
        rom[..4].copy_from_slice(b"NES\x1a");
        rom[4] = 2;
        rom[5] = 1;
        rom[6] = 0x22; // mapper 2 + battery flag
        rom[7] = 0x08; // NES 2.0 marker
        rom[8] = 0x30; // submapper 3, mapper high bits zero
        rom[10] = 0x70; // 8 KiB PRG NVRAM
        let (cart, _) = Cartridge::parse(&rom).unwrap();
        assert_eq!(cart.mapper, 2);
        assert_eq!(cart.submapper, 3);
        assert_eq!(cart.prg_ram.len(), 8 * 1024);
        assert_eq!(cart.battery_len, 8 * 1024);
    }

    #[test]
    fn nes2_exponent_multiplier_rom_size_is_checked() {
        let encoded = (10u8 << 2) | 1;
        assert_eq!(nes2_rom_size(encoded, 0x0f, 16 * 1024).unwrap(), 3072);
    }
}
