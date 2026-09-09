use crate::bus::Bus8;
use crate::cpu6502::Mos6502;
use crate::input::{DOWN, FACE_EAST, FACE_SOUTH, LEFT, RIGHT, SELECT, START, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;

const WIDTH: u32 = 256;
const HEIGHT: u32 = 240;
const CPU_CLOCK: f64 = 1_789_773.0;
const FRAME_RATE: f64 = 60.0988;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mirroring {
    Horizontal,
    Vertical,
}

struct Cartridge {
    prg_rom: Vec<u8>,
    prg_ram: Vec<u8>,
    mapper: u8,
    prg_bank: usize,
}

impl Cartridge {
    fn parse(rom: &[u8]) -> Result<(Self, Ppu), String> {
        if rom.len() < 16 || &rom[..4] != b"NES\x1a" {
            return Err("NES image is not an iNES/NES 2.0 file".into());
        }
        let prg_size = rom[4] as usize * 16 * 1024;
        let chr_size = rom[5] as usize * 8 * 1024;
        let flags6 = rom[6];
        let flags7 = rom[7];
        let mapper = (flags6 >> 4) | (flags7 & 0xf0);
        if !matches!(mapper, 0 | 2 | 3) {
            return Err(format!(
                "NES mapper {mapper} is not implemented in OmniCore yet"
            ));
        }
        if flags6 & 0x08 != 0 {
            return Err("four-screen NES mirroring is not implemented yet".into());
        }
        let trainer = if flags6 & 0x04 != 0 { 512 } else { 0 };
        let offset = 16 + trainer;
        match mapper {
            0 | 3 if prg_size != 16 * 1024 && prg_size != 32 * 1024 => {
                return Err(format!(
                    "mapper {mapper} requires 16 or 32 KiB PRG ROM, got {prg_size} bytes"
                ));
            }
            2 if prg_size < 32 * 1024 || !prg_size.is_multiple_of(16 * 1024) => {
                return Err(format!(
                    "UxROM requires 16 KiB PRG banks and at least 32 KiB, got {prg_size} bytes"
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
        let chr = if chr_size == 0 {
            vec![0; 8 * 1024]
        } else {
            rom[chr_start..chr_start + chr_size].to_vec()
        };
        let mirroring = if flags6 & 1 != 0 {
            Mirroring::Vertical
        } else {
            Mirroring::Horizontal
        };
        let ppu = Ppu::new(chr, chr_size == 0, mirroring);
        let mut prg_ram = vec![0; 8 * 1024];
        if trainer != 0 {
            prg_ram[0x1000..0x1200].copy_from_slice(&rom[16..16 + 512]);
        }
        Ok((
            Self {
                prg_rom,
                prg_ram,
                mapper,
                prg_bank: 0,
            },
            ppu,
        ))
    }

    fn read_prg(&self, address: u16) -> u8 {
        debug_assert!(address >= 0x8000);
        match self.mapper {
            2 => {
                let banks = self.prg_rom.len() / 0x4000;
                let bank = if address < 0xc000 {
                    self.prg_bank % banks
                } else {
                    banks - 1
                };
                self.prg_rom[bank * 0x4000 + (address as usize & 0x3fff)]
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

    fn write_mapper(&mut self, value: u8) -> Option<u8> {
        match self.mapper {
            2 => {
                self.prg_bank = value as usize;
                None
            }
            3 => Some(value),
            _ => None,
        }
    }

    fn read_ram(&self, address: u16) -> u8 {
        self.prg_ram[(address as usize - 0x6000) & 0x1fff]
    }

    fn write_ram(&mut self, address: u16, value: u8) {
        let index = (address as usize - 0x6000) & 0x1fff;
        self.prg_ram[index] = value;
    }
}

struct Ppu {
    chr: Vec<u8>,
    chr_ram: bool,
    chr_bank: usize,
    chr_bank_count: usize,
    mirroring: Mirroring,
    nametable: [u8; 2048],
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
            chr_bank_count: (chr.len() / 0x2000).max(1),
            chr_bank: 0,
            chr,
            chr_ram,
            mirroring,
            nametable: [0; 2048],
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
                let index = self.chr_bank * 0x2000 + address as usize;
                self.chr[index % self.chr.len()]
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
                let index = self.chr_bank * 0x2000 + address as usize;
                let len = self.chr.len();
                self.chr[index % len] = value;
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

    fn set_chr_bank(&mut self, bank: u8) {
        self.chr_bank = bank as usize % self.chr_bank_count;
    }

    fn take_nmi(&mut self) -> bool {
        let pending = self.nmi_pending;
        self.nmi_pending = false;
        pending
    }
    fn tick(&mut self, ticks: u32) {
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
                self.scanline += 1;
                if self.scanline >= 262 {
                    self.scanline = 0;
                }
            }
        }
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
    dma_page: Option<u8>,
}

impl NesBus {
    fn new(cartridge: Cartridge, ppu: Ppu) -> Self {
        Self {
            ram: [0; 2048],
            cartridge,
            ppu,
            controllers: [Controller::default(), Controller::default()],
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
            0x4015 => 0,
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
            0x4000..=0x4013 => {}
            0x4014 => self.dma_page = Some(value),
            0x4015 => {}
            0x4016 => {
                self.controllers[0].write_strobe(value);
                self.controllers[1].write_strobe(value);
            }
            0x4017 => {}
            0x6000..=0x7fff => self.cartridge.write_ram(address, value),
            0x8000..=0xffff => {
                if let Some(chr_bank) = self.cartridge.write_mapper(value) {
                    self.ppu.set_chr_bank(chr_bank);
                }
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

    fn clock_cpu_instruction(&mut self) -> u32 {
        let cycles = self.cpu.step(&mut self.bus);
        if cycles == 0 {
            return 0;
        }
        self.bus.ppu.tick(cycles * 3);
        if let Some(page) = self.bus.dma_page.take() {
            self.bus.perform_dma(page);
            let dma_cycles = 513 + (self.cpu.cycles as u32 & 1);
            self.cpu.cycles += dma_cycles as u64;
            self.bus.ppu.tick(dma_cycles * 3);
        }
        if self.bus.ppu.take_nmi() {
            let before = self.cpu.cycles;
            self.cpu.nmi(&mut self.bus);
            self.bus.ppu.tick(((self.cpu.cycles - before) as u32) * 3);
        }
        cycles
    }

    fn synth_silence(&mut self) {
        self.audio.begin_frame();
        let frames = (self.audio.sample_rate() as f64 / FRAME_RATE).round() as usize;
        for _ in 0..frames {
            self.audio.push_stereo(0.0, 0.0);
        }
    }
}

impl Machine for NesMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Nes
    }

    fn reset(&mut self) {
        self.bus.ppu.reset();
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
        self.synth_silence();
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
        Err("NES save states are disabled while the machine remains in foundation status".into())
    }

    fn load_state(&mut self, _bytes: &[u8]) -> Result<(), String> {
        Err("NES save states are disabled while the machine remains in foundation status".into())
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
}

#[cfg(test)]
mod mapper_tests {
    use super::*;

    fn mapper_rom(mapper: u8, prg_banks: u8, chr_banks: u8) -> Vec<u8> {
        let mut rom = vec![0u8; 16 + prg_banks as usize * 0x4000 + chr_banks as usize * 0x2000];
        rom[..4].copy_from_slice(b"NES\x1a");
        rom[4] = prg_banks;
        rom[5] = chr_banks;
        rom[6] = (mapper & 0x0f) << 4;
        rom[7] = mapper & 0xf0;
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
        assert_eq!(cart.write_mapper(2), None);
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
        let bank = cart
            .write_mapper(1)
            .expect("CNROM mapper should select CHR");
        ppu.set_chr_bank(bank);
        assert_eq!(ppu.read_vram(0), 0x77);
    }
}
