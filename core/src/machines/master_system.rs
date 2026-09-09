use super::sn76489::Sn76489;
use crate::cpu_z80::{Z80Bus, Z80};
use crate::input::{DOWN, FACE_EAST, FACE_SOUTH, LEFT, RIGHT, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 256;
const HEIGHT: u32 = 192;
const CPU_CLOCK: f64 = 3_579_545.0;
const FRAME_RATE: f64 = 59.922_743;
const STATE_VERSION: u32 = 1;

struct SegaCartridge {
    rom: Vec<u8>,
    sram: Vec<u8>,
    control: u8,
    pages: [u8; 3],
}

impl SegaCartridge {
    fn new(rom: &[u8]) -> Result<Self, String> {
        if rom.len() < 1024 {
            return Err("Master System cartridge image is too small".into());
        }
        let image = if rom.len() > 512 && rom.len() % 0x4000 == 512 {
            &rom[512..]
        } else {
            rom
        };
        Ok(Self {
            rom: image.to_vec(),
            sram: vec![0; 0x8000],
            control: 0,
            pages: [0, 1, 2],
        })
    }

    fn bank_count(&self) -> usize {
        self.rom.len().div_ceil(0x4000).max(1)
    }

    fn rom_byte(&self, bank: u8, offset: usize) -> u8 {
        let bank = usize::from(bank) % self.bank_count();
        self.rom[(bank * 0x4000 + offset) % self.rom.len()]
    }

    fn read(&self, address: u16) -> u8 {
        match address {
            0x0000..=0x03ff => self.rom[address as usize % self.rom.len()],
            0x0400..=0x3fff => self.rom_byte(self.pages[0], address as usize & 0x3fff),
            0x4000..=0x7fff => self.rom_byte(self.pages[1], address as usize & 0x3fff),
            0x8000..=0xbfff if self.control & 0x08 != 0 => {
                let bank = usize::from((self.control >> 2) & 1);
                self.sram[bank * 0x4000 + (address as usize & 0x3fff)]
            }
            0x8000..=0xbfff => self.rom_byte(self.pages[2], address as usize & 0x3fff),
            _ => 0xff,
        }
    }

    fn write_slot_two(&mut self, address: u16, value: u8) {
        if self.control & 0x08 == 0 {
            return;
        }
        let bank = usize::from((self.control >> 2) & 1);
        let index = bank * 0x4000 + (address as usize & 0x3fff);
        self.sram[index] = value;
    }

    fn write_mapper(&mut self, address: u16, value: u8) {
        match address {
            0xfffc => self.control = value,
            0xfffd => self.pages[0] = value,
            0xfffe => self.pages[1] = value,
            0xffff => self.pages[2] = value,
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.control);
        for page in self.pages {
            out.u8(page);
        }
        out.blob(&self.sram);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.control = input.u8()?;
        for page in &mut self.pages {
            *page = input.u8()?;
        }
        let sram = input.blob()?;
        if sram.len() != self.sram.len() {
            return Err("Master System save state has invalid cartridge RAM length".into());
        }
        self.sram.copy_from_slice(sram);
        Ok(())
    }
}

struct SmsVdp {
    vram: [u8; 0x4000],
    cram: [u8; 32],
    regs: [u8; 16],
    address: u16,
    code: u8,
    control_latch: Option<u8>,
    read_buffer: u8,
    status: u8,
    line_counter: u8,
    line_irq_pending: bool,
    dot: u16,
    dot_half: u8,
    scanline: u16,
    frame: u64,
    irq_pending: bool,
    video: VideoBuffer,
}

impl Default for SmsVdp {
    fn default() -> Self {
        Self {
            vram: [0; 0x4000],
            cram: [0; 32],
            regs: [0; 16],
            address: 0,
            code: 0,
            control_latch: None,
            read_buffer: 0,
            status: 0,
            line_counter: 0xff,
            line_irq_pending: false,
            dot: 0,
            dot_half: 0,
            scanline: 0,
            frame: 0,
            irq_pending: false,
            video: VideoBuffer::new(WIDTH, HEIGHT),
        }
    }
}

impl SmsVdp {
    fn reset(&mut self) {
        let vram = self.vram;
        let cram = self.cram;
        *self = Self::default();
        self.vram = vram;
        self.cram = cram;
    }

    fn write_control(&mut self, value: u8) {
        let Some(first) = self.control_latch.take() else {
            self.control_latch = Some(value);
            return;
        };
        self.code = value >> 6;
        if self.code == 2 {
            let register = (value & 0x0f) as usize;
            self.regs[register] = first;
            self.update_irq();
            return;
        }
        self.address = ((u16::from(value & 0x3f) << 8) | u16::from(first)) & 0x3fff;
        if self.code == 0 {
            self.read_buffer = self.vram[self.address as usize];
            self.address = (self.address + 1) & 0x3fff;
        }
    }

    fn write_data(&mut self, value: u8) {
        match self.code {
            3 => self.cram[(self.address as usize) & 0x1f] = value & 0x3f,
            _ => self.vram[self.address as usize] = value,
        }
        self.address = (self.address + 1) & 0x3fff;
        self.read_buffer = value;
        self.control_latch = None;
    }

    fn read_data(&mut self) -> u8 {
        let value = self.read_buffer;
        self.read_buffer = self.vram[self.address as usize];
        self.address = (self.address + 1) & 0x3fff;
        self.control_latch = None;
        value
    }

    fn read_status(&mut self) -> u8 {
        let value = self.status;
        self.status = 0;
        self.line_irq_pending = false;
        self.irq_pending = false;
        self.control_latch = None;
        value
    }

    fn update_irq(&mut self) {
        let frame_irq = self.status & 0x80 != 0 && self.regs[1] & 0x20 != 0;
        let line_irq = self.line_irq_pending && self.regs[0] & 0x10 != 0;
        self.irq_pending = frame_irq || line_irq;
    }

    fn clock_line_counter(&mut self) {
        if self.scanline < 192 {
            if self.line_counter == 0 {
                self.line_counter = self.regs[10];
                self.line_irq_pending = true;
            } else {
                self.line_counter = self.line_counter.wrapping_sub(1);
            }
        } else {
            self.line_counter = self.regs[10];
        }
        self.update_irq();
    }

    fn tick_cpu_cycles(&mut self, cycles: u32) {
        let scaled = cycles.saturating_mul(3) + u32::from(self.dot_half);
        self.dot_half = (scaled & 1) as u8;
        let mut dots = u32::from(self.dot) + scaled / 2;
        while dots >= 342 {
            dots -= 342;
            self.scanline = self.scanline.wrapping_add(1);
            self.clock_line_counter();
            if self.scanline == 192 {
                self.status |= 0x80;
                self.frame = self.frame.wrapping_add(1);
                self.render_frame();
                self.update_irq();
            }
            if self.scanline >= 262 {
                self.scanline = 0;
                self.line_counter = self.regs[10];
                self.update_irq();
            }
        }
        self.dot = dots as u16;
    }

    fn v_counter(&self) -> u8 {
        self.scanline as u8
    }

    fn h_counter(&self) -> u8 {
        ((u32::from(self.dot) * 256) / 342) as u8
    }

    fn render_frame(&mut self) {
        let backdrop = sms_color(self.cram[16 + usize::from(self.regs[7] & 0x0f)]);
        self.video.clear(backdrop);
        if self.regs[1] & 0x40 == 0 || self.regs[0] & 0x04 == 0 {
            return;
        }
        self.render_background();
        self.render_sprites();
    }

    fn render_background(&mut self) {
        let name_base = (u16::from(self.regs[2] & 0x0e) << 10) & 0x3fff;
        let base_scroll_x = usize::from(self.regs[8]);
        let base_scroll_y = usize::from(self.regs[9]);
        for y in 0..HEIGHT as usize {
            let scroll_x = if self.regs[0] & 0x40 != 0 && y < 16 {
                0
            } else {
                base_scroll_x
            };
            for x in 0..WIDTH as usize {
                let scroll_y = if self.regs[0] & 0x80 != 0 && x >= 192 {
                    0
                } else {
                    base_scroll_y
                };
                let world_y = (y + scroll_y) % 224;
                let tile_y = world_y / 8;
                let row = world_y & 7;
                let world_x = (x + 256 - scroll_x) & 0xff;
                let tile_x = world_x / 8;
                let entry = (name_base as usize + (tile_y * 32 + tile_x) * 2) & 0x3fff;
                let low = self.vram[entry];
                let high = self.vram[(entry + 1) & 0x3fff];
                let tile = u16::from(low) | (u16::from(high & 1) << 8);
                let flip_x = high & 0x02 != 0;
                let flip_y = high & 0x04 != 0;
                let palette = usize::from(high & 0x08 != 0);
                let source_y = if flip_y { 7 - row } else { row };
                let source_x = if flip_x {
                    world_x & 7
                } else {
                    7 - (world_x & 7)
                };
                let pattern = (usize::from(tile) * 32 + source_y * 4) & 0x3fff;
                let mut color = 0u8;
                for plane in 0..4 {
                    color |= ((self.vram[(pattern + plane) & 0x3fff] >> source_x) & 1) << plane;
                }
                let cram_index = palette * 16 + usize::from(color);
                let rgba = sms_color(self.cram[cram_index]);
                let offset = (y * WIDTH as usize + x) * 4;
                self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
            }
        }
        if self.regs[0] & 0x20 != 0 {
            for y in 0..HEIGHT as usize {
                for x in 0..8usize {
                    let offset = (y * WIDTH as usize + x) * 4;
                    let backdrop = sms_color(self.cram[16 + usize::from(self.regs[7] & 0x0f)]);
                    self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&backdrop);
                }
            }
        }
    }

    fn render_sprites(&mut self) {
        let base = (usize::from(self.regs[5] & 0x7e) << 7) & 0x3fff;
        let pattern_base = if self.regs[6] & 0x04 != 0 { 0x2000 } else { 0 };
        let sprite_height = if self.regs[1] & 0x02 != 0 { 16 } else { 8 };
        let zoom = if self.regs[1] & 0x01 != 0 { 2 } else { 1 };
        let mut occupied = vec![false; (WIDTH * HEIGHT) as usize];
        let mut line_count = [0u8; HEIGHT as usize];
        for sprite in 0..64usize {
            let raw_y = self.vram[(base + sprite) & 0x3fff];
            if raw_y == 0xd0 {
                break;
            }
            let mut y = i32::from(raw_y) + 1;
            if raw_y >= 0xe0 {
                y -= 256;
            }
            let pair = (base + 0x80 + sprite * 2) & 0x3fff;
            let mut x = i32::from(self.vram[pair]);
            if self.regs[0] & 0x08 != 0 {
                x -= 8;
            }
            let mut tile = self.vram[(pair + 1) & 0x3fff];
            if sprite_height == 16 {
                tile &= 0xfe;
            }
            for source_y in 0..sprite_height {
                for zy in 0..zoom {
                    let screen_y = y + (source_y * zoom + zy) as i32;
                    if !(0..HEIGHT as i32).contains(&screen_y) {
                        continue;
                    }
                    let line = screen_y as usize;
                    if line_count[line] >= 8 {
                        self.status |= 0x40;
                        continue;
                    }
                    let tile_offset = source_y / 8;
                    let row = source_y & 7;
                    let pattern = (pattern_base
                        + usize::from(tile.wrapping_add(tile_offset as u8)) * 32
                        + row * 4)
                        & 0x3fff;
                    for source_x in 0..8usize {
                        let bit = 7 - source_x;
                        let mut color = 0u8;
                        for plane in 0..4 {
                            color |= ((self.vram[(pattern + plane) & 0x3fff] >> bit) & 1) << plane;
                        }
                        if color == 0 {
                            continue;
                        }
                        for zx in 0..zoom {
                            let screen_x = x + (source_x * zoom + zx) as i32;
                            if !(0..WIDTH as i32).contains(&screen_x) {
                                continue;
                            }
                            let pixel = line * WIDTH as usize + screen_x as usize;
                            if occupied[pixel] {
                                self.status |= 0x20;
                            }
                            occupied[pixel] = true;
                            let rgba = sms_color(self.cram[16 + usize::from(color)]);
                            let offset = pixel * 4;
                            self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                        }
                    }
                    line_count[line] = line_count[line].saturating_add(1);
                }
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.vram);
        out.blob(&self.cram);
        out.blob(&self.regs);
        out.u16(self.address);
        out.u8(self.code);
        out.u8(self.control_latch.unwrap_or(0));
        out.u8(self.control_latch.is_some() as u8);
        out.u8(self.read_buffer);
        out.u8(self.status);
        out.u8(self.line_counter);
        out.u8(self.line_irq_pending as u8);
        out.u16(self.dot);
        out.u8(self.dot_half);
        out.u16(self.scanline);
        out.u64(self.frame);
        out.u8(self.irq_pending as u8);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let vram = input.blob()?;
        if vram.len() != self.vram.len() {
            return Err("invalid SMS VRAM state length".into());
        }
        self.vram.copy_from_slice(vram);
        let cram = input.blob()?;
        if cram.len() != self.cram.len() {
            return Err("invalid SMS CRAM state length".into());
        }
        self.cram.copy_from_slice(cram);
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("invalid SMS VDP register state length".into());
        }
        self.regs.copy_from_slice(regs);
        self.address = input.u16()? & 0x3fff;
        self.code = input.u8()? & 3;
        let latch = input.u8()?;
        self.control_latch = if input.u8()? != 0 { Some(latch) } else { None };
        self.read_buffer = input.u8()?;
        self.status = input.u8()?;
        self.line_counter = input.u8()?;
        self.line_irq_pending = input.u8()? != 0;
        self.dot = input.u16()?;
        self.dot_half = input.u8()?;
        self.scanline = input.u16()?;
        self.frame = input.u64()?;
        self.irq_pending = input.u8()? != 0;
        self.render_frame();
        Ok(())
    }
}

fn sms_color(value: u8) -> [u8; 4] {
    let r = (value & 3) * 85;
    let g = ((value >> 2) & 3) * 85;
    let b = ((value >> 4) & 3) * 85;
    [r, g, b, 255]
}

struct SmsBus {
    ram: [u8; 0x2000],
    cartridge: SegaCartridge,
    vdp: SmsVdp,
    psg: Sn76489,
    port_dc: u8,
    port_dd: u8,
    io_control: u8,
}

impl SmsBus {
    fn new(cartridge: SegaCartridge) -> Self {
        Self {
            ram: [0; 0x2000],
            cartridge,
            vdp: SmsVdp::default(),
            psg: Sn76489::default(),
            port_dc: 0xff,
            port_dd: 0xff,
            io_control: 0xff,
        }
    }

    fn set_inputs(&mut self, input: &InputState) {
        let p1 = input.buttons[0];
        let p2 = input.buttons[1];
        self.port_dc = 0xff;
        self.port_dd = 0xff;
        Self::clear_if_pressed(&mut self.port_dc, 0, p1 & UP != 0);
        Self::clear_if_pressed(&mut self.port_dc, 1, p1 & DOWN != 0);
        Self::clear_if_pressed(&mut self.port_dc, 2, p1 & LEFT != 0);
        Self::clear_if_pressed(&mut self.port_dc, 3, p1 & RIGHT != 0);
        Self::clear_if_pressed(&mut self.port_dc, 4, p1 & FACE_SOUTH != 0);
        Self::clear_if_pressed(&mut self.port_dc, 5, p1 & FACE_EAST != 0);
        Self::clear_if_pressed(&mut self.port_dc, 6, p2 & UP != 0);
        Self::clear_if_pressed(&mut self.port_dc, 7, p2 & DOWN != 0);
        Self::clear_if_pressed(&mut self.port_dd, 0, p2 & LEFT != 0);
        Self::clear_if_pressed(&mut self.port_dd, 1, p2 & RIGHT != 0);
        Self::clear_if_pressed(&mut self.port_dd, 2, p2 & FACE_SOUTH != 0);
        Self::clear_if_pressed(&mut self.port_dd, 3, p2 & FACE_EAST != 0);
    }

    fn clear_if_pressed(value: &mut u8, bit: u8, pressed: bool) {
        if pressed {
            *value &= !(1 << bit);
        }
    }

    fn tick(&mut self, cycles: u32) {
        self.vdp.tick_cpu_cycles(cycles);
        self.psg.tick_cpu_cycles(cycles);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        self.cartridge.save(out);
        self.vdp.save(out);
        self.psg.save(out);
        out.u8(self.port_dc);
        out.u8(self.port_dd);
        out.u8(self.io_control);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("invalid Master System RAM state length".into());
        }
        self.ram.copy_from_slice(ram);
        self.cartridge.load(input)?;
        self.vdp.load(input)?;
        self.psg.load(input)?;
        self.port_dc = input.u8()?;
        self.port_dd = input.u8()?;
        self.io_control = input.u8()?;
        Ok(())
    }
}

impl Z80Bus for SmsBus {
    fn mem_read(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0xbfff => self.cartridge.read(address),
            0xc000..=0xffff => self.ram[(address as usize) & 0x1fff],
        }
    }

    fn mem_write(&mut self, address: u16, value: u8) {
        match address {
            0x8000..=0xbfff => self.cartridge.write_slot_two(address, value),
            0xc000..=0xffff => {
                self.ram[(address as usize) & 0x1fff] = value;
                if address >= 0xfffc {
                    self.cartridge.write_mapper(address, value);
                }
            }
            _ => {}
        }
    }

    fn io_read(&mut self, port: u16) -> u8 {
        let low = port as u8;
        match low {
            0x40..=0x7f if low & 1 == 0 => self.vdp.v_counter(),
            0x40..=0x7f => self.vdp.h_counter(),
            0x80..=0xbf if low & 1 == 0 => self.vdp.read_data(),
            0x80..=0xbf => self.vdp.read_status(),
            0xc0..=0xff if low & 1 == 0 => self.port_dc,
            0xc0..=0xff => self.port_dd,
            _ => 0xff,
        }
    }

    fn io_write(&mut self, port: u16, value: u8) {
        let low = port as u8;
        match low {
            0x3f => self.io_control = value,
            0x40..=0x7f => self.psg.write(value),
            0x80..=0xbf if low & 1 == 0 => self.vdp.write_data(value),
            0x80..=0xbf => self.vdp.write_control(value),
            _ => {}
        }
    }
}

pub struct MasterSystemMachine {
    cpu: Z80,
    bus: SmsBus,
    audio: AudioBuffer,
    powered: bool,
}

impl MasterSystemMachine {
    pub fn from_rom(rom: &[u8]) -> Result<Self, String> {
        let cartridge = SegaCartridge::new(rom)?;
        let bus = SmsBus::new(cartridge);
        Ok(Self {
            cpu: Z80::default(),
            bus,
            audio: AudioBuffer::new(48_000, 2),
            powered: true,
        })
    }

    fn clock_instruction(&mut self) -> u32 {
        let cycles = self.cpu.step(&mut self.bus);
        self.bus.tick(cycles);
        if self.bus.vdp.irq_pending {
            let irq_cycles = self.cpu.irq(&mut self.bus, 0xff);
            if irq_cycles != 0 {
                self.bus.tick(irq_cycles);
            }
        }
        cycles
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        for sample in self.bus.psg.samples() {
            self.audio.push_stereo(*sample, *sample);
        }
    }
}

impl Machine for MasterSystemMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::MasterSystem
    }

    fn reset(&mut self) {
        self.cpu.reset();
        self.bus.ram = [0; 0x2000];
        self.bus.cartridge.control = 0;
        self.bus.cartridge.pages = [0, 1, 2];
        self.bus.vdp.reset();
        self.bus.psg.reset();
        self.powered = true;
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        self.bus.psg.begin_frame();
        let target = self.bus.vdp.frame.wrapping_add(1);
        let cycle_budget = (CPU_CLOCK / FRAME_RATE * 2.0).ceil() as u64;
        let deadline = self.cpu.cycles.saturating_add(cycle_budget);
        while self.bus.vdp.frame != target && self.cpu.cycles < deadline {
            self.clock_instruction();
        }
        if self.bus.vdp.frame != target {
            self.powered = false;
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }

    fn video(&self) -> &VideoBuffer {
        &self.bus.vdp.video
    }

    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::MasterSystem, STATE_VERSION);
        self.cpu.save(&mut out);
        self.bus.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::MasterSystem, STATE_VERSION)?;
        self.cpu.load(&mut input)?;
        self.bus.load(&mut input)?;
        self.powered = input.u8()? != 0;
        self.audio.begin_frame();
        input.finish()
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            self.bus.cartridge.sram.len()
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        let len = self.persistent_len(kind, slot);
        if len == 0 {
            return Err("Master System cartridge storage slot is unavailable".into());
        }
        if out.len() != len {
            return Err(format!(
                "persistent output has {} bytes; expected {len}",
                out.len()
            ));
        }
        out.copy_from_slice(&self.bus.cartridge.sram);
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
            return Err("Master System cartridge storage slot is unavailable".into());
        }
        if data.len() != len {
            return Err(format!(
                "persistent input has {} bytes; expected {len}",
                data.len()
            ));
        }
        self.bus.cartridge.sram.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_rom() -> Vec<u8> {
        let mut rom = vec![0; 0x8000];
        let program: &[u8] = &[
            0xf3, 0x31, 0xf0, 0xdf, 0x3e, 0x04, 0xd3, 0xbf, 0x3e, 0x80, 0xd3, 0xbf, 0x3e, 0xc0,
            0xd3, 0xbf, 0x3e, 0x81, 0xd3, 0xbf, 0x3e, 0xff, 0xd3, 0xbf, 0x3e, 0x82, 0xd3, 0xbf,
            0x3e, 0xff, 0xd3, 0xbf, 0x3e, 0x85, 0xd3, 0xbf, 0x3e, 0xfb, 0xd3, 0xbf, 0x3e, 0x86,
            0xd3, 0xbf, 0x3e, 0x01, 0xd3, 0xbf, 0x3e, 0xc0, 0xd3, 0xbf, 0x3e, 0x03, 0xd3, 0xbe,
            0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x40, 0xd3, 0xbf, 0x21, 0x00, 0x01, 0x06, 0x20, 0x7e,
            0xd3, 0xbe, 0x23, 0x10, 0xfa, 0xc3, 0x4b, 0x00,
        ];
        rom[..program.len()].copy_from_slice(program);
        for row in 0..8usize {
            let base = 0x100 + row * 4;
            rom[base] = 0xff;
        }
        rom
    }

    #[test]
    fn synthetic_cartridge_runs_z80_vdp_and_psg_frame() {
        let rom = synthetic_rom();
        let mut machine = MasterSystemMachine::from_rom(&rom).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.video().width(), WIDTH);
        assert_eq!(machine.video().height(), HEIGHT);
        assert!(machine
            .video()
            .pixels()
            .as_chunks::<4>()
            .0
            .iter()
            .any(|pixel| pixel[0] > pixel[1] && pixel[0] > pixel[2]));
        assert!(!machine.audio().samples().is_empty());
    }

    #[test]
    fn save_state_round_trip_restores_cpu_and_video_state() {
        let rom = synthetic_rom();
        let mut machine = MasterSystemMachine::from_rom(&rom).unwrap();
        machine.run_frame(&InputState::default());
        let saved = machine.save_state().unwrap();
        let pc = machine.cpu.pc;
        let frame = machine.bus.vdp.frame;
        machine.run_frame(&InputState::default());
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.cpu.pc, pc);
        assert_eq!(machine.bus.vdp.frame, frame);
    }

    #[test]
    fn sega_mapper_switches_rom_and_cartridge_ram() {
        let mut rom = vec![0; 0x10000];
        for bank in 0..4usize {
            rom[bank * 0x4000] = bank as u8;
        }
        let mut cart = SegaCartridge::new(&rom).unwrap();
        assert_eq!(cart.read(0x4000), 1);
        cart.write_mapper(0xfffe, 3);
        assert_eq!(cart.read(0x4000), 3);
        cart.write_mapper(0xfffc, 0x08);
        cart.write_slot_two(0x8000, 0x5a);
        assert_eq!(cart.read(0x8000), 0x5a);
        cart.write_mapper(0xfffc, 0x0c);
        cart.write_slot_two(0x8000, 0xa5);
        assert_eq!(cart.read(0x8000), 0xa5);
        cart.write_mapper(0xfffc, 0x08);
        assert_eq!(cart.read(0x8000), 0x5a);
    }
    #[test]
    fn line_interrupt_counter_asserts_and_status_acknowledges_it() {
        let mut vdp = SmsVdp::default();
        vdp.regs[0] = 0x14;
        vdp.regs[10] = 1;
        vdp.line_counter = 1;
        vdp.tick_cpu_cycles(456);
        assert!(vdp.line_irq_pending);
        assert!(vdp.irq_pending);
        let _ = vdp.read_status();
        assert!(!vdp.line_irq_pending);
        assert!(!vdp.irq_pending);
    }
}
