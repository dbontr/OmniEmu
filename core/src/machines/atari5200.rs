use super::pokey::Pokey;
use crate::bus::Bus8;
use crate::cpu6502::Mos6502;
use crate::input::{AXIS_LEFT_X, AXIS_LEFT_Y, DOWN, FACE_SOUTH, LEFT, RIGHT, SELECT, START, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 192;
const CPU_HZ: u64 = 1_789_790;
const FRAME_RATE: f64 = 59.94;
const CPU_CYCLES_PER_LINE: u16 = 114;
const SCANLINES: u16 = 262;
const STATE_VERSION: u32 = 1;

struct Atari5200Cartridge {
    rom: Vec<u8>,
}

impl Atari5200Cartridge {
    fn new(rom: &[u8]) -> Result<Self, String> {
        if rom.is_empty() || rom.len() > 0x8000 {
            return Err("Atari 5200 cartridge must contain 1-32 KiB".into());
        }
        Ok(Self { rom: rom.to_vec() })
    }

    fn read(&self, address: u16) -> u8 {
        let window = usize::from(address.saturating_sub(0x4000));
        if self.rom.len() == 0x8000 {
            self.rom[window & 0x7fff]
        } else {
            let base = 0x8000usize.saturating_sub(self.rom.len());
            if window < base {
                0xff
            } else {
                self.rom[(window - base) % self.rom.len()]
            }
        }
    }
}

#[derive(Clone)]
struct Gtia {
    write_regs: [u8; 32],
    triggers: [bool; 4],
    console: u8,
}

impl Default for Gtia {
    fn default() -> Self {
        Self {
            write_regs: [0; 32],
            triggers: [false; 4],
            console: 0x07,
        }
    }
}

impl Gtia {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn set_inputs(&mut self, input: &InputState) {
        for player in 0..4 {
            self.triggers[player] = input.buttons[player] & FACE_SOUTH != 0;
        }
        self.console = 0x07;
        if input.buttons[0] & START != 0 {
            self.console &= !0x01;
        }
        if input.buttons[0] & SELECT != 0 {
            self.console &= !0x02;
        }
    }

    fn read(&self, address: u16) -> u8 {
        match address & 0x1f {
            0x10..=0x13 => u8::from(!self.triggers[(address as usize) & 3]),
            0x1f => self.console,
            _ => 0,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        self.write_regs[(address as usize) & 0x1f] = value;
    }

    fn playfield_color(&self, index: usize) -> u8 {
        match index & 3 {
            0 => self.write_regs[0x16],
            1 => self.write_regs[0x17],
            2 => self.write_regs[0x18],
            _ => self.write_regs[0x19],
        }
    }

    fn background_color(&self) -> u8 {
        self.write_regs[0x1a]
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.write_regs);
        for trigger in self.triggers {
            out.u8(trigger as u8);
        }
        out.u8(self.console);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.write_regs.len() {
            return Err("GTIA state has invalid register length".into());
        }
        self.write_regs.copy_from_slice(regs);
        for trigger in &mut self.triggers {
            *trigger = input.u8()? != 0;
        }
        self.console = input.u8()?;
        Ok(())
    }
}

struct Antic {
    regs: [u8; 16],
    scanline: u16,
    cycle_in_line: u16,
    frame: u64,
    nmi_status: u8,
    nmi_pending: bool,
    wsync: bool,
    video: VideoBuffer,
}

impl Default for Antic {
    fn default() -> Self {
        Self {
            regs: [0; 16],
            scanline: 0,
            cycle_in_line: 0,
            frame: 0,
            nmi_status: 0,
            nmi_pending: false,
            wsync: false,
            video: VideoBuffer::new(WIDTH, HEIGHT),
        }
    }
}

impl Antic {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn read(&self, address: u16) -> u8 {
        match address & 0x0f {
            0x0b => (self.scanline / 2) as u8,
            0x0f => self.nmi_status,
            _ => 0xff,
        }
    }

    fn write(&mut self, address: u16, value: u8) {
        let register = (address & 0x0f) as usize;
        match register {
            0x0a => self.wsync = true,
            0x0f => {
                self.nmi_status = 0;
                self.nmi_pending = false;
            }
            _ => self.regs[register] = value,
        }
    }

    fn take_wsync(&mut self) -> bool {
        std::mem::take(&mut self.wsync)
    }

    fn cycles_until_line_end(&self) -> u32 {
        u32::from(
            CPU_CYCLES_PER_LINE
                .saturating_sub(self.cycle_in_line)
                .max(1),
        )
    }

    fn take_nmi(&mut self) -> bool {
        std::mem::take(&mut self.nmi_pending)
    }

    fn tick(&mut self, cycles: u32) -> bool {
        let mut frame_ready = false;
        let mut total = u32::from(self.cycle_in_line) + cycles;
        while total >= u32::from(CPU_CYCLES_PER_LINE) {
            total -= u32::from(CPU_CYCLES_PER_LINE);
            self.scanline = self.scanline.wrapping_add(1);
            if self.scanline == HEIGHT as u16 {
                frame_ready = true;
            }
            if self.scanline == 248 {
                self.nmi_status |= 0x40;
                if self.regs[0x0e] & 0x40 != 0 {
                    self.nmi_pending = true;
                }
            }
            if self.scanline >= SCANLINES {
                self.scanline = 0;
                self.frame = self.frame.wrapping_add(1);
            }
        }
        self.cycle_in_line = total as u16;
        frame_ready
    }

    fn dma_read(ram: &[u8; 0x4000], address: u16) -> u8 {
        ram[(address as usize) & 0x3fff]
    }

    fn mode_height(mode: u8) -> usize {
        match mode {
            2 | 4 | 6 => 8,
            3 => 10,
            5 | 7 => 16,
            8 => 8,
            9 | 10 => 4,
            11 | 13 => 2,
            _ => 1,
        }
    }

    fn mode_columns(mode: u8) -> usize {
        match mode {
            6 | 7 => 20,
            _ => 40,
        }
    }

    fn render_text_row(
        &mut self,
        ram: &[u8; 0x4000],
        gtia: &Gtia,
        mode: u8,
        screen: u16,
        y_start: usize,
    ) {
        let columns = Self::mode_columns(mode);
        let height = Self::mode_height(mode);
        let scale = WIDTH as usize / (columns * 8);
        let charset = u16::from(self.regs[9]) << 8;
        for row in 0..height.min(HEIGHT as usize - y_start) {
            let glyph_row = row & 7;
            for column in 0..columns {
                let character = Self::dma_read(ram, screen.wrapping_add(column as u16));
                let glyph_addr = charset
                    .wrapping_add(u16::from(character & 0x7f) * 8)
                    .wrapping_add(glyph_row as u16);
                let glyph = Self::dma_read(ram, glyph_addr);
                let color_index = if mode >= 4 {
                    usize::from(character >> 6)
                } else {
                    2
                };
                let foreground = atari_color(gtia.playfield_color(color_index));
                let background = atari_color(gtia.background_color());
                for bit in 0..8usize {
                    let rgba = if glyph & (0x80 >> bit) != 0 {
                        foreground
                    } else {
                        background
                    };
                    let x0 = (column * 8 + bit) * scale;
                    for dx in 0..scale {
                        let x = x0 + dx;
                        if x < WIDTH as usize {
                            let offset = ((y_start + row) * WIDTH as usize + x) * 4;
                            self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                        }
                    }
                }
            }
        }
    }

    fn render_bitmap_row(
        &mut self,
        ram: &[u8; 0x4000],
        gtia: &Gtia,
        mode: u8,
        screen: u16,
        y_start: usize,
    ) {
        let height = Self::mode_height(mode).min(HEIGHT as usize - y_start);
        let high_resolution = mode == 15;
        for row in 0..height {
            for byte_index in 0..40usize {
                let value = Self::dma_read(ram, screen.wrapping_add(byte_index as u16));
                if high_resolution {
                    for bit in 0..8usize {
                        let set = value & (0x80 >> bit) != 0;
                        let rgba = if set {
                            atari_color(gtia.playfield_color(2))
                        } else {
                            atari_color(gtia.background_color())
                        };
                        let x = byte_index * 8 + bit;
                        let offset = ((y_start + row) * WIDTH as usize + x) * 4;
                        self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                    }
                } else {
                    for pixel in 0..4usize {
                        let shift = 6 - pixel * 2;
                        let color = usize::from((value >> shift) & 3);
                        let rgba = if color == 0 {
                            atari_color(gtia.background_color())
                        } else {
                            atari_color(gtia.playfield_color(color - 1))
                        };
                        let x0 = (byte_index * 4 + pixel) * 2;
                        for dx in 0..2usize {
                            let x = x0 + dx;
                            let offset = ((y_start + row) * WIDTH as usize + x) * 4;
                            self.video.pixels_mut()[offset..offset + 4].copy_from_slice(&rgba);
                        }
                    }
                }
            }
        }
    }

    fn render_frame(&mut self, ram: &[u8; 0x4000], gtia: &Gtia) {
        self.video.clear(atari_color(gtia.background_color()));
        if self.regs[0] & 0x20 == 0 {
            return;
        }
        let mut display = u16::from(self.regs[2]) | (u16::from(self.regs[3]) << 8);
        let mut screen = 0u16;
        let mut y = 0usize;
        let mut guard = 0usize;
        while y < HEIGHT as usize && guard < 1024 {
            guard += 1;
            let instruction = Self::dma_read(ram, display);
            display = display.wrapping_add(1);
            let mode = instruction & 0x0f;
            if mode == 0 {
                let lines = usize::from((instruction >> 4) & 7) + 1;
                y = (y + lines).min(HEIGHT as usize);
                continue;
            }
            if mode == 1 {
                let lo = Self::dma_read(ram, display);
                let hi = Self::dma_read(ram, display.wrapping_add(1));
                display = u16::from_le_bytes([lo, hi]);
                if instruction & 0x40 != 0 {
                    break;
                }
                continue;
            }
            if instruction & 0x40 != 0 {
                let lo = Self::dma_read(ram, display);
                let hi = Self::dma_read(ram, display.wrapping_add(1));
                screen = u16::from_le_bytes([lo, hi]);
                display = display.wrapping_add(2);
            }
            if (2..=7).contains(&mode) {
                self.render_text_row(ram, gtia, mode, screen, y);
            } else {
                self.render_bitmap_row(ram, gtia, mode, screen, y);
            }
            let bytes = Self::mode_columns(mode) as u16;
            screen = screen.wrapping_add(bytes);
            y = (y + Self::mode_height(mode)).min(HEIGHT as usize);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
        out.u16(self.scanline);
        out.u16(self.cycle_in_line);
        out.u64(self.frame);
        out.u8(self.nmi_status);
        out.u8(self.nmi_pending as u8);
        out.u8(self.wsync as u8);
        out.blob(self.video.pixels());
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("ANTIC state has invalid register length".into());
        }
        self.regs.copy_from_slice(regs);
        self.scanline = input.u16()? % SCANLINES;
        self.cycle_in_line = input.u16()? % CPU_CYCLES_PER_LINE;
        self.frame = input.u64()?;
        self.nmi_status = input.u8()?;
        self.nmi_pending = input.u8()? != 0;
        self.wsync = input.u8()? != 0;
        let video = input.blob()?;
        if video.len() != self.video.pixels().len() {
            return Err("ANTIC state has invalid framebuffer length".into());
        }
        self.video.pixels_mut().copy_from_slice(video);
        Ok(())
    }
}

fn atari_color(value: u8) -> [u8; 4] {
    let hue = f32::from(value >> 4) / 16.0;
    let luma = f32::from(value & 0x0e) / 14.0;
    let saturation = if value & 0xf0 == 0 { 0.0 } else { 0.72 };
    let h = hue * 6.0;
    let c = luma * saturation;
    let x = c * (1.0 - ((h % 2.0) - 1.0).abs());
    let (r1, g1, b1) = match h as u8 % 6 {
        0 => (c, x, 0.0),
        1 => (x, c, 0.0),
        2 => (0.0, c, x),
        3 => (0.0, x, c),
        4 => (x, 0.0, c),
        _ => (c, 0.0, x),
    };
    let m = luma - c;
    [
        ((r1 + m).clamp(0.0, 1.0) * 255.0) as u8,
        ((g1 + m).clamp(0.0, 1.0) * 255.0) as u8,
        ((b1 + m).clamp(0.0, 1.0) * 255.0) as u8,
        255,
    ]
}

struct Atari5200Bus {
    ram: [u8; 0x4000],
    cartridge: Atari5200Cartridge,
    bios: [u8; 0x0800],
    gtia: Gtia,
    antic: Antic,
    pokey: Pokey,
}

impl Atari5200Bus {
    fn new(cartridge: Atari5200Cartridge, bios: &[u8]) -> Result<Self, String> {
        if bios.len() != 0x0800 {
            return Err(format!(
                "Atari 5200 BIOS must be exactly 2048 bytes, got {}",
                bios.len()
            ));
        }
        let mut bios_bytes = [0u8; 0x0800];
        bios_bytes.copy_from_slice(bios);
        Ok(Self {
            ram: [0; 0x4000],
            cartridge,
            bios: bios_bytes,
            gtia: Gtia::default(),
            antic: Antic::default(),
            pokey: Pokey::new(CPU_HZ),
        })
    }

    fn reset_devices(&mut self) {
        self.gtia.reset();
        self.antic.reset();
        self.pokey.reset();
    }

    fn tick(&mut self, cycles: u32) {
        self.pokey.tick(cycles);
        if self.antic.tick(cycles) {
            self.antic.render_frame(&self.ram, &self.gtia);
        }
    }

    fn set_inputs(&mut self, input: &InputState) {
        self.gtia.set_inputs(input);
        for player in 0..2usize {
            let mut x = input.axes[player][AXIS_LEFT_X];
            let mut y = input.axes[player][AXIS_LEFT_Y];
            let buttons = input.buttons[player];
            if buttons & LEFT != 0 {
                x = i16::MIN;
            } else if buttons & RIGHT != 0 {
                x = i16::MAX;
            }
            if buttons & UP != 0 {
                y = i16::MIN;
            } else if buttons & DOWN != 0 {
                y = i16::MAX;
            }
            self.pokey.set_pot(player * 2, axis_to_pot(x));
            self.pokey.set_pot(player * 2 + 1, axis_to_pot(y));
        }
        let keypad = if input.buttons[0] & START != 0 {
            0x0c
        } else if input.buttons[0] & SELECT != 0 {
            0x05
        } else {
            0xff
        };
        self.pokey.set_keyboard(keypad);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        self.gtia.save(out);
        self.antic.save(out);
        self.pokey.save(out);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("Atari 5200 state has invalid RAM length".into());
        }
        self.ram.copy_from_slice(ram);
        self.gtia.load(input)?;
        self.antic.load(input)?;
        self.pokey.load(input)
    }
}

fn axis_to_pot(value: i16) -> u8 {
    let normalized = i32::from(value) - i32::from(i16::MIN);
    ((normalized * 228) / 65_535).clamp(0, 228) as u8
}

impl Bus8 for Atari5200Bus {
    fn read8(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0x3fff => self.ram[address as usize],
            0x4000..=0xbfff => self.cartridge.read(address),
            0xc000..=0xc0ff => self.gtia.read(address),
            0xd400..=0xd4ff => self.antic.read(address),
            0xe800..=0xe8ff => self.pokey.read(address),
            0xf800..=0xffff => self.bios[(address as usize) & 0x07ff],
            _ => 0xff,
        }
    }

    fn write8(&mut self, address: u16, value: u8) {
        match address {
            0x0000..=0x3fff => self.ram[address as usize] = value,
            0xc000..=0xc0ff => self.gtia.write(address, value),
            0xd400..=0xd4ff => self.antic.write(address, value),
            0xe800..=0xe8ff => self.pokey.write(address, value),
            _ => {}
        }
    }
}

pub struct Atari5200Machine {
    cpu: Mos6502,
    bus: Atari5200Bus,
    audio: AudioBuffer,
    powered: bool,
}

impl Atari5200Machine {
    pub fn new(rom: &[u8], bios: &[u8]) -> Result<Self, String> {
        let cartridge = Atari5200Cartridge::new(rom)?;
        let mut bus = Atari5200Bus::new(cartridge, bios)?;
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
        if self.bus.antic.take_wsync() {
            let stall = self.bus.antic.cycles_until_line_end();
            self.cpu.cycles = self.cpu.cycles.saturating_add(u64::from(stall));
            self.bus.tick(stall);
        }
        if self.bus.antic.take_nmi() {
            let before = self.cpu.cycles;
            self.cpu.nmi(&mut self.bus);
            self.bus.tick((self.cpu.cycles - before) as u32);
        }
        if self.bus.pokey.irq_pending() {
            let before = self.cpu.cycles;
            self.cpu.irq(&mut self.bus);
            self.bus.tick((self.cpu.cycles - before) as u32);
        }
        cycles
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        let samples = self.bus.pokey.take_samples();
        for sample in samples {
            self.audio.push_stereo(sample, sample);
        }
        if self.audio.samples().is_empty() {
            self.audio.push_stereo(0.0, 0.0);
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

impl Machine for Atari5200Machine {
    fn platform(&self) -> PlatformId {
        PlatformId::Atari5200
    }

    fn reset(&mut self) {
        self.bus.reset_devices();
        self.cpu.reset(&mut self.bus);
        self.powered = true;
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        let target = self.bus.antic.frame.wrapping_add(1);
        let deadline = self
            .cpu
            .cycles
            .saturating_add((CPU_HZ as f64 / FRAME_RATE * 2.0) as u64);
        while self.bus.antic.frame != target && self.cpu.cycles < deadline {
            if self.clock_instruction() == 0 {
                self.powered = false;
                break;
            }
        }
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }

    fn video(&self) -> &VideoBuffer {
        &self.bus.antic.video
    }

    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Atari5200, STATE_VERSION);
        self.save_cpu(&mut out);
        self.bus.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Atari5200, STATE_VERSION)?;
        self.load_cpu(&mut input)?;
        self.bus.load(&mut input)?;
        self.powered = input.u8()? != 0;
        self.audio.begin_frame();
        input.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_bios() -> Vec<u8> {
        let mut bios = vec![0xea; 0x800];
        let program = [
            0x78, 0xd8, 0xa9, 0x22, 0x8d, 0x16, 0xc0, 0xa9, 0x84, 0x8d, 0x1a, 0xc0, 0xa9, 0x20,
            0x8d, 0x00, 0xd4, 0xa9, 0x00, 0x8d, 0x02, 0xd4, 0xa9, 0x20, 0x8d, 0x03, 0xd4, 0xa9,
            0x40, 0x8d, 0x0e, 0xd4, 0x4c, 0x00, 0x40,
        ];
        bios[..program.len()].copy_from_slice(&program);
        bios[0x7fa..0x800].copy_from_slice(&[0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8]);
        bios
    }

    fn synthetic_cart() -> Vec<u8> {
        let mut cart = vec![0xea; 0x8000];
        let code = [
            0xa9, 0x08, 0x8d, 0x00, 0xe8, 0xa9, 0xaf, 0x8d, 0x01, 0xe8, 0x4c, 0x0a, 0x40,
        ];
        cart[..code.len()].copy_from_slice(&code);
        cart
    }

    fn install_display_list(machine: &mut Atari5200Machine) {
        let list = 0x2000usize;
        let screen = 0x2100usize;
        machine.bus.ram[list] = 0x4f;
        machine.bus.ram[list + 1] = screen as u8;
        machine.bus.ram[list + 2] = (screen >> 8) as u8;
        for row in 1..HEIGHT as usize {
            machine.bus.ram[list + 2 + row] = 0x0f;
        }
        let end = list + 2 + HEIGHT as usize;
        machine.bus.ram[end] = 0x41;
        machine.bus.ram[end + 1] = list as u8;
        machine.bus.ram[end + 2] = (list >> 8) as u8;
        for row in 0..HEIGHT as usize {
            for byte in 0..40usize {
                machine.bus.ram[screen + row * 40 + byte] =
                    if (row + byte) & 1 == 0 { 0xaa } else { 0x55 };
            }
        }
    }

    #[test]
    fn synthetic_machine_runs_cpu_antic_gtia_and_pokey() {
        let mut machine = Atari5200Machine::new(&synthetic_cart(), &synthetic_bios()).unwrap();
        install_display_list(&mut machine);
        machine.run_frame(&InputState::default());
        assert_eq!(machine.video().width(), WIDTH);
        assert_eq!(machine.video().height(), HEIGHT);
        assert!(machine.video().pixels().iter().any(|&byte| byte != 0));
        assert!(machine.audio().samples().iter().any(|&sample| sample > 0.0));
        assert!(machine.powered);
    }

    #[test]
    fn analog_axes_feed_pokey_pot_inputs() {
        let mut machine = Atari5200Machine::new(&synthetic_cart(), &synthetic_bios()).unwrap();
        let mut input = InputState::default();
        input.axes[0][AXIS_LEFT_X] = i16::MIN;
        input.axes[0][AXIS_LEFT_Y] = i16::MAX;
        machine.bus.set_inputs(&input);
        assert_eq!(machine.bus.pokey.read(0), 0);
        assert_eq!(machine.bus.pokey.read(1), 228);
    }

    #[test]
    fn save_state_round_trip_restores_cpu_and_video() {
        let mut machine = Atari5200Machine::new(&synthetic_cart(), &synthetic_bios()).unwrap();
        install_display_list(&mut machine);
        machine.run_frame(&InputState::default());
        let state = machine.save_state().unwrap();
        let pc = machine.cpu.pc;
        let frame = machine.bus.antic.frame;
        let pixel = machine.video().pixels()[0];
        machine.cpu.pc = 0x1234;
        machine.bus.antic.video.pixels_mut()[0] ^= 0xff;
        machine.load_state(&state).unwrap();
        assert_eq!(machine.cpu.pc, pc);
        assert_eq!(machine.bus.antic.frame, frame);
        assert_eq!(machine.video().pixels()[0], pixel);
    }
}
