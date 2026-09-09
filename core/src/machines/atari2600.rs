use crate::bus::Bus8;
use crate::cpu6502::Mos6502;
use crate::input::{DOWN, FACE_SOUTH, LEFT, RIGHT, SELECT, START, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 160;
const HEIGHT: u32 = 192;
const CPU_HZ: u64 = 1_193_182;
const SAMPLE_RATE: u64 = 48_000;
const FRAME_RATE: f64 = 59.922_743;
const STATE_VERSION: u32 = 1;
const COLOR_CLOCKS_PER_LINE: u16 = 228;
const HBLANK_CLOCKS: u16 = 68;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CartScheme {
    TwoK,
    FourK,
    F8,
    F6,
    F4,
}

struct AtariCartridge {
    rom: Vec<u8>,
    scheme: CartScheme,
    bank: usize,
}

impl AtariCartridge {
    fn new(rom: &[u8]) -> Result<Self, String> {
        let scheme = match rom.len() {
            0x0800 => CartScheme::TwoK,
            0x1000 => CartScheme::FourK,
            0x2000 => CartScheme::F8,
            0x4000 => CartScheme::F6,
            0x8000 => CartScheme::F4,
            size => {
                return Err(format!(
                "Atari 2600 cartridge size {size} is not yet supported; expected 2/4/8/16/32 KiB"
            ))
            }
        };
        let bank = match scheme {
            CartScheme::TwoK | CartScheme::FourK => 0,
            CartScheme::F8 => 1,
            CartScheme::F6 => 3,
            CartScheme::F4 => 7,
        };
        Ok(Self {
            rom: rom.to_vec(),
            scheme,
            bank,
        })
    }

    fn select_hotspot(&mut self, address: u16) {
        match self.scheme {
            CartScheme::F8 if matches!(address, 0x1ff8 | 0x1ff9) => {
                self.bank = usize::from(address - 0x1ff8);
            }
            CartScheme::F6 if (0x1ff6..=0x1ff9).contains(&address) => {
                self.bank = usize::from(address - 0x1ff6);
            }
            CartScheme::F4 if (0x1ff4..=0x1ffb).contains(&address) => {
                self.bank = usize::from(address - 0x1ff4);
            }
            _ => {}
        }
    }

    fn reset(&mut self) {
        self.bank = match self.scheme {
            CartScheme::TwoK | CartScheme::FourK => 0,
            CartScheme::F8 => 1,
            CartScheme::F6 => 3,
            CartScheme::F4 => 7,
        };
    }

    fn read(&mut self, address: u16) -> u8 {
        let address = 0x1000 | (address & 0x0fff);
        self.select_hotspot(address);
        match self.scheme {
            CartScheme::TwoK => self.rom[(address as usize) & 0x07ff],
            CartScheme::FourK => self.rom[(address as usize) & 0x0fff],
            _ => self.rom[self.bank * 0x1000 + (address as usize & 0x0fff)],
        }
    }

    fn write(&mut self, address: u16, _value: u8) {
        self.select_hotspot(0x1000 | (address & 0x0fff));
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.bank as u8);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let bank = usize::from(input.u8()?);
        let banks = (self.rom.len() / 0x1000).max(1);
        if bank >= banks && !matches!(self.scheme, CartScheme::TwoK) {
            return Err("Atari 2600 state contains an invalid cartridge bank".into());
        }
        self.bank = bank.min(banks - 1);
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
        Self::clear(&mut self.swcha, 4, p0 & UP != 0);
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

    fn set_inputs(&mut self, input: &InputState) {
        self.fire[0] = input.buttons[0] & FACE_SOUTH != 0;
        self.fire[1] = input.buttons[1] & FACE_SOUTH != 0;
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
            8..=11 => 0x80,
            12 => {
                if self.fire[0] {
                    0
                } else {
                    0x80
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
        self.riot.tick(cycles);
        self.tia.tick_cpu_cycles(cycles);
    }
}

impl Bus8 for AtariBus {
    fn read8(&mut self, address: u16) -> u8 {
        let address = address & 0x1fff;
        if address & 0x1000 != 0 {
            return self.cartridge.read(address);
        }
        if address & 0x0080 == 0 {
            return self.tia.read(address);
        }
        if address & 0x0200 == 0 {
            self.riot.ram[address as usize & 0x7f]
        } else {
            self.riot.read_io(address)
        }
    }

    fn write8(&mut self, address: u16, value: u8) {
        let address = address & 0x1fff;
        if address & 0x1000 != 0 {
            self.cartridge.write(address, value);
            return;
        }
        if address & 0x0080 == 0 {
            self.tia.write(address, value);
            return;
        }
        if address & 0x0200 == 0 {
            self.riot.ram[address as usize & 0x7f] = value;
        } else {
            self.riot.write_io(address, value);
        }
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
        rom[0x1000..].fill(0x22);
        let mut cart = AtariCartridge::new(&rom).unwrap();
        assert_eq!(cart.read(0x1000), 0x22);
        let _ = cart.read(0x1ff8);
        assert_eq!(cart.read(0x1000), 0x11);
        let _ = cart.read(0x1ff9);
        assert_eq!(cart.read(0x1000), 0x22);
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
