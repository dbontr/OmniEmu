use crate::cpu68000::{Bus68000, M68000};
use crate::cpu_jaguar_risc::{JaguarRisc, JaguarRiscBus, JaguarRiscKind};
use crate::input::{
    DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, LEFT, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;
const MASTER_HZ: u64 = 26_590_906;
const FRAME_NUM: u64 = 60_000;
const FRAME_DEN: u64 = 1001;
const AUDIO_RATE: u32 = 48_000;
const BIOS_SIZE: usize = 128 * 1024;
const RAM_SIZE: usize = 2 * 1024 * 1024;
const CART_MAX: usize = 6 * 1024 * 1024;
const GPU_RAM_SIZE: usize = 4 * 1024;
const DSP_RAM_SIZE: usize = 8 * 1024;
const EEPROM_SIZE: usize = 128;
const STATE_VERSION: u32 = 3;
fn rgb16(value: u16) -> [u8; 4] {
    let r = ((value >> 11) & 0x1f) as u32;
    let g = ((value >> 5) & 0x3f) as u32;
    let b = (value & 0x1f) as u32;
    [
        ((r * 255 + 15) / 31) as u8,
        ((g * 255 + 31) / 63) as u8,
        ((b * 255 + 15) / 31) as u8,
        255,
    ]
}

fn sign12(value: u16) -> i32 {
    let raw = i32::from(value & 0x0fff);
    if raw & 0x0800 != 0 {
        raw - 0x1000
    } else {
        raw
    }
}

#[derive(Clone)]
struct Tom {
    regs: [u16; 128],
    clut: [u16; 256],
    line_a: Vec<u8>,
    line_b: Vec<u8>,
    irq_enable: u8,
    irq_pending: u8,
}
impl Default for Tom {
    fn default() -> Self {
        Self {
            regs: [0; 128],
            clut: [0; 256],
            line_a: vec![0; 1440],
            line_b: vec![0; 1440],
            irq_enable: 0,
            irq_pending: 0,
        }
    }
}

impl Tom {
    fn memcon1(&self) -> u16 {
        self.regs[0]
    }
    fn rom_high(&self) -> bool {
        self.memcon1() & 1 != 0
    }
    fn object_list(&self) -> u32 {
        (u32::from(self.regs[0x20 / 2]) << 16) | u32::from(self.regs[0x22 / 2])
    }
    fn video_enabled(&self) -> bool {
        self.regs[0x28 / 2] & 1 != 0
    }
    fn background(&self) -> u16 {
        self.regs[0x58 / 2]
    }
    fn host_irq(&self) -> bool {
        self.irq_pending & self.irq_enable != 0
    }

    fn read16(&self, address: u32) -> u16 {
        let offset = (address - 0x00f0_0000) as usize;
        match offset {
            0xe0 => u16::from(self.irq_enable) | (u16::from(self.irq_pending) << 8),
            0x400..=0x5ff => self.clut[(offset - 0x400) >> 1],
            0x600..=0x7ff => self.clut[(offset - 0x600) >> 1],
            0x800..=0x0d9f => {
                let index = offset - 0x800;
                u16::from_be_bytes([self.line_a[index], self.line_a[index + 1]])
            }
            0x1000..=0x159f => {
                let index = offset - 0x1000;
                u16::from_be_bytes([self.line_b[index], self.line_b[index + 1]])
            }
            _ if offset < 0x100 => self.regs[(offset & !1) >> 1],
            _ => 0xffff,
        }
    }

    fn write16(&mut self, address: u32, value: u16) {
        let offset = (address - 0x00f0_0000) as usize;
        match offset {
            0xe0 => {
                self.irq_enable = value as u8 & 0x1f;
                self.irq_pending &= !((value >> 8) as u8 & 0x1f);
            }
            0x400..=0x5ff => self.clut[(offset - 0x400) >> 1] = value,
            0x600..=0x7ff => self.clut[(offset - 0x600) >> 1] = value,
            0x800..=0x0d9f => {
                let index = offset - 0x800;
                self.line_a[index..index + 2].copy_from_slice(&value.to_be_bytes());
            }
            0x1000..=0x159f => {
                let index = offset - 0x1000;
                self.line_b[index..index + 2].copy_from_slice(&value.to_be_bytes());
            }
            _ if offset < 0x100 => self.regs[(offset & !1) >> 1] = value,
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        for value in self.regs {
            out.u16(value);
        }
        for value in self.clut {
            out.u16(value);
        }
        out.blob(&self.line_a);
        out.blob(&self.line_b);
        out.u8(self.irq_enable);
        out.u8(self.irq_pending);
    }
    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.regs {
            *value = input.u16()?;
        }
        for value in &mut self.clut {
            *value = input.u16()?;
        }
        for (name, target) in [("line A", &mut self.line_a), ("line B", &mut self.line_b)] {
            let data = input.blob()?;
            if data.len() != target.len() {
                return Err(format!("Jaguar TOM {name} state size mismatch"));
            }
            target.copy_from_slice(data);
        }
        self.irq_enable = input.u8()? & 0x1f;
        self.irq_pending = input.u8()? & 0x1f;
        Ok(())
    }
}

#[derive(Clone, Default)]
struct RiscControl {
    flags: u32,
    matrix_control: u32,
    matrix_pointer: u32,
    data_organization: u32,
    pc: u32,
    control: u32,
    high_data: u32,
    remain: u32,
    div_control: u32,
}

impl RiscControl {
    fn read32(&self, offset: u32) -> u32 {
        match offset & 0x1c {
            0x00 => self.flags,
            0x04 => self.matrix_control,
            0x08 => self.matrix_pointer,
            0x0c => self.data_organization,
            0x10 => self.pc,
            0x14 => self.control,
            0x18 => self.high_data,
            _ => self.remain,
        }
    }
    fn write32(&mut self, offset: u32, value: u32) {
        match offset & 0x1c {
            0x00 => self.flags = value,
            0x04 => self.matrix_control = value,
            0x08 => self.matrix_pointer = value & !3,
            0x0c => self.data_organization = value,
            0x10 => self.pc = value,
            0x14 => self.control = (self.control & 0xffff_f7c0) | (value & 0x0000_0839),
            0x18 => self.high_data = value,
            _ => self.div_control = value,
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u32(self.flags);
        out.u32(self.matrix_control);
        out.u32(self.matrix_pointer);
        out.u32(self.data_organization);
        out.u32(self.pc);
        out.u32(self.control);
        out.u32(self.high_data);
        out.u32(self.remain);
        out.u32(self.div_control);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.flags = input.u32()?;
        self.matrix_control = input.u32()?;
        self.matrix_pointer = input.u32()? & !3;
        self.data_organization = input.u32()?;
        self.pc = input.u32()?;
        self.control = input.u32()?;
        self.high_data = input.u32()?;
        self.remain = input.u32()?;
        self.div_control = input.u32()?;
        Ok(())
    }
}

struct JaguarBoard {
    ram: Box<[u8]>,
    cart: Vec<u8>,
    bios: Box<[u8]>,
    gpu_ram: Box<[u8]>,
    dsp_ram: Box<[u8]>,
    eeprom: [u8; EEPROM_SIZE],
    tom: Tom,
    gpu_ctrl: RiscControl,
    dsp_ctrl: RiscControl,
    jerry_regs: [u16; 0x20],
    jerry_irq_state: u8,
    jerry_irq_enable: u8,
    jerry_timer_countdown: [u64; 2],
    serial_frequency: u16,
    serial_mode: u8,
    serial_countdown: u64,
    dsp_irq_assert: u8,
    dsp_irq_clear: u8,
    gpu_irq_assert: u8,
    joy_latch: u16,
    joy_buttons: [u64; 2],
    dac_left: i16,
    dac_right: i16,
    audio_phase: u64,
    audio_samples: Vec<(f32, f32)>,
    frame: u64,
}

impl JaguarBoard {
    fn new(cart: &[u8], bios: &[u8]) -> Result<Self, String> {
        if cart.is_empty() || cart.len() > CART_MAX {
            return Err(format!(
                "Jaguar cartridge must contain 1..={CART_MAX} bytes"
            ));
        }
        if bios.len() != BIOS_SIZE {
            return Err(format!("Jaguar BIOS must be exactly {BIOS_SIZE} bytes"));
        }
        Ok(Self {
            ram: vec![0; RAM_SIZE].into_boxed_slice(),
            cart: cart.to_vec(),
            bios: bios.to_vec().into_boxed_slice(),
            gpu_ram: vec![0; GPU_RAM_SIZE].into_boxed_slice(),
            dsp_ram: vec![0; DSP_RAM_SIZE].into_boxed_slice(),
            eeprom: [0xff; EEPROM_SIZE],
            tom: Tom::default(),
            gpu_ctrl: RiscControl::default(),
            dsp_ctrl: RiscControl::default(),
            jerry_regs: [0; 0x20],
            jerry_irq_state: 0,
            jerry_irq_enable: 0,
            jerry_timer_countdown: [0; 2],
            serial_frequency: 0,
            serial_mode: 0,
            serial_countdown: 0,
            dsp_irq_assert: 0,
            dsp_irq_clear: 0,
            gpu_irq_assert: 0,
            joy_latch: 0xffff,
            joy_buttons: [0; 2],
            dac_left: 0,
            dac_right: 0,
            audio_phase: 0,
            audio_samples: Vec::with_capacity(1024),
            frame: 0,
        })
    }

    fn reset(&mut self) {
        self.ram.fill(0);
        self.gpu_ram.fill(0);
        self.dsp_ram.fill(0);
        self.tom = Tom::default();
        self.gpu_ctrl = RiscControl::default();
        self.dsp_ctrl = RiscControl::default();
        self.jerry_regs = [0; 0x20];
        self.jerry_irq_state = 0;
        self.jerry_irq_enable = 0;
        self.jerry_timer_countdown = [0; 2];
        self.serial_frequency = 0;
        self.serial_mode = 0;
        self.serial_countdown = 0;
        self.dsp_irq_assert = 0;
        self.dsp_irq_clear = 0;
        self.gpu_irq_assert = 0;
        self.joy_latch = 0xffff;
        self.dac_left = 0;
        self.dac_right = 0;
        self.audio_phase = 0;
        self.audio_samples.clear();
        self.frame = 0;
    }

    fn set_input(&mut self, input: &InputState) {
        self.joy_buttons.copy_from_slice(&input.buttons[..2]);
    }
    fn joy_read32(&self) -> u32 {
        let mut directions = 0xff6eu16;
        let mut buttons = 0xffefu16;
        for player in 0..2 {
            let state = self.joy_buttons[player];
            let base = if player == 0 { 0 } else { 4 };
            for column in 0..4 {
                if self.joy_latch & (1 << (base + column)) != 0 {
                    continue;
                }
                if column == 0 {
                    let masks = [(RIGHT, 11), (LEFT, 10), (DOWN, 9), (UP, 8)];
                    for (button, bit) in masks {
                        if state & button != 0 {
                            directions &= !(1 << bit);
                        }
                    }
                    if state & FACE_SOUTH != 0 {
                        buttons &= !(1 << (player * 2));
                    }
                    if state & START != 0 {
                        buttons &= !(1 << (player * 2 + 1));
                    }
                }
                if column == 1 && state & FACE_EAST != 0 {
                    buttons &= !(1 << (player * 2));
                }
                if column == 2 && state & FACE_NORTH != 0 {
                    buttons &= !(1 << (player * 2));
                }
                if column == 3 && (state & FACE_WEST != 0 || state & SELECT != 0) {
                    buttons &= !(1 << (player * 2 + 1));
                }
            }
        }
        (u32::from(directions) << 16) | u32::from(buttons)
    }
    fn control_read8(control: &RiscControl, base: u32, address: u32) -> u8 {
        let offset = address.wrapping_sub(base);
        let word = control.read32(offset & !3);
        let shift = (3 - (offset & 3)) * 8;
        (word >> shift) as u8
    }

    fn control_write8(control: &mut RiscControl, base: u32, address: u32, value: u8) {
        let offset = address.wrapping_sub(base);
        let aligned = offset & !3;
        let mut word = control.read32(aligned);
        let shift = (3 - (offset & 3)) * 8;
        word = (word & !(0xff << shift)) | (u32::from(value) << shift);
        control.write32(aligned, word);
    }

    fn jerry_timer_period(&self, timer: usize) -> u64 {
        let prescaler = u64::from(self.jerry_regs[timer * 2]);
        let divider = u64::from(self.jerry_regs[timer * 2 + 1]);
        if prescaler == 0 && divider == 0 {
            0
        } else {
            (prescaler + 1) * (divider + 1)
        }
    }

    fn update_jerry_timer(&mut self, timer: usize) {
        let period = self.jerry_timer_period(timer);
        self.jerry_timer_countdown[timer] = period;
        if period == 0 {
            self.dsp_irq_clear |= 1 << (2 + timer);
        }
    }

    fn serial_period(&self) -> u64 {
        if matches!(self.serial_mode, 0x05 | 0x15) {
            64 * (u64::from(self.serial_frequency) + 1)
        } else {
            0
        }
    }

    fn update_serial_timer(&mut self) {
        self.serial_countdown = self.serial_period();
        if self.serial_countdown == 0 {
            self.dsp_irq_clear |= 1 << 1;
        }
    }

    fn refresh_jerry_host_irq(&mut self) {
        if self.jerry_irq_state & self.jerry_irq_enable & 0x1f != 0 {
            self.tom.irq_pending |= 1 << 4;
        } else {
            self.tom.irq_pending &= !(1 << 4);
        }
    }

    fn jerry_read16(&self, address: u32) -> u16 {
        let index = ((address.wrapping_sub(0x00f1_0000)) >> 1) as usize;
        if index == 0x10 {
            u16::from(self.jerry_irq_state)
        } else {
            self.jerry_regs.get(index).copied().unwrap_or(0xffff)
        }
    }

    fn jerry_write16(&mut self, address: u32, value: u16) {
        let index = ((address.wrapping_sub(0x00f1_0000)) >> 1) as usize;
        if index >= self.jerry_regs.len() {
            return;
        }
        self.jerry_regs[index] = value;
        match index {
            0 | 1 => self.update_jerry_timer(0),
            2 | 3 => self.update_jerry_timer(1),
            0x10 => {
                self.jerry_irq_enable = value as u8 & 0x1f;
                self.jerry_irq_state &= !((value >> 8) as u8 & 0x1f);
                self.refresh_jerry_host_irq();
            }
            _ => {}
        }
    }

    fn tick_countdown(countdown: &mut u64, period: u64, cycles: u64) -> u32 {
        if *countdown == 0 || period == 0 {
            return 0;
        }
        let mut remaining = cycles;
        let mut events = 0u32;
        while remaining >= *countdown {
            remaining -= *countdown;
            events = events.saturating_add(1);
            *countdown = period;
        }
        *countdown -= remaining;
        events
    }

    fn tick_jerry(&mut self, master_cycles: u32) {
        let cycles = u64::from(master_cycles);
        for timer in 0..2 {
            let period = self.jerry_timer_period(timer);
            if Self::tick_countdown(&mut self.jerry_timer_countdown[timer], period, cycles) != 0 {
                let line = 2 + timer;
                self.dsp_irq_assert |= 1 << line;
                self.jerry_irq_state |= 1 << line;
            }
        }
        let serial_period = self.serial_period();
        if Self::tick_countdown(&mut self.serial_countdown, serial_period, cycles) != 0 {
            self.dsp_irq_assert |= 1 << 1;
        }
        self.refresh_jerry_host_irq();
    }

    fn take_dsp_irq_changes(&mut self) -> (u8, u8) {
        let clear = std::mem::take(&mut self.dsp_irq_clear);
        let assert = std::mem::take(&mut self.dsp_irq_assert);
        (clear, assert)
    }

    fn read8(&self, address: u32) -> u8 {
        let address = address & 0x00ff_ffff;
        match address {
            0x000000..=0x1fffff => {
                if !self.tom.rom_high() && (address as usize) < BIOS_SIZE {
                    self.bios[address as usize]
                } else {
                    self.ram[address as usize & (RAM_SIZE - 1)]
                }
            }
            0x800000..=0xdfffff => self
                .cart
                .get((address - 0x800000) as usize)
                .copied()
                .unwrap_or(0xff),
            0xe00000..=0xe1ffff => self.bios[(address - 0xe00000) as usize],
            0xf00000..=0xf01fff => {
                let aligned = address & !1;
                let word = self.tom.read16(aligned);
                if address & 1 == 0 {
                    (word >> 8) as u8
                } else {
                    word as u8
                }
            }
            0xf02100..=0xf0211f => Self::control_read8(&self.gpu_ctrl, 0xf02100, address),
            0xf03000..=0xf03fff => self.gpu_ram[(address - 0xf03000) as usize],
            0xf10000..=0xf103ff => {
                let word = self.jerry_read16(address & !1);
                if address & 1 == 0 {
                    (word >> 8) as u8
                } else {
                    word as u8
                }
            }
            0xf0b000..=0xf0bfff => self.gpu_ram[(address - 0xf0b000) as usize],
            0xf14000..=0xf14003 => {
                let value = self.joy_read32();
                let shift = (3 - (address - 0xf14000)) * 8;
                (value >> shift) as u8
            }
            0xf1a100..=0xf1a11f => Self::control_read8(&self.dsp_ctrl, 0xf1a100, address),
            0xf1a140..=0xf1a17f => 0,
            0xf1b000..=0xf1cfff => self.dsp_ram[(address - 0xf1b000) as usize],
            0xf20000..=0xffffff => self.bios[(address as usize - 0xf20000) & (BIOS_SIZE - 1)],
            _ => 0xff,
        }
    }
    fn write8(&mut self, address: u32, value: u8) {
        let address = address & 0x00ff_ffff;
        match address {
            0x000000..=0x1fffff => self.ram[address as usize & (RAM_SIZE - 1)] = value,
            0xf00000..=0xf01fff => {
                let aligned = address & !1;
                let mut word = self.tom.read16(aligned);
                if address & 1 == 0 {
                    word = (word & 0x00ff) | (u16::from(value) << 8);
                } else {
                    word = (word & 0xff00) | u16::from(value);
                }
                self.tom.write16(aligned, word);
                if aligned == 0xf000e0 {
                    self.refresh_jerry_host_irq();
                }
            }
            0xf02100..=0xf0211f => {
                Self::control_write8(&mut self.gpu_ctrl, 0xf02100, address, value)
            }
            0xf03000..=0xf03fff => self.gpu_ram[(address - 0xf03000) as usize] = value,
            0xf0b000..=0xf0bfff => self.gpu_ram[(address - 0xf0b000) as usize] = value,
            0xf10000..=0xf103ff => {
                let aligned = address & !1;
                let mut word = self.jerry_read16(aligned);
                if address & 1 == 0 {
                    word = (word & 0x00ff) | (u16::from(value) << 8);
                } else {
                    word = (word & 0xff00) | u16::from(value);
                }
                self.jerry_write16(aligned, word);
            }
            0xf14000..=0xf14003 => {
                if address & 1 != 0 {
                    self.joy_latch = (self.joy_latch & 0xff00) | u16::from(value);
                } else {
                    self.joy_latch = (self.joy_latch & 0x00ff) | (u16::from(value) << 8);
                }
            }
            0xf1a100..=0xf1a11f => {
                Self::control_write8(&mut self.dsp_ctrl, 0xf1a100, address, value)
            }
            0xf1a148..=0xf1a14b => {
                let byte = (address - 0xf1a148) as usize;
                let mut data = self.dac_right.to_be_bytes();
                if byte >= 2 {
                    data[byte - 2] = value;
                    self.dac_right = i16::from_be_bytes(data);
                }
            }
            0xf1a14c..=0xf1a14f => {
                let byte = (address - 0xf1a14c) as usize;
                let mut data = self.dac_left.to_be_bytes();
                if byte >= 2 {
                    data[byte - 2] = value;
                    self.dac_left = i16::from_be_bytes(data);
                }
            }
            0xf1a150..=0xf1a153 => {
                let byte = (address - 0xf1a150) as usize;
                if byte >= 2 {
                    let mut data = self.serial_frequency.to_be_bytes();
                    data[byte - 2] = value;
                    self.serial_frequency = u16::from_be_bytes(data);
                    self.update_serial_timer();
                }
            }
            0xf1a154..=0xf1a157 => {
                let byte = (address - 0xf1a154) as usize;
                if byte >= 2 {
                    let mut data = u16::from(self.serial_mode).to_be_bytes();
                    data[byte - 2] = value;
                    self.serial_mode = (u16::from_be_bytes(data) & 0x3f) as u8;
                    self.update_serial_timer();
                }
            }
            0xf1b000..=0xf1cfff => self.dsp_ram[(address - 0xf1b000) as usize] = value,
            _ => {}
        }
    }

    fn read16(&self, address: u32) -> u16 {
        let address = address & 0x00ff_ffff;
        match address {
            0xf10000..=0xf103fe if address & 1 == 0 => self.jerry_read16(address),
            0xf1a148 | 0xf1a14c | 0xf1a150 | 0xf1a154 => 0,
            _ => u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))]),
        }
    }
    fn read32(&self, address: u32) -> u32 {
        u32::from_be_bytes([
            self.read8(address),
            self.read8(address.wrapping_add(1)),
            self.read8(address.wrapping_add(2)),
            self.read8(address.wrapping_add(3)),
        ])
    }
    fn write16(&mut self, address: u32, value: u16) {
        let address = address & 0x00ff_ffff;
        match address {
            0xf10000..=0xf103fe if address & 1 == 0 => self.jerry_write16(address, value),
            0xf1a148 => self.dac_right = value as i16,
            0xf1a14c => self.dac_left = value as i16,
            0xf1a150 => {
                self.serial_frequency = value;
                self.update_serial_timer();
            }
            0xf1a154 => {
                self.serial_mode = (value & 0x3f) as u8;
                self.update_serial_timer();
            }
            _ => {
                for (offset, byte) in value.to_be_bytes().into_iter().enumerate() {
                    self.write8(address.wrapping_add(offset as u32), byte);
                }
            }
        }
    }
    fn write32(&mut self, address: u32, value: u32) {
        let address = address & 0x00ff_ffff;
        match address {
            0xf1a148 | 0xf1a14c | 0xf1a150 | 0xf1a154 => {
                self.write16(address, value as u16);
            }
            _ => {
                self.write16(address, (value >> 16) as u16);
                self.write16(address.wrapping_add(2), value as u16);
            }
        }
    }

    fn sync_from_risc(&mut self, gpu: &JaguarRisc, dsp: &JaguarRisc) {
        Self::sync_control_from_cpu(&mut self.gpu_ctrl, gpu);
        Self::sync_control_from_cpu(&mut self.dsp_ctrl, dsp);
    }

    fn sync_control_from_cpu(control: &mut RiscControl, cpu: &JaguarRisc) {
        control.flags = cpu.flags_word();
        control.matrix_control = cpu.matrix_control();
        control.matrix_pointer = cpu.matrix_pointer();
        control.data_organization = cpu.data_organization();
        control.pc = cpu.pc;
        let latch = cpu.interrupt_latch();
        let latch_bits = (u32::from(latch & 0x1f) << 6)
            | if cpu.kind() == JaguarRiscKind::Dsp {
                u32::from(latch & 0x20) << 11
            } else {
                0
            };
        control.control = (control.control & !0x0001_07c1) | u32::from(cpu.running) | latch_bits;
        control.high_data = if cpu.kind() == JaguarRiscKind::Dsp {
            cpu.modulo()
        } else {
            cpu.high_data()
        };
        control.remain = cpu.remain();
        control.div_control = cpu.div_control();
    }

    fn apply_control_to_cpu(control: &RiscControl, cpu: &mut JaguarRisc) {
        cpu.set_flags_word(control.flags);
        cpu.set_matrix_control(control.matrix_control);
        cpu.set_matrix_pointer(control.matrix_pointer);
        cpu.set_data_organization(control.data_organization);
        cpu.pc = control.pc;
        if cpu.kind() == JaguarRiscKind::Dsp {
            cpu.set_modulo(control.high_data);
        } else {
            cpu.set_high_data(control.high_data);
        }
        cpu.set_div_control(control.div_control);
        cpu.running = control.control & 1 != 0;
    }

    fn begin_frame(&mut self) {
        self.audio_samples.clear();
    }
    fn tick_audio(&mut self, master_cycles: u32) {
        self.audio_phase = self
            .audio_phase
            .saturating_add(u64::from(master_cycles) * u64::from(AUDIO_RATE));
        while self.audio_phase >= MASTER_HZ {
            self.audio_phase -= MASTER_HZ;
            self.audio_samples.push((
                f32::from(self.dac_left) / 32768.0,
                f32::from(self.dac_right) / 32768.0,
            ));
        }
    }

    fn pixel_byte(&self, address: u32) -> u8 {
        self.read8(address)
    }
    fn pixel_word(&self, address: u32) -> u16 {
        self.read16(address)
    }

    fn read_phrase(&self, address: u32) -> u64 {
        let high = u64::from(self.read32(address));
        let low = u64::from(self.read32(address.wrapping_add(4)));
        (high << 32) | low
    }
    fn object_pixel(&self, address: u32, depth: u8, pixel: usize, index: u8) -> Option<[u8; 4]> {
        match depth {
            2 => {
                let byte = self.pixel_byte(address.wrapping_add((pixel >> 1) as u32));
                let value = if pixel & 1 == 0 {
                    byte >> 4
                } else {
                    byte & 0x0f
                };
                Some(rgb16(
                    self.tom.clut[(usize::from(index) + usize::from(value)) & 0xff],
                ))
            }
            3 => {
                let value = self.pixel_byte(address.wrapping_add(pixel as u32));
                Some(rgb16(
                    self.tom.clut[(usize::from(index) + usize::from(value)) & 0xff],
                ))
            }
            4 => Some(rgb16(
                self.pixel_word(address.wrapping_add((pixel * 2) as u32)),
            )),
            _ => None,
        }
    }

    fn render_bitmap(&self, video: &mut VideoBuffer, p0: u64, p1: u64, scale: Option<u64>) {
        let y_pos = ((p0 >> 3) & 0x07ff) as i32 / 2;
        let height = ((p0 >> 14) & 0x03ff) as usize;
        let data = (((p0 >> 43) & 0x1f_ffff) << 3) as u32;
        let x_pos = sign12(p1 as u16);
        let depth = ((p1 >> 12) & 7) as u8;
        let data_width = ((p1 >> 18) & 0x03ff).max(1) as usize;
        let image_width = ((p1 >> 28) & 0x03ff).max(1) as usize;
        let palette_index = ((p1 >> 38) & 0x7f) as u8;
        let reflect = p1 & (1 << 45) != 0;
        let transparent = p1 & (1 << 47) != 0;
        let first_pixel = ((p1 >> 49) & 0x3f) as usize;
        let phrase_pixels = match depth {
            2 => 16,
            3 => 8,
            4 => 4,
            _ => return,
        };
        let source_width = image_width
            .saturating_mul(phrase_pixels)
            .saturating_sub(first_pixel);
        let (hscale, vscale) = scale.map_or((32usize, 32usize), |phrase| {
            (
                ((phrase & 0xff) as usize).max(1),
                (((phrase >> 8) & 0xff) as usize).max(1),
            )
        });
        for source_y in 0..height {
            let target_y = y_pos + ((source_y * vscale) / 32) as i32;
            if !(0..HEIGHT as i32).contains(&target_y) {
                continue;
            }
            let line = data.wrapping_add((source_y * data_width * 8) as u32);
            for source_x in 0..source_width {
                let logical_x = if reflect {
                    source_width - 1 - source_x
                } else {
                    source_x
                };
                let Some(color) =
                    self.object_pixel(line, depth, first_pixel + logical_x, palette_index)
                else {
                    continue;
                };
                if transparent && color[..3] == [0, 0, 0] {
                    continue;
                }
                let start_x = x_pos + ((source_x * hscale) / 32) as i32;
                let end_x = x_pos
                    + (((source_x + 1) * hscale) / 32).max((source_x * hscale) / 32 + 1) as i32;
                for target_x in start_x..end_x {
                    if !(0..WIDTH as i32).contains(&target_x) {
                        continue;
                    }
                    let offset = (target_y as usize * WIDTH as usize + target_x as usize) * 4;
                    video.pixels_mut()[offset..offset + 4].copy_from_slice(&color);
                }
            }
        }
    }
    fn render(&mut self, video: &mut VideoBuffer) {
        let background = if self.tom.video_enabled() {
            rgb16(self.tom.background())
        } else {
            [0, 0, 0, 255]
        };
        video.clear(background);
        if !self.tom.video_enabled() {
            return;
        }
        let mut pointer = self.tom.object_list() & 0x00ff_fff8;
        for _ in 0..256 {
            let p0 = self.read_phrase(pointer);
            match (p0 & 7) as u8 {
                0 | 1 => {
                    let p1 = self.read_phrase(pointer.wrapping_add(8));
                    let scaled = p0 & 7 == 1;
                    let scale = scaled.then(|| self.read_phrase(pointer.wrapping_add(16)));
                    self.render_bitmap(video, p0, p1, scale);
                    let link = (((p0 >> 24) & 0x7ffff) << 3) as u32;
                    pointer = if link == 0 {
                        pointer.wrapping_add(if scaled { 24 } else { 16 })
                    } else {
                        link
                    };
                }
                2 => {
                    self.gpu_ctrl.control |= 0x40;
                    pointer = pointer.wrapping_add(8);
                }
                3 => pointer = pointer.wrapping_add(8),
                4 => break,
                _ => break,
            }
        }
    }

    fn end_frame(&mut self, video: &mut VideoBuffer) {
        self.render(video);
        self.tom.irq_pending |= 1;
        self.frame = self.frame.wrapping_add(1);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.ram.as_ref());
        out.blob(self.gpu_ram.as_ref());
        out.blob(self.dsp_ram.as_ref());
        out.blob(&self.eeprom);
        self.tom.save(out);
        self.gpu_ctrl.save(out);
        self.dsp_ctrl.save(out);
        for value in self.jerry_regs {
            out.u16(value);
        }
        out.u8(self.jerry_irq_state);
        out.u8(self.jerry_irq_enable);
        for countdown in self.jerry_timer_countdown {
            out.u64(countdown);
        }
        out.u16(self.serial_frequency);
        out.u8(self.serial_mode);
        out.u64(self.serial_countdown);
        out.u8(self.dsp_irq_assert);
        out.u8(self.dsp_irq_clear);
        out.u16(self.joy_latch);
        out.u32(self.dac_left as i32 as u32);
        out.u32(self.dac_right as i32 as u32);
        out.u64(self.audio_phase);
        out.u64(self.frame);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for (name, target) in [
            ("DRAM", self.ram.as_mut()),
            ("GPU RAM", self.gpu_ram.as_mut()),
            ("DSP RAM", self.dsp_ram.as_mut()),
        ] {
            let data = input.blob()?;
            if data.len() != target.len() {
                return Err(format!("Jaguar {name} state size mismatch"));
            }
            target.copy_from_slice(data);
        }
        let eeprom = input.blob()?;
        if eeprom.len() != EEPROM_SIZE {
            return Err("Jaguar EEPROM state size mismatch".into());
        }
        self.eeprom.copy_from_slice(eeprom);
        self.tom.load(input)?;
        self.gpu_ctrl.load(input)?;
        self.dsp_ctrl.load(input)?;
        for value in &mut self.jerry_regs {
            *value = input.u16()?;
        }
        self.jerry_irq_state = input.u8()?;
        self.jerry_irq_enable = input.u8()?;
        if self.jerry_irq_state & !0x1f != 0 || self.jerry_irq_enable & !0x1f != 0 {
            return Err("Jaguar Jerry state contains invalid interrupt bits".into());
        }
        for countdown in &mut self.jerry_timer_countdown {
            *countdown = input.u64()?;
        }
        self.serial_frequency = input.u16()?;
        self.serial_mode = input.u8()?;
        if self.serial_mode & !0x3f != 0 {
            return Err("Jaguar serial state contains invalid mode bits".into());
        }
        self.serial_countdown = input.u64()?;
        self.dsp_irq_assert = input.u8()? & 0x3f;
        self.dsp_irq_clear = input.u8()? & 0x3f;
        self.joy_latch = input.u16()?;
        self.dac_left = input.u32()? as i32 as i16;
        self.dac_right = input.u32()? as i32 as i16;
        self.audio_phase = input.u64()?;
        self.frame = input.u64()?;
        self.audio_samples.clear();
        self.refresh_jerry_host_irq();
        Ok(())
    }
}

struct MainBus<'a> {
    board: &'a mut JaguarBoard,
}
impl Bus68000 for MainBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        self.board.read8(address)
    }
    fn write8(&mut self, address: u32, value: u8) {
        self.board.write8(address, value);
    }
    fn read16(&mut self, address: u32) -> u16 {
        self.board.read16(address)
    }
    fn read32(&mut self, address: u32) -> u32 {
        self.board.read32(address)
    }
    fn write16(&mut self, address: u32, value: u16) {
        self.board.write16(address, value);
    }
    fn write32(&mut self, address: u32, value: u32) {
        self.board.write32(address, value);
    }
}
struct RiscBus<'a> {
    board: &'a mut JaguarBoard,
}
impl JaguarRiscBus for RiscBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        self.board.read8(address)
    }
    fn write8(&mut self, address: u32, value: u8) {
        self.board.write8(address, value);
    }
    fn read16(&mut self, address: u32) -> u16 {
        self.board.read16(address)
    }
    fn write16(&mut self, address: u32, value: u16) {
        self.board.write16(address, value);
    }
    fn read32(&mut self, address: u32) -> u32 {
        self.board.read32(address)
    }
    fn write32(&mut self, address: u32, value: u32) {
        self.board.write32(address, value);
    }
}

pub struct JaguarMachine {
    cpu: M68000,
    gpu: JaguarRisc,
    dsp: JaguarRisc,
    board: JaguarBoard,
    video: VideoBuffer,
    audio: AudioBuffer,
    frame_phase: u128,
    master_credit: i64,
    gpu_credit: i64,
    dsp_credit: i64,
}

impl JaguarMachine {
    pub fn from_images(cart: &[u8], bios: &[u8]) -> Result<Self, String> {
        let mut board = JaguarBoard::new(cart, bios)?;
        let mut cpu = M68000::default();
        {
            let mut bus = MainBus { board: &mut board };
            cpu.reset(&mut bus);
        }
        let mut gpu = JaguarRisc::new(JaguarRiscKind::Gpu);
        let mut dsp = JaguarRisc::new(JaguarRiscKind::Dsp);
        gpu.reset(0x00f0_3000);
        dsp.reset(0x00f1_b000);
        Ok(Self {
            cpu,
            gpu,
            dsp,
            board,
            video: VideoBuffer::new(WIDTH, HEIGHT),
            audio: AudioBuffer::new(AUDIO_RATE, 2),
            frame_phase: 0,
            master_credit: 0,
            gpu_credit: 0,
            dsp_credit: 0,
        })
    }

    fn run_risc(cpu: &mut JaguarRisc, board: &mut JaguarBoard, credit: &mut i64) {
        while *credit > 0 && cpu.running {
            let used = {
                let mut bus = RiscBus { board };
                cpu.step(&mut bus)
            };
            *credit -= i64::from(used.max(1));
            cpu.running = match cpu.kind() {
                JaguarRiscKind::Gpu => board.gpu_ctrl.control & 1 != 0,
                JaguarRiscKind::Dsp => board.dsp_ctrl.control & 1 != 0,
            };
        }
        if !cpu.running {
            *credit = 0;
        }
    }

    fn apply_dsp_irq_changes(&mut self) {
        let (clear, assert) = self.board.take_dsp_irq_changes();
        for line in 0u8..=5 {
            let bit = 1u8 << line;
            if clear & bit != 0 {
                self.dsp.set_interrupt_line(line, false);
            }
            if assert & bit != 0 {
                self.dsp.set_interrupt_line(line, true);
            }
        }
    }

    fn run_master_credit(&mut self) {
        while self.master_credit > 0 {
            self.board.sync_from_risc(&self.gpu, &self.dsp);
            let interrupt_cycles = if self.board.tom.host_irq() {
                let mut bus = MainBus {
                    board: &mut self.board,
                };
                self.cpu.interrupt(&mut bus, 2, 26)
            } else {
                0
            };
            let cpu_cycles = if interrupt_cycles != 0 {
                interrupt_cycles
            } else {
                let mut bus = MainBus {
                    board: &mut self.board,
                };
                self.cpu.step(&mut bus)
            };
            let master_cycles = cpu_cycles.max(1).saturating_mul(2);
            JaguarBoard::apply_control_to_cpu(&self.board.gpu_ctrl, &mut self.gpu);
            JaguarBoard::apply_control_to_cpu(&self.board.dsp_ctrl, &mut self.dsp);
            self.apply_dsp_irq_changes();
            self.gpu_credit += i64::from(master_cycles);
            self.dsp_credit += i64::from(master_cycles);
            Self::run_risc(&mut self.gpu, &mut self.board, &mut self.gpu_credit);
            Self::run_risc(&mut self.dsp, &mut self.board, &mut self.dsp_credit);
            self.board.sync_from_risc(&self.gpu, &self.dsp);
            self.board.tick_jerry(master_cycles);
            self.apply_dsp_irq_changes();
            self.board.tick_audio(master_cycles);
            self.master_credit -= i64::from(master_cycles);
        }
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        for &(left, right) in &self.board.audio_samples {
            self.audio.push_stereo(left, right);
        }
    }

    fn reset_cpus(&mut self) {
        {
            let mut bus = MainBus {
                board: &mut self.board,
            };
            self.cpu.reset(&mut bus);
        }
        self.gpu.reset(0x00f0_3000);
        self.dsp.reset(0x00f1_b000);
        self.frame_phase = 0;
        self.master_credit = 0;
        self.gpu_credit = 0;
        self.dsp_credit = 0;
    }
}

impl Machine for JaguarMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Jaguar
    }

    fn reset(&mut self) {
        self.board.reset();
        self.reset_cpus();
        self.video.clear([0, 0, 0, 255]);
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.board.set_input(input);
        self.board.begin_frame();
        self.frame_phase += u128::from(MASTER_HZ) * u128::from(FRAME_DEN);
        let budget = self.frame_phase / u128::from(FRAME_NUM);
        self.frame_phase %= u128::from(FRAME_NUM);
        self.master_credit += budget as i64;
        self.run_master_credit();
        self.board.end_frame(&mut self.video);
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_NUM as f64 / FRAME_DEN as f64
    }
    fn video(&self) -> &VideoBuffer {
        &self.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Jaguar, STATE_VERSION);
        self.cpu.save(&mut out);
        self.gpu.save(&mut out);
        self.dsp.save(&mut out);
        self.board.save(&mut out);
        out.u128(self.frame_phase);
        out.u64(self.master_credit as u64);
        out.u64(self.gpu_credit as u64);
        out.u64(self.dsp_credit as u64);
        Ok(out.finish())
    }
    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Jaguar, STATE_VERSION)?;
        self.cpu.load(&mut input)?;
        self.gpu.load(&mut input)?;
        self.dsp.load(&mut input)?;
        self.board.load(&mut input)?;
        self.frame_phase = input.u128()?;
        self.master_credit = input.u64()? as i64;
        self.gpu_credit = input.u64()? as i64;
        self.dsp_credit = input.u64()? as i64;
        input.finish()?;
        self.board.sync_from_risc(&self.gpu, &self.dsp);
        self.board.render(&mut self.video);
        self.flush_audio();
        Ok(())
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            EEPROM_SIZE
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 || out.len() != EEPROM_SIZE {
            return Err("Jaguar persistent resource must be slot-0 EEPROM".into());
        }
        out.copy_from_slice(&self.board.eeprom);
        Ok(())
    }
    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 || data.len() != EEPROM_SIZE {
            return Err("Jaguar persistent resource must be slot-0 EEPROM".into());
        }
        self.board.eeprom.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bios_with_loop() -> Vec<u8> {
        let mut bios = vec![0xff; BIOS_SIZE];
        bios[0..4].copy_from_slice(&0x001f_ff00u32.to_be_bytes());
        bios[4..8].copy_from_slice(&0x00e0_0100u32.to_be_bytes());
        bios[0x100..0x102].copy_from_slice(&0x60feu16.to_be_bytes());
        bios
    }

    fn cart() -> Vec<u8> {
        vec![0xff; 0x20000]
    }

    fn phrase(board: &mut JaguarBoard, address: u32, value: u64) {
        board.write32(address, (value >> 32) as u32);
        board.write32(address + 4, value as u32);
    }
    #[test]
    fn romhi_switch_exposes_dram_under_boot_overlay() {
        let bios = bios_with_loop();
        let mut board = JaguarBoard::new(&cart(), &bios).unwrap();
        assert_eq!(board.read32(0), 0x001f_ff00);
        board.write32(0, 0x1234_5678);
        assert_eq!(board.read32(0), 0x001f_ff00);
        board.write16(0x00f0_0000, 1);
        assert_eq!(board.read32(0), 0x1234_5678);
        assert_eq!(board.read32(0x00e0_0000), 0x001f_ff00);
    }

    #[test]
    fn controller_matrix_is_active_low() {
        let bios = bios_with_loop();
        let mut board = JaguarBoard::new(&cart(), &bios).unwrap();
        board.joy_latch = 0xfffe;
        board.joy_buttons[0] = UP | RIGHT | FACE_SOUTH | START;
        let value = board.joy_read32();
        assert_eq!((value >> 24) as u8 & 0x09, 0);
        assert_eq!(value as u8 & 0x03, 0);
    }
    #[test]
    fn tom_bitmap_object_reaches_video_surface() {
        let bios = bios_with_loop();
        let mut board = JaguarBoard::new(&cart(), &bios).unwrap();
        board.write16(0x00f0_0000, 1);
        board.write16(0x00f0_0020, 0x0002);
        board.write16(0x00f0_0022, 0x0000);
        board.write16(0x00f0_0028, 0x0007);
        let object = 0x0002_0000u32;
        let stop = object + 16;
        let pixels = 0x0002_1000u32;
        board.write16(pixels, 0xf800);
        board.write16(pixels + 2, 0x07e0);
        let p0 = (1u64 << 14) | (u64::from(stop >> 3) << 24) | (u64::from(pixels >> 3) << 43);
        let p1 = (4u64 << 12) | (1u64 << 18) | (1u64 << 28);
        phrase(&mut board, object, p0);
        phrase(&mut board, object + 8, p1);
        phrase(&mut board, stop, 4);
        let mut video = VideoBuffer::new(WIDTH, HEIGHT);
        board.render(&mut video);
        assert!(video.pixels()[0] > video.pixels()[1]);
        assert!(video.pixels()[4 + 1] > video.pixels()[4]);
    }
    fn put_risc(words: &mut [u8], opcodes: &[u16]) {
        for (index, opcode) in opcodes.iter().copied().enumerate() {
            words[index * 2..index * 2 + 2].copy_from_slice(&opcode.to_be_bytes());
        }
    }

    #[test]
    fn jerry_pit_reload_irq_ack_and_disable_follow_register_contract() {
        let bios = bios_with_loop();
        let mut board = JaguarBoard::new(&cart(), &bios).unwrap();

        board.write16(0x00f0_00e0, 0x0010);
        board.write16(0x00f1_0020, 0x0004);
        board.write16(0x00f1_0000, 1);
        board.write16(0x00f1_0002, 2);
        assert_eq!(board.jerry_timer_countdown[0], 6);

        board.tick_jerry(5);
        assert_eq!(board.jerry_irq_state & 0x04, 0);
        board.tick_jerry(1);
        assert_ne!(board.jerry_irq_state & 0x04, 0);
        assert_eq!(board.read16(0x00f1_0020) & 0x04, 0x04);
        assert!(board.tom.host_irq());
        let (clear, assert) = board.take_dsp_irq_changes();
        assert_eq!(clear & 0x04, 0);
        assert_ne!(assert & 0x04, 0);

        board.write16(0x00f1_0020, 0x0404);
        assert_eq!(board.jerry_irq_state & 0x04, 0);
        assert!(!board.tom.host_irq());

        board.write16(0x00f1_0000, 0);
        board.write16(0x00f1_0002, 0);
        let (clear, _) = board.take_dsp_irq_changes();
        assert_ne!(clear & 0x04, 0);
        assert_eq!(board.jerry_timer_countdown[0], 0);
    }

    #[test]
    fn serial_clock_and_dac_use_low_half_of_dsp_registers() {
        let bios = bios_with_loop();
        let mut board = JaguarBoard::new(&cart(), &bios).unwrap();

        board.write32(0x00f1_a148, 0xdead_1234);
        board.write32(0x00f1_a14c, 0xbeef_8000);
        assert_eq!(board.dac_right, 0x1234);
        assert_eq!(board.dac_left, i16::MIN);

        board.write32(0x00f1_a150, 1);
        board.write32(0x00f1_a154, 0x05);
        assert_eq!(board.serial_countdown, 128);

        board.tick_jerry(127);
        let (_, assert) = board.take_dsp_irq_changes();
        assert_eq!(assert & 0x02, 0);

        board.tick_jerry(1);
        let (_, assert) = board.take_dsp_irq_changes();
        assert_ne!(assert & 0x02, 0);
        assert_eq!(board.serial_countdown, 128);

        board.write16(0x00f1_a154, 0);
        let (clear, _) = board.take_dsp_irq_changes();
        assert_ne!(clear & 0x02, 0);
        assert_eq!(board.serial_countdown, 0);
    }

    #[test]
    fn jerry_timer_drives_dsp_interrupt_vector_through_scheduler() {
        let bios = bios_with_loop();
        let mut machine = JaguarMachine::from_images(&cart(), &bios).unwrap();
        machine.dsp.running = true;
        machine.dsp.regs[31] = 0x0000_2000;
        machine.dsp.set_flags_word(1 << 6);
        machine.board.write16(0x00f1_0002, 1);
        machine.master_credit = 24;
        machine.run_master_credit();

        assert_ne!(machine.dsp.interrupt_latch() & 0x04, 0);
        assert_ne!(machine.dsp.flags_word() & 0x08, 0);
        assert!(machine.dsp.pc >= 0x00f1_b020);
        assert!(machine.dsp.regs[31] <= 0x0000_1ffc);
    }

    #[test]
    fn machine_runs_68000_gpu_dsp_video_and_dac() {
        let bios = bios_with_loop();
        let mut machine = JaguarMachine::from_images(&cart(), &bios).unwrap();
        let moveq = ((35u16) << 10) | (5 << 5) | 1;
        let jump = (52u16) << 10;
        let nop = (57u16) << 10;
        put_risc(&mut machine.board.gpu_ram[..6], &[moveq, jump, nop]);
        put_risc(&mut machine.board.dsp_ram[..6], &[moveq, jump, nop]);
        machine.gpu.regs[0] = 0x00f0_3000;
        machine.dsp.regs[0] = 0x00f1_b000;
        machine.gpu.running = true;
        machine.dsp.running = true;
        machine.board.dac_left = 8192;
        machine.board.dac_right = -8192;
        machine.run_frame(&InputState::default());
        assert_eq!(machine.gpu.regs[1], 5);
        assert_eq!(machine.dsp.regs[1], 5);
        assert_eq!(machine.video().width(), WIDTH);
        assert!(machine
            .audio()
            .samples()
            .iter()
            .any(|sample| sample.abs() > 0.1));
    }
    #[test]
    fn state_and_eeprom_round_trip_are_deterministic() {
        let bios = bios_with_loop();
        let mut machine = JaguarMachine::from_images(&cart(), &bios).unwrap();
        let mut persisted = [0u8; EEPROM_SIZE];
        for (index, byte) in persisted.iter_mut().enumerate() {
            *byte = index as u8;
        }
        machine
            .write_persistent(ResourceKind::Storage, 0, &persisted)
            .unwrap();
        machine.board.write16(0x00f1_0000, 0xffff);
        machine.board.write16(0x00f1_0002, 0xffff);
        machine.board.write16(0x00f1_a150, 0xffff);
        machine.board.write16(0x00f1_a154, 0x0005);
        machine.run_frame(&InputState::default());
        let state = machine.save_state().unwrap();
        machine.board.ram[0x1234] = 0xaa;
        machine.board.eeprom.fill(0);
        machine.board.write16(0x00f1_0000, 0);
        machine.board.write16(0x00f1_0002, 0);
        machine.board.write16(0x00f1_a154, 0);
        machine.load_state(&state).unwrap();
        assert_eq!(machine.save_state().unwrap(), state);
        let mut restored = [0u8; EEPROM_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut restored)
            .unwrap();
        assert_eq!(restored, persisted);
    }
}
