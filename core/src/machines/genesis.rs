use crate::cpu68000::{Bus68000, M68000};
use crate::cpu_z80::{Z80Bus, Z80};
use crate::input::{DOWN, FACE_EAST, FACE_SOUTH, FACE_WEST, LEFT, RIGHT, START, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

use super::genesis_vdp::GenesisVdp;
use super::sn76489::Sn76489;
use super::ym2612::Ym2612;

const M68K_HZ: u64 = 7_670_454;
const Z80_HZ: u64 = 3_579_545;
const FRAME_RATE: f64 = 59.922_743;
const STATE_VERSION: u32 = 1;
const MAX_ROM: usize = 8 * 1024 * 1024;

struct GenesisCartridge {
    rom: Vec<u8>,
    sram: Vec<u8>,
    sram_start: u32,
    sram_end: u32,
    sram_enabled: bool,
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

    fn new(image: &[u8]) -> Result<Self, String> {
        let rom = Self::decode_image(image)?;
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
        Ok(Self {
            rom,
            sram,
            sram_start,
            sram_end,
            sram_enabled: true,
        })
    }
    fn read(&self, address: u32) -> u8 {
        if self.sram_enabled
            && !self.sram.is_empty()
            && (self.sram_start..=self.sram_end).contains(&address)
        {
            let offset = usize::try_from(address - self.sram_start).unwrap_or(usize::MAX);
            return self.sram.get(offset).copied().unwrap_or(0xff);
        }
        self.rom.get(address as usize).copied().unwrap_or(0xff)
    }

    fn write(&mut self, address: u32, value: u8) {
        if self.sram_enabled
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
        self.sram.len()
    }
    fn persistent(&self) -> &[u8] {
        &self.sram
    }
    fn set_persistent(&mut self, data: &[u8]) -> Result<(), String> {
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
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let sram = input.blob()?;
        if sram.len() != self.sram.len() {
            return Err("Genesis state SRAM size mismatch".into());
        }
        self.sram.copy_from_slice(sram);
        self.sram_enabled = input.u8()? != 0;
        Ok(())
    }
}
#[derive(Clone)]
struct GenesisIo {
    data: [u8; 3],
    control: [u8; 3],
    input: InputState,
}

impl Default for GenesisIo {
    fn default() -> Self {
        Self {
            data: [0x7f; 3],
            control: [0; 3],
            input: InputState::default(),
        }
    }
}

impl GenesisIo {
    fn set_input(&mut self, input: &InputState) {
        self.input = input.clone();
    }

    fn controller_lines(&self, player: usize, th: bool) -> u8 {
        let buttons = self.input.buttons[player.min(3)];
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
        let th = self.data[port] & 0x40 != 0;
        let inputs = self.controller_lines(port, th) | if th { 0x40 } else { 0 };
        (inputs & !self.control[port]) | (self.data[port] & self.control[port])
    }
    fn read(&self, offset: u32) -> u8 {
        match offset & 0x1f {
            0x01 => 0xa0,
            0x03 => self.read_data(0),
            0x05 => self.read_data(1),
            0x07 => self.read_data(2),
            0x09 => self.control[0],
            0x0b => self.control[1],
            0x0d => self.control[2],
            _ => 0xff,
        }
    }

    fn write(&mut self, offset: u32, value: u8) {
        match offset & 0x1f {
            0x03 => self.data[0] = value,
            0x05 => self.data[1] = value,
            0x07 => self.data[2] = value,
            0x09 => self.control[0] = value,
            0x0b => self.control[1] = value,
            0x0d => self.control[2] = value,
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.data);
        out.blob(&self.control);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let data = input.blob()?;
        let control = input.blob()?;
        if data.len() != 3 || control.len() != 3 {
            return Err("invalid Genesis I/O state".into());
        }
        self.data.copy_from_slice(data);
        self.control.copy_from_slice(control);
        Ok(())
    }
}

fn clear(value: &mut u8, bit: u8, pressed: bool) {
    if pressed {
        *value &= !(1 << bit);
    }
}
struct GenesisBus {
    cartridge: GenesisCartridge,
    main_ram: Box<[u8; 0x10000]>,
    z80_ram: Box<[u8; 0x2000]>,
    io: GenesisIo,
    vdp: GenesisVdp,
    ym: Ym2612,
    psg: Sn76489,
    z80_bus_requested: bool,
    z80_running: bool,
    z80_bank: u32,
}

impl GenesisBus {
    fn new(cartridge: GenesisCartridge) -> Self {
        Self {
            cartridge,
            main_ram: Box::new([0; 0x10000]),
            z80_ram: Box::new([0; 0x2000]),
            io: GenesisIo::default(),
            vdp: GenesisVdp::default(),
            ym: Ym2612::new(M68K_HZ),
            psg: Sn76489::default(),
            z80_bus_requested: false,
            z80_running: false,
            z80_bank: 0,
        }
    }

    fn reset_devices(&mut self) {
        self.vdp.reset();
        self.ym.reset();
        self.psg.reset();
        self.io = GenesisIo::default();
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

    fn read_main8(&mut self, address: u32) -> u8 {
        let address = address & 0x00ff_ffff;
        match address {
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
            0xa130f0..=0xa130f1 => self.cartridge.sram_enabled = value & 1 != 0,
            0xa14000..=0xa14003 => {}
            0xc00000..=0xc0001f => {
                let word = u16::from(value) * 0x0101;
                self.write_vdp_word(address & !1, word);
            }
            0xff0000..=0xffffff => self.main_ram[(address as usize) & 0xffff] = value,
            _ => {}
        }
    }

    fn tick_main_cycles(&mut self, cycles: u32) {
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
    fn read16(&mut self, address: u32) -> u16 {
        let address = address & 0x00ff_ffff;
        match address {
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
        match address {
            0xa11100 => self.z80_bus_requested = value & 0x0100 != 0,
            0xa11200 => self.z80_running = value & 0x0100 != 0,
            0xa130f0 => self.cartridge.sram_enabled = value & 1 != 0,
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
            0x000000..=0x7fffff => self.cartridge.read(target),
            0xff0000..=0xffffff => self.main_ram[(target as usize) & 0xffff],
            _ => 0xff,
        }
    }

    fn banked_write(&mut self, address: u16, value: u8) {
        let target = self.banked_address(address) & 0x00ff_ffff;
        match target {
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
    main_cpu: M68000,
    z80: Z80,
    bus: GenesisBus,
    audio: AudioBuffer,
    z80_phase: u64,
    z80_cycle_credit: i64,
    powered: bool,
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
    fn run_z80_for_main_cycles(&mut self, main_cycles: u32) {
        self.z80_phase = self
            .z80_phase
            .saturating_add(u64::from(main_cycles) * Z80_HZ);
        let z80_cycles = self.z80_phase / M68K_HZ;
        self.z80_phase %= M68K_HZ;
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
    fn flush_audio(&mut self) {
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

    fn begin_audio_frame(&mut self) {
        self.bus.ym.begin_frame();
        self.bus.psg.begin_frame();
        self.audio.begin_frame();
    }

    fn save_bus(&self, out: &mut StateWriter) {
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

    fn load_bus(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
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
        let deadline = self
            .main_cpu
            .cycles
            .saturating_add((M68K_HZ as f64 / FRAME_RATE * 2.0).ceil() as u64);
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
        FRAME_RATE
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
        self.z80_phase = input.u64()? % M68K_HZ;
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
