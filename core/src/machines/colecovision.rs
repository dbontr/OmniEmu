use super::sn76489::Sn76489;
use super::tms9918::Tms9918;
use crate::cpu_z80::{Z80Bus, Z80};
use crate::input::{DOWN, FACE_EAST, FACE_SOUTH, LEFT, RIGHT, UP};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const CPU_CLOCK: f64 = 3_579_545.0;
const FRAME_RATE: f64 = 59.922_743;
const STATE_VERSION: u32 = 1;

struct ColecoBus {
    bios: Vec<u8>,
    rom: Vec<u8>,
    ram: [u8; 1024],
    vdp: Tms9918,
    psg: Sn76489,
    keypad_mode: bool,
    controllers: [u8; 2],
}

impl ColecoBus {
    fn new(bios: &[u8], rom: &[u8]) -> Result<Self, String> {
        if bios.len() != 0x2000 {
            return Err(format!(
                "ColecoVision BIOS must be exactly 8192 bytes, got {}",
                bios.len()
            ));
        }
        if rom.is_empty() || rom.len() > 0x8000 {
            return Err(format!(
                "ColecoVision cartridge must be 1..=32768 bytes, got {}",
                rom.len()
            ));
        }
        Ok(Self {
            bios: bios.to_vec(),
            rom: rom.to_vec(),
            ram: [0; 1024],
            vdp: Tms9918::default(),
            psg: Sn76489::default(),
            keypad_mode: false,
            controllers: [0xff; 2],
        })
    }

    fn set_inputs(&mut self, input: &InputState) {
        for player in 0..2 {
            let mask = input.buttons[player];
            let mut value = 0x7f;
            if mask & UP != 0 {
                value &= !0x01;
            }
            if mask & RIGHT != 0 {
                value &= !0x02;
            }
            if mask & DOWN != 0 {
                value &= !0x04;
            }
            if mask & LEFT != 0 {
                value &= !0x08;
            }
            if mask & FACE_SOUTH != 0 {
                value &= !0x40;
            }
            if mask & FACE_EAST != 0 {
                value &= !0x10;
            }
            self.controllers[player] = value;
        }
    }

    fn tick(&mut self, cycles: u32) {
        self.vdp.tick_cpu_cycles(cycles);
        self.psg.tick_cpu_cycles(cycles);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        self.vdp.save(out);
        self.psg.save(out);
        out.u8(self.keypad_mode as u8);
        out.u8(self.controllers[0]);
        out.u8(self.controllers[1]);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("invalid ColecoVision RAM state length".into());
        }
        self.ram.copy_from_slice(ram);
        self.vdp.load(input)?;
        self.psg.load(input)?;
        self.keypad_mode = input.u8()? != 0;
        self.controllers[0] = input.u8()?;
        self.controllers[1] = input.u8()?;
        Ok(())
    }
}

impl Z80Bus for ColecoBus {
    fn mem_read(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0x1fff => self.bios[address as usize],
            0x6000..=0x7fff => self.ram[address as usize & 0x03ff],
            0x8000..=0xffff => {
                let index = (address as usize - 0x8000) % self.rom.len();
                self.rom[index]
            }
            _ => 0xff,
        }
    }

    fn mem_write(&mut self, address: u16, value: u8) {
        if (0x6000..=0x7fff).contains(&address) {
            self.ram[address as usize & 0x03ff] = value;
        }
    }

    fn io_read(&mut self, port: u16) -> u8 {
        let low = port as u8;
        match low {
            0xa0..=0xbf if low & 1 == 0 => self.vdp.read_data(),
            0xa0..=0xbf => self.vdp.read_status(),
            0xe0..=0xff => {
                let player = usize::from((low & 0x02) != 0);
                if self.keypad_mode {
                    0x7f
                } else {
                    self.controllers[player]
                }
            }
            _ => 0xff,
        }
    }

    fn io_write(&mut self, port: u16, value: u8) {
        let low = port as u8;
        match low {
            0x80..=0x9f => self.keypad_mode = true,
            0xa0..=0xbf if low & 1 == 0 => self.vdp.write_data(value),
            0xa0..=0xbf => self.vdp.write_control(value),
            0xc0..=0xdf => self.keypad_mode = false,
            0xe0..=0xff => self.psg.write(value),
            _ => {}
        }
    }
}

pub struct ColecoVisionMachine {
    cpu: Z80,
    bus: ColecoBus,
    audio: AudioBuffer,
    powered: bool,
}

impl ColecoVisionMachine {
    pub fn from_images(bios: &[u8], rom: &[u8]) -> Result<Self, String> {
        Ok(Self {
            cpu: Z80::default(),
            bus: ColecoBus::new(bios, rom)?,
            audio: AudioBuffer::new(48_000, 2),
            powered: true,
        })
    }

    fn clock_instruction(&mut self) -> u32 {
        let cycles = self.cpu.step(&mut self.bus);
        self.bus.tick(cycles);
        if self.bus.vdp.take_irq_edge() {
            let interrupt_cycles = self.cpu.nmi(&mut self.bus);
            self.bus.tick(interrupt_cycles);
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

impl Machine for ColecoVisionMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::ColecoVision
    }

    fn reset(&mut self) {
        self.cpu.reset();
        self.bus.ram = [0; 1024];
        self.bus.vdp.reset();
        self.bus.psg.reset();
        self.powered = true;
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.bus.set_inputs(input);
        self.bus.psg.begin_frame();
        let target = self.bus.vdp.frame().wrapping_add(1);
        let deadline = self
            .cpu
            .cycles
            .saturating_add((CPU_CLOCK / FRAME_RATE * 2.0).ceil() as u64);
        while self.bus.vdp.frame() != target && self.cpu.cycles < deadline {
            self.clock_instruction();
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
        let mut out = StateWriter::new(PlatformId::ColecoVision, STATE_VERSION);
        self.cpu.save(&mut out);
        self.bus.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::ColecoVision, STATE_VERSION)?;
        self.cpu.load(&mut input)?;
        self.bus.load(&mut input)?;
        self.powered = input.u8()? != 0;
        self.audio.begin_frame();
        input.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_images() -> (Vec<u8>, Vec<u8>) {
        let mut bios = vec![0; 0x2000];
        bios[..3].copy_from_slice(&[0xc3, 0x00, 0x80]);
        bios[0x66..0x68].copy_from_slice(&[0xed, 0x45]);
        let mut rom = vec![0; 0x2000];
        let program: &[u8] = &[
            0xf3, 0x31, 0xff, 0x7f, 0x3e, 0x02, 0xd3, 0xbf, 0x3e, 0x80, 0xd3, 0xbf, 0x3e, 0x40,
            0xd3, 0xbf, 0x3e, 0x81, 0xd3, 0xbf, 0x3e, 0x06, 0xd3, 0xbf, 0x3e, 0x82, 0xd3, 0xbf,
            0x3e, 0x80, 0xd3, 0xbf, 0x3e, 0x83, 0xd3, 0xbf, 0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x84,
            0xd3, 0xbf, 0x3e, 0x36, 0xd3, 0xbf, 0x3e, 0x85, 0xd3, 0xbf, 0x3e, 0x07, 0xd3, 0xbf,
            0x3e, 0x86, 0xd3, 0xbf, 0x3e, 0x01, 0xd3, 0xbf, 0x3e, 0x87, 0xd3, 0xbf,
        ];
        rom[..program.len()].copy_from_slice(program);
        let tail: &[u8] = &[
            0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x40, 0xd3, 0xbf, 0x21, 0x00, 0x81, 0x06, 0x08, 0x7e,
            0xd3, 0xbe, 0x23, 0x10, 0xfa, 0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x60, 0xd3, 0xbf, 0x21,
            0x10, 0x81, 0x06, 0x08, 0x7e, 0xd3, 0xbe, 0x23, 0x10, 0xfa, 0x3e, 0x00, 0xd3, 0xbf,
            0x3e, 0x58, 0xd3, 0xbf, 0xaf, 0xd3, 0xbe, 0x3e, 0x84, 0xd3, 0xff, 0x3e, 0x10, 0xd3,
            0xff, 0x3e, 0x90, 0xd3, 0xff,
        ];
        let tail_start = program.len();
        rom[tail_start..tail_start + tail.len()].copy_from_slice(tail);
        let loop_address = 0x8000u16 + (tail_start + tail.len()) as u16;
        let [lo, hi] = loop_address.to_le_bytes();
        let loop_offset = tail_start + tail.len();
        rom[loop_offset..loop_offset + 3].copy_from_slice(&[0xc3, lo, hi]);
        rom[0x100..0x108].fill(0xff);
        rom[0x110..0x118].fill(0xf1);
        (bios, rom)
    }

    #[test]
    fn synthetic_cartridge_runs_shared_z80_vdp_and_psg() {
        let (bios, rom) = synthetic_images();
        let mut machine = ColecoVisionMachine::from_images(&bios, &rom).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.video().width(), 256);
        assert_eq!(machine.video().height(), 192);
        assert!(machine
            .video()
            .pixels()
            .as_chunks::<4>()
            .0
            .iter()
            .any(|pixel| { pixel[0] > 180 && pixel[1] > 180 && pixel[2] > 180 }));
        assert!(!machine.audio().samples().is_empty());
    }

    #[test]
    fn save_state_round_trip_restores_cpu_and_vdp_frame() {
        let (bios, rom) = synthetic_images();
        let mut machine = ColecoVisionMachine::from_images(&bios, &rom).unwrap();
        machine.run_frame(&InputState::default());
        let snapshot = machine.save_state().unwrap();
        let pc = machine.cpu.pc;
        let frame = machine.bus.vdp.frame();
        machine.run_frame(&InputState::default());
        machine.load_state(&snapshot).unwrap();
        assert_eq!(machine.cpu.pc, pc);
        assert_eq!(machine.bus.vdp.frame(), frame);
    }
}
