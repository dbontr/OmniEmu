use super::sn76489::Sn76489;
use super::tms9918::Tms9918;
use crate::cpu_z80::{Z80Bus, Z80};
use crate::input::{
    AXIS_RIGHT_X, DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, KEYPAD_0, KEYPAD_1, KEYPAD_2,
    KEYPAD_3, KEYPAD_4, KEYPAD_5, KEYPAD_6, KEYPAD_7, KEYPAD_8, KEYPAD_9, KEYPAD_HASH, KEYPAD_STAR,
    LEFT, RIGHT, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::state::{StateReader, StateWriter};

const CPU_CLOCK: f64 = 3_579_545.0;
const FRAME_RATE: f64 = 59.922_743;
const STATE_VERSION: u32 = 4;

struct ColecoBus {
    bios: Vec<u8>,
    rom: Vec<u8>,
    megacart_banks: usize,
    megacart_bank: usize,
    ram: [u8; 1024],
    vdp: Tms9918,
    psg: Sn76489,
    keypad_mode: bool,
    controllers: [u8; 2],
    keypads: [u8; 2],
    spinner_reload: [u8; 2],
    spinner_status: [u8; 2],
    spinner_countdown: [u32; 2],
    spinner_d7_cycles: [u32; 2],
    spinner_irq_cycles: [u32; 2],
}

impl ColecoBus {
    fn new(bios: &[u8], rom: &[u8]) -> Result<Self, String> {
        if bios.len() != 0x2000 {
            return Err(format!(
                "ColecoVision BIOS must be exactly 8192 bytes, got {}",
                bios.len()
            ));
        }
        if rom.is_empty() {
            return Err("ColecoVision cartridge cannot be empty".into());
        }
        let megacart_banks = if rom.len() > 0x8000 {
            if !(0x10000..=0x100000).contains(&rom.len())
                || !rom.len().is_power_of_two()
                || !rom.len().is_multiple_of(0x4000)
            {
                return Err(format!(
                    "ColecoVision MegaCart must be a 64 KiB to 1 MiB power-of-two image, got {} bytes",
                    rom.len()
                ));
            }
            rom.len() / 0x4000
        } else {
            0
        };
        Ok(Self {
            bios: bios.to_vec(),
            rom: rom.to_vec(),
            megacart_banks,
            megacart_bank: 0,
            ram: [0; 1024],
            vdp: Tms9918::default(),
            psg: Sn76489::default(),
            keypad_mode: false,
            controllers: [0x7f; 2],
            keypads: [0x7f; 2],
            spinner_reload: [0; 2],
            spinner_status: [0; 2],
            spinner_countdown: [0; 2],
            spinner_d7_cycles: [0; 2],
            spinner_irq_cycles: [0; 2],
        })
    }

    fn keypad_nibble(mask: u64) -> u8 {
        let mut value = 0x0f;
        let mapping = [
            (KEYPAD_0, 0x0a),
            (KEYPAD_1, 0x0d),
            (KEYPAD_2, 0x07),
            (KEYPAD_3, 0x0c),
            (KEYPAD_4, 0x02),
            (KEYPAD_5, 0x03),
            (KEYPAD_6, 0x0e),
            (KEYPAD_7, 0x05),
            (KEYPAD_8, 0x01),
            (KEYPAD_9, 0x0b),
            (KEYPAD_HASH, 0x06),
            (KEYPAD_STAR, 0x09),
        ];
        for (button, encoded) in mapping {
            if mask & button != 0 {
                value &= encoded;
            }
        }
        value
    }

    fn spinner_reload_from_axis(axis: i16) -> u8 {
        let magnitude = i32::from(axis).unsigned_abs();
        if magnitude < 1024 {
            return 0;
        }
        let scaled = (magnitude * 127).div_ceil(32_767).clamp(1, 127) as u8;
        if axis < 0 {
            0u8.wrapping_sub(scaled)
        } else {
            scaled
        }
    }

    fn spinner_interval_cycles(reload: u8) -> u32 {
        let magnitude = u32::from((reload as i8).unsigned_abs()).max(1);
        ((CPU_CLOCK * 0.5 / f64::from(magnitude)).round() as u32).max(1)
    }

    fn set_inputs(&mut self, input: &InputState) {
        for player in 0..2 {
            let mask = input.buttons[player];

            let mut joystick = 0x7f;
            if mask & UP != 0 {
                joystick &= !0x01;
            }
            if mask & RIGHT != 0 {
                joystick &= !0x02;
            }
            if mask & DOWN != 0 {
                joystick &= !0x04;
            }
            if mask & LEFT != 0 {
                joystick &= !0x08;
            }
            if mask & FACE_EAST != 0 {
                joystick &= !0x40;
            }
            self.controllers[player] = joystick;

            let mut keypad = 0x70 | Self::keypad_nibble(mask);
            if mask & FACE_NORTH != 0 {
                keypad &= 0x74;
            }
            if mask & FACE_WEST != 0 {
                keypad &= 0x78;
            }
            if mask & FACE_SOUTH != 0 {
                keypad &= !0x40;
            }
            self.keypads[player] = keypad;

            let reload = Self::spinner_reload_from_axis(input.axes[player][AXIS_RIGHT_X]);
            if reload != self.spinner_reload[player] {
                self.spinner_reload[player] = reload;
                self.spinner_countdown[player] = if reload == 0 {
                    0
                } else {
                    Self::spinner_interval_cycles(reload)
                };
            }
        }
    }

    fn trigger_spinner_pulse(&mut self, player: usize) {
        self.spinner_status[player] = self.spinner_reload[player];
        self.spinner_d7_cycles[player] = (CPU_CLOCK * 0.000_500).round() as u32;
        self.spinner_irq_cycles[player] = (CPU_CLOCK * 0.000_011).round() as u32;
    }

    fn tick_spinner(&mut self, cycles: u32) {
        for player in 0..2 {
            let prior_d7 = self.spinner_d7_cycles[player];
            self.spinner_d7_cycles[player] = prior_d7.saturating_sub(cycles);
            if prior_d7 != 0 && self.spinner_d7_cycles[player] == 0 {
                self.spinner_status[player] = 0;
            }
            self.spinner_irq_cycles[player] =
                self.spinner_irq_cycles[player].saturating_sub(cycles);

            let reload = self.spinner_reload[player];
            if reload == 0 {
                self.spinner_countdown[player] = 0;
                continue;
            }

            let interval = Self::spinner_interval_cycles(reload);
            let mut remaining = cycles;
            if self.spinner_countdown[player] == 0 {
                self.spinner_countdown[player] = interval;
            }
            while remaining >= self.spinner_countdown[player] {
                remaining -= self.spinner_countdown[player];
                self.trigger_spinner_pulse(player);
                self.spinner_countdown[player] = interval;
            }
            self.spinner_countdown[player] -= remaining;
        }
    }

    fn spinner_irq_active(&self) -> bool {
        self.spinner_irq_cycles.iter().any(|&cycles| cycles != 0)
    }

    fn tick(&mut self, cycles: u32) {
        self.vdp.tick_cpu_cycles(cycles);
        self.psg.tick_cpu_cycles(cycles);
        self.tick_spinner(cycles);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        self.vdp.save(out);
        self.psg.save(out);
        out.u8(self.keypad_mode as u8);
        out.u8(self.controllers[0]);
        out.u8(self.controllers[1]);
        out.u8(self.keypads[0]);
        out.u8(self.keypads[1]);
        for value in self.spinner_reload {
            out.u8(value);
        }
        for value in self.spinner_status {
            out.u8(value);
        }
        for value in self.spinner_countdown {
            out.u32(value);
        }
        for value in self.spinner_d7_cycles {
            out.u32(value);
        }
        for value in self.spinner_irq_cycles {
            out.u32(value);
        }
        out.u8(self.megacart_bank as u8);
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
        self.keypads[0] = input.u8()?;
        self.keypads[1] = input.u8()?;
        for value in &mut self.spinner_reload {
            *value = input.u8()?;
        }
        for value in &mut self.spinner_status {
            *value = input.u8()?;
        }
        for value in &mut self.spinner_countdown {
            *value = input.u32()?;
        }
        for value in &mut self.spinner_d7_cycles {
            *value = input.u32()?;
        }
        for value in &mut self.spinner_irq_cycles {
            *value = input.u32()?;
        }
        let megacart_bank = usize::from(input.u8()?);
        if self.megacart_banks == 0 {
            if megacart_bank != 0 {
                return Err(
                    "ColecoVision state selects a MegaCart bank for a standard cartridge".into(),
                );
            }
        } else if megacart_bank >= self.megacart_banks {
            return Err("ColecoVision state contains an invalid MegaCart bank".into());
        }
        self.megacart_bank = megacart_bank;
        Ok(())
    }
}

impl Z80Bus for ColecoBus {
    fn mem_read(&mut self, address: u16) -> u8 {
        match address {
            0x0000..=0x1fff => self.bios[address as usize],
            0x6000..=0x7fff => self.ram[address as usize & 0x03ff],
            0x8000..=0xbfff if self.megacart_banks != 0 => {
                let bank = self.megacart_banks - 1;
                self.rom[bank * 0x4000 + usize::from(address - 0x8000)]
            }
            0xc000..=0xffbf if self.megacart_banks != 0 => {
                self.rom[self.megacart_bank * 0x4000 + usize::from(address - 0xc000)]
            }
            0xffc0..=0xffff if self.megacart_banks != 0 => {
                let selector = usize::from(0xffff - address) & (self.megacart_banks - 1);
                self.megacart_bank = self.megacart_banks - selector - 1;
                0
            }
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
                    self.keypads[player]
                } else {
                    let mut data = self.controllers[player];
                    let status = self.spinner_status[player];
                    if status & 0x80 != 0 {
                        data ^= 0x30;
                    } else if status != 0 {
                        data ^= 0x10;
                    }
                    if self.spinner_d7_cycles[player] != 0 {
                        data | 0x80
                    } else {
                        data
                    }
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
        if self.bus.spinner_irq_active() {
            let interrupt_cycles = self.cpu.irq(&mut self.bus, 0xff);
            if interrupt_cycles != 0 {
                self.bus.tick(interrupt_cycles);
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

impl Machine for ColecoVisionMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::ColecoVision
    }

    fn reset(&mut self) {
        self.cpu.reset();
        self.bus.ram = [0; 1024];
        self.bus.vdp.reset();
        self.bus.psg.reset();
        self.bus.keypad_mode = false;
        self.bus.controllers = [0x7f; 2];
        self.bus.keypads = [0x7f; 2];
        self.bus.spinner_reload = [0; 2];
        self.bus.spinner_status = [0; 2];
        self.bus.spinner_countdown = [0; 2];
        self.bus.spinner_d7_cycles = [0; 2];
        self.bus.spinner_irq_cycles = [0; 2];
        self.bus.megacart_bank = 0;
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

    fn synthetic_megacart(banks: usize) -> Vec<u8> {
        assert!(banks.is_power_of_two() && (4..=64).contains(&banks));
        let mut rom = vec![0; banks * 0x4000];
        for bank in 0..banks {
            rom[bank * 0x4000..(bank + 1) * 0x4000].fill(bank as u8);
        }
        rom
    }

    #[test]
    fn megacart_maps_fixed_last_bank_and_read_selected_switchable_bank() {
        let (bios, _) = synthetic_images();
        let rom = synthetic_megacart(4);
        let mut bus = ColecoBus::new(&bios, &rom).unwrap();

        assert_eq!(bus.megacart_banks, 4);
        assert_eq!(bus.mem_read(0x8000), 3);
        assert_eq!(bus.mem_read(0xbfff), 3);
        assert_eq!(bus.mem_read(0xc000), 0);

        assert_eq!(bus.mem_read(0xfffe), 0);
        assert_eq!(bus.megacart_bank, 2);
        assert_eq!(bus.mem_read(0xc000), 2);
        assert_eq!(bus.mem_read(0xfffd), 0);
        assert_eq!(bus.megacart_bank, 1);
        assert_eq!(bus.mem_read(0xc000), 1);
        assert_eq!(bus.mem_read(0xffff), 0);
        assert_eq!(bus.megacart_bank, 3);
        assert_eq!(bus.mem_read(0xc000), 3);
        assert_eq!(bus.mem_read(0x8000), 3);
    }

    #[test]
    fn megacart_bank_round_trips_through_state_and_reset() {
        let (bios, _) = synthetic_images();
        let rom = synthetic_megacart(8);
        let mut machine = ColecoVisionMachine::from_images(&bios, &rom).unwrap();

        machine.bus.mem_read(0xfffa);
        assert_eq!(machine.bus.megacart_bank, 2);
        let snapshot = machine.save_state().unwrap();

        machine.bus.mem_read(0xffff);
        assert_eq!(machine.bus.megacart_bank, 7);
        machine.load_state(&snapshot).unwrap();
        assert_eq!(machine.bus.megacart_bank, 2);
        assert_eq!(machine.bus.mem_read(0xc000), 2);

        machine.reset();
        assert_eq!(machine.bus.megacart_bank, 0);
        assert_eq!(machine.bus.mem_read(0xc000), 0);
    }

    #[test]
    fn megacart_rejects_non_power_of_two_and_oversized_images() {
        let (bios, _) = synthetic_images();
        assert!(ColecoBus::new(&bios, &vec![0; 0xc000])
            .err()
            .unwrap()
            .contains("MegaCart"));
        assert!(ColecoBus::new(&bios, &vec![0; 0x200000])
            .err()
            .unwrap()
            .contains("MegaCart"));
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
    fn controller_strobes_select_joystick_and_keypad_halves() {
        let (bios, rom) = synthetic_images();
        let mut bus = ColecoBus::new(&bios, &rom).unwrap();
        let mut input = InputState::default();
        input.buttons[0] = UP | RIGHT | FACE_SOUTH | FACE_EAST | KEYPAD_1;
        bus.set_inputs(&input);

        assert_eq!(bus.io_read(0xfc), 0x3c);

        bus.io_write(0x80, 0);
        assert_eq!(bus.io_read(0xfc), 0x3d);

        bus.io_write(0xc0, 0);
        assert_eq!(bus.io_read(0xfc), 0x3c);
    }

    #[test]
    fn keypad_matrix_uses_hardware_codes_and_multi_key_diode_behavior() {
        let (bios, rom) = synthetic_images();
        let mut bus = ColecoBus::new(&bios, &rom).unwrap();
        bus.io_write(0x80, 0);

        let cases = [
            (KEYPAD_0, 0x0a),
            (KEYPAD_1, 0x0d),
            (KEYPAD_2, 0x07),
            (KEYPAD_3, 0x0c),
            (KEYPAD_4, 0x02),
            (KEYPAD_5, 0x03),
            (KEYPAD_6, 0x0e),
            (KEYPAD_7, 0x05),
            (KEYPAD_8, 0x01),
            (KEYPAD_9, 0x0b),
            (KEYPAD_HASH, 0x06),
            (KEYPAD_STAR, 0x09),
        ];
        for (button, code) in cases {
            let mut input = InputState::default();
            input.buttons[0] = button;
            bus.set_inputs(&input);
            assert_eq!(bus.io_read(0xfc), 0x70 | code);
        }

        let mut input = InputState::default();
        input.buttons[0] = KEYPAD_1 | KEYPAD_4;
        bus.set_inputs(&input);
        assert_eq!(bus.io_read(0xfc), 0x70);
    }

    #[test]
    fn super_action_extra_fire_buttons_share_keypad_matrix() {
        let (bios, rom) = synthetic_images();
        let mut bus = ColecoBus::new(&bios, &rom).unwrap();
        bus.io_write(0x80, 0);

        let mut input = InputState::default();
        input.buttons[0] = FACE_NORTH;
        bus.set_inputs(&input);
        assert_eq!(bus.io_read(0xfc), 0x74);

        input.buttons[0] = FACE_WEST;
        bus.set_inputs(&input);
        assert_eq!(bus.io_read(0xfc), 0x78);

        input.buttons[0] = FACE_NORTH | FACE_WEST;
        bus.set_inputs(&input);
        assert_eq!(bus.io_read(0xfc), 0x70);
    }

    #[test]
    fn driving_controller_combines_digital_pedal_with_spinner_wheel() {
        let (bios, rom) = synthetic_images();
        let mut bus = ColecoBus::new(&bios, &rom).unwrap();
        let mut input = InputState::default();
        input.buttons[0] = FACE_EAST;
        input.axes[0][AXIS_RIGHT_X] = i16::MAX;
        bus.set_inputs(&input);

        assert_eq!(bus.io_read(0xfc) & 0x40, 0);
        let interval = ColecoBus::spinner_interval_cycles(bus.spinner_reload[0]);
        bus.tick(interval);
        let driven = bus.io_read(0xfc);
        assert_eq!(driven & 0x40, 0);
        assert_eq!(driven & 0x10, 0);
        assert_ne!(driven & 0x80, 0);

        input.buttons[0] = 0;
        bus.set_inputs(&input);
        assert_ne!(bus.io_read(0xfc) & 0x40, 0);
    }

    #[test]
    fn spinner_pulses_encode_direction_irq_and_round_trip_state() {
        let (bios, rom) = synthetic_images();
        let mut machine = ColecoVisionMachine::from_images(&bios, &rom).unwrap();

        let mut input = InputState::default();
        input.axes[0][AXIS_RIGHT_X] = i16::MAX;
        machine.bus.set_inputs(&input);
        assert_eq!(machine.bus.spinner_reload[0], 127);

        let interval = ColecoBus::spinner_interval_cycles(machine.bus.spinner_reload[0]);
        machine.bus.tick(interval);
        assert_eq!(machine.bus.spinner_status[0], 127);
        assert!(machine.bus.spinner_irq_active());
        assert_ne!(machine.bus.io_read(0xfc) & 0x80, 0);
        assert_eq!(machine.bus.io_read(0xfc) & 0x30, 0x20);

        let snapshot = machine.save_state().unwrap();
        let saved_d7 = machine.bus.spinner_d7_cycles[0];
        let saved_irq = machine.bus.spinner_irq_cycles[0];

        machine.bus.tick(saved_d7.max(saved_irq));
        assert_eq!(machine.bus.spinner_status[0], 0);
        assert!(!machine.bus.spinner_irq_active());

        machine.load_state(&snapshot).unwrap();
        assert_eq!(machine.bus.spinner_status[0], 127);
        assert_eq!(machine.bus.spinner_d7_cycles[0], saved_d7);
        assert_eq!(machine.bus.spinner_irq_cycles[0], saved_irq);

        input.axes[0][AXIS_RIGHT_X] = i16::MIN;
        machine.bus.set_inputs(&input);
        let interval = ColecoBus::spinner_interval_cycles(machine.bus.spinner_reload[0]);
        machine.bus.tick(interval);
        assert_ne!(machine.bus.spinner_status[0] & 0x80, 0);
        assert_eq!(machine.bus.io_read(0xfc) & 0x30, 0x00);
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
