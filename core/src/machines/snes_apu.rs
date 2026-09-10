use crate::cpu_spc700::{Spc700, Spc700Bus};
use crate::state::{StateReader, StateWriter};

const SPC_HZ: u64 = 1_024_000;
const MAIN_APPROX_HZ: u64 = 3_579_545;

#[derive(Clone, Copy, Default)]
struct SmpTimer {
    enabled: bool,
    target: u8,
    divider: u32,
    stage: u16,
    output: u8,
}

impl SmpTimer {
    fn set_enabled(&mut self, enabled: bool) {
        if enabled && !self.enabled {
            self.divider = 0;
            self.stage = 0;
            self.output = 0;
        }
        self.enabled = enabled;
    }

    fn tick(&mut self, cycles: u32, base_divider: u32) {
        if !self.enabled {
            return;
        }
        self.divider = self.divider.saturating_add(cycles);
        while self.divider >= base_divider {
            self.divider -= base_divider;
            self.stage = self.stage.wrapping_add(1);
            let target = if self.target == 0 {
                256
            } else {
                u16::from(self.target)
            };
            if self.stage >= target {
                self.stage = 0;
                self.output = self.output.wrapping_add(1) & 0x0f;
            }
        }
    }

    fn read_output(&mut self) -> u8 {
        let value = self.output & 0x0f;
        self.output = 0;
        value
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.enabled as u8);
        out.u8(self.target);
        out.u32(self.divider);
        out.u16(self.stage);
        out.u8(self.output);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.enabled = input.u8()? != 0;
        self.target = input.u8()?;
        self.divider = input.u32()?;
        self.stage = input.u16()?;
        self.output = input.u8()? & 0x0f;
        Ok(())
    }
}
struct SnesDsp {
    regs: [u8; 128],
}

impl Default for SnesDsp {
    fn default() -> Self {
        Self { regs: [0; 128] }
    }
}

impl SnesDsp {
    fn read(&self, address: u8) -> u8 {
        self.regs[usize::from(address & 0x7f)]
    }

    fn write(&mut self, address: u8, value: u8) {
        let index = usize::from(address & 0x7f);
        match index {
            0x7c => self.regs[index] &= !value,
            _ => self.regs[index] = value,
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("invalid SNES DSP register state length".into());
        }
        self.regs.copy_from_slice(regs);
        Ok(())
    }
}
struct SmpBus {
    ram: Vec<u8>,
    from_cpu: [u8; 4],
    from_apu: [u8; 4],
    control: u8,
    dsp_addr: u8,
    dsp: SnesDsp,
    timers: [SmpTimer; 3],
}

impl Default for SmpBus {
    fn default() -> Self {
        Self {
            ram: vec![0; 65_536],
            from_cpu: [0; 4],
            from_apu: [0xaa, 0xbb, 0, 0],
            control: 0xb0,
            dsp_addr: 0,
            dsp: SnesDsp::default(),
            timers: [SmpTimer::default(); 3],
        }
    }
}

impl SmpBus {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn write_control(&mut self, value: u8) {
        let previous = self.control;
        self.control = value;
        for index in 0..3 {
            let enabled = value & (1 << index) != 0;
            let was_enabled = previous & (1 << index) != 0;
            if enabled != was_enabled {
                self.timers[index].set_enabled(enabled);
            }
        }
        if value & 0x10 != 0 {
            self.from_cpu[0] = 0;
            self.from_cpu[1] = 0;
        }
        if value & 0x20 != 0 {
            self.from_cpu[2] = 0;
            self.from_cpu[3] = 0;
        }
    }

    fn tick(&mut self, cycles: u32) {
        self.timers[0].tick(cycles, 128);
        self.timers[1].tick(cycles, 128);
        self.timers[2].tick(cycles, 16);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        out.blob(&self.from_cpu);
        out.blob(&self.from_apu);
        out.u8(self.control);
        out.u8(self.dsp_addr);
        self.dsp.save(out);
        for timer in &self.timers {
            timer.save(out);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let ram = input.blob()?;
        if ram.len() != self.ram.len() {
            return Err("invalid S-SMP RAM state length".into());
        }
        self.ram.copy_from_slice(ram);
        let from_cpu = input.blob()?;
        if from_cpu.len() != 4 {
            return Err("invalid S-SMP CPU port state length".into());
        }
        self.from_cpu.copy_from_slice(from_cpu);
        let from_apu = input.blob()?;
        if from_apu.len() != 4 {
            return Err("invalid S-SMP APU port state length".into());
        }
        self.from_apu.copy_from_slice(from_apu);
        self.control = input.u8()?;
        self.dsp_addr = input.u8()?;
        self.dsp.load(input)?;
        for timer in &mut self.timers {
            timer.load(input)?;
        }
        Ok(())
    }
}

impl Spc700Bus for SmpBus {
    fn read8(&mut self, address: u16) -> u8 {
        match address {
            0x00f0 => 0,
            0x00f1 => 0,
            0x00f2 => self.dsp_addr,
            0x00f3 => self.dsp.read(self.dsp_addr),
            0x00f4..=0x00f7 => self.from_cpu[usize::from(address - 0x00f4)],
            0x00fa..=0x00fc => 0,
            0x00fd..=0x00ff => self.timers[usize::from(address - 0x00fd)].read_output(),
            _ => self.ram[usize::from(address)],
        }
    }
    fn write8(&mut self, address: u16, value: u8) {
        match address {
            0x00f0 => {}
            0x00f1 => self.write_control(value),
            0x00f2 => self.dsp_addr = value & 0x7f,
            0x00f3 => self.dsp.write(self.dsp_addr, value),
            0x00f4..=0x00f7 => self.from_apu[usize::from(address - 0x00f4)] = value,
            0x00fa..=0x00fc => self.timers[usize::from(address - 0x00fa)].target = value,
            0x00fd..=0x00ff => {}
            _ => self.ram[usize::from(address)] = value,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum IplHle {
    WaitingCommand,
    Transfer { address: u16, expected: u8 },
    Running,
}

pub struct SnesApu {
    cpu: Spc700,
    bus: SmpBus,
    hle: IplHle,
    clock_phase: u64,
    cycle_budget: i64,
}
impl Default for SnesApu {
    fn default() -> Self {
        Self::new()
    }
}

impl SnesApu {
    pub fn new() -> Self {
        Self {
            cpu: Spc700::default(),
            bus: SmpBus::default(),
            hle: IplHle::WaitingCommand,
            clock_phase: 0,
            cycle_budget: 0,
        }
    }

    pub fn reset(&mut self) {
        self.cpu = Spc700::default();
        self.bus.reset();
        self.hle = IplHle::WaitingCommand;
        self.clock_phase = 0;
        self.cycle_budget = 0;
    }

    pub fn read_cpu_port(&self, index: usize) -> u8 {
        self.bus.from_apu[index & 3]
    }

    pub fn write_cpu_port(&mut self, index: usize, value: u8) {
        let index = index & 3;
        self.bus.from_cpu[index] = value;
        if index == 0 {
            self.observe_ipl_token(value);
        }
    }
    fn observe_ipl_token(&mut self, token: u8) {
        match self.hle {
            IplHle::WaitingCommand => {
                if token == 0xcc {
                    self.begin_ipl_command(token);
                }
            }
            IplHle::Transfer {
                mut address,
                expected,
            } => {
                if token == expected {
                    self.bus.ram[usize::from(address)] = self.bus.from_cpu[1];
                    address = address.wrapping_add(1);
                    self.bus.from_apu[0] = token;
                    self.hle = IplHle::Transfer {
                        address,
                        expected: expected.wrapping_add(1),
                    };
                } else {
                    self.begin_ipl_command(token);
                }
            }
            IplHle::Running => {}
        }
    }

    fn begin_ipl_command(&mut self, token: u8) {
        let address = u16::from_le_bytes([self.bus.from_cpu[2], self.bus.from_cpu[3]]);
        self.bus.from_apu[0] = token;
        if self.bus.from_cpu[1] == 0 {
            self.cpu.pc = address;
            self.cpu.sp = 0xef;
            self.cpu.sleeping = false;
            self.cpu.stopped = false;
            self.hle = IplHle::Running;
        } else {
            self.hle = IplHle::Transfer {
                address,
                expected: 0,
            };
        }
    }
    pub fn tick_main_cycles(&mut self, main_cycles: u32) {
        self.clock_phase = self
            .clock_phase
            .saturating_add(u64::from(main_cycles) * SPC_HZ);
        let spc_cycles = self.clock_phase / MAIN_APPROX_HZ;
        self.clock_phase %= MAIN_APPROX_HZ;
        if self.hle != IplHle::Running {
            return;
        }
        self.cycle_budget = self.cycle_budget.saturating_add(spc_cycles as i64);
        while self.cycle_budget > 0 {
            let used = self.cpu.step(&mut self.bus);
            self.bus.tick(used);
            self.cycle_budget -= i64::from(used);
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        self.cpu.save(out);
        self.bus.save(out);
        match self.hle {
            IplHle::WaitingCommand => out.u8(0),
            IplHle::Transfer { address, expected } => {
                out.u8(1);
                out.u16(address);
                out.u8(expected);
            }
            IplHle::Running => out.u8(2),
        }
        out.u64(self.clock_phase);
        out.u64(self.cycle_budget as u64);
    }
    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.cpu.load(input)?;
        self.bus.load(input)?;
        self.hle = match input.u8()? {
            0 => IplHle::WaitingCommand,
            1 => IplHle::Transfer {
                address: input.u16()?,
                expected: input.u8()?,
            },
            2 => IplHle::Running,
            _ => return Err("invalid SNES IPL HLE state".into()),
        };
        self.clock_phase = input.u64()? % MAIN_APPROX_HZ;
        self.cycle_budget = input.u64()? as i64;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hle_ipl_uploads_and_starts_spc700_without_bundled_firmware() {
        let mut apu = SnesApu::new();
        assert_eq!(apu.read_cpu_port(0), 0xaa);
        assert_eq!(apu.read_cpu_port(1), 0xbb);
        apu.write_cpu_port(2, 0x00);
        apu.write_cpu_port(3, 0x02);
        apu.write_cpu_port(1, 1);
        apu.write_cpu_port(0, 0xcc);
        assert_eq!(apu.read_cpu_port(0), 0xcc);
        let program = [0xe8, 0x5a, 0xc4, 0xf4, 0x2f, 0xfc];
        for (token, byte) in program.into_iter().enumerate() {
            apu.write_cpu_port(1, byte);
            apu.write_cpu_port(0, token as u8);
            assert_eq!(apu.read_cpu_port(0), token as u8);
        }
        apu.write_cpu_port(2, 0x00);
        apu.write_cpu_port(3, 0x02);
        apu.write_cpu_port(1, 0);
        apu.write_cpu_port(0, 7);
        assert_eq!(apu.hle, IplHle::Running);
        apu.tick_main_cycles(2_000);
        assert_eq!(apu.read_cpu_port(0), 0x5a);
    }

    #[test]
    fn smp_timers_divide_clock_and_clear_on_read() {
        let mut bus = SmpBus::default();
        bus.write8(0x00fa, 1);
        bus.write8(0x00f1, 1);
        bus.tick(128);
        assert_eq!(bus.read8(0x00fd), 1);
        assert_eq!(bus.read8(0x00fd), 0);
        bus.write8(0x00fc, 2);
        bus.write8(0x00f1, 5);
        bus.tick(32);
        assert_eq!(bus.read8(0x00ff), 1);
    }
    #[test]
    fn dsp_registers_are_visible_through_smp_io() {
        let mut bus = SmpBus::default();
        bus.write8(0x00f2, 0x0c);
        bus.write8(0x00f3, 0x7f);
        assert_eq!(bus.read8(0x00f3), 0x7f);
    }
}
