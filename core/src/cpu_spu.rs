use crate::state::{StateReader, StateWriter};

pub const SPU_LOCAL_STORE_SIZE: u32 = 256 * 1024;
const LS_MASK: u32 = SPU_LOCAL_STORE_SIZE - 1;
const INSTRUCTION_MASK: u32 = LS_MASK & !3;
const QUAD_MASK: u32 = LS_MASK & !15;

pub trait SpuBus {
    fn read8(&mut self, address: u32) -> u8;
    fn write8(&mut self, address: u32, value: u8);

    fn read32(&mut self, address: u32) -> u32 {
        let bytes = [
            self.read8(address),
            self.read8(address.wrapping_add(1) & LS_MASK),
            self.read8(address.wrapping_add(2) & LS_MASK),
            self.read8(address.wrapping_add(3) & LS_MASK),
        ];
        u32::from_be_bytes(bytes)
    }

    fn read_quad(&mut self, address: u32) -> [u8; 16] {
        let mut bytes = [0; 16];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read8(address.wrapping_add(index as u32) & LS_MASK);
        }
        bytes
    }

    fn write_quad(&mut self, address: u32, value: [u8; 16]) {
        for (index, byte) in value.into_iter().enumerate() {
            self.write8(address.wrapping_add(index as u32) & LS_MASK, byte);
        }
    }

    fn read_channel(&mut self, _channel: u8) -> Option<u32> {
        None
    }

    fn write_channel(&mut self, _channel: u8, _value: u32) -> bool {
        false
    }

    fn channel_count(&self, _channel: u8) -> u32 {
        0
    }
}

#[derive(Clone)]
pub struct SpuCpu {
    pub gpr: [[u8; 16]; 128],
    pub pc: u32,
    pub cycles: u64,
    halted: bool,
    invalid_instruction: bool,
    pub stop_code: Option<u16>,
}

impl Default for SpuCpu {
    fn default() -> Self {
        Self::new()
    }
}

impl SpuCpu {
    pub fn new() -> Self {
        Self {
            gpr: [[0; 16]; 128],
            pc: 0,
            cycles: 0,
            halted: true,
            invalid_instruction: false,
            stop_code: None,
        }
    }

    pub fn reset_to(&mut self, pc: u32) {
        *self = Self::new();
        self.pc = pc & INSTRUCTION_MASK;
        self.halted = false;
    }

    pub fn halted(&self) -> bool {
        self.halted
    }

    pub fn invalid_instruction(&self) -> bool {
        self.invalid_instruction
    }

    pub fn wake(&mut self) {
        self.halted = false;
        self.stop_code = None;
    }

    fn word(register: &[u8; 16], lane: usize) -> u32 {
        let start = lane * 4;
        u32::from_be_bytes(register[start..start + 4].try_into().unwrap())
    }

    fn set_word(register: &mut [u8; 16], lane: usize, value: u32) {
        let start = lane * 4;
        register[start..start + 4].copy_from_slice(&value.to_be_bytes());
    }

    pub fn scalar(&self, register: usize) -> u32 {
        Self::word(&self.gpr[register & 127], 3)
    }

    fn splat_word(value: u32) -> [u8; 16] {
        let mut result = [0; 16];
        for lane in 0..4 {
            Self::set_word(&mut result, lane, value);
        }
        result
    }

    fn ri10(instruction: u32) -> (usize, usize, i32) {
        let rt = (instruction & 127) as usize;
        let ra = ((instruction >> 7) & 127) as usize;
        let raw = ((instruction >> 14) & 0x3ff) as i32;
        let immediate = (raw ^ 0x200) - 0x200;
        (rt, ra, immediate)
    }

    fn ri16(instruction: u32) -> (usize, i32) {
        let rt = (instruction & 127) as usize;
        let raw = ((instruction >> 7) & 0xffff) as i32;
        (rt, (raw ^ 0x8000) - 0x8000)
    }

    fn rr(instruction: u32) -> (usize, usize, usize) {
        (
            (instruction & 127) as usize,
            ((instruction >> 7) & 127) as usize,
            ((instruction >> 14) & 127) as usize,
        )
    }

    fn relative_target(pc: u32, immediate: i32) -> u32 {
        pc.wrapping_add_signed(immediate << 2) & INSTRUCTION_MASK
    }

    fn absolute_target(immediate: i32) -> u32 {
        ((immediate << 2) as u32) & INSTRUCTION_MASK
    }

    pub fn step<B: SpuBus>(&mut self, bus: &mut B) {
        if self.halted {
            self.cycles = self.cycles.wrapping_add(1);
            return;
        }
        self.invalid_instruction = false;
        let current_pc = self.pc;
        let instruction = bus.read32(current_pc);
        self.pc = current_pc.wrapping_add(4) & INSTRUCTION_MASK;

        let top8 = instruction >> 24;
        let top9 = instruction >> 23;
        let top7 = instruction >> 25;
        let top11 = instruction >> 21;
        if matches!(top8, 0x1c | 0x34 | 0x24) {
            self.execute_ri10(bus, instruction, top8);
        } else if matches!(top9, 0x81 | 0x64 | 0x60 | 0x66 | 0x62 | 0x42 | 0x40) {
            self.execute_ri16(bus, instruction, top9, current_pc);
        } else if top7 == 0x21 {
            let rt = (instruction & 127) as usize;
            self.gpr[rt] = Self::splat_word((instruction >> 7) & 0x3ffff);
        } else {
            self.execute_rr(bus, instruction, top11, current_pc);
        }
        self.cycles = self.cycles.wrapping_add(1);
    }

    fn execute_ri10<B: SpuBus>(&mut self, bus: &mut B, instruction: u32, opcode: u32) {
        let (rt, ra, immediate) = Self::ri10(instruction);
        match opcode {
            0x1c => {
                let source = self.gpr[ra];
                let mut result = [0; 16];
                for lane in 0..4 {
                    let value = Self::word(&source, lane).wrapping_add_signed(immediate);
                    Self::set_word(&mut result, lane, value);
                }
                self.gpr[rt] = result;
            }
            0x34 => {
                let address = self.scalar(ra).wrapping_add_signed(immediate << 4) & QUAD_MASK;
                self.gpr[rt] = bus.read_quad(address);
            }
            0x24 => {
                let address = self.scalar(ra).wrapping_add_signed(immediate << 4) & QUAD_MASK;
                bus.write_quad(address, self.gpr[rt]);
            }
            _ => self.invalid_instruction = true,
        }
    }

    fn execute_ri16<B: SpuBus>(
        &mut self,
        _bus: &mut B,
        instruction: u32,
        opcode: u32,
        current_pc: u32,
    ) {
        let (rt, immediate) = Self::ri16(instruction);
        match opcode {
            0x81 => self.gpr[rt] = Self::splat_word((immediate as i16 as i32) as u32),
            0x64 => self.pc = Self::relative_target(current_pc, immediate),
            0x60 => self.pc = Self::absolute_target(immediate),
            0x66 => {
                self.gpr[rt] = Self::splat_word(current_pc.wrapping_add(4));
                self.pc = Self::relative_target(current_pc, immediate);
            }
            0x62 => {
                self.gpr[rt] = Self::splat_word(current_pc.wrapping_add(4));
                self.pc = Self::absolute_target(immediate);
            }
            0x42 if self.scalar(rt) != 0 => {
                self.pc = Self::relative_target(current_pc, immediate);
            }
            0x40 if self.scalar(rt) == 0 => {
                self.pc = Self::relative_target(current_pc, immediate);
            }
            0x42 | 0x40 => {}
            _ => self.invalid_instruction = true,
        }
    }

    fn scalar_word(value: u32) -> [u8; 16] {
        let mut result = [0; 16];
        Self::set_word(&mut result, 3, value);
        result
    }

    fn binary_bits(lhs: [u8; 16], rhs: [u8; 16], operation: u8) -> [u8; 16] {
        let mut result = [0; 16];
        for index in 0..16 {
            result[index] = match operation {
                0 => lhs[index] & rhs[index],
                1 => !(lhs[index] & rhs[index]),
                2 => lhs[index] | rhs[index],
                3 => !(lhs[index] | rhs[index]),
                4 => lhs[index] ^ rhs[index],
                5 => !(lhs[index] ^ rhs[index]),
                6 => lhs[index] & !rhs[index],
                7 => lhs[index] | !rhs[index],
                _ => unreachable!(),
            };
        }
        result
    }

    fn add_words(lhs: [u8; 16], rhs: [u8; 16], subtract_from: bool) -> [u8; 16] {
        let mut result = [0; 16];
        for lane in 0..4 {
            let a = Self::word(&lhs, lane);
            let b = Self::word(&rhs, lane);
            let value = if subtract_from {
                b.wrapping_sub(a)
            } else {
                a.wrapping_add(b)
            };
            Self::set_word(&mut result, lane, value);
        }
        result
    }

    fn compare_words(lhs: [u8; 16], rhs: [u8; 16], operation: u8) -> [u8; 16] {
        let mut result = [0; 16];
        for lane in 0..4 {
            let a = Self::word(&lhs, lane);
            let b = Self::word(&rhs, lane);
            let matches = match operation {
                0 => a == b,
                1 => (a as i32) > (b as i32),
                2 => a > b,
                _ => unreachable!(),
            };
            Self::set_word(&mut result, lane, if matches { u32::MAX } else { 0 });
        }
        result
    }

    fn execute_rr<B: SpuBus>(
        &mut self,
        bus: &mut B,
        instruction: u32,
        opcode: u32,
        current_pc: u32,
    ) {
        let (rt, ra, rb) = Self::rr(instruction);
        match opcode {
            0x000 => {
                self.stop_code = Some((instruction & 0x3fff) as u16);
                self.halted = true;
            }
            0x001 | 0x002 | 0x003 | 0x201 => {}
            0x00d => {
                let channel = ra as u8;
                if let Some(value) = bus.read_channel(channel) {
                    self.gpr[rt] = Self::scalar_word(value);
                } else {
                    self.pc = current_pc;
                }
            }
            0x00f => {
                self.gpr[rt] = Self::scalar_word(bus.channel_count(ra as u8));
            }
            0x10d => {
                if !bus.write_channel(ra as u8, self.scalar(rt)) {
                    self.pc = current_pc;
                }
            }
            0x0c0 => self.gpr[rt] = Self::add_words(self.gpr[ra], self.gpr[rb], false),
            0x040 => self.gpr[rt] = Self::add_words(self.gpr[ra], self.gpr[rb], true),
            0x0c1 => self.gpr[rt] = Self::binary_bits(self.gpr[ra], self.gpr[rb], 0),
            0x0c9 => self.gpr[rt] = Self::binary_bits(self.gpr[ra], self.gpr[rb], 1),
            0x041 => self.gpr[rt] = Self::binary_bits(self.gpr[ra], self.gpr[rb], 2),
            0x049 => self.gpr[rt] = Self::binary_bits(self.gpr[ra], self.gpr[rb], 3),
            0x241 => self.gpr[rt] = Self::binary_bits(self.gpr[ra], self.gpr[rb], 4),
            0x249 => self.gpr[rt] = Self::binary_bits(self.gpr[ra], self.gpr[rb], 5),
            0x2c1 => self.gpr[rt] = Self::binary_bits(self.gpr[ra], self.gpr[rb], 6),
            0x2c9 => self.gpr[rt] = Self::binary_bits(self.gpr[ra], self.gpr[rb], 7),
            0x3c0 => self.gpr[rt] = Self::compare_words(self.gpr[ra], self.gpr[rb], 0),
            0x240 => self.gpr[rt] = Self::compare_words(self.gpr[ra], self.gpr[rb], 1),
            0x2c0 => self.gpr[rt] = Self::compare_words(self.gpr[ra], self.gpr[rb], 2),
            0x1c4 => {
                let address = self.scalar(ra).wrapping_add(self.scalar(rb)) & QUAD_MASK;
                self.gpr[rt] = bus.read_quad(address);
            }
            0x144 => {
                let address = self.scalar(ra).wrapping_add(self.scalar(rb)) & QUAD_MASK;
                bus.write_quad(address, self.gpr[rt]);
            }
            0x1a8 => self.pc = self.scalar(ra) & INSTRUCTION_MASK,
            0x1a9 => {
                self.gpr[rt] = Self::splat_word(current_pc.wrapping_add(4));
                self.pc = self.scalar(ra) & INSTRUCTION_MASK;
            }
            0x128 if self.scalar(rt) == 0 => {
                self.pc = self.scalar(ra) & INSTRUCTION_MASK;
            }
            0x129 if self.scalar(rt) != 0 => {
                self.pc = self.scalar(ra) & INSTRUCTION_MASK;
            }
            0x128 | 0x129 => {}
            _ => self.invalid_instruction = true,
        }
    }

    pub fn save(&self, out: &mut StateWriter) {
        for register in &self.gpr {
            out.blob(register);
        }
        out.u32(self.pc);
        out.u64(self.cycles);
        out.u8(u8::from(self.halted));
        out.u8(u8::from(self.invalid_instruction));
        match self.stop_code {
            Some(code) => {
                out.u8(1);
                out.u16(code);
            }
            None => out.u8(0),
        }
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for register in &mut self.gpr {
            let bytes = input.blob()?;
            if bytes.len() != register.len() {
                return Err("SPU register state has the wrong width".into());
            }
            register.copy_from_slice(bytes);
        }
        self.pc = input.u32()?;
        self.cycles = input.u64()?;
        self.halted = input.u8()? != 0;
        self.invalid_instruction = input.u8()? != 0;
        self.stop_code = if input.u8()? != 0 {
            Some(input.u16()?)
        } else {
            None
        };
        if self.pc & !INSTRUCTION_MASK != 0 || self.pc & 3 != 0 {
            return Err("SPU program counter is outside local store".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    struct Bus {
        bytes: Vec<u8>,
        inbound: Option<u32>,
        outbound: Option<u32>,
    }

    impl Bus {
        fn new() -> Self {
            Self {
                bytes: vec![0; SPU_LOCAL_STORE_SIZE as usize],
                inbound: None,
                outbound: None,
            }
        }
    }

    impl SpuBus for Bus {
        fn read8(&mut self, address: u32) -> u8 {
            self.bytes[(address & LS_MASK) as usize]
        }

        fn write8(&mut self, address: u32, value: u8) {
            self.bytes[(address & LS_MASK) as usize] = value;
        }

        fn read_channel(&mut self, channel: u8) -> Option<u32> {
            (channel == 29).then(|| self.inbound.take()).flatten()
        }

        fn write_channel(&mut self, channel: u8, value: u32) -> bool {
            if channel == 28 && self.outbound.is_none() {
                self.outbound = Some(value);
                true
            } else {
                false
            }
        }

        fn channel_count(&self, channel: u8) -> u32 {
            match channel {
                28 => u32::from(self.outbound.is_none()),
                29 => u32::from(self.inbound.is_some()),
                _ => 0,
            }
        }
    }

    fn rr(opcode: u32, rt: u32, ra: u32, rb: u32) -> u32 {
        (opcode << 21) | (rb << 14) | (ra << 7) | rt
    }

    fn ri10(opcode: u32, rt: u32, ra: u32, immediate: i32) -> u32 {
        (opcode << 24) | (((immediate as u32) & 0x3ff) << 14) | (ra << 7) | rt
    }

    fn ri16(opcode: u32, rt: u32, immediate: i32) -> u32 {
        (opcode << 23) | (((immediate as u32) & 0xffff) << 7) | rt
    }

    fn ri18(opcode: u32, rt: u32, immediate: u32) -> u32 {
        (opcode << 25) | ((immediate & 0x3ffff) << 7) | rt
    }

    fn install(bus: &mut Bus, program: &[u32]) {
        for (index, instruction) in program.iter().enumerate() {
            let start = index * 4;
            bus.bytes[start..start + 4].copy_from_slice(&instruction.to_be_bytes());
        }
    }

    #[test]
    fn integer_local_store_program_executes() {
        let program = [
            ri18(0x21, 1, 0x100),
            ri16(0x81, 3, 5),
            ri10(0x1c, 4, 3, -2),
            rr(0x0c0, 5, 3, 4),
            ri10(0x24, 5, 1, 0),
            ri10(0x34, 6, 1, 0),
            0x123,
        ];
        let mut bus = Bus::new();
        install(&mut bus, &program);
        let mut cpu = SpuCpu::new();
        cpu.reset_to(0);
        while !cpu.halted() && !cpu.invalid_instruction() {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.scalar(5), 8);
        assert_eq!(cpu.gpr[6], SpuCpu::splat_word(8));
        assert_eq!(bus.read_quad(0x100), SpuCpu::splat_word(8));
        assert_eq!(cpu.stop_code, Some(0x123));
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn conditional_and_link_branches_use_local_store_targets() {
        let program = [
            ri16(0x81, 3, 0),
            ri16(0x40, 3, 2),
            ri16(0x81, 4, 0x11),
            ri16(0x66, 5, 2),
            ri16(0x81, 4, 0x22),
            0x321,
        ];
        let mut bus = Bus::new();
        install(&mut bus, &program);
        let mut cpu = SpuCpu::new();
        cpu.reset_to(0);
        while !cpu.halted() && !cpu.invalid_instruction() {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.scalar(4), 0);
        assert_eq!(cpu.scalar(5), 16);
        assert_eq!(cpu.stop_code, Some(0x321));
        assert!(!cpu.invalid_instruction());
    }

    #[test]
    fn mailbox_channels_stall_until_ready() {
        let program = [rr(0x00d, 3, 29, 0), rr(0x10d, 3, 28, 0), 0x55];
        let mut bus = Bus::new();
        install(&mut bus, &program);
        let mut cpu = SpuCpu::new();
        cpu.reset_to(0);
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 0);
        assert_eq!(cpu.scalar(3), 0);
        bus.inbound = Some(0xdead_beef);
        cpu.step(&mut bus);
        assert_eq!(cpu.pc, 4);
        assert_eq!(cpu.scalar(3), 0xdead_beef);
        cpu.step(&mut bus);
        assert_eq!(bus.outbound, Some(0xdead_beef));
        cpu.step(&mut bus);
        assert!(cpu.halted());
        assert_eq!(cpu.stop_code, Some(0x55));
    }

    #[test]
    fn state_round_trip_preserves_vector_registers_and_stop_state() {
        let mut cpu = SpuCpu::new();
        cpu.reset_to(0x120);
        cpu.gpr[7] = [0x5a; 16];
        cpu.cycles = 1234;
        cpu.stop_code = Some(0x77);
        cpu.halted = true;

        let mut writer = StateWriter::new(PlatformId::PlayStation3, 88);
        cpu.save(&mut writer);
        let state = writer.finish();
        let mut reader = StateReader::new(&state, PlatformId::PlayStation3, 88).unwrap();
        let mut restored = SpuCpu::new();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.gpr, cpu.gpr);
        assert_eq!(restored.pc, cpu.pc);
        assert_eq!(restored.cycles, cpu.cycles);
        assert_eq!(restored.halted, cpu.halted);
        assert_eq!(restored.stop_code, cpu.stop_code);
        assert_eq!(restored.invalid_instruction, cpu.invalid_instruction);
    }
}
