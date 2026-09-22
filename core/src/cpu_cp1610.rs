use crate::state::{StateReader, StateWriter};

pub trait Cp1610Bus: Send {
    fn read(&mut self, address: u16) -> u16;
    fn write(&mut self, address: u16, value: u16);
    fn external_condition(&mut self, _condition: u8) -> bool {
        false
    }
}

#[derive(Debug, Clone)]
pub struct Cp1610 {
    pub registers: [u16; 8],
    pub sign: bool,
    pub zero: bool,
    pub overflow: bool,
    pub carry: bool,
    pub interrupt_enabled: bool,
    pub double_data: bool,
    pub halted: bool,
    pub cycles: u64,
}

impl Default for Cp1610 {
    fn default() -> Self {
        let mut cpu = Self {
            registers: [0; 8],
            sign: false,
            zero: false,
            overflow: false,
            carry: false,
            interrupt_enabled: false,
            double_data: false,
            halted: false,
            cycles: 0,
        };
        cpu.registers[6] = 0x02f1;
        cpu.registers[7] = 0x1000;
        cpu
    }
}

impl Cp1610 {
    pub fn reset(&mut self) {
        *self = Self::default();
    }

    pub fn pc(&self) -> u16 {
        self.registers[7]
    }

    pub fn set_pc(&mut self, value: u16) {
        self.registers[7] = value;
    }

    fn fetch<B: Cp1610Bus>(&mut self, bus: &mut B) -> u16 {
        let address = self.registers[7];
        let value = bus.read(address);
        self.registers[7] = address.wrapping_add(1);
        value
    }

    fn set_sz(&mut self, value: u16) {
        self.sign = value & 0x8000 != 0;
        self.zero = value == 0;
    }

    fn set_shift_right_sz(&mut self, value: u16) {
        self.sign = value & 0x0080 != 0;
        self.zero = value == 0;
    }

    const fn register_cycle_penalty(register: usize) -> u32 {
        if register >= 6 {
            1
        } else {
            0
        }
    }

    fn set_add_flags(&mut self, left: u16, right: u16, result: u16, wide: u32) {
        self.set_sz(result);
        self.carry = wide > 0xffff;
        self.overflow = (!(left ^ right) & (left ^ result) & 0x8000) != 0;
    }

    fn set_sub_flags(&mut self, left: u16, right: u16, result: u16) {
        self.set_sz(result);
        self.carry = left >= right;
        self.overflow = ((left ^ right) & (left ^ result) & 0x8000) != 0;
    }

    fn add(&mut self, left: u16, right: u16) -> u16 {
        let wide = u32::from(left) + u32::from(right);
        let result = wide as u16;
        self.set_add_flags(left, right, result, wide);
        result
    }

    fn sub(&mut self, left: u16, right: u16) -> u16 {
        let result = left.wrapping_sub(right);
        self.set_sub_flags(left, right, result);
        result
    }

    fn status_nibble(&self) -> u16 {
        (u16::from(self.sign) << 3)
            | (u16::from(self.zero) << 2)
            | (u16::from(self.overflow) << 1)
            | u16::from(self.carry)
    }

    fn read_direct<B: Cp1610Bus>(&mut self, bus: &mut B) -> u16 {
        let address = self.fetch(bus);
        bus.read(address)
    }

    fn read_indirect<B: Cp1610Bus>(&mut self, bus: &mut B, pointer: usize, double: bool) -> u16 {
        if pointer == 6 {
            self.registers[6] = self.registers[6].wrapping_sub(1);
        }
        let address = self.registers[pointer];
        let first = bus.read(address);
        if matches!(pointer, 4 | 5 | 7) {
            self.registers[pointer] = self.registers[pointer].wrapping_add(1);
        }
        if !double {
            return first;
        }

        let low = first & 0xff;
        if matches!(pointer, 4 | 5 | 7) {
            let high = bus.read(address.wrapping_add(1)) & 0xff;
            self.registers[pointer] = self.registers[pointer].wrapping_add(1);
            low | (high << 8)
        } else {
            low | (low << 8)
        }
    }

    fn read_operand<B: Cp1610Bus>(&mut self, bus: &mut B, mode: usize, double: bool) -> u16 {
        if mode == 0 {
            self.read_direct(bus)
        } else {
            self.read_indirect(bus, mode, double)
        }
    }

    fn write_operand<B: Cp1610Bus>(&mut self, bus: &mut B, mode: usize, value: u16) {
        if mode == 0 {
            let address = self.fetch(bus);
            bus.write(address, value);
            return;
        }
        let address = self.registers[mode];
        bus.write(address, value);
        if matches!(mode, 4..=7) {
            self.registers[mode] = self.registers[mode].wrapping_add(1);
        }
    }

    pub fn interrupt<B: Cp1610Bus>(&mut self, bus: &mut B) -> u32 {
        if !self.interrupt_enabled {
            return 0;
        }
        self.halted = false;
        let pc = self.registers[7];
        self.write_operand(bus, 6, pc);
        self.registers[7] = 0x1004;
        self.cycles = self.cycles.wrapping_add(12);
        12
    }

    pub fn step<B: Cp1610Bus>(&mut self, bus: &mut B) -> u32 {
        if self.halted {
            self.cycles = self.cycles.wrapping_add(4);
            return 4;
        }
        let double = self.double_data;
        let opcode = self.fetch(bus);
        if opcode > 0x03ff {
            self.halted = true;
            self.cycles = self.cycles.wrapping_add(4);
            return 4;
        }
        let used = self.execute(bus, opcode, double);
        if double {
            self.double_data = false;
        }
        self.cycles = self.cycles.wrapping_add(u64::from(used));
        used
    }

    fn execute<B: Cp1610Bus>(&mut self, bus: &mut B, opcode: u16, double: bool) -> u32 {
        match opcode {
            0x0000 => {
                self.halted = true;
                4
            }
            0x0001 => {
                self.double_data = true;
                4
            }
            0x0002 => {
                self.interrupt_enabled = true;
                4
            }
            0x0003 => {
                self.interrupt_enabled = false;
                4
            }
            0x0004 => self.jump(bus),
            0x0005 => 4,
            0x0006 => {
                self.carry = false;
                4
            }
            0x0007 => {
                self.carry = true;
                4
            }
            0x0008..=0x000f => {
                let register = (opcode & 7) as usize;
                self.registers[register] = self.registers[register].wrapping_add(1);
                self.set_sz(self.registers[register]);
                6 + Self::register_cycle_penalty(register)
            }
            0x0010..=0x0017 => {
                let register = (opcode & 7) as usize;
                self.registers[register] = self.registers[register].wrapping_sub(1);
                self.set_sz(self.registers[register]);
                6 + Self::register_cycle_penalty(register)
            }
            0x0018..=0x001f => {
                let register = (opcode & 7) as usize;
                self.registers[register] = !self.registers[register];
                self.set_sz(self.registers[register]);
                6 + Self::register_cycle_penalty(register)
            }
            0x0020..=0x0027 => {
                let register = (opcode & 7) as usize;
                let value = self.registers[register];
                self.registers[register] = self.sub(0, value);
                6 + Self::register_cycle_penalty(register)
            }
            0x0028..=0x002f => {
                let register = (opcode & 7) as usize;
                let input_carry = self.carry;
                let value = self.registers[register];
                let addend = u16::from(input_carry);
                self.registers[register] = self.add(value, addend);
                6 + Self::register_cycle_penalty(register)
            }
            0x0030..=0x0033 => {
                let register = (opcode & 3) as usize;
                let nibble = self.status_nibble();
                self.registers[register] = (nibble << 12) | (nibble << 4);
                6
            }
            0x0034..=0x0037 => 6,
            0x0038..=0x003f => {
                let value = self.registers[(opcode & 7) as usize];
                self.sign = value & 0x0080 != 0;
                self.zero = value & 0x0040 != 0;
                self.overflow = value & 0x0020 != 0;
                self.carry = value & 0x0010 != 0;
                6
            }
            0x0040..=0x007f => self.shift(opcode),
            0x0080..=0x01ff => self.register_operation(opcode),
            0x0200..=0x023f => self.branch(bus, opcode),
            0x0240..=0x027f => {
                let mode = ((opcode >> 3) & 7) as usize;
                let source = (opcode & 7) as usize;
                let value = self.registers[source];
                self.write_operand(bus, mode, value);
                if mode == 0 {
                    11
                } else {
                    9
                }
            }
            0x0280..=0x02bf => {
                let mode = ((opcode >> 3) & 7) as usize;
                let destination = (opcode & 7) as usize;
                let value = self.read_operand(bus, mode, double);
                self.registers[destination] = value;
                if mode == 0 {
                    10 + Self::register_cycle_penalty(destination)
                } else {
                    (if double { 10 } else { 8 })
                        + Self::register_cycle_penalty(destination)
                        + if mode == 6 { 3 } else { 0 }
                }
            }
            0x02c0..=0x03ff => self.memory_operation(bus, opcode, double),
            _ => 4,
        }
    }

    fn jump<B: Cp1610Bus>(&mut self, bus: &mut B) -> u32 {
        let high = self.fetch(bus);
        let low = self.fetch(bus);
        let return_register = ((high >> 8) & 3) as usize;
        let flag_mode = high & 3;
        let target = (((high >> 2) & 0x3f) << 10) | (low & 0x03ff);
        if return_register < 3 {
            self.registers[4 + return_register] = self.registers[7];
        }
        match flag_mode {
            1 => self.interrupt_enabled = true,
            2 => self.interrupt_enabled = false,
            _ => {}
        }
        self.registers[7] = target;
        13
    }

    fn shift(&mut self, opcode: u16) -> u32 {
        let register = (opcode & 3) as usize;
        let count = if opcode & 4 != 0 { 2 } else { 1 };
        let family = (opcode >> 3) & 7;
        let mut value = self.registers[register];
        match family {
            0 => {
                let lower = value & 0x00ff;
                value = if count == 1 {
                    value.rotate_left(8)
                } else {
                    (lower << 8) | lower
                };
                self.set_shift_right_sz(value);
            }
            1 => {
                value = value.wrapping_shl(count);
                self.set_sz(value);
            }
            2 => {
                let old_carry = self.carry;
                let old_overflow = self.overflow;
                self.carry = value & 0x8000 != 0;
                if count == 2 {
                    self.overflow = value & 0x4000 != 0;
                    value = (value << 2) | (u16::from(old_carry) << 1) | u16::from(old_overflow);
                } else {
                    value = (value << 1) | u16::from(old_carry);
                }
                self.set_sz(value);
            }
            3 => {
                self.carry = value & 0x8000 != 0;
                if count == 2 {
                    self.overflow = value & 0x4000 != 0;
                }
                value <<= count;
                self.set_sz(value);
            }
            4 => {
                value >>= count;
                self.set_shift_right_sz(value);
            }
            5 => {
                value = ((value as i16) >> count) as u16;
                self.set_shift_right_sz(value);
            }
            6 => {
                let old_carry = self.carry;
                let old_overflow = self.overflow;
                self.carry = value & 1 != 0;
                if count == 2 {
                    self.overflow = value & 2 != 0;
                    value = (value >> 2)
                        | (u16::from(old_overflow) << 15)
                        | (u16::from(old_carry) << 14);
                } else {
                    value = (value >> 1) | (u16::from(old_carry) << 15);
                }
                self.set_shift_right_sz(value);
            }
            _ => {
                self.carry = value & 1 != 0;
                if count == 2 {
                    self.overflow = value & 2 != 0;
                }
                value = ((value as i16) >> count) as u16;
                self.set_shift_right_sz(value);
            }
        }
        self.registers[register] = value;
        if count == 2 {
            8
        } else {
            6
        }
    }

    fn register_operation(&mut self, opcode: u16) -> u32 {
        let source = ((opcode >> 3) & 7) as usize;
        let destination = (opcode & 7) as usize;
        let source_value = self.registers[source];
        let destination_value = self.registers[destination];
        match (opcode >> 6) & 7 {
            2 => {
                self.registers[destination] = source_value;
                self.set_sz(source_value);
            }
            3 => self.registers[destination] = self.add(destination_value, source_value),
            4 => self.registers[destination] = self.sub(destination_value, source_value),
            5 => {
                self.sub(destination_value, source_value);
            }
            6 => {
                let value = destination_value & source_value;
                self.registers[destination] = value;
                self.set_sz(value);
            }
            7 => {
                let value = destination_value ^ source_value;
                self.registers[destination] = value;
                self.set_sz(value);
            }
            _ => unreachable!(),
        }
        6 + Self::register_cycle_penalty(destination)
    }

    fn branch<B: Cp1610Bus>(&mut self, bus: &mut B, opcode: u16) -> u32 {
        let reverse = opcode & 0x20 != 0;
        let external = opcode & 0x10 != 0;
        let negate = opcode & 0x08 != 0;
        let condition = (opcode & 7) as u8;
        let offset = self.fetch(bus);
        let mut take = if external {
            bus.external_condition(condition)
        } else {
            match condition {
                0 => true,
                1 => self.carry,
                2 => self.overflow,
                3 => !self.sign,
                4 => self.zero,
                5 => self.sign != self.overflow,
                6 => self.zero || self.sign != self.overflow,
                _ => self.sign != self.carry,
            }
        };
        if negate {
            take = !take;
        }
        if take {
            self.registers[7] = if reverse {
                self.registers[7].wrapping_sub(offset).wrapping_sub(1)
            } else {
                self.registers[7].wrapping_add(offset)
            };
            9
        } else {
            7
        }
    }

    fn memory_operation<B: Cp1610Bus>(&mut self, bus: &mut B, opcode: u16, double: bool) -> u32 {
        let operation = ((opcode - 0x02c0) >> 6) as u8;
        let mode = ((opcode >> 3) & 7) as usize;
        let destination = (opcode & 7) as usize;
        let operand = self.read_operand(bus, mode, double);
        let original = self.registers[destination];
        match operation {
            0 => self.registers[destination] = self.add(original, operand),
            1 => self.registers[destination] = self.sub(original, operand),
            2 => {
                self.sub(original, operand);
            }
            3 => {
                let result = original & operand;
                self.registers[destination] = result;
                self.set_sz(result);
            }
            _ => {
                let result = original ^ operand;
                self.registers[destination] = result;
                self.set_sz(result);
            }
        }
        Self::memory_cycles(mode, double, destination)
    }

    fn memory_cycles(mode: usize, double: bool, destination: usize) -> u32 {
        if mode == 0 {
            return 10 + Self::register_cycle_penalty(destination);
        }
        let addressing_penalty = match mode {
            6 => 4,
            7 => 1,
            _ => 0,
        };
        (if double { 10 } else { 8 }) + addressing_penalty
    }

    pub fn save(&self, out: &mut StateWriter) {
        for value in self.registers {
            out.u16(value);
        }
        out.u8(u8::from(self.sign));
        out.u8(u8::from(self.zero));
        out.u8(u8::from(self.overflow));
        out.u8(u8::from(self.carry));
        out.u8(u8::from(self.interrupt_enabled));
        out.u8(u8::from(self.double_data));
        out.u8(u8::from(self.halted));
        out.u64(self.cycles);
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.registers {
            *value = input.u16()?;
        }
        self.sign = input.u8()? != 0;
        self.zero = input.u8()? != 0;
        self.overflow = input.u8()? != 0;
        self.carry = input.u8()? != 0;
        self.interrupt_enabled = input.u8()? != 0;
        self.double_data = input.u8()? != 0;
        self.halted = input.u8()? != 0;
        self.cycles = input.u64()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct TestBus {
        words: Box<[u16; 65536]>,
    }

    impl Default for TestBus {
        fn default() -> Self {
            Self {
                words: Box::new([0; 65536]),
            }
        }
    }

    impl Cp1610Bus for TestBus {
        fn read(&mut self, address: u16) -> u16 {
            self.words[address as usize]
        }
        fn write(&mut self, address: u16, value: u16) {
            self.words[address as usize] = value;
        }
    }

    #[test]
    fn reset_and_jump_match_exec_entry_sequence() {
        let mut bus = TestBus::default();
        bus.words[0x1000] = 0x0004;
        bus.words[0x1001] = 0x0112;
        bus.words[0x1002] = 0x0026;
        let mut cpu = Cp1610::default();
        assert_eq!(cpu.pc(), 0x1000);
        assert_eq!(cpu.registers[6], 0x02f1);
        assert_eq!(cpu.step(&mut bus), 13);
        assert_eq!(cpu.pc(), 0x1026);
        assert_eq!(cpu.registers[5], 0x1003);
        assert!(!cpu.interrupt_enabled);
    }

    #[test]
    fn register_arithmetic_and_relative_branch_execute() {
        let mut bus = TestBus::default();
        let start = 0x1000usize;
        bus.words[start] = 0x02b8;
        bus.words[start + 1] = 3;
        bus.words[start + 2] = 0x0008;
        bus.words[start + 3] = 0x0010;
        bus.words[start + 4] = 0x020c;
        bus.words[start + 5] = 1;
        bus.words[start + 6] = 0x0000;
        bus.words[start + 7] = 0x0007;
        let mut cpu = Cp1610::default();
        for _ in 0..4 {
            cpu.step(&mut bus);
        }
        assert_eq!(cpu.registers[0], 3);
        assert_eq!(cpu.pc(), 0x1007);
        cpu.step(&mut bus);
        assert!(cpu.carry);
    }

    #[test]
    fn stack_and_sdbd_addressing_follow_cp1610_rules() {
        let mut bus = TestBus::default();
        bus.words[0x1000] = 0x0001;
        bus.words[0x1001] = 0x02a0;
        bus.words[0x0200] = 0x0034;
        bus.words[0x0201] = 0x0012;
        let mut cpu = Cp1610::default();
        cpu.registers[4] = 0x0200;
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.registers[0], 0x1234);
        assert_eq!(cpu.registers[4], 0x0202);

        cpu.registers[6] = 0x0300;
        cpu.registers[1] = 0xabcd;
        cpu.write_operand(&mut bus, 6, cpu.registers[1]);
        assert_eq!(bus.words[0x0300], 0xabcd);
        assert_eq!(cpu.registers[6], 0x0301);
        assert_eq!(cpu.read_indirect(&mut bus, 6, false), 0xabcd);
        assert_eq!(cpu.registers[6], 0x0300);
    }

    #[test]
    fn accepted_interrupt_pushes_pc_and_vectors_to_exec() {
        let mut bus = TestBus::default();
        let mut cpu = Cp1610::default();
        cpu.registers[6] = 0x02f1;
        cpu.registers[7] = 0x5000;
        cpu.interrupt_enabled = true;
        assert_eq!(cpu.interrupt(&mut bus), 12);
        assert_eq!(bus.words[0x02f1], 0x5000);
        assert_eq!(cpu.registers[6], 0x02f2);
        assert_eq!(cpu.pc(), 0x1004);
    }

    #[test]
    fn rswd_swap_and_right_shift_flags_match_cp1610_word_rules() {
        let mut bus = TestBus::default();
        let mut cpu = Cp1610::default();
        cpu.registers[4] = 0x00b0;
        bus.words[0x1000] = 0x003c;
        cpu.step(&mut bus);
        assert!(cpu.sign);
        assert!(!cpu.zero);
        assert!(cpu.overflow);
        assert!(cpu.carry);

        cpu.set_pc(0x1001);
        cpu.registers[0] = 0x12a5;
        bus.words[0x1001] = 0x0044;
        cpu.step(&mut bus);
        assert_eq!(cpu.registers[0], 0xa5a5);
        assert!(cpu.sign);

        cpu.set_pc(0x1002);
        cpu.registers[0] = 0x0100;
        bus.words[0x1002] = 0x0060;
        cpu.step(&mut bus);
        assert_eq!(cpu.registers[0], 0x0080);
        assert!(cpu.sign);
    }

    #[test]
    fn sdbd_replicates_non_autoincrement_source_and_rejects_invalid_opcode_words() {
        let mut bus = TestBus::default();
        let mut cpu = Cp1610::default();
        cpu.registers[1] = 0x0200;
        bus.words[0x0200] = 0x0034;
        bus.words[0x1000] = 0x0001;
        bus.words[0x1001] = 0x0288;
        cpu.step(&mut bus);
        cpu.step(&mut bus);
        assert_eq!(cpu.registers[0], 0x3434);
        assert_eq!(cpu.registers[1], 0x0200);

        cpu.set_pc(0x1002);
        cpu.halted = false;
        bus.words[0x1002] = 0x0400;
        assert_eq!(cpu.step(&mut bus), 4);
        assert!(cpu.halted);
    }

    #[test]
    fn memory_instruction_cycles_include_cp1610_addressing_penalties() {
        let mut bus = TestBus::default();
        let mut cpu = Cp1610::default();
        cpu.registers[6] = 0x0301;
        bus.words[0x0300] = 1;
        bus.words[0x1000] = 0x02f0;
        assert_eq!(cpu.step(&mut bus), 12);

        cpu.reset();
        bus.words[0x1000] = 0x02f8;
        bus.words[0x1001] = 1;
        assert_eq!(cpu.step(&mut bus), 9);
    }
}
