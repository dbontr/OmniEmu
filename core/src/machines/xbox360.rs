use crate::blueprint::GuestIsa;
use crate::clock::ClockRate;
use crate::cluster::ProcessorCluster;
use crate::cpu_powerpc64::{PowerPc64, PowerPc64Bus};
use crate::executable::{load_elf64_be, EM_PPC64};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceBlob;
use crate::sparse_memory::SparseMemory;
use crate::state::{StateReader, StateWriter};

const STATE_VERSION: u32 = 2;
const RAM_SIZE: u64 = 512 * 1024 * 1024;
const FRAME_RATE: f64 = 60.0;
const FRAMES_PER_SECOND: u64 = 60;
const AUDIO_RATE: u32 = 48_000;
const STEPS_PER_FRAME: u64 = 100_000;
const SCHEDULER_QUANTUM: u64 = 1_024;
const CORE_COUNT: usize = 3;
const VIDEO_WIDTH: u32 = 1280;
const VIDEO_HEIGHT: u32 = 720;

struct Xbox360Board {
    memory: SparseMemory,
    image: ResourceBlob,
    video: VideoBuffer,
    audio: AudioBuffer,
    frame: u64,
}

impl Xbox360Board {
    fn new(image: ResourceBlob) -> Result<(Self, u64), String> {
        let mut memory = SparseMemory::new(RAM_SIZE)?;
        let entry = load_elf64_be(&image, &mut memory, EM_PPC64, |address| {
            (address < RAM_SIZE).then_some(address)
        })?;
        Ok((
            Self {
                memory,
                image,
                video: {
                    let mut video = VideoBuffer::new(VIDEO_WIDTH, VIDEO_HEIGHT);
                    video.clear([0, 0, 0, 255]);
                    video
                },
                audio: AudioBuffer::new(AUDIO_RATE, 2),
                frame: 0,
            },
            entry,
        ))
    }

    fn reset(&mut self) -> Result<u64, String> {
        self.memory.clear();
        let entry = load_elf64_be(&self.image, &mut self.memory, EM_PPC64, |address| {
            (address < RAM_SIZE).then_some(address)
        })?;
        self.video.clear([0, 0, 0, 255]);
        self.audio.begin_frame();
        self.frame = 0;
        Ok(entry)
    }

    fn end_frame(&mut self) {
        self.audio.begin_frame();
        for _ in 0..(AUDIO_RATE / 60) {
            self.audio.push_stereo(0.0, 0.0);
        }
        self.frame = self.frame.wrapping_add(1);
    }

    fn save(&self, out: &mut StateWriter) {
        self.memory.save(out);
        out.u64(self.frame);
    }
    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.memory.load(input)?;
        self.frame = input.u64()?;
        self.video.clear([0, 0, 0, 255]);
        self.audio.begin_frame();
        Ok(())
    }
}

impl PowerPc64Bus for Xbox360Board {
    fn read8(&mut self, address: u64) -> u8 {
        if address < RAM_SIZE {
            self.memory.read8(address)
        } else {
            0
        }
    }
    fn write8(&mut self, address: u64, value: u8) {
        if address < RAM_SIZE {
            self.memory.write8(address, value);
        }
    }
}

pub struct Xbox360Machine {
    cpus: [PowerPc64; CORE_COUNT],
    board: Xbox360Board,
    scheduler: ProcessorCluster,
    powered: bool,
}

impl Xbox360Machine {
    pub fn from_elf(image: ResourceBlob) -> Result<Self, String> {
        let (board, entry) = Xbox360Board::new(image)?;
        let mut cpus = std::array::from_fn(|_| PowerPc64::new());
        Self::boot_cpus(&mut cpus, entry);
        Ok(Self {
            cpus,
            board,
            scheduler: Self::new_scheduler(),
            powered: true,
        })
    }

    fn new_scheduler() -> ProcessorCluster {
        let mut scheduler =
            ProcessorCluster::new(ClockRate::hz(FRAMES_PER_SECOND), SCHEDULER_QUANTUM)
                .expect("Xbox 360 scheduler constants are valid");
        for index in 0..CORE_COUNT {
            let id = scheduler.add_processor(
                GuestIsa::PowerPc64,
                ClockRate::hz(STEPS_PER_FRAME * FRAMES_PER_SECOND),
            );
            if index != 0 {
                scheduler.set_halted(id, true).unwrap();
            }
        }
        scheduler
    }

    fn boot_cpus(cpus: &mut [PowerPc64; CORE_COUNT], entry: u64) {
        for cpu in cpus.iter_mut() {
            cpu.reset_to(0);
        }
        cpus[0].reset_to(entry);
        cpus[0].gpr[1] = RAM_SIZE - 0x40;
    }

    fn run_cores(&mut self) {
        for batch in self.scheduler.advance_batches(1) {
            for slice in batch.slices {
                let cpu = &mut self.cpus[slice.processor.0 as usize];
                let target = cpu.cycles.saturating_add(slice.cycles);
                while cpu.cycles < target && !cpu.halted() && !cpu.invalid_instruction() {
                    cpu.step(&mut self.board);
                }
                if cpu.halted() || cpu.invalid_instruction() {
                    self.scheduler.set_halted(slice.processor, true).unwrap();
                }
            }
        }
    }
}

impl Machine for Xbox360Machine {
    fn platform(&self) -> PlatformId {
        PlatformId::Xbox360
    }
    fn reset(&mut self) {
        match self.board.reset() {
            Ok(entry) => {
                Self::boot_cpus(&mut self.cpus, entry);
                self.scheduler = Self::new_scheduler();
                self.powered = true;
            }
            Err(_) => self.powered = false,
        }
    }
    fn run_frame(&mut self, input: &InputState) {
        let _ = input;
        if !self.powered {
            return;
        }
        self.run_cores();
        self.board.end_frame();
    }
    fn frame_rate(&self) -> f64 {
        FRAME_RATE
    }
    fn video(&self) -> &VideoBuffer {
        &self.board.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.board.audio
    }
    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Xbox360, STATE_VERSION);
        for cpu in &self.cpus {
            cpu.save(&mut out);
        }
        self.scheduler.save(&mut out);
        self.board.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }
    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Xbox360, STATE_VERSION)?;
        for cpu in &mut self.cpus {
            cpu.load(&mut input)?;
        }
        self.scheduler.load(&mut input)?;
        self.board.load(&mut input)?;
        self.powered = input.u8()? != 0;
        input.finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn addi(rt: u32, ra: u32, imm: u16) -> u32 {
        (14 << 26) | (rt << 21) | (ra << 16) | u32::from(imm)
    }
    fn std(rs: u32, ra: u32, disp: u16) -> u32 {
        (62 << 26) | (rs << 21) | (ra << 16) | (u32::from(disp) & 0xfffc)
    }
    fn elf(program: &[u32]) -> ResourceBlob {
        let payload = program
            .iter()
            .flat_map(|instruction| instruction.to_be_bytes())
            .collect::<Vec<_>>();
        let mut bytes = vec![0; 0x100 + payload.len()];
        bytes[..4].copy_from_slice(b"\x7fELF");
        bytes[4] = 2;
        bytes[5] = 2;
        bytes[6] = 1;
        bytes[16..18].copy_from_slice(&2u16.to_be_bytes());
        bytes[18..20].copy_from_slice(&EM_PPC64.to_be_bytes());
        bytes[20..24].copy_from_slice(&1u32.to_be_bytes());
        bytes[24..32].copy_from_slice(&0x1000u64.to_be_bytes());
        bytes[32..40].copy_from_slice(&64u64.to_be_bytes());
        bytes[52..54].copy_from_slice(&64u16.to_be_bytes());
        bytes[54..56].copy_from_slice(&56u16.to_be_bytes());
        bytes[56..58].copy_from_slice(&1u16.to_be_bytes());
        bytes[64..68].copy_from_slice(&1u32.to_be_bytes());
        bytes[68..72].copy_from_slice(&5u32.to_be_bytes());
        bytes[72..80].copy_from_slice(&0x100u64.to_be_bytes());
        bytes[80..88].copy_from_slice(&0x1000u64.to_be_bytes());
        bytes[88..96].copy_from_slice(&0x1000u64.to_be_bytes());
        bytes[96..104].copy_from_slice(&(payload.len() as u64).to_be_bytes());
        bytes[104..112].copy_from_slice(&(payload.len() as u64).to_be_bytes());
        bytes[112..120].copy_from_slice(&0x1000u64.to_be_bytes());
        bytes[0x100..].copy_from_slice(&payload);
        ResourceBlob::from_bytes(&bytes)
    }
    #[test]
    fn xenon_primary_core_executes_64_bit_elf() {
        let image = elf(&[addi(3, 0, 0x2000), addi(4, 0, 0x1234), std(4, 3, 0), 0]);
        let mut machine = Xbox360Machine::from_elf(image).unwrap();
        machine.run_frame(&InputState::default());
        let mut bytes = [0; 8];
        machine.board.memory.read(0x2000, &mut bytes).unwrap();
        assert_eq!(u64::from_be_bytes(bytes), 0x1234);
        assert!(machine.cpus[0].halted());
        assert_eq!(machine.cpus.len(), 3);
    }

    #[test]
    fn released_secondary_core_receives_deterministic_frame_budget() {
        let image = elf(&[0x4800_0000]);
        let mut machine = Xbox360Machine::from_elf(image).unwrap();
        machine.cpus[1].reset_to(0x1000);
        machine
            .scheduler
            .set_halted(crate::cluster::ProcessorId(1), false)
            .unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.cpus[0].cycles, STEPS_PER_FRAME);
        assert_eq!(machine.cpus[1].cycles, STEPS_PER_FRAME);
        assert_eq!(machine.cpus[2].cycles, 0);
    }

    #[test]
    fn sparse_unified_memory_and_state_round_trip() {
        let image = elf(&[0]);
        let mut machine = Xbox360Machine::from_elf(image.clone()).unwrap();
        machine
            .board
            .memory
            .write(0x1fff_f000, &[0xaa, 0xbb])
            .unwrap();
        machine.cpus[0].gpr[7] = 0x1122_3344_5566_7788;
        assert!(machine.board.memory.allocated_pages() < 16);
        let state = machine.save_state().unwrap();
        let mut restored = Xbox360Machine::from_elf(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.board.memory.read8(0x1fff_f000), 0xaa);
        assert_eq!(restored.cpus[0].gpr[7], 0x1122_3344_5566_7788);
        assert_eq!(restored.platform(), PlatformId::Xbox360);
    }
}
