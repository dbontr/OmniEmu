use crate::blueprint::GuestIsa;
use crate::clock::ClockRate;
use crate::cluster::ProcessorCluster;
use crate::cpu_powerpc750::{PowerPc750, PowerPcBus};
use crate::executable::{load_elf32_be, EM_PPC};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceBlob;
use crate::sparse_memory::SparseMemory;
use crate::state::{StateReader, StateWriter};

const STATE_VERSION: u32 = 2;
const MEM1_SIZE: u64 = 32 * 1024 * 1024;
const MEM2_SIZE: u64 = 2 * 1024 * 1024 * 1024;
const BACKING_SIZE: u64 = MEM1_SIZE + MEM2_SIZE;
const MEM2_GUEST_BASE: u64 = 0x1000_0000;
const MEM2_GUEST_END: u64 = MEM2_GUEST_BASE + MEM2_SIZE;
const CORE_COUNT: usize = 3;
const FRAME_RATE: f64 = 60.0;
const FRAMES_PER_SECOND: u64 = 60;
const AUDIO_RATE: u32 = 48_000;
const STEPS_PER_FRAME: u64 = 100_000;
const SCHEDULER_QUANTUM: u64 = 1_024;
const VIDEO_WIDTH: u32 = 1280;
const VIDEO_HEIGHT: u32 = 720;

fn guest_to_backing(address: u64) -> Option<u64> {
    if address < MEM1_SIZE {
        Some(address)
    } else if (MEM2_GUEST_BASE..MEM2_GUEST_END).contains(&address) {
        Some(MEM1_SIZE + address - MEM2_GUEST_BASE)
    } else {
        None
    }
}
struct WiiUBoard {
    memory: SparseMemory,
    image: ResourceBlob,
    video: VideoBuffer,
    audio: AudioBuffer,
    frame: u64,
}

impl WiiUBoard {
    fn new(image: ResourceBlob) -> Result<(Self, u32), String> {
        let mut memory = SparseMemory::new(BACKING_SIZE)?;
        let entry = load_elf32_be(&image, &mut memory, EM_PPC, guest_to_backing)?;
        let entry = u32::try_from(entry)
            .map_err(|_| "Wii U ELF entry exceeds 32-bit address space".to_string())?;
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

    fn reset(&mut self) -> Result<u32, String> {
        self.memory.clear();
        let entry = load_elf32_be(&self.image, &mut self.memory, EM_PPC, guest_to_backing)?;
        self.video.clear([0, 0, 0, 255]);
        self.audio.begin_frame();
        self.frame = 0;
        u32::try_from(entry).map_err(|_| "Wii U ELF entry exceeds 32-bit address space".to_string())
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

impl PowerPcBus for WiiUBoard {
    fn read8(&mut self, address: u32) -> u8 {
        guest_to_backing(u64::from(address)).map_or(0, |mapped| self.memory.read8(mapped))
    }
    fn write8(&mut self, address: u32, value: u8) {
        if let Some(mapped) = guest_to_backing(u64::from(address)) {
            self.memory.write8(mapped, value);
        }
    }
}
pub struct WiiUMachine {
    cpus: [PowerPc750; CORE_COUNT],
    board: WiiUBoard,
    scheduler: ProcessorCluster,
    powered: bool,
}

impl WiiUMachine {
    pub fn from_elf(image: ResourceBlob) -> Result<Self, String> {
        let (board, entry) = WiiUBoard::new(image)?;
        let mut cpus = std::array::from_fn(|_| PowerPc750::new());
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
                .expect("Wii U scheduler constants are valid");
        for index in 0..CORE_COUNT {
            let id = scheduler.add_processor(
                GuestIsa::PowerPc32,
                ClockRate::hz(STEPS_PER_FRAME * FRAMES_PER_SECOND),
            );
            if index != 0 {
                scheduler.set_halted(id, true).unwrap();
            }
        }
        scheduler
    }

    fn boot_cpus(cpus: &mut [PowerPc750; CORE_COUNT], entry: u32) {
        for cpu in cpus.iter_mut() {
            cpu.reset_to(0);
        }
        cpus[0].reset_to(entry);
        cpus[0].gpr[1] = 0x8fff_ffc0;
    }

    fn run_cores(&mut self) {
        for batch in self.scheduler.advance_batches(1) {
            for slice in batch.slices {
                let cpu = &mut self.cpus[slice.processor.0 as usize];
                let target = cpu.cycles.saturating_add(slice.cycles);
                while cpu.cycles < target && !cpu.halted() {
                    cpu.step(&mut self.board);
                }
                if cpu.halted() {
                    self.scheduler.set_halted(slice.processor, true).unwrap();
                }
            }
        }
    }
}

impl Machine for WiiUMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::WiiU
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
        let mut out = StateWriter::new(PlatformId::WiiU, STATE_VERSION);
        for cpu in &self.cpus {
            cpu.save(&mut out);
        }
        self.scheduler.save(&mut out);
        self.board.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }
    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::WiiU, STATE_VERSION)?;
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

    fn addis(rt: u32, ra: u32, imm: u16) -> u32 {
        (15 << 26) | (rt << 21) | (ra << 16) | u32::from(imm)
    }
    fn addi(rt: u32, ra: u32, imm: u16) -> u32 {
        (14 << 26) | (rt << 21) | (ra << 16) | u32::from(imm)
    }
    fn stw(rs: u32, ra: u32, disp: u16) -> u32 {
        (36 << 26) | (rs << 21) | (ra << 16) | u32::from(disp)
    }
    fn elf(program: &[u32]) -> ResourceBlob {
        let payload = program
            .iter()
            .flat_map(|instruction| instruction.to_be_bytes())
            .collect::<Vec<_>>();
        let mut bytes = vec![0; 0x100 + payload.len()];
        bytes[..4].copy_from_slice(b"\x7fELF");
        bytes[4] = 1;
        bytes[5] = 2;
        bytes[6] = 1;
        bytes[16..18].copy_from_slice(&2u16.to_be_bytes());
        bytes[18..20].copy_from_slice(&EM_PPC.to_be_bytes());
        bytes[20..24].copy_from_slice(&1u32.to_be_bytes());
        bytes[24..28].copy_from_slice(&0x1000u32.to_be_bytes());
        bytes[28..32].copy_from_slice(&52u32.to_be_bytes());
        bytes[40..42].copy_from_slice(&52u16.to_be_bytes());
        bytes[42..44].copy_from_slice(&32u16.to_be_bytes());
        bytes[44..46].copy_from_slice(&1u16.to_be_bytes());
        bytes[52..56].copy_from_slice(&1u32.to_be_bytes());
        bytes[56..60].copy_from_slice(&0x100u32.to_be_bytes());
        bytes[60..64].copy_from_slice(&0x1000u32.to_be_bytes());
        bytes[64..68].copy_from_slice(&0x1000u32.to_be_bytes());
        bytes[68..72].copy_from_slice(&(payload.len() as u32).to_be_bytes());
        bytes[72..76].copy_from_slice(&(payload.len() as u32).to_be_bytes());
        bytes[76..80].copy_from_slice(&5u32.to_be_bytes());
        bytes[80..84].copy_from_slice(&0x1000u32.to_be_bytes());
        bytes[0x100..].copy_from_slice(&payload);
        ResourceBlob::from_bytes(&bytes)
    }

    #[test]
    fn espresso_primary_core_executes_elf_into_mem2() {
        let image = elf(&[
            addis(3, 0, 0x5000),
            addi(4, 0, 0x1234),
            stw(4, 3, 0),
            0x4800_0000,
        ]);
        let mut machine = WiiUMachine::from_elf(image).unwrap();
        machine.run_frame(&InputState::default());
        let mapped = guest_to_backing(0x5000_0000).unwrap();
        let mut bytes = [0; 4];
        machine.board.memory.read(mapped, &mut bytes).unwrap();
        assert_eq!(u32::from_be_bytes(bytes), 0x1234);
        assert_eq!(machine.cpus.len(), 3);
    }

    #[test]
    fn released_secondary_core_receives_deterministic_frame_budget() {
        let image = elf(&[0x4800_0000]);
        let mut machine = WiiUMachine::from_elf(image).unwrap();
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
    fn wiiu_sparse_mem1_mem2_and_state_round_trip() {
        let image = elf(&[0x4800_0000]);
        let mut machine = WiiUMachine::from_elf(image.clone()).unwrap();
        let mem1 = guest_to_backing(0x0010_0000).unwrap();
        let mem2 = guest_to_backing(0x5000_1000).unwrap();
        machine.board.memory.write(mem1, &[0xaa]).unwrap();
        machine.board.memory.write(mem2, &[0xbb]).unwrap();
        machine.cpus[0].gpr[7] = 0xfeed_beef;
        assert!(machine.board.memory.allocated_pages() < 16);
        let state = machine.save_state().unwrap();
        let mut restored = WiiUMachine::from_elf(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.board.memory.read8(mem1), 0xaa);
        assert_eq!(restored.board.memory.read8(mem2), 0xbb);
        assert_eq!(restored.cpus[0].gpr[7], 0xfeed_beef);
        assert_eq!(restored.platform(), PlatformId::WiiU);
    }
}
