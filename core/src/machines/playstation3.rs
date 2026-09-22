use crate::blueprint::GuestIsa;
use crate::clock::ClockRate;
use crate::cluster::ProcessorCluster;
use crate::cpu_powerpc64::{PowerPc64, PowerPc64Bus};
use crate::cpu_spu::{SpuBus, SpuCpu, SPU_LOCAL_STORE_SIZE};
use crate::executable::{load_elf64_be, EM_PPC64};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceBlob;
use crate::sparse_memory::SparseMemory;
use crate::state::{StateReader, StateWriter};

const STATE_VERSION: u32 = 2;
const XDR_SIZE: u64 = 256 * 1024 * 1024;
const RSX_SIZE: u64 = 256 * 1024 * 1024;
const SPU_STORE_SIZE: u64 = SPU_LOCAL_STORE_SIZE as u64;
const SPU_COUNT: usize = 6;
const FRAME_RATE: f64 = 60.0;
const FRAMES_PER_SECOND: u64 = 60;
const AUDIO_RATE: u32 = 48_000;
const STEPS_PER_FRAME: u64 = 100_000;
const SCHEDULER_QUANTUM: u64 = 1_024;
const VIDEO_WIDTH: u32 = 1280;
const VIDEO_HEIGHT: u32 = 720;

#[derive(Clone, Copy, Default)]
struct SpuMfcState {
    lsa: u32,
    eah: u32,
    eal: u32,
    size: u32,
    tag: u8,
    tag_mask: u32,
    completed_tags: u32,
    inbound_mailbox: Option<u32>,
    outbound_mailbox: Option<u32>,
    outbound_interrupt_mailbox: Option<u32>,
    faulted: bool,
}

fn valid_dma_size(size: u32) -> bool {
    matches!(size, 1 | 2 | 4 | 8) || (16..=16_384).contains(&size) && size.is_multiple_of(16)
}

fn is_get_command(command: u8) -> bool {
    matches!(command, 0x40 | 0x41 | 0x42 | 0x48 | 0x49 | 0x4a)
}

fn is_put_command(command: u8) -> bool {
    matches!(
        command,
        0x20 | 0x21 | 0x22 | 0x28 | 0x29 | 0x2a | 0x30 | 0x31 | 0x32
    )
}

impl SpuMfcState {
    fn save(&self, out: &mut StateWriter) {
        out.u32(self.lsa);
        out.u32(self.eah);
        out.u32(self.eal);
        out.u32(self.size);
        out.u8(self.tag);
        out.u32(self.tag_mask);
        out.u32(self.completed_tags);
        for mailbox in [
            self.inbound_mailbox,
            self.outbound_mailbox,
            self.outbound_interrupt_mailbox,
        ] {
            out.u8(u8::from(mailbox.is_some()));
            if let Some(value) = mailbox {
                out.u32(value);
            }
        }
        out.u8(u8::from(self.faulted));
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.lsa = input.u32()?;
        self.eah = input.u32()?;
        self.eal = input.u32()?;
        self.size = input.u32()?;
        self.tag = input.u8()?;
        self.tag_mask = input.u32()?;
        self.completed_tags = input.u32()?;
        self.inbound_mailbox = Self::load_mailbox(input)?;
        self.outbound_mailbox = Self::load_mailbox(input)?;
        self.outbound_interrupt_mailbox = Self::load_mailbox(input)?;
        self.faulted = input.u8()? != 0;
        Ok(())
    }

    fn load_mailbox(input: &mut StateReader<'_>) -> Result<Option<u32>, String> {
        Ok(if input.u8()? != 0 {
            Some(input.u32()?)
        } else {
            None
        })
    }
}

struct PlayStation3Board {
    xdr: SparseMemory,
    rsx: SparseMemory,
    spu_local_store: [SparseMemory; SPU_COUNT],
    spu_mfc: [SpuMfcState; SPU_COUNT],
    image: ResourceBlob,
    video: VideoBuffer,
    audio: AudioBuffer,
    frame: u64,
}

impl PlayStation3Board {
    fn new(image: ResourceBlob) -> Result<(Self, u64), String> {
        let mut xdr = SparseMemory::new(XDR_SIZE)?;
        let entry = load_elf64_be(&image, &mut xdr, EM_PPC64, |address| {
            (address < XDR_SIZE).then_some(address)
        })?;
        Ok((
            Self {
                xdr,
                rsx: SparseMemory::new(RSX_SIZE)?,
                spu_local_store: std::array::from_fn(|_| {
                    SparseMemory::new(SPU_STORE_SIZE).unwrap()
                }),
                spu_mfc: [SpuMfcState::default(); SPU_COUNT],
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
        self.xdr.clear();
        self.rsx.clear();
        for store in &mut self.spu_local_store {
            store.clear();
        }
        self.spu_mfc = [SpuMfcState::default(); SPU_COUNT];
        let entry = load_elf64_be(&self.image, &mut self.xdr, EM_PPC64, |address| {
            (address < XDR_SIZE).then_some(address)
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
        self.xdr.save(out);
        self.rsx.save(out);
        for store in &self.spu_local_store {
            store.save(out);
        }
        for mfc in &self.spu_mfc {
            mfc.save(out);
        }
        out.u64(self.frame);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.xdr.load(input)?;
        self.rsx.load(input)?;
        for store in &mut self.spu_local_store {
            store.load(input)?;
        }
        for mfc in &mut self.spu_mfc {
            mfc.load(input)?;
        }
        self.frame = input.u64()?;
        self.video.clear([0, 0, 0, 255]);
        self.audio.begin_frame();
        Ok(())
    }
}

impl PowerPc64Bus for PlayStation3Board {
    fn read8(&mut self, address: u64) -> u8 {
        if address < XDR_SIZE {
            self.xdr.read8(address)
        } else {
            0
        }
    }
    fn write8(&mut self, address: u64, value: u8) {
        if address < XDR_SIZE {
            self.xdr.write8(address, value);
        }
    }
}

struct SpuExecutionBus<'a> {
    store: &'a mut SparseMemory,
    xdr: &'a mut SparseMemory,
    mfc: &'a mut SpuMfcState,
}

impl SpuExecutionBus<'_> {
    fn execute_dma(&mut self, command: u8) {
        let size = self.mfc.size;
        let lsa = self.mfc.lsa & (SPU_LOCAL_STORE_SIZE - 1);
        let ea = (u64::from(self.mfc.eah) << 32) | u64::from(self.mfc.eal);
        let alignment = if size >= 16 { 16 } else { size.max(1) };
        let valid = valid_dma_size(size)
            && lsa
                .checked_add(size)
                .is_some_and(|end| end <= SPU_LOCAL_STORE_SIZE)
            && ea
                .checked_add(u64::from(size))
                .is_some_and(|end| end <= XDR_SIZE)
            && lsa.is_multiple_of(alignment)
            && ea.is_multiple_of(u64::from(alignment));
        if !valid || (!is_get_command(command) && !is_put_command(command)) {
            self.mfc.faulted = true;
            return;
        }
        let mut bytes = vec![0; size as usize];
        let result = if is_get_command(command) {
            self.xdr
                .read(ea, &mut bytes)
                .and_then(|_| self.store.write(u64::from(lsa), &bytes))
        } else {
            self.store
                .read(u64::from(lsa), &mut bytes)
                .and_then(|_| self.xdr.write(ea, &bytes))
        };
        if result.is_err() {
            self.mfc.faulted = true;
        } else {
            self.mfc.completed_tags |= 1u32 << self.mfc.tag;
        }
    }
}

impl SpuBus for SpuExecutionBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        self.store.read8(u64::from(address))
    }

    fn write8(&mut self, address: u32, value: u8) {
        self.store.write8(u64::from(address), value);
    }

    fn read_channel(&mut self, channel: u8) -> Option<u32> {
        match channel {
            12 => Some(self.mfc.tag_mask),
            13 => Some(1),
            24 => Some(self.mfc.completed_tags & self.mfc.tag_mask),
            25 | 27 => Some(0),
            29 => self.mfc.inbound_mailbox.take(),
            _ => None,
        }
    }

    fn write_channel(&mut self, channel: u8, value: u32) -> bool {
        match channel {
            16 => self.mfc.lsa = value,
            17 => self.mfc.eah = value,
            18 => self.mfc.eal = value,
            19 => self.mfc.size = value & 0x7fff,
            20 => self.mfc.tag = (value & 31) as u8,
            21 => self.execute_dma(value as u8),
            22 => self.mfc.tag_mask = value,
            23 => {}
            28 if self.mfc.outbound_mailbox.is_none() => self.mfc.outbound_mailbox = Some(value),
            30 if self.mfc.outbound_interrupt_mailbox.is_none() => {
                self.mfc.outbound_interrupt_mailbox = Some(value)
            }
            28 | 30 => return false,
            _ => return false,
        }
        true
    }

    fn channel_count(&self, channel: u8) -> u32 {
        match channel {
            12 | 13 | 16..=27 => 1,
            28 => u32::from(self.mfc.outbound_mailbox.is_none()),
            29 => u32::from(self.mfc.inbound_mailbox.is_some()),
            30 => u32::from(self.mfc.outbound_interrupt_mailbox.is_none()),
            _ => 0,
        }
    }
}

pub struct PlayStation3Machine {
    ppe: PowerPc64,
    spus: [SpuCpu; SPU_COUNT],
    board: PlayStation3Board,
    scheduler: ProcessorCluster,
    powered: bool,
}

impl PlayStation3Machine {
    pub fn from_elf(image: ResourceBlob) -> Result<Self, String> {
        let (board, entry) = PlayStation3Board::new(image)?;
        let mut ppe = PowerPc64::new();
        Self::boot_ppe(&mut ppe, entry);
        Ok(Self {
            ppe,
            spus: std::array::from_fn(|_| SpuCpu::new()),
            board,
            scheduler: Self::new_scheduler(),
            powered: true,
        })
    }

    fn new_scheduler() -> ProcessorCluster {
        let mut scheduler =
            ProcessorCluster::new(ClockRate::hz(FRAMES_PER_SECOND), SCHEDULER_QUANTUM)
                .expect("PlayStation 3 scheduler constants are valid");
        scheduler.add_processor(
            GuestIsa::PowerPc64,
            ClockRate::hz(STEPS_PER_FRAME * FRAMES_PER_SECOND),
        );
        for _ in 0..SPU_COUNT {
            let id = scheduler.add_processor(
                GuestIsa::Spu,
                ClockRate::hz(STEPS_PER_FRAME * FRAMES_PER_SECOND),
            );
            scheduler.set_halted(id, true).unwrap();
        }
        scheduler
    }

    fn boot_ppe(ppe: &mut PowerPc64, entry: u64) {
        ppe.reset_to(entry);
        ppe.gpr[1] = XDR_SIZE - 0x40;
    }

    fn run_spu_slice(&mut self, index: usize, cycles: u64) -> bool {
        let cpu = &mut self.spus[index];
        let board = &mut self.board;
        let target = cpu.cycles.saturating_add(cycles);
        while cpu.cycles < target && !cpu.halted() && !cpu.invalid_instruction() {
            let mut bus = SpuExecutionBus {
                store: &mut board.spu_local_store[index],
                xdr: &mut board.xdr,
                mfc: &mut board.spu_mfc[index],
            };
            cpu.step(&mut bus);
            if board.spu_mfc[index].faulted {
                break;
            }
        }
        cpu.halted() || cpu.invalid_instruction() || board.spu_mfc[index].faulted
    }

    fn run_processors(&mut self) {
        for batch in self.scheduler.advance_batches(1) {
            for slice in batch.slices {
                if slice.processor.0 == 0 {
                    let target = self.ppe.cycles.saturating_add(slice.cycles);
                    while self.ppe.cycles < target
                        && !self.ppe.halted()
                        && !self.ppe.invalid_instruction()
                    {
                        self.ppe.step(&mut self.board);
                    }
                    if self.ppe.halted() || self.ppe.invalid_instruction() {
                        self.scheduler.set_halted(slice.processor, true).unwrap();
                    }
                    if self.ppe.invalid_instruction() {
                        self.powered = false;
                    }
                } else {
                    let index = slice.processor.0 as usize - 1;
                    if self.run_spu_slice(index, slice.cycles) {
                        self.scheduler.set_halted(slice.processor, true).unwrap();
                    }
                }
            }
        }
    }
}

impl Machine for PlayStation3Machine {
    fn platform(&self) -> PlatformId {
        PlatformId::PlayStation3
    }
    fn reset(&mut self) {
        match self.board.reset() {
            Ok(entry) => {
                Self::boot_ppe(&mut self.ppe, entry);
                self.spus = std::array::from_fn(|_| SpuCpu::new());
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
        self.run_processors();
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
        let mut out = StateWriter::new(PlatformId::PlayStation3, STATE_VERSION);
        self.ppe.save(&mut out);
        for spu in &self.spus {
            spu.save(&mut out);
        }
        self.scheduler.save(&mut out);
        self.board.save(&mut out);
        out.u8(self.powered as u8);
        Ok(out.finish())
    }
    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::PlayStation3, STATE_VERSION)?;
        self.ppe.load(&mut input)?;
        for spu in &mut self.spus {
            spu.load(&mut input)?;
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

    fn spu_rr(opcode: u32, rt: u32, ra: u32, rb: u32) -> u32 {
        (opcode << 21) | (rb << 14) | (ra << 7) | rt
    }

    fn spu_ri10(opcode: u32, rt: u32, ra: u32, immediate: i32) -> u32 {
        (opcode << 24) | (((immediate as u32) & 0x3ff) << 14) | (ra << 7) | rt
    }

    fn spu_ri16(opcode: u32, rt: u32, immediate: i32) -> u32 {
        (opcode << 23) | (((immediate as u32) & 0xffff) << 7) | rt
    }

    fn install_spu_program(machine: &mut PlayStation3Machine, index: usize, program: &[u32]) {
        let bytes = program
            .iter()
            .flat_map(|instruction| instruction.to_be_bytes())
            .collect::<Vec<_>>();
        machine.board.spu_local_store[index]
            .write(0, &bytes)
            .unwrap();
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
    fn cell_ppe_executes_elf_into_xdr() {
        let image = elf(&[addi(3, 0, 0x3000), addi(4, 0, 0x5678), std(4, 3, 0), 0]);
        let mut machine = PlayStation3Machine::from_elf(image).unwrap();
        machine.run_frame(&InputState::default());
        let mut bytes = [0; 8];
        machine.board.xdr.read(0x3000, &mut bytes).unwrap();
        assert_eq!(u64::from_be_bytes(bytes), 0x5678);
        assert!(machine.ppe.halted());
    }

    #[test]
    fn released_spu_executes_mfc_get_and_consumes_local_store_data() {
        let image = elf(&[0]);
        let mut machine = PlayStation3Machine::from_elf(image).unwrap();
        let payload = [
            0x10, 0x21, 0x32, 0x43, 0x54, 0x65, 0x76, 0x87, 0x98, 0xa9, 0xba, 0xcb, 0xdc, 0xed,
            0xfe, 0x0f,
        ];
        machine.board.xdr.write(0x4000, &payload).unwrap();
        let program = [
            spu_ri16(0x81, 3, 0x100),
            spu_rr(0x10d, 3, 16, 0),
            spu_ri16(0x81, 3, 0),
            spu_rr(0x10d, 3, 17, 0),
            spu_ri16(0x81, 3, 0x4000),
            spu_rr(0x10d, 3, 18, 0),
            spu_ri16(0x81, 3, 16),
            spu_rr(0x10d, 3, 19, 0),
            spu_ri16(0x81, 3, 2),
            spu_rr(0x10d, 3, 20, 0),
            spu_ri16(0x81, 3, 0x40),
            spu_rr(0x10d, 3, 21, 0),
            spu_ri16(0x81, 6, 0x100),
            spu_ri10(0x34, 4, 6, 0),
            0x0123,
        ];
        install_spu_program(&mut machine, 0, &program);
        machine.spus[0].reset_to(0);
        machine
            .scheduler
            .set_halted(crate::cluster::ProcessorId(1), false)
            .unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.spus[0].gpr[4], payload);
        assert_ne!(machine.board.spu_mfc[0].completed_tags & (1 << 2), 0);
        assert!(!machine.board.spu_mfc[0].faulted);
        assert_eq!(machine.spus[0].stop_code, Some(0x123));
    }

    #[test]
    fn spu_mfc_put_and_invalid_dma_boundaries_are_enforced() {
        let image = elf(&[0]);
        let mut machine = PlayStation3Machine::from_elf(image).unwrap();
        let payload = [0xa5; 16];
        machine.board.spu_local_store[0]
            .write(0x200, &payload)
            .unwrap();
        {
            let mut bus = SpuExecutionBus {
                store: &mut machine.board.spu_local_store[0],
                xdr: &mut machine.board.xdr,
                mfc: &mut machine.board.spu_mfc[0],
            };
            assert!(bus.write_channel(16, 0x200));
            assert!(bus.write_channel(17, 0));
            assert!(bus.write_channel(18, 0x5000));
            assert!(bus.write_channel(19, 16));
            assert!(bus.write_channel(20, 4));
            assert!(bus.write_channel(21, 0x20));
        }
        let mut copied = [0; 16];
        machine.board.xdr.read(0x5000, &mut copied).unwrap();
        assert_eq!(copied, payload);
        assert_eq!(machine.board.spu_mfc[0].completed_tags, 1 << 4);
        assert!(!machine.board.spu_mfc[0].faulted);

        let mut bus = SpuExecutionBus {
            store: &mut machine.board.spu_local_store[0],
            xdr: &mut machine.board.xdr,
            mfc: &mut machine.board.spu_mfc[0],
        };
        assert!(bus.write_channel(19, 3));
        assert!(bus.write_channel(21, 0x40));
        assert!(bus.mfc.faulted);
    }

    #[test]
    fn ps3_state_preserves_xdr_rsx_spu_and_ppe() {
        let image = elf(&[0]);
        let mut machine = PlayStation3Machine::from_elf(image.clone()).unwrap();
        machine.board.xdr.write(0x4000, &[0x11]).unwrap();
        machine.board.rsx.write(0x5000, &[0x22]).unwrap();
        machine.board.spu_local_store[3]
            .write(0x6000, &[0x33])
            .unwrap();
        machine.spus[3].reset_to(0x120);
        machine.spus[3].gpr[9] = [0x44; 16];
        machine.board.spu_mfc[3].tag_mask = 0x20;
        machine.board.spu_mfc[3].completed_tags = 0x20;
        machine.board.spu_mfc[3].inbound_mailbox = Some(0x5566_7788);
        machine
            .scheduler
            .set_halted(crate::cluster::ProcessorId(4), false)
            .unwrap();
        machine.ppe.gpr[8] = 0xfeed_face_cafe_beef;
        let state = machine.save_state().unwrap();
        let mut restored = PlayStation3Machine::from_elf(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.board.xdr.read8(0x4000), 0x11);
        assert_eq!(restored.board.rsx.read8(0x5000), 0x22);
        assert_eq!(restored.board.spu_local_store[3].read8(0x6000), 0x33);
        assert_eq!(restored.spus[3].pc, 0x120);
        assert_eq!(restored.spus[3].gpr[9], [0x44; 16]);
        assert_eq!(restored.board.spu_mfc[3].tag_mask, 0x20);
        assert_eq!(restored.board.spu_mfc[3].completed_tags, 0x20);
        assert_eq!(restored.board.spu_mfc[3].inbound_mailbox, Some(0x5566_7788));
        assert_eq!(restored.ppe.gpr[8], 0xfeed_face_cafe_beef);
        assert_eq!(restored.platform(), PlatformId::PlayStation3);
    }
}
