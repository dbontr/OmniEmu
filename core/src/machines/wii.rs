use crate::cpu_powerpc750::{PowerPc750, PowerPcBus};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceBlob;
use crate::state::{StateReader, StateWriter};

const STATE_VERSION: u32 = 1;
const MEM1_SIZE: usize = 24 * 1024 * 1024;
const MEM2_SIZE: usize = 64 * 1024 * 1024;
const HW_SIZE: usize = 0x1_0000;
const FRAME_RATE: u64 = 60;
const CPU_STEPS_PER_FRAME: u64 = 300_000;
const AUDIO_RATE: u32 = 48_000;
const VIDEO_WIDTH: u32 = 640;
const VIDEO_HEIGHT: u32 = 480;

const FLIPPER_BASE: u32 = 0xcc00_0000;
const HOLLYWOOD_BASE: u32 = 0xcd00_0000;
const PI_CAUSE: usize = 0x3000;
const PI_MASK: usize = 0x3004;
const PI_INT_VI: u32 = 0x0100;
const PI_INT_WII_IPC: u32 = 0x4000;
const VI_FB_LEFT_TOP_HI: usize = 0x201c;
const VI_FB_LEFT_TOP_LO: usize = 0x201e;
const VI_PRERETRACE_HI: usize = 0x2030;
const VI_POSTRETRACE_HI: usize = 0x2034;
const IPC_PPCMSG: usize = 0x00;
const IPC_PPCCTRL: usize = 0x04;
const IPC_ARMMSG: usize = 0x08;
const IPC_PPCSPEED: usize = 0x18;
const IPC_PPC_IRQFLAG: usize = 0x30;
const IPC_PPC_IRQMASK: usize = 0x34;
const IPC_AHBPROT: usize = 0x64;
const IPC_HW_RESETS: usize = 0x194;
const IPC_IRQ_BROADWAY: u32 = 0x4000_0000;

fn be32(bytes: &[u8], offset: usize) -> Result<u32, String> {
    let end = offset
        .checked_add(4)
        .ok_or_else(|| "Wii DOL offset overflow".to_string())?;
    let value = bytes
        .get(offset..end)
        .ok_or_else(|| "Wii DOL header is truncated".to_string())?;
    Ok(u32::from_be_bytes(value.try_into().unwrap()))
}

#[derive(Clone, Copy)]
enum MemoryRegion {
    Mem1(usize),
    Mem2(usize),
}
fn memory_region(address: u32) -> Option<MemoryRegion> {
    match address {
        0x0000_0000..=0x017f_ffff => Some(MemoryRegion::Mem1(address as usize)),
        0x8000_0000..=0x817f_ffff | 0xc000_0000..=0xc17f_ffff => {
            Some(MemoryRegion::Mem1((address & 0x01ff_ffff) as usize))
        }
        0x1000_0000..=0x13ff_ffff => Some(MemoryRegion::Mem2((address - 0x1000_0000) as usize)),
        0x9000_0000..=0x93ff_ffff | 0xd000_0000..=0xd3ff_ffff => {
            Some(MemoryRegion::Mem2((address & 0x03ff_ffff) as usize))
        }
        _ => None,
    }
}

fn memory_range(address: u32, len: usize) -> Result<(MemoryRegion, usize), String> {
    let region = memory_region(address)
        .ok_or_else(|| format!("Wii DOL section address {address:#010x} is outside MEM1/MEM2"))?;
    let start = match region {
        MemoryRegion::Mem1(index) | MemoryRegion::Mem2(index) => index,
    };
    let limit = match region {
        MemoryRegion::Mem1(_) => MEM1_SIZE,
        MemoryRegion::Mem2(_) => MEM2_SIZE,
    };
    let end = start
        .checked_add(len)
        .filter(|end| *end <= limit)
        .ok_or_else(|| "Wii DOL section exceeds its memory region".to_string())?;
    Ok((region, end))
}
fn load_dol(image: &ResourceBlob, mem1: &mut [u8], mem2: &mut [u8]) -> Result<u32, String> {
    let mut header = [0; 0x100];
    image.read(0, &mut header)?;
    let bss_address = be32(&header, 0xd8)?;
    let bss_size = be32(&header, 0xdc)? as usize;
    let entry = be32(&header, 0xe0)?;
    if entry == 0 {
        return Err("Wii DOL entry point is zero".into());
    }
    if bss_size != 0 {
        let (region, end) = memory_range(bss_address, bss_size)?;
        match region {
            MemoryRegion::Mem1(start) => mem1[start..end].fill(0),
            MemoryRegion::Mem2(start) => mem2[start..end].fill(0),
        }
    }
    for section in 0..18usize {
        let (offset_field, address_field, size_field) = if section < 7 {
            (section * 4, 0x48 + section * 4, 0x90 + section * 4)
        } else {
            let data = section - 7;
            (0x1c + data * 4, 0x64 + data * 4, 0xac + data * 4)
        };
        let file_offset = be32(&header, offset_field)? as u64;
        let address = be32(&header, address_field)?;
        let size = be32(&header, size_field)? as usize;
        if size == 0 {
            continue;
        }
        let end = file_offset
            .checked_add(size as u64)
            .filter(|end| *end <= image.len())
            .ok_or_else(|| "Wii DOL section exceeds the staged file".to_string())?;
        let (region, memory_end) = memory_range(address, size)?;
        let mut bytes = vec![0; size];
        image.read(file_offset, &mut bytes)?;
        debug_assert_eq!(end - file_offset, size as u64);
        match region {
            MemoryRegion::Mem1(start) => mem1[start..memory_end].copy_from_slice(&bytes),
            MemoryRegion::Mem2(start) => mem2[start..memory_end].copy_from_slice(&bytes),
        }
    }
    Ok(entry)
}

struct WiiBoard {
    mem1: Box<[u8]>,
    mem2: Box<[u8]>,
    flipper: Box<[u8]>,
    hollywood: Box<[u8]>,
    image: ResourceBlob,
    video: VideoBuffer,
    audio: AudioBuffer,
    frame_counter: u64,
}
impl WiiBoard {
    fn new(image: ResourceBlob) -> Result<(Self, u32), String> {
        let mut mem1 = vec![0; MEM1_SIZE].into_boxed_slice();
        let mut mem2 = vec![0; MEM2_SIZE].into_boxed_slice();
        let entry = load_dol(&image, &mut mem1, &mut mem2)?;
        let mut board = Self {
            mem1,
            mem2,
            flipper: vec![0; HW_SIZE].into_boxed_slice(),
            hollywood: vec![0; HW_SIZE].into_boxed_slice(),
            image,
            video: VideoBuffer::new(VIDEO_WIDTH, VIDEO_HEIGHT),
            audio: AudioBuffer::new(AUDIO_RATE, 2),
            frame_counter: 0,
        };
        board.reset_registers();
        board.initialize_low_memory();
        Ok((board, entry))
    }

    fn reset_registers(&mut self) {
        self.flipper.fill(0);
        self.hollywood.fill(0);
        self.write_hollywood_u32(IPC_PPCSPEED, 1);
        self.write_hollywood_u32(IPC_PPC_IRQMASK, IPC_IRQ_BROADWAY);
        self.write_hollywood_u32(IPC_AHBPROT, u32::MAX);
        self.write_hollywood_u32(IPC_HW_RESETS, u32::MAX);
    }
    fn initialize_low_memory(&mut self) {
        self.write_memory_u32(0x0000_0028, MEM1_SIZE as u32);
        self.write_memory_u32(0x0000_00f8, 243_000_000);
        self.write_memory_u32(0x0000_00fc, 729_000_000);
        self.write_memory_u32(0x0000_3118, MEM2_SIZE as u32);
        self.write_memory_u32(0x0000_311c, MEM2_SIZE as u32);
        self.write_memory_u32(0x0000_3120, 0x1400_0000);
    }

    fn write_memory_u32(&mut self, address: u32, value: u32) {
        let bytes = value.to_be_bytes();
        for (offset, byte) in bytes.into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }

    fn read_flipper_u16(&self, offset: usize) -> u16 {
        u16::from_be_bytes(self.flipper[offset..offset + 2].try_into().unwrap())
    }

    fn read_flipper_u32(&self, offset: usize) -> u32 {
        u32::from_be_bytes(self.flipper[offset..offset + 4].try_into().unwrap())
    }

    fn write_flipper_u32(&mut self, offset: usize, value: u32) {
        self.flipper[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
    }
    fn read_hollywood_u32(&self, offset: usize) -> u32 {
        u32::from_be_bytes(self.hollywood[offset..offset + 4].try_into().unwrap())
    }

    fn write_hollywood_u32(&mut self, offset: usize, value: u32) {
        self.hollywood[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
    }

    fn set_pi_cause(&mut self, mask: u32, active: bool) {
        let mut cause = self.read_flipper_u32(PI_CAUSE);
        if active {
            cause |= mask;
        } else {
            cause &= !mask;
        }
        self.write_flipper_u32(PI_CAUSE, cause);
    }

    fn interrupt_pending(&self) -> bool {
        self.read_flipper_u32(PI_CAUSE) & self.read_flipper_u32(PI_MASK) != 0
    }

    fn service_ipc_request(&mut self) {
        let ctrl = self.read_hollywood_u32(IPC_PPCCTRL) as u8;
        if ctrl & 1 == 0 {
            return;
        }
        let message = self.read_hollywood_u32(IPC_PPCMSG);
        self.write_hollywood_u32(IPC_ARMMSG, message);
        let reply = (ctrl & !1) | 0x06;
        self.write_hollywood_u32(IPC_PPCCTRL, u32::from(reply));
        let flags = self.read_hollywood_u32(IPC_PPC_IRQFLAG) | IPC_IRQ_BROADWAY;
        self.write_hollywood_u32(IPC_PPC_IRQFLAG, flags);
        self.refresh_ipc_interrupt();
    }
    fn refresh_ipc_interrupt(&mut self) {
        let flags = self.read_hollywood_u32(IPC_PPC_IRQFLAG);
        let mask = self.read_hollywood_u32(IPC_PPC_IRQMASK);
        self.set_pi_cause(PI_INT_WII_IPC, flags & mask != 0);
    }

    fn write_hollywood_register(&mut self, offset: usize, value: u32) {
        match offset {
            IPC_PPCCTRL => {
                self.write_hollywood_u32(offset, value & 0x3f);
                self.service_ipc_request();
            }
            IPC_PPC_IRQFLAG => {
                let flags = self.read_hollywood_u32(offset) & !value;
                self.write_hollywood_u32(offset, flags);
                self.refresh_ipc_interrupt();
            }
            IPC_PPC_IRQMASK => {
                self.write_hollywood_u32(offset, value);
                self.refresh_ipc_interrupt();
            }
            IPC_AHBPROT => {}
            _ => self.write_hollywood_u32(offset, value),
        }
    }

    fn xfb_address(&self) -> Option<u32> {
        let raw = (u32::from(self.read_flipper_u16(VI_FB_LEFT_TOP_HI)) << 16)
            | u32::from(self.read_flipper_u16(VI_FB_LEFT_TOP_LO));
        let fbb = raw & 0x00ff_ffff;
        let address = if raw & (1 << 28) != 0 { fbb << 5 } else { fbb };
        (address != 0).then_some(address)
    }
    fn clamp_byte(value: i32) -> u8 {
        value.clamp(0, 255) as u8
    }

    fn present_video(&mut self) {
        let Some(address) = self.xfb_address() else {
            self.video.clear([0, 0, 0, 255]);
            return;
        };
        let byte_len = (VIDEO_WIDTH * VIDEO_HEIGHT * 2) as usize;
        let Some(MemoryRegion::Mem1(start)) = memory_region(address) else {
            self.video.clear([0, 0, 0, 255]);
            return;
        };
        let Some(end) = start
            .checked_add(byte_len)
            .filter(|end| *end <= self.mem1.len())
        else {
            self.video.clear([0, 0, 0, 255]);
            return;
        };
        let source = &self.mem1[start..end];
        let pixels = self.video.pixels_mut();
        for (pair, rgba) in source
            .as_chunks::<4>()
            .0
            .iter()
            .zip(pixels.as_chunks_mut::<8>().0.iter_mut())
        {
            let y0 = i32::from(pair[0]);
            let u = i32::from(pair[1]) - 128;
            let y1 = i32::from(pair[2]);
            let v = i32::from(pair[3]) - 128;
            let convert = |y: i32| {
                let c = (y - 16).max(0);
                [
                    Self::clamp_byte((298 * c + 409 * v + 128) >> 8),
                    Self::clamp_byte((298 * c - 100 * u - 208 * v + 128) >> 8),
                    Self::clamp_byte((298 * c + 516 * u + 128) >> 8),
                    255,
                ]
            };
            rgba[..4].copy_from_slice(&convert(y0));
            rgba[4..].copy_from_slice(&convert(y1));
        }
    }

    fn raise_vi_retrace(&mut self) {
        let mut active = false;
        for offset in [VI_PRERETRACE_HI, VI_POSTRETRACE_HI] {
            let register = self.read_flipper_u32(offset);
            if register & (1 << 28) != 0 {
                self.write_flipper_u32(offset, register | (1 << 31));
                active = true;
            }
        }
        self.set_pi_cause(PI_INT_VI, active);
    }

    fn end_frame(&mut self) {
        self.present_video();
        self.audio.begin_frame();
        for _ in 0..(AUDIO_RATE / FRAME_RATE as u32) {
            self.audio.push_stereo(0.0, 0.0);
        }
        self.raise_vi_retrace();
        self.frame_counter = self.frame_counter.wrapping_add(1);
    }
    fn reset(&mut self) -> Result<u32, String> {
        self.mem1.fill(0);
        self.mem2.fill(0);
        let entry = load_dol(&self.image, &mut self.mem1, &mut self.mem2)?;
        self.reset_registers();
        self.initialize_low_memory();
        self.video.clear([0, 0, 0, 255]);
        self.audio.begin_frame();
        self.frame_counter = 0;
        Ok(entry)
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.mem1);
        out.blob(&self.mem2);
        out.blob(&self.flipper);
        out.blob(&self.hollywood);
        out.u64(self.frame_counter);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        Self::load_blob(input, &mut self.mem1, "MEM1")?;
        Self::load_blob(input, &mut self.mem2, "MEM2")?;
        Self::load_blob(input, &mut self.flipper, "Flipper registers")?;
        Self::load_blob(input, &mut self.hollywood, "Hollywood registers")?;
        self.frame_counter = input.u64()?;
        self.present_video();
        self.audio.begin_frame();
        Ok(())
    }
    fn load_blob(
        input: &mut StateReader<'_>,
        target: &mut [u8],
        label: &str,
    ) -> Result<(), String> {
        let bytes = input.blob()?;
        if bytes.len() != target.len() {
            return Err(format!(
                "Wii {label} state has {} bytes; expected {}",
                bytes.len(),
                target.len()
            ));
        }
        target.copy_from_slice(bytes);
        Ok(())
    }

    fn read_region8(&self, region: MemoryRegion) -> u8 {
        match region {
            MemoryRegion::Mem1(index) => self.mem1[index],
            MemoryRegion::Mem2(index) => self.mem2[index],
        }
    }

    fn write_region8(&mut self, region: MemoryRegion, value: u8) {
        match region {
            MemoryRegion::Mem1(index) => self.mem1[index] = value,
            MemoryRegion::Mem2(index) => self.mem2[index] = value,
        }
    }
}
impl PowerPcBus for WiiBoard {
    fn read8(&mut self, address: u32) -> u8 {
        if let Some(region) = memory_region(address) {
            return self.read_region8(region);
        }
        if let Some(offset) = address
            .checked_sub(FLIPPER_BASE)
            .filter(|offset| *offset < HW_SIZE as u32)
        {
            return self.flipper[offset as usize];
        }
        if let Some(offset) = address
            .checked_sub(HOLLYWOOD_BASE)
            .filter(|offset| *offset < HW_SIZE as u32)
        {
            return self.hollywood[offset as usize];
        }
        0
    }

    fn write8(&mut self, address: u32, value: u8) {
        if let Some(region) = memory_region(address) {
            self.write_region8(region, value);
            return;
        }
        if let Some(offset) = address
            .checked_sub(FLIPPER_BASE)
            .filter(|offset| *offset < HW_SIZE as u32)
        {
            self.flipper[offset as usize] = value;
            return;
        }
        if let Some(offset) = address
            .checked_sub(HOLLYWOOD_BASE)
            .filter(|offset| *offset < HW_SIZE as u32)
        {
            self.hollywood[offset as usize] = value;
        }
    }

    fn read32(&mut self, address: u32) -> u32 {
        if let Some(offset) = address
            .checked_sub(FLIPPER_BASE)
            .filter(|offset| (*offset as usize) + 4 <= HW_SIZE)
        {
            return self.read_flipper_u32(offset as usize);
        }
        if let Some(offset) = address
            .checked_sub(HOLLYWOOD_BASE)
            .filter(|offset| (*offset as usize) + 4 <= HW_SIZE)
        {
            return self.read_hollywood_u32(offset as usize);
        }
        let mut bytes = [0; 4];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read8(address.wrapping_add(index as u32));
        }
        u32::from_be_bytes(bytes)
    }

    fn write32(&mut self, address: u32, value: u32) {
        if let Some(offset) = address
            .checked_sub(FLIPPER_BASE)
            .filter(|offset| (*offset as usize) + 4 <= HW_SIZE)
        {
            let offset = offset as usize;
            if offset != PI_CAUSE {
                self.write_flipper_u32(offset, value);
            }
            return;
        }
        if let Some(offset) = address
            .checked_sub(HOLLYWOOD_BASE)
            .filter(|offset| (*offset as usize) + 4 <= HW_SIZE)
        {
            self.write_hollywood_register(offset as usize, value);
            return;
        }
        for (index, byte) in value.to_be_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(index as u32), byte);
        }
    }
}

pub struct WiiMachine {
    cpu: PowerPc750,
    board: WiiBoard,
    powered: bool,
}
impl WiiMachine {
    pub fn from_dol(image: ResourceBlob) -> Result<Self, String> {
        let (board, entry) = WiiBoard::new(image)?;
        let mut cpu = PowerPc750::new();
        Self::boot_cpu(&mut cpu, entry);
        Ok(Self {
            cpu,
            board,
            powered: true,
        })
    }

    fn boot_cpu(cpu: &mut PowerPc750, entry: u32) {
        cpu.reset_to(entry);
        cpu.msr |= 0x0000_2000;
        cpu.hid2 = 0xe000_0000;
        cpu.gpr[1] = 0x817f_ffc0;
    }

    fn run_cpu_frame(&mut self) {
        let target = self.cpu.cycles.saturating_add(CPU_STEPS_PER_FRAME);
        while self.powered && self.cpu.cycles < target {
            self.cpu
                .set_external_interrupt(self.board.interrupt_pending());
            self.cpu.step(&mut self.board);
        }
    }
}

impl Machine for WiiMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Wii
    }
    fn reset(&mut self) {
        match self.board.reset() {
            Ok(entry) => {
                Self::boot_cpu(&mut self.cpu, entry);
                self.powered = true;
            }
            Err(_) => self.powered = false,
        }
    }

    fn run_frame(&mut self, input: &InputState) {
        if !self.powered {
            return;
        }
        let _ = input;
        self.run_cpu_frame();
        self.board.end_frame();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE as f64
    }

    fn video(&self) -> &VideoBuffer {
        &self.board.video
    }

    fn audio(&self) -> &AudioBuffer {
        &self.board.audio
    }
    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Wii, STATE_VERSION);
        self.cpu.save(&mut out);
        self.board.save(&mut out);
        out.u8(u8::from(self.powered));
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Wii, STATE_VERSION)?;
        self.cpu.load(&mut input)?;
        self.board.load(&mut input)?;
        self.powered = input.u8()? != 0;
        input.finish()?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn addi(rd: u32, ra: u32, immediate: u16) -> u32 {
        (14 << 26) | (rd << 21) | (ra << 16) | u32::from(immediate)
    }

    fn stw(rs: u32, ra: u32, displacement: u16) -> u32 {
        (36 << 26) | (rs << 21) | (ra << 16) | u32::from(displacement)
    }
    fn addis(rd: u32, ra: u32, immediate: u16) -> u32 {
        (15 << 26) | (rd << 21) | (ra << 16) | u32::from(immediate)
    }

    fn dol_bytes(program: &[u32], mem2_data: &[u8]) -> Vec<u8> {
        let text_offset = 0x100usize;
        let data_offset = text_offset + program.len() * 4;
        let mut image = vec![0; data_offset + mem2_data.len()];
        image[0x00..0x04].copy_from_slice(&(text_offset as u32).to_be_bytes());
        image[0x1c..0x20].copy_from_slice(&(data_offset as u32).to_be_bytes());
        image[0x48..0x4c].copy_from_slice(&0x8000_3100u32.to_be_bytes());
        image[0x64..0x68].copy_from_slice(&0x9000_1000u32.to_be_bytes());
        image[0x90..0x94].copy_from_slice(&((program.len() * 4) as u32).to_be_bytes());
        image[0xac..0xb0].copy_from_slice(&(mem2_data.len() as u32).to_be_bytes());
        image[0xd8..0xdc].copy_from_slice(&0x9000_2000u32.to_be_bytes());
        image[0xdc..0xe0].copy_from_slice(&0x100u32.to_be_bytes());
        image[0xe0..0xe4].copy_from_slice(&0x8000_3100u32.to_be_bytes());
        for (index, instruction) in program.iter().enumerate() {
            let start = text_offset + index * 4;
            image[start..start + 4].copy_from_slice(&instruction.to_be_bytes());
        }
        image[data_offset..].copy_from_slice(mem2_data);
        image
    }
    fn idle_image() -> ResourceBlob {
        ResourceBlob::from_bytes(&dol_bytes(&[0x4800_0000], &[]))
    }

    #[test]
    fn dol_loads_mem2_and_broadway_executes_into_mem2() {
        let image = ResourceBlob::from_bytes(&dol_bytes(
            &[addis(4, 0, 0x9000), addi(3, 0, 0x1234), stw(3, 4, 0x3000)],
            &[0xaa, 0xbb, 0xcc, 0xdd],
        ));
        let (mut board, entry) = WiiBoard::new(image).unwrap();
        assert_eq!(&board.mem2[0x1000..0x1004], &[0xaa, 0xbb, 0xcc, 0xdd]);
        assert!(board.mem2[0x2000..0x2100].iter().all(|byte| *byte == 0));
        let mut cpu = PowerPc750::new();
        cpu.reset_to(entry);
        for _ in 0..3 {
            cpu.step(&mut board);
        }
        assert_eq!(
            u32::from_be_bytes(board.mem2[0x3000..0x3004].try_into().unwrap()),
            0x1234
        );
    }
    #[test]
    fn ipc_boundary_acknowledges_and_interrupts_broadway() {
        let mut board = WiiBoard::new(idle_image()).unwrap().0;
        board.write32(FLIPPER_BASE + PI_MASK as u32, PI_INT_WII_IPC);
        board.write32(HOLLYWOOD_BASE + IPC_PPCMSG as u32, 0x8000_4000);
        board.write32(HOLLYWOOD_BASE + IPC_PPCCTRL as u32, 1);
        assert_eq!(
            board.read32(HOLLYWOOD_BASE + IPC_ARMMSG as u32),
            0x8000_4000
        );
        assert_eq!(board.read32(HOLLYWOOD_BASE + IPC_PPCCTRL as u32) & 7, 6);
        assert_ne!(
            board.read32(HOLLYWOOD_BASE + IPC_PPC_IRQFLAG as u32) & IPC_IRQ_BROADWAY,
            0
        );
        assert!(board.interrupt_pending());
        board.write32(HOLLYWOOD_BASE + IPC_PPC_IRQFLAG as u32, IPC_IRQ_BROADWAY);
        assert!(!board.interrupt_pending());
        assert_eq!(board.read32(HOLLYWOOD_BASE + IPC_AHBPROT as u32), u32::MAX);
    }

    #[test]
    fn xfb_scanout_reaches_public_video() {
        let mut board = WiiBoard::new(idle_image()).unwrap().0;
        board.mem1[0x4000..0x4004].copy_from_slice(&[235, 128, 235, 128]);
        board.write16(FLIPPER_BASE + VI_FB_LEFT_TOP_HI as u32, 0);
        board.write16(FLIPPER_BASE + VI_FB_LEFT_TOP_LO as u32, 0x4000);
        board.present_video();
        let pixel = &board.video.pixels()[..4];
        assert!(pixel[0] > 250 && pixel[1] > 250 && pixel[2] > 250);
        assert_eq!(pixel[3], 255);
    }
    #[test]
    fn state_round_trip_preserves_cpu_memories_and_ipc() {
        let image = idle_image();
        let mut machine = WiiMachine::from_dol(image.clone()).unwrap();
        machine.cpu.gpr[7] = 0xfeed_beef;
        machine.board.mem1[0x5000..0x5004].copy_from_slice(&0x1122_3344u32.to_be_bytes());
        machine.board.mem2[0x6000..0x6004].copy_from_slice(&0x5566_7788u32.to_be_bytes());
        machine
            .board
            .write32(HOLLYWOOD_BASE + IPC_PPCMSG as u32, 0x8000_7000);
        let saved = machine.save_state().unwrap();

        let mut restored = WiiMachine::from_dol(image).unwrap();
        restored.load_state(&saved).unwrap();
        assert_eq!(restored.cpu.gpr[7], 0xfeed_beef);
        assert_eq!(
            &restored.board.mem1[0x5000..0x5004],
            &0x1122_3344u32.to_be_bytes()
        );
        assert_eq!(
            &restored.board.mem2[0x6000..0x6004],
            &0x5566_7788u32.to_be_bytes()
        );
        assert_eq!(
            restored.board.read32(HOLLYWOOD_BASE + IPC_PPCMSG as u32),
            0x8000_7000
        );
        assert_eq!(restored.platform(), PlatformId::Wii);
        assert_eq!(restored.frame_rate(), 60.0);
    }
}
