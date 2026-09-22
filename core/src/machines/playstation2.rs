use crate::cpu_mips_r5900::{MipsR5900, R5900Bus};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::{ResourceBlob, ResourceKind};
use crate::state::{StateReader, StateWriter};

const STATE_VERSION: u32 = 1;
const EE_RAM_SIZE: usize = 32 * 1024 * 1024;
const EE_SCRATCHPAD_SIZE: usize = 16 * 1024;
const BIOS_SIZE: usize = 4 * 1024 * 1024;
const GS_VRAM_SIZE: usize = 4 * 1024 * 1024;
const HW_SIZE: usize = 0x1_0000;
const MEMORY_CARD_SIZE: usize = 8 * 1024 * 1024;
const SECTOR_SIZE: u64 = 2048;
const FRAME_RATE: u64 = 60;
const CPU_STEPS_PER_FRAME: u64 = 300_000;
const AUDIO_RATE: u32 = 48_000;
const VIDEO_WIDTH: u32 = 640;
const VIDEO_HEIGHT: u32 = 448;

const EE_HW_BASE: u32 = 0x1000_0000;
const VU_BASE: u32 = 0x1100_0000;
const GS_BASE: u32 = 0x1200_0000;
const BIOS_BASE: u32 = 0x1fc0_0000;
const SCRATCHPAD_BASE: u32 = 0x7000_0000;
const INTC_STAT: usize = 0xf000;
const INTC_MASK: usize = 0xf010;
const INTC_VBLANK_S: u32 = 1 << 2;
const INTC_VBLANK_E: u32 = 1 << 3;
const INTC_TIMER0: u32 = 1 << 9;

const GS_PMODE: u32 = 0x0000;
const GS_DISPFB1: u32 = 0x0070;
const GS_BGCOLOR: u32 = 0x00e0;
const GS_CSR: u32 = 0x1000;
const GS_IMR: u32 = 0x1010;

const GIF_FIFO_START: usize = 0x6000;
const GIF_FIFO_END: usize = 0x7000;
const D2_CHCR: usize = 0xa000;
const D2_MADR: usize = 0xa010;
const D2_QWC: usize = 0xa020;
const D2_TADR: usize = 0xa030;
const D2_ASR0: usize = 0xa040;
const D2_ASR1: usize = 0xa050;
const DMAC_CTRL: usize = 0xe000;
const DMAC_STAT: usize = 0xe010;
const DMAC_GIF: u32 = 1 << 2;
const DMA_CHCR_MOD_MASK: u32 = 3 << 2;
const DMA_CHCR_ASP_MASK: u32 = 3 << 4;
const DMA_CHCR_TTE: u32 = 1 << 6;
const DMA_CHCR_TIE: u32 = 1 << 7;
const DMA_CHCR_STR: u32 = 1 << 8;
const GS_REG_BITBLTBUF: u8 = 0x50;
const GS_REG_TRXPOS: u8 = 0x51;
const GS_REG_TRXREG: u8 = 0x52;
const GS_REG_TRXDIR: u8 = 0x53;

fn le16(bytes: &[u8], offset: usize) -> Result<u16, String> {
    let data = bytes
        .get(offset..offset + 2)
        .ok_or_else(|| "PS2 image is truncated".to_string())?;
    Ok(u16::from_le_bytes(data.try_into().unwrap()))
}

fn le32(bytes: &[u8], offset: usize) -> Result<u32, String> {
    let data = bytes
        .get(offset..offset + 4)
        .ok_or_else(|| "PS2 image is truncated".to_string())?;
    Ok(u32::from_le_bytes(data.try_into().unwrap()))
}
fn read_blob(image: &ResourceBlob, offset: u64, length: usize) -> Result<Vec<u8>, String> {
    let end = offset
        .checked_add(length as u64)
        .filter(|end| *end <= image.len())
        .ok_or_else(|| "PS2 media read exceeds staged image".to_string())?;
    let mut bytes = vec![0; length];
    image.read(offset, &mut bytes)?;
    debug_assert_eq!(end - offset, bytes.len() as u64);
    Ok(bytes)
}

#[derive(Clone, Debug)]
struct IsoEntry {
    name: String,
    extent: u32,
    size: u32,
    directory: bool,
}

fn iso_name(bytes: &[u8]) -> String {
    String::from_utf8_lossy(bytes)
        .trim_end_matches(";1")
        .trim_end_matches('.')
        .to_ascii_uppercase()
}

fn parse_directory(bytes: &[u8]) -> Result<Vec<IsoEntry>, String> {
    let mut entries = Vec::new();
    let mut cursor = 0usize;
    while cursor < bytes.len() {
        let length = bytes[cursor] as usize;
        if length == 0 {
            cursor = ((cursor / SECTOR_SIZE as usize) + 1) * SECTOR_SIZE as usize;
            continue;
        }
        let record = bytes
            .get(cursor..cursor + length)
            .ok_or_else(|| "PS2 ISO directory record is truncated".to_string())?;
        if record.len() < 34 {
            return Err("PS2 ISO directory record is malformed".into());
        }
        let name_len = record[32] as usize;
        let name = record
            .get(33..33 + name_len)
            .ok_or_else(|| "PS2 ISO directory name is truncated".to_string())?;
        if !(name_len == 1 && matches!(name[0], 0 | 1)) {
            entries.push(IsoEntry {
                name: iso_name(name),
                extent: le32(record, 2)?,
                size: le32(record, 10)?,
                directory: record[25] & 0x02 != 0,
            });
        }
        cursor += length;
    }
    Ok(entries)
}

fn iso_root(image: &ResourceBlob) -> Result<IsoEntry, String> {
    let pvd = read_blob(image, 16 * SECTOR_SIZE, SECTOR_SIZE as usize)?;
    if pvd.first() != Some(&1) || pvd.get(1..6) != Some(b"CD001") {
        return Err("PS2 disc is not an ISO9660 image with a primary volume descriptor".into());
    }
    let record_len = pvd[156] as usize;
    let record = pvd
        .get(156..156 + record_len)
        .ok_or_else(|| "PS2 ISO root directory record is truncated".to_string())?;
    Ok(IsoEntry {
        name: String::new(),
        extent: le32(record, 2)?,
        size: le32(record, 10)?,
        directory: true,
    })
}

fn iso_find(image: &ResourceBlob, path: &str) -> Result<IsoEntry, String> {
    let clean = path
        .trim()
        .trim_start_matches("cdrom0:")
        .trim_start_matches("CDROM0:")
        .trim_start_matches(['\\', '/'])
        .replace('\\', "/");
    let mut current = iso_root(image)?;
    for (index, segment) in clean.split('/').filter(|part| !part.is_empty()).enumerate() {
        if !current.directory {
            return Err(format!(
                "PS2 ISO path enters non-directory {}",
                current.name
            ));
        }
        let bytes = read_blob(
            image,
            u64::from(current.extent) * SECTOR_SIZE,
            current.size as usize,
        )?;
        let wanted = iso_name(segment.as_bytes());
        let entries = parse_directory(&bytes)?;
        current = entries
            .into_iter()
            .find(|entry| entry.name == wanted)
            .ok_or_else(|| format!("PS2 ISO path component {segment} was not found"))?;
        if index > 16 {
            return Err("PS2 ISO path is unreasonably deep".into());
        }
    }
    Ok(current)
}

fn boot_path(image: &ResourceBlob) -> Result<IsoEntry, String> {
    let system = iso_find(image, "SYSTEM.CNF")?;
    let bytes = read_blob(
        image,
        u64::from(system.extent) * SECTOR_SIZE,
        system.size as usize,
    )?;
    let text = String::from_utf8_lossy(&bytes);
    let path = text
        .lines()
        .find_map(|line| {
            let (key, value) = line.split_once('=')?;
            key.trim()
                .eq_ignore_ascii_case("BOOT2")
                .then(|| value.trim())
        })
        .ok_or_else(|| "PS2 SYSTEM.CNF has no BOOT2 entry".to_string())?;
    iso_find(image, path)
}

fn is_elf(image: &ResourceBlob) -> Result<bool, String> {
    Ok(read_blob(image, 0, 4)?.as_slice() == b"\x7fELF")
}
fn ee_ram_range(address: u32, length: usize) -> Result<std::ops::Range<usize>, String> {
    let physical = address & 0x1fff_ffff;
    if physical >= EE_RAM_SIZE as u32 {
        return Err(format!(
            "PS2 ELF segment address {address:#010x} is outside EE RAM"
        ));
    }
    let start = physical as usize;
    let end = start
        .checked_add(length)
        .filter(|end| *end <= EE_RAM_SIZE)
        .ok_or_else(|| "PS2 ELF segment exceeds EE RAM".to_string())?;
    Ok(start..end)
}

fn load_elf(
    image: &ResourceBlob,
    file_offset: u64,
    file_size: u64,
    ram: &mut [u8],
) -> Result<u32, String> {
    let header = read_blob(image, file_offset, 52)?;
    if header.get(0..4) != Some(b"\x7fELF") || header[4] != 1 || header[5] != 1 {
        return Err("PS2 executable is not a 32-bit little-endian ELF".into());
    }
    if le16(&header, 18)? != 8 {
        return Err("PS2 ELF does not declare the MIPS machine type".into());
    }
    let entry = le32(&header, 24)?;
    let phoff = u64::from(le32(&header, 28)?);
    let phentsize = u64::from(le16(&header, 42)?);
    let phnum = u64::from(le16(&header, 44)?);
    if phentsize < 32 || phnum > 256 {
        return Err("PS2 ELF program-header table is malformed".into());
    }
    for index in 0..phnum {
        let offset = phoff
            .checked_add(index * phentsize)
            .filter(|offset| offset.checked_add(32).is_some_and(|end| end <= file_size))
            .ok_or_else(|| "PS2 ELF program-header table exceeds the executable".to_string())?;
        let program = read_blob(image, file_offset + offset, 32)?;
        if le32(&program, 0)? != 1 {
            continue;
        }
        let segment_offset = u64::from(le32(&program, 4)?);
        let virtual_address = le32(&program, 8)?;
        let physical_address = le32(&program, 12)?;
        let file_bytes = le32(&program, 16)? as usize;
        let memory_bytes = le32(&program, 20)? as usize;
        if file_bytes > memory_bytes
            || segment_offset
                .checked_add(file_bytes as u64)
                .is_none_or(|end| end > file_size)
        {
            return Err("PS2 ELF load segment is malformed".into());
        }
        let address = if physical_address != 0 {
            physical_address
        } else {
            virtual_address
        };
        let memory = ee_ram_range(address, memory_bytes)?;
        ram[memory.clone()].fill(0);
        if file_bytes != 0 {
            let source = read_blob(image, file_offset + segment_offset, file_bytes)?;
            ram[memory.start..memory.start + file_bytes].copy_from_slice(&source);
        }
    }
    Ok(entry)
}

fn load_executable(image: &ResourceBlob, ram: &mut [u8]) -> Result<u32, String> {
    if is_elf(image)? {
        return load_elf(image, 0, image.len(), ram);
    }
    let boot = boot_path(image)?;
    load_elf(
        image,
        u64::from(boot.extent) * SECTOR_SIZE,
        u64::from(boot.size),
        ram,
    )
}
#[derive(Clone, Copy, Default)]
struct GifState {
    buffer: [u8; 16],
    fill: usize,
    remaining: u32,
    format: u8,
    regs: [u8; 16],
    nreg: u8,
    reg_index: u8,
    bitbltbuf: u64,
    trxpos: u64,
    trxreg: u64,
    trxdir: u64,
    image_pixel: u32,
}

struct Ps2Board {
    ram: Box<[u8]>,
    scratchpad: Box<[u8]>,
    bios: Box<[u8]>,
    vu: Box<[u8]>,
    hw: Box<[u8]>,
    gs_regs: Box<[u8]>,
    gs_vram: Box<[u8]>,
    disc: ResourceBlob,
    memory_card: Box<[u8]>,
    gif: GifState,
    video: VideoBuffer,
    audio: AudioBuffer,
    frame_counter: u64,
}
impl Ps2Board {
    fn new(image: ResourceBlob, bios: Option<&[u8]>) -> Result<(Self, u32), String> {
        let mut ram = vec![0; EE_RAM_SIZE].into_boxed_slice();
        let entry = load_executable(&image, &mut ram)?;
        let mut bios_image = vec![0; BIOS_SIZE].into_boxed_slice();
        if let Some(source) = bios {
            if source.len() != BIOS_SIZE {
                return Err("PlayStation 2 BIOS must be exactly 4 MiB".into());
            }
            bios_image.copy_from_slice(source);
        }
        let mut board = Self {
            ram,
            scratchpad: vec![0; EE_SCRATCHPAD_SIZE].into_boxed_slice(),
            bios: bios_image,
            vu: vec![0; 0x1_0000].into_boxed_slice(),
            hw: vec![0; HW_SIZE].into_boxed_slice(),
            gs_regs: vec![0; 0x2000].into_boxed_slice(),
            gs_vram: vec![0; GS_VRAM_SIZE].into_boxed_slice(),
            disc: image,
            memory_card: vec![0xff; MEMORY_CARD_SIZE].into_boxed_slice(),
            gif: GifState::default(),
            video: VideoBuffer::new(VIDEO_WIDTH, VIDEO_HEIGHT),
            audio: AudioBuffer::new(AUDIO_RATE, 2),
            frame_counter: 0,
        };
        board.reset_registers();
        Ok((board, entry))
    }

    fn reset_registers(&mut self) {
        self.hw.fill(0);
        self.gs_regs.fill(0);
        self.gif = GifState::default();
        self.write_gs_u64(GS_CSR, 0x551b_4000);
        self.write_gs_u64(GS_IMR, 0x0000_7f00);
    }
    fn read_hw_u32(&self, offset: usize) -> u32 {
        self.hw
            .get(offset..offset + 4)
            .map(|bytes| u32::from_le_bytes(bytes.try_into().unwrap()))
            .unwrap_or(0)
    }

    fn write_hw_u32_raw(&mut self, offset: usize, value: u32) {
        if let Some(bytes) = self.hw.get_mut(offset..offset + 4) {
            bytes.copy_from_slice(&value.to_le_bytes());
        }
    }

    fn read_gs_u64(&self, offset: u32) -> u64 {
        let start = offset as usize;
        self.gs_regs
            .get(start..start + 8)
            .map(|bytes| u64::from_le_bytes(bytes.try_into().unwrap()))
            .unwrap_or(0)
    }

    fn write_gs_u64(&mut self, offset: u32, value: u64) {
        let start = offset as usize;
        if let Some(bytes) = self.gs_regs.get_mut(start..start + 8) {
            bytes.copy_from_slice(&value.to_le_bytes());
        }
    }

    fn interrupt_pending(&self) -> bool {
        let intc = self.read_hw_u32(INTC_STAT) & self.read_hw_u32(INTC_MASK) != 0;
        let dmac = self.read_hw_u32(DMAC_STAT);
        let dmac_pending = (dmac & 0xffff) & (dmac >> 16) != 0;
        intc || dmac_pending
    }

    fn raise_intc(&mut self, mask: u32) {
        let value = self.read_hw_u32(INTC_STAT) | mask;
        self.write_hw_u32_raw(INTC_STAT, value);
    }

    fn read_dma_qword(&self, address: u32) -> Option<u128> {
        if address & 0x8000_0000 != 0 {
            let start = (address as usize) & (EE_SCRATCHPAD_SIZE - 1) & !0xf;
            let bytes = self.scratchpad.get(start..start + 16)?;
            return Some(u128::from_le_bytes(bytes.try_into().ok()?));
        }
        let start = (address as usize) & !0xf;
        let bytes = self.ram.get(start..start + 16)?;
        Some(u128::from_le_bytes(bytes.try_into().ok()?))
    }

    fn transfer_gif_qwords(&mut self, address: u32, qwc: u32) -> bool {
        for index in 0..qwc {
            let Some(qword) = self.read_dma_qword(address.wrapping_add(index.saturating_mul(16)))
            else {
                return false;
            };
            self.process_gif_qword(qword);
        }
        true
    }

    fn finish_gif_dma(&mut self, mut chcr: u32) {
        chcr &= !DMA_CHCR_STR;
        self.write_hw_u32_raw(D2_CHCR, chcr);
        self.write_hw_u32_raw(D2_QWC, 0);
        let stat = self.read_hw_u32(DMAC_STAT) | DMAC_GIF;
        self.write_hw_u32_raw(DMAC_STAT, stat);
    }

    fn run_gif_dma_normal(&mut self, chcr: u32) {
        let madr = self.read_hw_u32(D2_MADR) & !0xf;
        let qwc = self.read_hw_u32(D2_QWC) & 0xffff;
        if !self.transfer_gif_qwords(madr, qwc) {
            return;
        }
        self.write_hw_u32_raw(D2_MADR, madr.wrapping_add(qwc.saturating_mul(16)));
        self.finish_gif_dma(chcr);
    }

    fn gif_dma_push_return(&mut self, chcr: &mut u32, address: u32) -> bool {
        let asp = (*chcr & DMA_CHCR_ASP_MASK) >> 4;
        match asp {
            0 => self.write_hw_u32_raw(D2_ASR0, address),
            1 => self.write_hw_u32_raw(D2_ASR1, address),
            _ => return false,
        }
        *chcr = (*chcr & !DMA_CHCR_ASP_MASK) | ((asp + 1) << 4);
        true
    }

    fn gif_dma_pop_return(&mut self, chcr: &mut u32) -> Option<u32> {
        let asp = (*chcr & DMA_CHCR_ASP_MASK) >> 4;
        let next = match asp {
            1 => self.read_hw_u32(D2_ASR0),
            2 => self.read_hw_u32(D2_ASR1),
            _ => return None,
        };
        *chcr = (*chcr & !DMA_CHCR_ASP_MASK) | ((asp - 1) << 4);
        Some(next)
    }

    fn run_gif_dma_chain(&mut self, mut chcr: u32) {
        let mut tadr = self.read_hw_u32(D2_TADR) & !0xf;
        for _ in 0..4096 {
            let Some(tag) = self.read_dma_qword(tadr) else {
                return;
            };
            let words = tag.to_le_bytes();
            let tag0 = u32::from_le_bytes(words[0..4].try_into().unwrap());
            let tag1 = u32::from_le_bytes(words[4..8].try_into().unwrap());
            let qwc = tag0 & 0xffff;
            let id = (tag0 >> 28) & 7;
            let irq = tag0 & (1 << 31) != 0;
            let address = tag1 & 0x7fff_ffff;
            let spr = tag1 & 0x8000_0000;
            let inline = tadr.wrapping_add(16);
            let (madr, next_tadr, end) = match id {
                0 => (address | spr, tadr.wrapping_add(16), true),
                1 => (inline, inline.wrapping_add(qwc.saturating_mul(16)), false),
                2 => (inline, address | spr, false),
                3 | 4 => (address | spr, tadr.wrapping_add(16), false),
                5 => {
                    let return_address = inline.wrapping_add(qwc.saturating_mul(16));
                    if !self.gif_dma_push_return(&mut chcr, return_address) {
                        return;
                    }
                    (inline, address | spr, false)
                }
                6 => match self.gif_dma_pop_return(&mut chcr) {
                    Some(return_address) => (inline, return_address, false),
                    None => (inline, inline.wrapping_add(qwc.saturating_mul(16)), true),
                },
                7 => (inline, inline.wrapping_add(qwc.saturating_mul(16)), true),
                _ => unreachable!(),
            };
            if chcr & DMA_CHCR_TTE != 0 {
                let tag_data = tag >> 64;
                self.process_gif_qword(tag_data);
            }
            self.write_hw_u32_raw(D2_MADR, madr);
            self.write_hw_u32_raw(D2_QWC, qwc);
            if !self.transfer_gif_qwords(madr, qwc) {
                return;
            }
            self.write_hw_u32_raw(D2_MADR, madr.wrapping_add(qwc.saturating_mul(16)));
            self.write_hw_u32_raw(D2_QWC, 0);
            chcr = (chcr & 0x0000_ffff) | (tag0 & 0xffff_0000);
            self.write_hw_u32_raw(D2_CHCR, chcr);
            if irq && chcr & DMA_CHCR_TIE != 0 {
                self.write_hw_u32_raw(D2_TADR, next_tadr);
                self.finish_gif_dma(chcr);
                return;
            }
            tadr = next_tadr;
            self.write_hw_u32_raw(D2_TADR, tadr);
            if end {
                self.finish_gif_dma(chcr);
                return;
            }
        }
    }

    fn service_gif_dma(&mut self) {
        let chcr = self.read_hw_u32(D2_CHCR);
        if chcr & DMA_CHCR_STR == 0 || self.read_hw_u32(DMAC_CTRL) & 1 == 0 {
            return;
        }
        match (chcr & DMA_CHCR_MOD_MASK) >> 2 {
            0 => self.run_gif_dma_normal(chcr),
            1 => self.run_gif_dma_chain(chcr),
            _ => {}
        }
    }

    fn write_dmac_stat(&mut self, value: u32) {
        let current = self.read_hw_u32(DMAC_STAT);
        let status = (current & 0xffff) & !(value & 0xffff);
        let masks = ((current >> 16) ^ (value >> 16)) & 0xffff;
        self.write_hw_u32_raw(DMAC_STAT, status | (masks << 16));
    }

    fn write_gs_internal(&mut self, register: u8, value: u64) {
        match register {
            GS_REG_BITBLTBUF => self.gif.bitbltbuf = value,
            GS_REG_TRXPOS => self.gif.trxpos = value,
            GS_REG_TRXREG => self.gif.trxreg = value,
            GS_REG_TRXDIR => {
                self.gif.trxdir = value;
                self.gif.image_pixel = 0;
            }
            _ => {}
        }
    }

    fn process_gif_qword(&mut self, qword: u128) {
        if self.gif.remaining == 0 {
            let loops = (qword & 0x7fff) as u32;
            self.gif.format = ((qword >> 58) & 3) as u8;
            self.gif.nreg = ((qword >> 60) & 0xf) as u8;
            if self.gif.nreg == 0 {
                self.gif.nreg = 16;
            }
            for index in 0..16 {
                self.gif.regs[index] = ((qword >> (64 + index * 4)) & 0xf) as u8;
            }
            self.gif.reg_index = 0;
            self.gif.remaining = match self.gif.format {
                0 => loops.saturating_mul(u32::from(self.gif.nreg)),
                1 => loops.saturating_mul(u32::from(self.gif.nreg).div_ceil(2)),
                2 | 3 => loops,
                _ => 0,
            };
            return;
        }

        match self.gif.format {
            0 => self.process_gif_packed(qword),
            2 | 3 => self.upload_image_qword(qword),
            _ => {}
        }
        self.gif.remaining = self.gif.remaining.saturating_sub(1);
    }
    fn process_gif_packed(&mut self, qword: u128) {
        let register = self.gif.regs[self.gif.reg_index as usize];
        self.gif.reg_index = (self.gif.reg_index + 1) % self.gif.nreg;
        if register == 0x0e {
            let address = ((qword >> 64) & 0xff) as u8;
            self.write_gs_internal(address, qword as u64);
        }
    }

    fn upload_image_qword(&mut self, qword: u128) {
        if self.gif.trxdir & 3 != 0 {
            return;
        }
        let dbp = ((self.gif.bitbltbuf >> 32) & 0x3fff) as usize * 256;
        let dbw = (((self.gif.bitbltbuf >> 48) & 0x3f) as usize).max(1) * 64;
        let psm = ((self.gif.bitbltbuf >> 56) & 0x3f) as u8;
        if psm != 0 {
            return;
        }
        let dsax = ((self.gif.trxpos >> 32) & 0x7ff) as usize;
        let dsay = ((self.gif.trxpos >> 48) & 0x7ff) as usize;
        let width = ((self.gif.trxreg & 0xfff) as usize).max(1);
        let height = (((self.gif.trxreg >> 32) & 0xfff) as usize).max(1);
        let bytes = qword.to_le_bytes();
        for pixel in bytes.as_chunks::<4>().0 {
            let cursor = self.gif.image_pixel as usize;
            if cursor >= width.saturating_mul(height) {
                break;
            }
            let x = cursor % width;
            let y = cursor / width;
            let offset = dbp
                .saturating_add((dsay + y).saturating_mul(dbw).saturating_mul(4))
                .saturating_add((dsax + x).saturating_mul(4));
            if let Some(target) = self.gs_vram.get_mut(offset..offset + 4) {
                target.copy_from_slice(pixel);
            }
            self.gif.image_pixel = self.gif.image_pixel.wrapping_add(1);
        }
    }
    fn write_gif_byte(&mut self, byte: u8) {
        if self.gif.fill >= self.gif.buffer.len() {
            self.gif.fill = 0;
        }
        self.gif.buffer[self.gif.fill] = byte;
        self.gif.fill += 1;
        if self.gif.fill == self.gif.buffer.len() {
            let qword = u128::from_le_bytes(self.gif.buffer);
            self.gif.fill = 0;
            self.process_gif_qword(qword);
        }
    }

    fn write_gs_privileged(&mut self, offset: u32, value: u64) {
        match offset {
            GS_CSR => {
                let mut csr = self.read_gs_u64(GS_CSR);
                csr &= !(value & 0x1f);
                if value & (1 << 9) != 0 {
                    self.gs_vram.fill(0);
                    self.gif = GifState::default();
                    csr = 0x551b_4000;
                }
                self.write_gs_u64(GS_CSR, csr);
            }
            GS_IMR => self.write_gs_u64(GS_IMR, value & 0x7f00),
            _ => self.write_gs_u64(offset, value),
        }
    }

    fn present_video(&mut self) {
        let pmode = self.read_gs_u64(GS_PMODE);
        if pmode & 1 == 0 {
            let color = self.read_gs_u64(GS_BGCOLOR);
            self.video
                .clear([color as u8, (color >> 8) as u8, (color >> 16) as u8, 255]);
            return;
        }
        let dispfb = self.read_gs_u64(GS_DISPFB1);
        let base = (dispfb & 0x1ff) as usize * 2048;
        let width = (((dispfb >> 9) & 0x3f) as usize).max(1) * 64;
        let psm = ((dispfb >> 15) & 0x1f) as u8;
        let x0 = ((dispfb >> 32) & 0x7ff) as usize;
        let y0 = ((dispfb >> 43) & 0x7ff) as usize;
        if psm != 0 {
            self.video.clear([0, 0, 0, 255]);
            return;
        }
        for y in 0..VIDEO_HEIGHT as usize {
            for x in 0..VIDEO_WIDTH as usize {
                let source = base.saturating_add(((y0 + y) * width + x0 + x) * 4);
                let target = (y * VIDEO_WIDTH as usize + x) * 4;
                let Some(pixel) = self.gs_vram.get(source..source + 4) else {
                    continue;
                };
                self.video.pixels_mut()[target..target + 3].copy_from_slice(&pixel[..3]);
                self.video.pixels_mut()[target + 3] = 255;
            }
        }
    }

    fn tick_timer0(&mut self) {
        let mode = self.read_hw_u32(0x0010);
        if mode & (1 << 7) == 0 {
            return;
        }
        let old = self.read_hw_u32(0x0000) & 0xffff;
        let target = self.read_hw_u32(0x0020) & 0xffff;
        let mut next = old.wrapping_add(1) & 0xffff;
        let mut next_mode = mode;
        if target != 0 && next >= target {
            next_mode |= 1 << 10;
            if mode & (1 << 8) != 0 {
                self.raise_intc(INTC_TIMER0);
            }
            if mode & (1 << 6) != 0 {
                next = 0;
            }
        }
        if next == 0 && old == 0xffff {
            next_mode |= 1 << 11;
            if mode & (1 << 9) != 0 {
                self.raise_intc(INTC_TIMER0);
            }
        }
        self.write_hw_u32_raw(0x0000, next);
        self.write_hw_u32_raw(0x0010, next_mode);
    }

    fn begin_frame(&mut self, input: &InputState) {
        let _ = input;
        self.raise_intc(INTC_VBLANK_S);
    }

    fn end_frame(&mut self) {
        self.tick_timer0();
        self.present_video();
        self.audio.begin_frame();
        for _ in 0..(AUDIO_RATE / FRAME_RATE as u32) {
            self.audio.push_stereo(0.0, 0.0);
        }
        let csr = self.read_gs_u64(GS_CSR) | (1 << 3);
        self.write_gs_u64(GS_CSR, csr);
        self.raise_intc(INTC_VBLANK_E);
        self.frame_counter = self.frame_counter.wrapping_add(1);
    }

    fn reset(&mut self) -> Result<u32, String> {
        self.ram.fill(0);
        self.scratchpad.fill(0);
        self.vu.fill(0);
        self.gs_vram.fill(0);
        let entry = load_executable(&self.disc, &mut self.ram)?;
        self.reset_registers();
        self.frame_counter = 0;
        self.video.clear([0, 0, 0, 255]);
        self.audio.begin_frame();
        Ok(entry)
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        out.blob(&self.scratchpad);
        out.blob(&self.bios);
        out.blob(&self.vu);
        out.blob(&self.hw);
        out.blob(&self.gs_regs);
        out.blob(&self.gs_vram);
        out.blob(&self.memory_card);
        out.blob(&self.gif.buffer);
        out.u32(self.gif.fill as u32);
        out.u32(self.gif.remaining);
        out.u8(self.gif.format);
        out.blob(&self.gif.regs);
        out.u8(self.gif.nreg);
        out.u8(self.gif.reg_index);
        out.u64(self.gif.bitbltbuf);
        out.u64(self.gif.trxpos);
        out.u64(self.gif.trxreg);
        out.u64(self.gif.trxdir);
        out.u32(self.gif.image_pixel);
        out.u64(self.frame_counter);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        Self::load_blob(input, &mut self.ram, "EE RAM")?;
        Self::load_blob(input, &mut self.scratchpad, "scratchpad")?;
        Self::load_blob(input, &mut self.bios, "BIOS")?;
        Self::load_blob(input, &mut self.vu, "VU memory")?;
        Self::load_blob(input, &mut self.hw, "hardware registers")?;
        Self::load_blob(input, &mut self.gs_regs, "GS registers")?;
        Self::load_blob(input, &mut self.gs_vram, "GS VRAM")?;
        Self::load_blob(input, &mut self.memory_card, "memory card")?;
        let gif_buffer = input.blob()?;
        if gif_buffer.len() != self.gif.buffer.len() {
            return Err("PS2 GIF state buffer has the wrong size".into());
        }
        self.gif.buffer.copy_from_slice(gif_buffer);
        self.gif.fill = input.u32()? as usize;
        self.gif.remaining = input.u32()?;
        self.gif.format = input.u8()?;
        let gif_regs = input.blob()?;
        if gif_regs.len() != self.gif.regs.len() {
            return Err("PS2 GIF register-list state has the wrong size".into());
        }
        self.gif.regs.copy_from_slice(gif_regs);
        self.gif.nreg = input.u8()?;
        self.gif.reg_index = input.u8()?;
        self.gif.bitbltbuf = input.u64()?;
        self.gif.trxpos = input.u64()?;
        self.gif.trxreg = input.u64()?;
        self.gif.trxdir = input.u64()?;
        self.gif.image_pixel = input.u32()?;
        self.frame_counter = input.u64()?;
        if self.gif.fill > self.gif.buffer.len() || self.gif.nreg > 16 {
            return Err("PS2 GIF state is invalid".into());
        }
        self.present_video();
        self.audio.begin_frame();
        Ok(())
    }

    fn load_blob(
        input: &mut StateReader<'_>,
        target: &mut [u8],
        label: &str,
    ) -> Result<(), String> {
        let blob = input.blob()?;
        if blob.len() != target.len() {
            return Err(format!(
                "PS2 {label} state has {} bytes; expected {}",
                blob.len(),
                target.len()
            ));
        }
        target.copy_from_slice(blob);
        Ok(())
    }
}
impl R5900Bus for Ps2Board {
    fn read8(&mut self, address: u32) -> u8 {
        match address {
            0x0000_0000..=0x01ff_ffff => self.ram[address as usize],
            SCRATCHPAD_BASE..=0x7000_3fff => self.scratchpad[(address - SCRATCHPAD_BASE) as usize],
            EE_HW_BASE..=0x1000_ffff => self.hw[(address - EE_HW_BASE) as usize],
            VU_BASE..=0x1100_ffff => self.vu[(address - VU_BASE) as usize],
            GS_BASE..=0x1200_1fff => self.gs_regs[(address - GS_BASE) as usize],
            BIOS_BASE..=0x1fff_ffff => self.bios[(address - BIOS_BASE) as usize],
            _ => 0,
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        match address {
            0x0000_0000..=0x01ff_ffff => self.ram[address as usize] = value,
            SCRATCHPAD_BASE..=0x7000_3fff => {
                self.scratchpad[(address - SCRATCHPAD_BASE) as usize] = value;
            }
            EE_HW_BASE..=0x1000_ffff => {
                let offset = (address - EE_HW_BASE) as usize;
                if (GIF_FIFO_START..GIF_FIFO_END).contains(&offset) {
                    self.write_gif_byte(value);
                } else {
                    self.hw[offset] = value;
                }
            }
            VU_BASE..=0x1100_ffff => self.vu[(address - VU_BASE) as usize] = value,
            GS_BASE..=0x1200_1fff => self.gs_regs[(address - GS_BASE) as usize] = value,
            _ => {}
        }
    }

    fn write32(&mut self, address: u32, value: u32) {
        match address {
            0x1000_f000 => {
                let current = self.read_hw_u32(INTC_STAT);
                self.write_hw_u32_raw(INTC_STAT, current & !value);
            }
            0x1000_a000 => {
                self.write_hw_u32_raw(D2_CHCR, value);
                self.service_gif_dma();
            }
            0x1000_a010 => self.write_hw_u32_raw(D2_MADR, value & !0xf),
            0x1000_a020 => self.write_hw_u32_raw(D2_QWC, value & 0xffff),
            0x1000_a030 => self.write_hw_u32_raw(D2_TADR, value & !0xf),
            0x1000_a040 => self.write_hw_u32_raw(D2_ASR0, value & !0xf),
            0x1000_a050 => self.write_hw_u32_raw(D2_ASR1, value & !0xf),
            0x1000_e000 => {
                self.write_hw_u32_raw(DMAC_CTRL, value);
                self.service_gif_dma();
            }
            0x1000_e010 => self.write_dmac_stat(value),
            0x1000_f010 => {
                let current = self.read_hw_u32(INTC_MASK);
                self.write_hw_u32_raw(INTC_MASK, current ^ value);
            }
            _ => {
                for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
                    self.write8(address.wrapping_add(offset as u32), byte);
                }
            }
        }
    }

    fn write64(&mut self, address: u32, value: u64) {
        if let Some(offset) = address
            .checked_sub(GS_BASE)
            .filter(|offset| *offset <= 0x1ff8)
        {
            self.write_gs_privileged(offset, value);
            return;
        }
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }

    fn write128(&mut self, address: u32, value: u128) {
        if (0x1000_6000..0x1000_7000).contains(&address) {
            self.process_gif_qword(value);
            return;
        }
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }
}
pub struct PlayStation2Machine {
    ee: MipsR5900,
    board: Ps2Board,
    powered: bool,
}

impl PlayStation2Machine {
    pub fn from_images(image: ResourceBlob, bios: Option<&[u8]>) -> Result<Self, String> {
        let (board, entry) = Ps2Board::new(image, bios)?;
        let mut ee = MipsR5900::default();
        ee.reset_to(u64::from(entry));
        ee.set_hle_direct_map(true);
        ee.regs[29] = 0x01ff_fff0;
        Ok(Self {
            ee,
            board,
            powered: true,
        })
    }

    fn boot_ee(&mut self, entry: u32) {
        self.ee.reset_to(u64::from(entry));
        self.ee.set_hle_direct_map(true);
        self.ee.regs[29] = 0x01ff_fff0;
        self.powered = true;
    }

    fn run_ee_frame(&mut self) {
        let target = self.ee.cycles.saturating_add(CPU_STEPS_PER_FRAME);
        while self.powered && self.ee.cycles < target {
            self.ee.set_irq_line(2, self.board.interrupt_pending());
            self.ee.step(&mut self.board);
        }
    }
}
impl Machine for PlayStation2Machine {
    fn platform(&self) -> PlatformId {
        PlatformId::PlayStation2
    }

    fn reset(&mut self) {
        match self.board.reset() {
            Ok(entry) => self.boot_ee(entry),
            Err(_) => self.powered = false,
        }
    }

    fn run_frame(&mut self, input: &InputState) {
        if !self.powered {
            return;
        }
        self.board.begin_frame(input);
        self.run_ee_frame();
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
        let mut out = StateWriter::new(PlatformId::PlayStation2, STATE_VERSION);
        self.ee.save(&mut out);
        self.board.save(&mut out);
        out.u8(u8::from(self.powered));
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::PlayStation2, STATE_VERSION)?;
        self.ee.load_state(&mut input)?;
        self.board.load(&mut input)?;
        self.powered = input.u8()? != 0;
        input.finish()?;
        Ok(())
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::MemoryCard && slot == 0 {
            MEMORY_CARD_SIZE
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        if kind != ResourceKind::MemoryCard || slot != 0 {
            return Err("PS2 persistent resource is MemoryCard slot 0".into());
        }
        if out.len() != MEMORY_CARD_SIZE {
            return Err("PS2 memory-card output has the wrong size".into());
        }
        out.copy_from_slice(&self.board.memory_card);
        Ok(())
    }
    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        if kind != ResourceKind::MemoryCard || slot != 0 {
            return Err("PS2 persistent resource is MemoryCard slot 0".into());
        }
        if data.len() != MEMORY_CARD_SIZE {
            return Err("PS2 memory-card image has the wrong size".into());
        }
        self.board.memory_card.copy_from_slice(data);
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    fn elf_bytes(program: &[u32]) -> Vec<u8> {
        let file_size = 0x100 + program.len() * 4;
        let mut image = vec![0; file_size];
        image[0..4].copy_from_slice(b"\x7fELF");
        image[4] = 1;
        image[5] = 1;
        image[6] = 1;
        image[16..18].copy_from_slice(&2u16.to_le_bytes());
        image[18..20].copy_from_slice(&8u16.to_le_bytes());
        image[20..24].copy_from_slice(&1u32.to_le_bytes());
        image[24..28].copy_from_slice(&0x0010_0000u32.to_le_bytes());
        image[28..32].copy_from_slice(&52u32.to_le_bytes());
        image[40..42].copy_from_slice(&52u16.to_le_bytes());
        image[42..44].copy_from_slice(&32u16.to_le_bytes());
        image[44..46].copy_from_slice(&1u16.to_le_bytes());
        let ph = 52usize;
        image[ph..ph + 4].copy_from_slice(&1u32.to_le_bytes());
        image[ph + 4..ph + 8].copy_from_slice(&0x100u32.to_le_bytes());
        image[ph + 8..ph + 12].copy_from_slice(&0x0010_0000u32.to_le_bytes());
        image[ph + 12..ph + 16].copy_from_slice(&0x0010_0000u32.to_le_bytes());
        image[ph + 16..ph + 20].copy_from_slice(&((program.len() * 4) as u32).to_le_bytes());
        image[ph + 20..ph + 24].copy_from_slice(&0x100u32.to_le_bytes());
        image[ph + 24..ph + 28].copy_from_slice(&5u32.to_le_bytes());
        image[ph + 28..ph + 32].copy_from_slice(&0x1000u32.to_le_bytes());
        for (index, instruction) in program.iter().enumerate() {
            let start = 0x100 + index * 4;
            image[start..start + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        image
    }

    fn iso_record(name: &[u8], extent: u32, size: u32, directory: bool) -> Vec<u8> {
        let padding = usize::from(name.len().is_multiple_of(2));
        let length = 33 + name.len() + padding;
        let mut record = vec![0; length];
        record[0] = length as u8;
        record[2..6].copy_from_slice(&extent.to_le_bytes());
        record[6..10].copy_from_slice(&extent.to_be_bytes());
        record[10..14].copy_from_slice(&size.to_le_bytes());
        record[14..18].copy_from_slice(&size.to_be_bytes());
        record[25] = if directory { 2 } else { 0 };
        record[28..30].copy_from_slice(&1u16.to_le_bytes());
        record[30..32].copy_from_slice(&1u16.to_be_bytes());
        record[32] = name.len() as u8;
        record[33..33 + name.len()].copy_from_slice(name);
        record
    }

    fn iso_bytes(program: &[u32]) -> Vec<u8> {
        const PVD: usize = 16;
        const ROOT: u32 = 20;
        const SYSTEM: u32 = 21;
        const GAME: u32 = 22;
        let elf = elf_bytes(program);
        let system = b"BOOT2 = cdrom0:\\GAME.ELF;1\r\nVER = 1.00\r\n";
        let mut image = vec![0; 24 * SECTOR_SIZE as usize];
        let pvd = PVD * SECTOR_SIZE as usize;
        image[pvd] = 1;
        image[pvd + 1..pvd + 6].copy_from_slice(b"CD001");
        image[pvd + 6] = 1;
        let root = iso_record(&[0], ROOT, SECTOR_SIZE as u32, true);
        image[pvd + 156..pvd + 156 + root.len()].copy_from_slice(&root);
        let root_start = ROOT as usize * SECTOR_SIZE as usize;
        let records = [
            iso_record(&[0], ROOT, SECTOR_SIZE as u32, true),
            iso_record(&[1], ROOT, SECTOR_SIZE as u32, true),
            iso_record(b"SYSTEM.CNF;1", SYSTEM, system.len() as u32, false),
            iso_record(b"GAME.ELF;1", GAME, elf.len() as u32, false),
        ];
        let mut cursor = root_start;
        for record in records {
            image[cursor..cursor + record.len()].copy_from_slice(&record);
            cursor += record.len();
        }
        let system_start = SYSTEM as usize * SECTOR_SIZE as usize;
        image[system_start..system_start + system.len()].copy_from_slice(system);
        let game_start = GAME as usize * SECTOR_SIZE as usize;
        image[game_start..game_start + elf.len()].copy_from_slice(&elf);
        image
    }

    fn addiu(rt: u32, rs: u32, immediate: u16) -> u32 {
        (0x09 << 26) | (rs << 21) | (rt << 16) | u32::from(immediate)
    }

    fn sw(rt: u32, rs: u32, immediate: u16) -> u32 {
        (0x2b << 26) | (rs << 21) | (rt << 16) | u32::from(immediate)
    }

    fn idle_image() -> ResourceBlob {
        ResourceBlob::from_bytes(&elf_bytes(&[0]))
    }

    #[test]
    fn raw_elf_boots_ee_and_writes_ram() {
        let image = ResourceBlob::from_bytes(&elf_bytes(&[addiu(3, 0, 0x1234), sw(3, 0, 0x0200)]));
        let (mut board, entry) = Ps2Board::new(image, None).unwrap();
        let mut ee = MipsR5900::default();
        ee.reset_to(u64::from(entry));
        ee.set_hle_direct_map(true);
        ee.step(&mut board);
        ee.step(&mut board);
        assert_eq!(
            u32::from_le_bytes(board.ram[0x200..0x204].try_into().unwrap()),
            0x1234
        );
    }

    #[test]
    fn iso_system_cnf_resolves_boot2_elf() {
        let image = ResourceBlob::from_bytes(&iso_bytes(&[addiu(4, 0, 0x55aa)]));
        let boot = boot_path(&image).unwrap();
        assert_eq!(boot.name, "GAME.ELF");
        let mut ram = vec![0; EE_RAM_SIZE];
        let entry = load_executable(&image, &mut ram).unwrap();
        assert_eq!(entry, 0x0010_0000);
        assert_eq!(
            u32::from_le_bytes(ram[0x0010_0000..0x0010_0004].try_into().unwrap()),
            addiu(4, 0, 0x55aa)
        );
    }

    #[test]
    fn intc_write_semantics_and_timer_irq_are_live() {
        let mut board = Ps2Board::new(idle_image(), None).unwrap().0;
        board.write32(0x1000_f010, INTC_VBLANK_S | INTC_TIMER0);
        board.raise_intc(INTC_VBLANK_S);
        assert!(board.interrupt_pending());
        board.write32(0x1000_f000, INTC_VBLANK_S);
        assert!(!board.interrupt_pending());

        board.write_hw_u32_raw(0x0010, (1 << 7) | (1 << 8) | (1 << 6));
        board.write_hw_u32_raw(0x0020, 1);
        board.tick_timer0();
        assert_eq!(board.read_hw_u32(0x0000), 0);
        assert_ne!(board.read_hw_u32(0x0010) & (1 << 10), 0);
        assert_ne!(board.read_hw_u32(INTC_STAT) & INTC_TIMER0, 0);
    }

    fn gif_image_packet(pixels: [u8; 16]) -> [u128; 7] {
        [
            4u128 | (1u128 << 60) | (0x0eu128 << 64),
            u128::from(1u64 << 48) | (u128::from(GS_REG_BITBLTBUF) << 64),
            u128::from(GS_REG_TRXPOS) << 64,
            u128::from(2u64 | (2u64 << 32)) | (u128::from(GS_REG_TRXREG) << 64),
            u128::from(GS_REG_TRXDIR) << 64,
            1u128 | (2u128 << 58),
            u128::from_le_bytes(pixels),
        ]
    }

    fn write_dma_qwords(board: &mut Ps2Board, address: usize, qwords: &[u128]) {
        for (index, qword) in qwords.iter().enumerate() {
            let start = address + index * 16;
            board.ram[start..start + 16].copy_from_slice(&qword.to_le_bytes());
        }
    }

    #[test]
    fn gif_dmac_normal_mode_moves_ram_packet_and_raises_masked_irq() {
        let mut board = Ps2Board::new(idle_image(), None).unwrap().0;
        let pixels = [
            10, 20, 30, 255, 40, 50, 60, 255, 70, 80, 90, 255, 100, 110, 120, 255,
        ];
        let packet = gif_image_packet(pixels);
        write_dma_qwords(&mut board, 0x2000, &packet);
        board.write32(0x1000_e000, 1);
        board.write32(0x1000_e010, DMAC_GIF << 16);
        board.write32(0x1000_a010, 0x2000);
        board.write32(0x1000_a020, packet.len() as u32);
        board.write32(0x1000_a000, DMA_CHCR_STR);

        assert_eq!(board.read_hw_u32(D2_QWC), 0);
        assert_eq!(board.read_hw_u32(D2_MADR), 0x2070);
        assert_eq!(board.read_hw_u32(D2_CHCR) & DMA_CHCR_STR, 0);
        assert_ne!(board.read_hw_u32(DMAC_STAT) & DMAC_GIF, 0);
        assert!(board.interrupt_pending());
        board.write32(0x1000_e010, DMAC_GIF);
        assert!(!board.interrupt_pending());
        board.write_gs_privileged(GS_PMODE, 1);
        board.write_gs_privileged(GS_DISPFB1, 1 << 9);
        board.present_video();
        assert_eq!(&board.video.pixels()[..4], &[10, 20, 30, 255]);
    }

    #[test]
    fn gif_dmac_end_tag_transfers_inline_packet_and_finishes_chain() {
        let mut board = Ps2Board::new(idle_image(), None).unwrap().0;
        let pixels = [1, 2, 3, 255, 4, 5, 6, 255, 7, 8, 9, 255, 10, 11, 12, 255];
        let packet = gif_image_packet(pixels);
        let tag = u128::from(packet.len() as u32 | (7 << 28));
        board.ram[0x3000..0x3010].copy_from_slice(&tag.to_le_bytes());
        write_dma_qwords(&mut board, 0x3010, &packet);
        board.write32(0x1000_e000, 1);
        board.write32(0x1000_a030, 0x3000);
        board.write32(0x1000_a000, DMA_CHCR_STR | (1 << 2));

        assert_eq!(board.read_hw_u32(D2_TADR), 0x3080);
        assert_eq!(board.read_hw_u32(D2_MADR), 0x3080);
        assert_eq!(board.read_hw_u32(D2_CHCR) & DMA_CHCR_STR, 0);
        assert_ne!(board.read_hw_u32(DMAC_STAT) & DMAC_GIF, 0);
        board.write_gs_privileged(GS_PMODE, 1);
        board.write_gs_privileged(GS_DISPFB1, 1 << 9);
        board.present_video();
        assert_eq!(&board.video.pixels()[..4], &[1, 2, 3, 255]);
    }

    #[test]
    fn gif_dmac_call_and_ret_use_two_entry_source_chain_stack() {
        let mut board = Ps2Board::new(idle_image(), None).unwrap().0;
        let call = u128::from(5u32 << 28) | (u128::from(0x3040u32) << 32);
        let end = u128::from(7u32 << 28);
        let ret = u128::from(6u32 << 28);
        board.ram[0x3000..0x3010].copy_from_slice(&call.to_le_bytes());
        board.ram[0x3010..0x3020].copy_from_slice(&end.to_le_bytes());
        board.ram[0x3040..0x3050].copy_from_slice(&ret.to_le_bytes());
        board.write32(0x1000_e000, 1);
        board.write32(0x1000_a030, 0x3000);
        board.write32(0x1000_a000, DMA_CHCR_STR | (1 << 2));

        assert_eq!(board.read_hw_u32(D2_ASR0), 0x3010);
        assert_eq!(board.read_hw_u32(D2_CHCR) & DMA_CHCR_ASP_MASK, 0);
        assert_eq!(board.read_hw_u32(D2_TADR), 0x3020);
        assert_eq!(board.read_hw_u32(D2_CHCR) & DMA_CHCR_STR, 0);
    }

    #[test]
    fn gif_ad_image_upload_reaches_display_framebuffer() {
        let mut board = Ps2Board::new(idle_image(), None).unwrap().0;
        let packed_tag = 4u128 | (1u128 << 60) | (0x0eu128 << 64);
        board.process_gif_qword(packed_tag);
        let registers = [
            (GS_REG_BITBLTBUF, 1u64 << 48),
            (GS_REG_TRXPOS, 0),
            (GS_REG_TRXREG, 2 | (2u64 << 32)),
            (GS_REG_TRXDIR, 0),
        ];
        for (register, value) in registers {
            board.process_gif_qword(u128::from(value) | (u128::from(register) << 64));
        }
        board.process_gif_qword(1u128 | (2u128 << 58));
        let pixels = [
            10, 20, 30, 255, 40, 50, 60, 255, 70, 80, 90, 255, 100, 110, 120, 255,
        ];
        board.process_gif_qword(u128::from_le_bytes(pixels));
        board.write_gs_privileged(GS_PMODE, 1);
        board.write_gs_privileged(GS_DISPFB1, 1 << 9);
        board.present_video();
        assert_eq!(&board.video.pixels()[..4], &[10, 20, 30, 255]);
        assert_eq!(&board.gs_vram[..8], &pixels[..8]);
        assert_eq!(&board.gs_vram[256..264], &pixels[8..]);
    }

    #[test]
    fn machine_state_and_memory_card_round_trip_are_deterministic() {
        let image = idle_image();
        let mut machine = PlayStation2Machine::from_images(image.clone(), None).unwrap();
        machine.ee.regs[7] = (0x1122_3344_5566_7788u128 << 64) | 0x99aa_bbcc_ddee_ff00u128;
        machine.board.ram[0x6000..0x6004].copy_from_slice(&0x89ab_cdefu32.to_le_bytes());
        let card = vec![0x5a; MEMORY_CARD_SIZE];
        machine
            .write_persistent(ResourceKind::MemoryCard, 0, &card)
            .unwrap();
        let saved = machine.save_state().unwrap();

        let mut restored = PlayStation2Machine::from_images(image, None).unwrap();
        restored.load_state(&saved).unwrap();
        assert_eq!(restored.ee.regs[7], machine.ee.regs[7]);
        assert_eq!(
            u32::from_le_bytes(restored.board.ram[0x6000..0x6004].try_into().unwrap()),
            0x89ab_cdef
        );
        let mut out = vec![0; MEMORY_CARD_SIZE];
        restored
            .read_persistent(ResourceKind::MemoryCard, 0, &mut out)
            .unwrap();
        assert_eq!(out, card);
        assert_eq!(restored.platform(), PlatformId::PlayStation2);
        assert_eq!(restored.frame_rate(), 60.0);
    }
}
