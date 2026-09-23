use crate::cpu_powerpc750::{PowerPc750, PowerPcBus};
use crate::input::{
    AXIS_LEFT_TRIGGER, AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_TRIGGER, AXIS_RIGHT_X, AXIS_RIGHT_Y,
    DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, L1, LEFT, R1, RIGHT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::{ResourceBlob, ResourceKind};
use crate::state::{StateReader, StateWriter};

const STATE_VERSION: u32 = 6;
const MEM1_SIZE: usize = 24 * 1024 * 1024;
const ARAM_SIZE: usize = 16 * 1024 * 1024;
const EFB_SIZE: usize = 2 * 1024 * 1024;
const MEMORY_CARD_SIZE: usize = 512 * 1024;
const MMIO_SIZE: usize = 0x1_0000;
const FRAME_RATE: u64 = 60;
const CPU_STEPS_PER_FRAME: u64 = 250_000;
const AUDIO_RATE: u32 = 48_000;
const AUDIO_SAMPLES_PER_FRAME: u32 = AUDIO_RATE / FRAME_RATE as u32;
const DTK_BLOCK_BYTES: u64 = 32;
const DTK_SAMPLES_PER_BLOCK: u32 = 28;
const VIDEO_WIDTH: u32 = 640;
const VIDEO_HEIGHT: u32 = 480;

const DISC_MAGIC: u32 = 0xc233_9f3d;
const MMIO_BASE: u32 = 0xcc00_0000;
const EFB_BASE: u32 = 0xc800_0000;
const PI_CAUSE: usize = 0x3000;
const PI_MASK: usize = 0x3004;
const PI_INT_DI: u32 = 0x0004;
const PI_INT_SI: u32 = 0x0008;
const PI_INT_EXI: u32 = 0x0010;
const PI_INT_DSP: u32 = 0x0040;
const PI_INT_VI: u32 = 0x0100;

const DI_STATUS: usize = 0x6000;
const DI_COVER: usize = 0x6004;
const DI_COMMAND_0: usize = 0x6008;
const DI_COMMAND_1: usize = 0x600c;
const DI_COMMAND_2: usize = 0x6010;
const DI_DMA_ADDRESS: usize = 0x6014;
const DI_DMA_LENGTH: usize = 0x6018;
const DI_DMA_CONTROL: usize = 0x601c;
const DI_IMMEDIATE: usize = 0x6020;
const DI_CONFIG: usize = 0x6024;
const DI_ERROR_NONE: u32 = 0x00000;
const DI_ERROR_MOTOR_STOPPED: u32 = 0x20400;
const DI_ERROR_NO_DISC_ID: u32 = 0x20401;
const DI_ERROR_MEDIUM_NOT_PRESENT: u32 = 0x23a00;
const DI_ERROR_INVALID_COMMAND: u32 = 0x52000;
const DI_ERROR_NO_AUDIO_BUFFER: u32 = 0x52001;
const DI_ERROR_BLOCK_OOB: u32 = 0x52100;
const DI_ERROR_INVALID_AUDIO_COMMAND: u32 = 0x52401;
const DI_ERROR_INVALID_PERIOD: u32 = 0x52402;
const DI_ERROR_MEDIUM_CHANGED: u32 = 0x62800;

const DSP_CONTROL: usize = 0x500a;
const AR_DMA_MMADDR_H: usize = 0x5020;
const AR_DMA_MMADDR_L: usize = 0x5022;
const AR_DMA_ARADDR_H: usize = 0x5024;
const AR_DMA_ARADDR_L: usize = 0x5026;
const AR_DMA_CNT_H: usize = 0x5028;
const AR_DMA_CNT_L: usize = 0x502a;
const AUDIO_DMA_START_HI: usize = 0x5030;
const AUDIO_DMA_START_LO: usize = 0x5032;
const AUDIO_DMA_CONTROL_LEN: usize = 0x5036;
const AUDIO_DMA_BLOCKS_LEFT: usize = 0x503a;

const SI_CHANNEL_STRIDE: usize = 0x0c;
const SI_CHANNEL_0_OUT: usize = 0x6400;
const SI_COM_CSR: usize = 0x6434;
const SI_STATUS: usize = 0x6438;
const SI_IO_BUFFER: usize = 0x6480;

const EXI_STATUS: usize = 0x6800;
const EXI_DMA_ADDRESS: usize = 0x6804;
const EXI_DMA_LENGTH: usize = 0x6808;
const EXI_DMA_CONTROL: usize = 0x680c;
const EXI_IMM_DATA: usize = 0x6810;
const EXI_CHANNEL_STRIDE: usize = 0x14;
const EXI_STATUS_EXIINTMASK: u32 = 1 << 0;
const EXI_STATUS_EXIINT: u32 = 1 << 1;
const EXI_STATUS_TCINTMASK: u32 = 1 << 2;
const EXI_STATUS_TCINT: u32 = 1 << 3;
const EXI_STATUS_CHIP_SELECT_MASK: u32 = 0x7 << 7;
const EXI_STATUS_EXTINTMASK: u32 = 1 << 10;
const EXI_STATUS_EXTINT: u32 = 1 << 11;
const EXI_STATUS_EXT: u32 = 1 << 12;
const EXI_STATUS_ROMDIS: u32 = 1 << 13;
const EXI_CARD_BLOCK_SIZE: usize = 8 * 1024;
const EXI_CARD_STATUS_BUSY: u8 = 0x80;
const EXI_CARD_STATUS_UNLOCKED: u8 = 0x40;
const EXI_CARD_STATUS_ERASE_ERROR: u8 = 0x10;
const EXI_CARD_STATUS_PROGRAM_ERROR: u8 = 0x08;
const EXI_CARD_STATUS_READY: u8 = 0x01;
const EXI_IPL_ROM_SIZE: usize = 2 * 1024 * 1024;
const EXI_IPL_ENCRYPTED_START: usize = 0x100;
const EXI_IPL_ENCRYPTED_END: usize = 0x1aff00;
const EXI_IPL_SRAM_BASE: u32 = 0x0080_0000;
const EXI_IPL_SRAM_SIZE: usize = 0x44;

const VI_CONTROL: usize = 0x2002;
const VI_FB_LEFT_TOP_HI: usize = 0x201c;
const VI_FB_LEFT_TOP_LO: usize = 0x201e;
const VI_PRERETRACE_HI: usize = 0x2030;
const VI_POSTRETRACE_HI: usize = 0x2034;

const PAD_LEFT: u16 = 0x0001;
const PAD_RIGHT: u16 = 0x0002;
const PAD_DOWN: u16 = 0x0004;
const PAD_UP: u16 = 0x0008;
const PAD_Z: u16 = 0x0010;
const PAD_R: u16 = 0x0020;
const PAD_L: u16 = 0x0040;
const PAD_USE_ORIGIN: u16 = 0x0080;
const PAD_A: u16 = 0x0100;
const PAD_B: u16 = 0x0200;
const PAD_X: u16 = 0x0400;
const PAD_Y: u16 = 0x0800;
const PAD_START: u16 = 0x1000;
fn be32(bytes: &[u8], offset: usize) -> Result<u32, String> {
    let end = offset
        .checked_add(4)
        .ok_or_else(|| "GameCube executable offset overflow".to_string())?;
    let value = bytes
        .get(offset..end)
        .ok_or_else(|| "GameCube executable header is truncated".to_string())?;
    Ok(u32::from_be_bytes(value.try_into().unwrap()))
}

fn memory_index(address: u32) -> Option<usize> {
    let physical = match address {
        0x0000_0000..=0x017f_ffff => address,
        0x8000_0000..=0x817f_ffff | 0xc000_0000..=0xc17f_ffff => address & 0x01ff_ffff,
        _ => return None,
    };
    ((physical as usize) < MEM1_SIZE).then_some(physical as usize)
}

fn memory_range(address: u32, len: usize) -> Result<std::ops::Range<usize>, String> {
    let start = memory_index(address)
        .ok_or_else(|| format!("GameCube DOL section address {address:#010x} is outside MEM1"))?;
    let end = start
        .checked_add(len)
        .filter(|end| *end <= MEM1_SIZE)
        .ok_or_else(|| "GameCube DOL section exceeds MEM1".to_string())?;
    Ok(start..end)
}
fn read_resource_u32(image: &ResourceBlob, offset: u64) -> Result<u32, String> {
    let mut bytes = [0; 4];
    image.read(offset, &mut bytes)?;
    Ok(u32::from_be_bytes(bytes))
}

fn is_disc_image(image: &ResourceBlob) -> Result<bool, String> {
    Ok(image.len() >= 0x424 && read_resource_u32(image, 0x1c)? == DISC_MAGIC)
}

fn locate_dol(image: &ResourceBlob) -> Result<u64, String> {
    if is_disc_image(image)? {
        return Ok(u64::from(read_resource_u32(image, 0x420)?));
    }
    Ok(0)
}

fn load_dol(image: &ResourceBlob, mem1: &mut [u8]) -> Result<u32, String> {
    let dol_offset = locate_dol(image)?;
    let mut header = [0; 0x100];
    image.read(dol_offset, &mut header)?;
    let bss_address = be32(&header, 0xd8)?;
    let bss_size = be32(&header, 0xdc)? as usize;
    let entry = be32(&header, 0xe0)?;
    if entry == 0 {
        return Err("GameCube DOL entry point is zero".into());
    }
    if bss_size != 0 {
        let range = memory_range(bss_address, bss_size)?;
        mem1[range].fill(0);
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
        let source = dol_offset
            .checked_add(file_offset)
            .ok_or_else(|| "GameCube DOL section file offset overflow".to_string())?;
        let source_end = source
            .checked_add(size as u64)
            .filter(|end| *end <= image.len())
            .ok_or_else(|| "GameCube DOL section exceeds the staged image".to_string())?;
        let range = memory_range(address, size)?;
        debug_assert_eq!(source_end - source, range.len() as u64);
        image.read(source, &mut mem1[range])?;
    }
    Ok(entry)
}

#[derive(Clone, Copy, Default)]
struct AudioDma {
    source: u32,
    cursor: u32,
    blocks: u16,
    remaining: u16,
    enabled: bool,
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum DiDriveState {
    Ready = 0,
    ReadyNoReadsMade = 1,
    CoverOpened = 2,
    DiscChangeDetected = 3,
    NoMediumPresent = 4,
    MotorStopped = 5,
    #[default]
    DiscIdNotRead = 6,
}

impl DiDriveState {
    fn error_state(self) -> u32 {
        match self {
            Self::Ready | Self::ReadyNoReadsMade => 0,
            _ => u32::from(self as u8) - 1,
        }
    }

    fn from_u8(value: u8) -> Result<Self, String> {
        match value {
            0 => Ok(Self::Ready),
            1 => Ok(Self::ReadyNoReadsMade),
            2 => Ok(Self::CoverOpened),
            3 => Ok(Self::DiscChangeDetected),
            4 => Ok(Self::NoMediumPresent),
            5 => Ok(Self::MotorStopped),
            6 => Ok(Self::DiscIdNotRead),
            _ => Err(format!("GameCube DI state has invalid drive state {value}")),
        }
    }
}

#[derive(Clone, Copy, Default)]
struct DiState {
    drive_state: DiDriveState,
    error: u32,
    enable_dtk: bool,
    dtk_buffer_length: u8,
    dtk_sample_remainder: u8,
    stream: bool,
    stop_at_track_end: bool,
    audio_position: u64,
    current_start: u64,
    current_length: u32,
    next_start: u64,
    next_length: u32,
}

fn descramble_ipl_code(bytes: &mut [u8]) {
    let mut t = 0x2953u16;
    let mut u = 0xd9c2u16;
    let mut v = 0x3ff1u16;
    let mut x = 1u8;

    for byte in bytes {
        let mut key = 0u8;
        for _ in 0..8 {
            let t0 = (t & 1) as u8;
            let t1 = ((t >> 1) & 1) as u8;
            let u0 = (u & 1) as u8;
            let u1 = ((u >> 1) & 1) as u8;
            let v0 = (v & 1) as u8;

            x ^= t1 ^ v0;
            x ^= u0 | u1;
            x ^= (t0 ^ u1 ^ v0) & (t0 ^ u0);

            if t0 == u0 {
                v >>= 1;
                if v0 != 0 {
                    v ^= 0xb3d0;
                }
            }
            if t0 == 0 {
                u >>= 1;
                if u0 != 0 {
                    u ^= 0xfb10;
                }
            }
            t >>= 1;
            if t0 != 0 {
                t ^= 0xa740;
            }

            key = (key << 1) | x;
        }
        *byte ^= key;
    }
}

fn prepare_ipl_rom(bytes: &[u8]) -> Result<Box<[u8]>, String> {
    if bytes.len() != EXI_IPL_ROM_SIZE {
        return Err(format!(
            "GameCube IPL ROM must be exactly {EXI_IPL_ROM_SIZE} bytes"
        ));
    }
    let mut rom = bytes.to_vec();
    descramble_ipl_code(&mut rom[EXI_IPL_ENCRYPTED_START..EXI_IPL_ENCRYPTED_END]);
    Ok(rom.into_boxed_slice())
}

fn default_exi_sram() -> [u8; EXI_IPL_SRAM_SIZE] {
    let mut sram = [0u8; EXI_IPL_SRAM_SIZE];
    sram[4..6].copy_from_slice(&0x002cu16.to_be_bytes());
    sram[6..8].copy_from_slice(&0xffd0u16.to_be_bytes());
    sram[23] = 0x2c;
    sram[24..36].copy_from_slice(b"DOLPHINSLOTA");
    sram[36..48].copy_from_slice(b"DOLPHINSLOTB");
    sram[62] = 0x6e;
    sram[63] = 0x6d;
    sram
}

#[derive(Clone)]
struct ExiIplState {
    command: u32,
    command_bytes: u8,
    cursor: u32,
    rtc_phase: u8,
    sram: [u8; EXI_IPL_SRAM_SIZE],
}

impl Default for ExiIplState {
    fn default() -> Self {
        Self {
            command: 0,
            command_bytes: 0,
            cursor: 0,
            rtc_phase: 0,
            sram: default_exi_sram(),
        }
    }
}

#[derive(Clone)]
struct ExiCardState {
    status: u8,
    interrupt_switch: u8,
    interrupt_set: bool,
    command: u8,
    position: u32,
    address: u32,
    programming_buffer: [u8; 128],
}

impl Default for ExiCardState {
    fn default() -> Self {
        Self {
            status: EXI_CARD_STATUS_BUSY | EXI_CARD_STATUS_UNLOCKED | EXI_CARD_STATUS_READY,
            interrupt_switch: 0,
            interrupt_set: false,
            command: 0,
            position: 0,
            address: 0,
            programming_buffer: [0; 128],
        }
    }
}

#[derive(Clone, Default)]
struct ExiChannelState {
    status: u32,
    dma_address: u32,
    dma_length: u32,
    control: u32,
    imm_data: u32,
    card: ExiCardState,
}

#[derive(Clone, Default)]
struct ExiState {
    channel0: ExiChannelState,
    channel1: ExiChannelState,
    ipl: ExiIplState,
}

struct GameCubeBoard {
    mem1: Box<[u8]>,
    aram: Box<[u8]>,
    efb: Box<[u8]>,
    mmio: Box<[u8]>,
    disc: ResourceBlob,
    ipl_rom: Option<Box<[u8]>>,
    memory_cards: [Box<[u8]>; 2],
    input: InputState,
    audio_dma: AudioDma,
    di: DiState,
    exi: ExiState,
    video: VideoBuffer,
    frame_counter: u64,
}

impl GameCubeBoard {
    #[cfg(test)]
    fn new(image: ResourceBlob) -> Result<(Self, u32), String> {
        Self::new_with_ipl(image, None)
    }

    fn new_with_ipl(
        image: ResourceBlob,
        ipl_rom: Option<Box<[u8]>>,
    ) -> Result<(Self, u32), String> {
        let mut mem1 = vec![0; MEM1_SIZE].into_boxed_slice();
        let entry = load_dol(&image, &mut mem1)?;
        let mut board = Self {
            mem1,
            aram: vec![0; ARAM_SIZE].into_boxed_slice(),
            efb: vec![0; EFB_SIZE].into_boxed_slice(),
            mmio: vec![0; MMIO_SIZE].into_boxed_slice(),
            disc: image,
            ipl_rom,
            memory_cards: [
                vec![0xff; MEMORY_CARD_SIZE].into_boxed_slice(),
                vec![0xff; MEMORY_CARD_SIZE].into_boxed_slice(),
            ],
            input: InputState::default(),
            audio_dma: AudioDma::default(),
            di: DiState::default(),
            exi: ExiState::default(),
            video: VideoBuffer::new(VIDEO_WIDTH, VIDEO_HEIGHT),
            frame_counter: 0,
        };
        board.reset_mmio();
        board.initialize_low_memory();
        board.initialize_hle_boot_state()?;
        Ok((board, entry))
    }

    fn initialize_low_memory(&mut self) {
        self.write_mem_u32(0x0000_0028, MEM1_SIZE as u32);
        self.write_mem_u32(0x0000_00f8, 162_000_000);
        self.write_mem_u32(0x0000_00fc, 486_000_000);
    }

    fn initialize_hle_boot_state(&mut self) -> Result<(), String> {
        if !is_disc_image(&self.disc)? {
            self.di.drive_state = DiDriveState::CoverOpened;
            return Ok(());
        }

        let mut disc_id = [0u8; 0x20];
        self.disc.read(0, &mut disc_id)?;
        self.mem1[..disc_id.len()].copy_from_slice(&disc_id);
        let streaming = disc_id[8] != 0;
        let streaming_size = if streaming {
            let configured = disc_id[9] & 0x0f;
            if configured == 0 {
                10
            } else {
                configured
            }
        } else {
            0
        };
        self.di.drive_state = DiDriveState::Ready;
        self.di.enable_dtk = streaming;
        self.di.dtk_buffer_length = streaming_size;
        Ok(())
    }

    fn reset_mmio(&mut self) {
        self.mmio.fill(0);
        self.mmio_write_u32_raw(DI_COVER, 0);
        self.mmio_write_u32_raw(DI_CONFIG, 1);
        self.mmio_write_u16_raw(VI_CONTROL, 1);
        self.audio_dma = AudioDma::default();
        self.di = DiState::default();
        let ipl = self.exi.ipl.clone();
        self.exi = ExiState::default();
        self.exi.ipl = ipl;
        self.exi.channel0.status = EXI_STATUS_EXTINT | EXI_STATUS_EXT;
        self.exi.channel1.status = EXI_STATUS_EXTINT | EXI_STATUS_EXT | (1 << 7);
    }

    fn write_mem_u32(&mut self, address: u32, value: u32) {
        let Some(index) = memory_index(address) else {
            return;
        };
        if index + 4 <= self.mem1.len() {
            self.mem1[index..index + 4].copy_from_slice(&value.to_be_bytes());
        }
    }

    fn mmio_read_u16_raw(&self, offset: usize) -> u16 {
        let bytes = [self.mmio[offset], self.mmio[offset + 1]];
        u16::from_be_bytes(bytes)
    }

    fn mmio_write_u16_raw(&mut self, offset: usize, value: u16) {
        self.mmio[offset..offset + 2].copy_from_slice(&value.to_be_bytes());
    }
    fn mmio_read_u32_raw(&self, offset: usize) -> u32 {
        let bytes = [
            self.mmio[offset],
            self.mmio[offset + 1],
            self.mmio[offset + 2],
            self.mmio[offset + 3],
        ];
        u32::from_be_bytes(bytes)
    }

    fn mmio_write_u32_raw(&mut self, offset: usize, value: u32) {
        self.mmio[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
    }

    fn exi_card_select(card: &mut ExiCardState, memory_card: &mut [u8], selected: bool) {
        if selected {
            card.position = 0;
            return;
        }

        match card.command {
            0xf1 if card.position > 2 => {
                let base =
                    (card.address as usize & (MEMORY_CARD_SIZE - 1)) & !(EXI_CARD_BLOCK_SIZE - 1);
                memory_card[base..base + EXI_CARD_BLOCK_SIZE].fill(0xff);
                card.status |= EXI_CARD_STATUS_READY;
                card.status &= !EXI_CARD_STATUS_BUSY;
                card.interrupt_set = true;
            }
            0xf2 if card.position >= 5 => {
                let count = (card.position - 5).min(128) as usize;
                let mut address = card.address;
                for index in 0..count {
                    memory_card[address as usize & (MEMORY_CARD_SIZE - 1)] =
                        card.programming_buffer[index];
                    address = (address & !0x1ff) | (address.wrapping_add(1) & 0x1ff);
                }
                card.status |= EXI_CARD_STATUS_READY;
                card.status &= !EXI_CARD_STATUS_BUSY;
                card.interrupt_set = true;
            }
            0xf4 if card.position > 1 => {
                memory_card.fill(0xff);
                card.status |= EXI_CARD_STATUS_READY;
                card.status &= !EXI_CARD_STATUS_BUSY;
                card.interrupt_set = true;
            }
            _ => {}
        }
    }

    fn exi_card_transfer_byte(card: &mut ExiCardState, memory_card: &mut [u8], input: u8) -> u8 {
        let position = card.position;
        let mut output = 0xff;
        if position == 0 {
            card.command = input;
            if input == 0x89 {
                card.status &= !(EXI_CARD_STATUS_PROGRAM_ERROR | EXI_CARD_STATUS_ERASE_ERROR);
                card.status |= EXI_CARD_STATUS_READY;
                card.interrupt_set = false;
                card.position = 0;
            }
        } else {
            match card.command {
                0x00 => {
                    output = if position == 1 {
                        0x80
                    } else {
                        let shift = 24 - (((position - 2) & 3) * 8);
                        (4u32 >> shift) as u8
                    };
                }
                0x52 => {
                    match position {
                        1 => card.address = u32::from(input) << 17,
                        2 => card.address |= u32::from(input) << 9,
                        3 => card.address |= u32::from(input & 3) << 7,
                        4 => card.address |= u32::from(input & 0x7f),
                        _ => {}
                    }
                    if position > 1 {
                        output = memory_card[card.address as usize & (MEMORY_CARD_SIZE - 1)];
                        if position >= 9 {
                            card.address =
                                (card.address & !0x1ff) | (card.address.wrapping_add(1) & 0x1ff);
                        }
                    }
                }
                0x81 => {
                    if position == 1 {
                        card.interrupt_switch = input;
                    }
                }
                0x83 => output = card.status,
                0x85 => {
                    output = if position == 1 || position & 1 == 0 {
                        0xc2
                    } else {
                        0x21
                    };
                }
                0xf1 => match position {
                    1 => card.address = u32::from(input) << 17,
                    2 => card.address |= u32::from(input) << 9,
                    _ => {}
                },
                0xf2 => {
                    match position {
                        1 => card.address = u32::from(input) << 17,
                        2 => card.address |= u32::from(input) << 9,
                        3 => card.address |= u32::from(input & 3) << 7,
                        4 => card.address |= u32::from(input & 0x7f),
                        _ => {}
                    }
                    if position >= 5 {
                        card.programming_buffer[((position - 5) & 0x7f) as usize] = input;
                    }
                }
                _ => {}
            }
        }
        card.position = card.position.wrapping_add(1);
        output
    }

    fn exi_ipl_select(&mut self) {
        self.exi.ipl.command = 0;
        self.exi.ipl.command_bytes = 0;
        self.exi.ipl.cursor = 0;
    }

    fn exi_ipl_transfer_byte(&mut self, input: u8) -> u8 {
        if self.exi.ipl.command_bytes < 4 {
            self.exi.ipl.command = (self.exi.ipl.command << 8) | u32::from(input);
            self.exi.ipl.command_bytes += 1;
            return 0xff;
        }

        let address = (self.exi.ipl.command >> 6) & 0x01ff_ffff;
        let write = self.exi.ipl.command >> 31 != 0;
        if address < EXI_IPL_ROM_SIZE as u32 {
            if write {
                return 0xff;
            }
            let offset = (address.wrapping_add(self.exi.ipl.cursor) as usize) % EXI_IPL_ROM_SIZE;
            self.exi.ipl.cursor = self.exi.ipl.cursor.wrapping_add(1);
            return self.ipl_rom.as_ref().map_or(0xff, |rom| rom[offset]);
        }
        if (EXI_IPL_SRAM_BASE..EXI_IPL_SRAM_BASE + EXI_IPL_SRAM_SIZE as u32).contains(&address) {
            let offset = ((address - EXI_IPL_SRAM_BASE).wrapping_add(self.exi.ipl.cursor) as usize)
                % EXI_IPL_SRAM_SIZE;
            self.exi.ipl.cursor = self.exi.ipl.cursor.wrapping_add(1);
            if write {
                self.exi.ipl.sram[offset] = input;
                0xff
            } else {
                self.exi.ipl.sram[offset]
            }
        } else {
            0xff
        }
    }

    fn exi_channel(&self, channel: usize) -> &ExiChannelState {
        match channel {
            0 => &self.exi.channel0,
            1 => &self.exi.channel1,
            _ => unreachable!("GameCube EXI channel {channel} is not modeled"),
        }
    }

    fn exi_channel_mut(&mut self, channel: usize) -> &mut ExiChannelState {
        match channel {
            0 => &mut self.exi.channel0,
            1 => &mut self.exi.channel1,
            _ => unreachable!("GameCube EXI channel {channel} is not modeled"),
        }
    }

    fn exi_chip_select(&self, channel: usize) -> u8 {
        ((self.exi_channel(channel).status & EXI_STATUS_CHIP_SELECT_MASK) >> 7) as u8
    }

    fn exi_transfer_byte(&mut self, channel: usize, input: u8) -> u8 {
        match (channel, self.exi_chip_select(channel)) {
            (0, 1) => Self::exi_card_transfer_byte(
                &mut self.exi.channel0.card,
                &mut self.memory_cards[0],
                input,
            ),
            (0, 2) => self.exi_ipl_transfer_byte(input),
            (1, 1) => Self::exi_card_transfer_byte(
                &mut self.exi.channel1.card,
                &mut self.memory_cards[1],
                input,
            ),
            _ => 0xff,
        }
    }

    fn tick_exi_rtc(&mut self) {
        self.exi.ipl.rtc_phase += 1;
        if self.exi.ipl.rtc_phase < FRAME_RATE as u8 {
            return;
        }
        self.exi.ipl.rtc_phase = 0;
        let rtc = u32::from_be_bytes(self.exi.ipl.sram[0..4].try_into().unwrap()).wrapping_add(1);
        self.exi.ipl.sram[0..4].copy_from_slice(&rtc.to_be_bytes());
    }

    fn set_pi_cause(&mut self, mask: u32, set: bool) {
        let mut cause = self.mmio_read_u32_raw(PI_CAUSE);
        if set {
            cause |= mask;
        } else {
            cause &= !mask;
        }
        self.mmio_write_u32_raw(PI_CAUSE, cause);
    }

    fn refresh_exi_interrupt(&mut self) {
        for channel in 0..=1 {
            let state = self.exi_channel_mut(channel);
            if state.card.interrupt_switch != 0 && state.card.interrupt_set {
                state.status |= EXI_STATUS_EXIINT;
            }
        }
        let active = [self.exi.channel0.status, self.exi.channel1.status]
            .into_iter()
            .any(|status| {
                status & EXI_STATUS_EXIINT != 0 && status & EXI_STATUS_EXIINTMASK != 0
                    || status & EXI_STATUS_TCINT != 0 && status & EXI_STATUS_TCINTMASK != 0
                    || status & EXI_STATUS_EXTINT != 0 && status & EXI_STATUS_EXTINTMASK != 0
            });
        self.set_pi_cause(PI_INT_EXI, active);
    }

    fn write_exi_status(&mut self, channel: usize, value: u32) {
        let previous_select = self.exi_chip_select(channel);
        let writable = EXI_STATUS_EXIINTMASK
            | EXI_STATUS_TCINTMASK
            | (0x7 << 4)
            | EXI_STATUS_CHIP_SELECT_MASK
            | EXI_STATUS_EXTINTMASK
            | if channel == 0 { EXI_STATUS_ROMDIS } else { 0 };
        {
            let state = self.exi_channel_mut(channel);
            let mut status = (state.status & !writable) | (value & writable) | EXI_STATUS_EXT;
            if value & EXI_STATUS_EXIINT != 0 {
                status &= !EXI_STATUS_EXIINT;
            }
            if value & EXI_STATUS_TCINT != 0 {
                status &= !EXI_STATUS_TCINT;
            }
            if value & EXI_STATUS_EXTINT != 0 {
                status &= !EXI_STATUS_EXTINT;
            }
            state.status = status;
        }

        let selected = self.exi_chip_select(channel);
        if previous_select == 1 && selected != 1 {
            match channel {
                0 => Self::exi_card_select(
                    &mut self.exi.channel0.card,
                    &mut self.memory_cards[0],
                    false,
                ),
                1 => Self::exi_card_select(
                    &mut self.exi.channel1.card,
                    &mut self.memory_cards[1],
                    false,
                ),
                _ => {}
            }
        }
        if previous_select != 1 && selected == 1 {
            match channel {
                0 => Self::exi_card_select(
                    &mut self.exi.channel0.card,
                    &mut self.memory_cards[0],
                    true,
                ),
                1 => Self::exi_card_select(
                    &mut self.exi.channel1.card,
                    &mut self.memory_cards[1],
                    true,
                ),
                _ => {}
            }
        }
        if channel == 0 && previous_select != 2 && selected == 2 {
            self.exi_ipl_select();
        }
        self.refresh_exi_interrupt();
    }

    fn exi_dma_read(&mut self, channel: usize) {
        let (dma_address, dma_length, card_command, card_address) = {
            let state = self.exi_channel(channel);
            (
                state.dma_address,
                state.dma_length,
                state.card.command,
                state.card.address,
            )
        };
        let Some(start) = memory_index(dma_address) else {
            return;
        };
        let count = (dma_length as usize).min(self.mem1.len().saturating_sub(start));
        if self.exi_chip_select(channel) == 1 && card_command == 0x52 {
            let address = card_address as usize;
            for index in 0..count {
                self.mem1[start + index] =
                    self.memory_cards[channel][(address + index) & (MEMORY_CARD_SIZE - 1)];
            }
        } else {
            for index in 0..count {
                self.mem1[start + index] = self.exi_transfer_byte(channel, 0);
            }
        }
    }

    fn exi_dma_write(&mut self, channel: usize) {
        let (dma_address, dma_length, card_command, card_address) = {
            let state = self.exi_channel(channel);
            (
                state.dma_address,
                state.dma_length,
                state.card.command,
                state.card.address,
            )
        };
        let Some(start) = memory_index(dma_address) else {
            return;
        };
        let count = (dma_length as usize).min(self.mem1.len().saturating_sub(start));
        if self.exi_chip_select(channel) == 1 && card_command == 0xf2 {
            let address = card_address as usize;
            for index in 0..count {
                self.memory_cards[channel][(address + index) & (MEMORY_CARD_SIZE - 1)] =
                    self.mem1[start + index];
            }
            let state = self.exi_channel_mut(channel);
            state.card.status |= EXI_CARD_STATUS_READY;
            state.card.status &= !EXI_CARD_STATUS_BUSY;
            state.card.interrupt_set = true;
            state.card.position = 0;
        } else {
            for index in 0..count {
                let byte = self.mem1[start + index];
                self.exi_transfer_byte(channel, byte);
            }
        }
    }

    fn write_exi_control(&mut self, channel: usize, value: u32) {
        self.exi_channel_mut(channel).control = value;
        if value & 1 == 0 || self.exi_chip_select(channel) == 0 {
            return;
        }
        let rw = (value >> 2) & 3;
        if value & 2 != 0 {
            match rw {
                0 => self.exi_dma_read(channel),
                1 => self.exi_dma_write(channel),
                _ => {}
            }
        } else {
            let count = (((value >> 4) & 3) + 1) as usize;
            let input = self.exi_channel(channel).imm_data.to_be_bytes();
            let mut output = [0u8; 4];
            match rw {
                0 => {
                    for byte in output.iter_mut().take(count) {
                        *byte = self.exi_transfer_byte(channel, 0);
                    }
                    self.exi_channel_mut(channel).imm_data = u32::from_be_bytes(output);
                }
                1 => {
                    for byte in input.into_iter().take(count) {
                        self.exi_transfer_byte(channel, byte);
                    }
                }
                2 => {
                    for (index, byte) in input.into_iter().take(count).enumerate() {
                        output[index] = self.exi_transfer_byte(channel, byte);
                    }
                    self.exi_channel_mut(channel).imm_data = u32::from_be_bytes(output);
                }
                _ => {}
            }
        }
        let state = self.exi_channel_mut(channel);
        state.control &= !1;
        state.status |= EXI_STATUS_TCINT;
        self.refresh_exi_interrupt();
    }

    fn interrupt_pending(&self) -> bool {
        self.mmio_read_u32_raw(PI_CAUSE) & self.mmio_read_u32_raw(PI_MASK) != 0
    }

    fn axis_byte(value: i16, invert: bool) -> u8 {
        let value = i32::from(if invert {
            value.saturating_neg()
        } else {
            value
        });
        if value >= 0 {
            (128 + value * 127 / 32767).clamp(128, 255) as u8
        } else {
            (128 + value * 128 / 32768).clamp(0, 127) as u8
        }
    }
    fn trigger_byte(value: i16) -> u8 {
        (i32::from(value).clamp(0, 32767) * 255 / 32767) as u8
    }

    fn controller_words(&self, player: usize) -> (u32, u32) {
        let buttons = self.input.buttons[player];
        let mut pad = PAD_USE_ORIGIN;
        pad |= u16::from(buttons & FACE_SOUTH != 0) * PAD_A;
        pad |= u16::from(buttons & FACE_EAST != 0) * PAD_B;
        pad |= u16::from(buttons & FACE_WEST != 0) * PAD_X;
        pad |= u16::from(buttons & FACE_NORTH != 0) * PAD_Y;
        pad |= u16::from(buttons & START != 0) * PAD_START;
        pad |= u16::from(buttons & LEFT != 0) * PAD_LEFT;
        pad |= u16::from(buttons & RIGHT != 0) * PAD_RIGHT;
        pad |= u16::from(buttons & DOWN != 0) * PAD_DOWN;
        pad |= u16::from(buttons & UP != 0) * PAD_UP;
        pad |= u16::from(buttons & L1 != 0) * PAD_L;
        pad |= u16::from(buttons & R1 != 0) * PAD_R;
        pad |= u16::from(buttons & crate::input::R2 != 0) * PAD_Z;
        let axes = self.input.axes[player];
        let stick_x = Self::axis_byte(axes[AXIS_LEFT_X], false);
        let stick_y = Self::axis_byte(axes[AXIS_LEFT_Y], true);
        let c_x = Self::axis_byte(axes[AXIS_RIGHT_X], false);
        let c_y = Self::axis_byte(axes[AXIS_RIGHT_Y], true);
        let l = Self::trigger_byte(axes[AXIS_LEFT_TRIGGER]).max(if buttons & L1 != 0 {
            255
        } else {
            0
        });
        let r = Self::trigger_byte(axes[AXIS_RIGHT_TRIGGER]).max(if buttons & R1 != 0 {
            255
        } else {
            0
        });
        let analog_a: u8 = if pad & PAD_A != 0 { 255 } else { 0 };
        let analog_b: u8 = if pad & PAD_B != 0 { 255 } else { 0 };
        let high = u32::from(stick_y) | (u32::from(stick_x) << 8) | (u32::from(pad) << 16);
        let low = u32::from(analog_b >> 4)
            | (u32::from(analog_a >> 4) << 4)
            | (u32::from(r >> 4) << 8)
            | (u32::from(l >> 4) << 12)
            | (u32::from(c_y) << 16)
            | (u32::from(c_x) << 24);
        (high, low)
    }

    fn refresh_si_channels(&mut self) {
        let mut status = self.mmio_read_u32_raw(SI_STATUS);
        for player in 0..4 {
            let (high, low) = self.controller_words(player);
            let base = SI_CHANNEL_0_OUT + player * SI_CHANNEL_STRIDE;
            self.mmio_write_u32_raw(base + 4, high);
            self.mmio_write_u32_raw(base + 8, low);
            status |= 0x2000_0000u32 >> (player * 8);
        }
        self.mmio_write_u32_raw(SI_STATUS, status);
        let mut csr = self.mmio_read_u32_raw(SI_COM_CSR);
        csr |= 1 << 28;
        self.mmio_write_u32_raw(SI_COM_CSR, csr);
        self.set_pi_cause(PI_INT_SI, csr & (1 << 27) != 0);
    }

    fn service_si_transfer(&mut self) {
        let mut csr = self.mmio_read_u32_raw(SI_COM_CSR);
        if csr & 1 == 0 {
            return;
        }
        let player = ((csr >> 1) & 3) as usize;
        let command = self.mmio[SI_IO_BUFFER];
        let (high, low) = self.controller_words(player);
        let response = match command {
            0x00 | 0xff => {
                self.mmio[SI_IO_BUFFER..SI_IO_BUFFER + 3].copy_from_slice(&[0x09, 0x00, 0x00]);
                3usize
            }
            0x40 => {
                self.mmio[SI_IO_BUFFER..SI_IO_BUFFER + 4].copy_from_slice(&high.to_be_bytes());
                self.mmio[SI_IO_BUFFER + 4..SI_IO_BUFFER + 8].copy_from_slice(&low.to_be_bytes());
                8
            }
            0x41 | 0x42 => {
                self.write_pad_origin(player);
                10
            }
            0x1d => 0,
            _ => 0,
        };
        let _ = response;
        csr &= !1;
        csr |= 1 << 31;
        self.mmio_write_u32_raw(SI_COM_CSR, csr);
        self.set_pi_cause(PI_INT_SI, csr & (1 << 30) != 0);
    }

    fn write_pad_origin(&mut self, _player: usize) {
        let origin = [0, 0, 0x80, 0x80, 0x80, 0x80, 0, 0, 0, 0];
        self.mmio[SI_IO_BUFFER..SI_IO_BUFFER + origin.len()].copy_from_slice(&origin);
    }

    fn fail_di(&mut self, error: u32) {
        self.di.error = error;
        self.finish_di(false);
    }

    fn check_di_read_preconditions(&mut self) -> bool {
        let error = match self.di.drive_state {
            DiDriveState::Ready | DiDriveState::ReadyNoReadsMade => return true,
            DiDriveState::DiscIdNotRead => DI_ERROR_NO_DISC_ID,
            DiDriveState::MotorStopped => DI_ERROR_MOTOR_STOPPED,
            DiDriveState::CoverOpened | DiDriveState::NoMediumPresent => {
                DI_ERROR_MEDIUM_NOT_PRESENT
            }
            DiDriveState::DiscChangeDetected => DI_ERROR_MEDIUM_CHANGED,
        };
        self.di.error = error;
        false
    }

    fn finish_di(&mut self, success: bool) {
        let mut control = self.mmio_read_u32_raw(DI_DMA_CONTROL);
        control &= !1;
        self.mmio_write_u32_raw(DI_DMA_CONTROL, control);
        let mut status = self.mmio_read_u32_raw(DI_STATUS);
        if success {
            status |= 1 << 4;
        } else {
            status |= 1 << 2;
        }
        self.mmio_write_u32_raw(DI_STATUS, status);
        let pending = (status & (1 << 2) != 0 && status & (1 << 1) != 0)
            || (status & (1 << 4) != 0 && status & (1 << 3) != 0)
            || (status & (1 << 6) != 0 && status & (1 << 5) != 0);
        self.set_pi_cause(PI_INT_DI, pending);
    }

    fn service_di(&mut self) {
        if self.mmio_read_u32_raw(DI_DMA_CONTROL) & 1 == 0 {
            return;
        }
        let command0 = self.mmio_read_u32_raw(DI_COMMAND_0);
        let command = (command0 >> 24) as u8;
        match command {
            0x12 => self.di_inquiry(),
            0xa8 => match command0 as u8 {
                0x00 => self.di_read(),
                0x40 => self.di_read_disc_id(),
                _ => self.fail_di(DI_ERROR_INVALID_COMMAND),
            },
            0xab => self.di_seek(),
            0xe0 => self.di_request_error(),
            0xe1 => self.di_audio_stream(),
            0xe2 => self.di_audio_status(),
            0xe3 => self.di_stop_motor(),
            0xe4 => self.di_audio_buffer_config(),
            _ => self.fail_di(DI_ERROR_INVALID_COMMAND),
        }
    }
    fn di_inquiry(&mut self) {
        let address = self.mmio_read_u32_raw(DI_DMA_ADDRESS);
        let length = self.mmio_read_u32_raw(DI_DMA_LENGTH).min(0x20) as usize;
        let Ok(range) = memory_range(address, length) else {
            self.fail_di(DI_ERROR_INVALID_COMMAND);
            return;
        };
        let mut response = [0u8; 0x20];
        response[..12].copy_from_slice(&[
            0x00, 0x00, 0x00, 0x02, 0x20, 0x06, 0x05, 0x26, 0x41, 0x00, 0x00, 0x00,
        ]);
        self.mem1[range].copy_from_slice(&response[..length]);
        self.mmio_write_u32_raw(DI_DMA_ADDRESS, address.wrapping_add(length as u32));
        self.mmio_write_u32_raw(DI_DMA_LENGTH, 0);
        self.finish_di(true);
    }

    fn di_read(&mut self) {
        if !self.check_di_read_preconditions() {
            self.finish_di(false);
            return;
        }
        let dvd_offset = u64::from(self.mmio_read_u32_raw(DI_COMMAND_1)) << 2;
        let requested = self.mmio_read_u32_raw(DI_COMMAND_2);
        let dma_length = self.mmio_read_u32_raw(DI_DMA_LENGTH);
        let length = requested.min(dma_length) as usize;
        let address = self.mmio_read_u32_raw(DI_DMA_ADDRESS);
        if dvd_offset
            .checked_add(length as u64)
            .is_none_or(|end| end > self.disc.len())
        {
            self.fail_di(DI_ERROR_BLOCK_OOB);
            return;
        }
        let Ok(range) = memory_range(address, length) else {
            self.fail_di(DI_ERROR_INVALID_COMMAND);
            return;
        };
        if self.disc.read(dvd_offset, &mut self.mem1[range]).is_err() {
            return;
        }
        if self.di.drive_state == DiDriveState::ReadyNoReadsMade {
            self.di.drive_state = DiDriveState::Ready;
        }
        self.mmio_write_u32_raw(DI_DMA_ADDRESS, address.wrapping_add(length as u32));
        self.mmio_write_u32_raw(DI_DMA_LENGTH, 0);
        self.finish_di(true);
    }

    fn di_read_disc_id(&mut self) {
        match self.di.drive_state {
            DiDriveState::MotorStopped => {
                self.fail_di(DI_ERROR_MOTOR_STOPPED);
                return;
            }
            DiDriveState::CoverOpened | DiDriveState::NoMediumPresent => {
                self.fail_di(DI_ERROR_MEDIUM_NOT_PRESENT);
                return;
            }
            DiDriveState::DiscChangeDetected => {
                self.fail_di(DI_ERROR_MEDIUM_CHANGED);
                return;
            }
            DiDriveState::Ready | DiDriveState::ReadyNoReadsMade | DiDriveState::DiscIdNotRead => {}
        }

        let address = self.mmio_read_u32_raw(DI_DMA_ADDRESS);
        let length = self.mmio_read_u32_raw(DI_DMA_LENGTH).min(0x20) as usize;
        let Ok(range) = memory_range(address, length) else {
            self.fail_di(DI_ERROR_INVALID_COMMAND);
            return;
        };
        if self.disc.read(0, &mut self.mem1[range]).is_err() {
            return;
        }

        self.di.drive_state = match self.di.drive_state {
            DiDriveState::DiscIdNotRead => DiDriveState::ReadyNoReadsMade,
            DiDriveState::ReadyNoReadsMade => DiDriveState::Ready,
            state => state,
        };
        self.mmio_write_u32_raw(DI_DMA_ADDRESS, address.wrapping_add(length as u32));
        self.mmio_write_u32_raw(DI_DMA_LENGTH, 0);
        self.finish_di(true);
    }

    fn di_seek(&mut self) {
        if !self.check_di_read_preconditions() {
            self.finish_di(false);
            return;
        }
        if self.di.drive_state == DiDriveState::ReadyNoReadsMade {
            self.di.drive_state = DiDriveState::Ready;
        }
        self.finish_di(true);
    }

    fn di_request_error(&mut self) {
        let result = (self.di.drive_state.error_state() << 24) | self.di.error;
        self.mmio_write_u32_raw(DI_IMMEDIATE, result);
        self.di.error = DI_ERROR_NONE;
        self.finish_di(true);
    }

    fn di_audio_stream(&mut self) {
        if !self.check_di_read_preconditions() {
            self.finish_di(false);
            return;
        }
        if !self.di.enable_dtk {
            self.fail_di(DI_ERROR_NO_AUDIO_BUFFER);
            return;
        }
        if self.di.drive_state == DiDriveState::ReadyNoReadsMade {
            self.di.drive_state = DiDriveState::Ready;
        }

        match (self.mmio_read_u32_raw(DI_COMMAND_0) >> 16) & 0xff {
            0 => {
                let start = u64::from(self.mmio_read_u32_raw(DI_COMMAND_1)) << 2;
                let length = self.mmio_read_u32_raw(DI_COMMAND_2);
                if start == 0 && length == 0 {
                    self.di.stop_at_track_end = true;
                } else if !self.di.stop_at_track_end {
                    self.di.next_start = start;
                    self.di.next_length = length;
                    if !self.di.stream {
                        self.di.current_start = start;
                        self.di.current_length = length;
                        self.di.audio_position = start;
                        self.di.dtk_sample_remainder = 0;
                        self.di.stream = true;
                    }
                }
                self.finish_di(true);
            }
            1 => {
                self.di.stop_at_track_end = false;
                self.di.stream = false;
                self.di.dtk_sample_remainder = 0;
                self.finish_di(true);
            }
            _ => self.fail_di(DI_ERROR_INVALID_AUDIO_COMMAND),
        }
    }

    fn di_audio_status(&mut self) {
        if !self.check_di_read_preconditions() {
            self.finish_di(false);
            return;
        }
        if !self.di.enable_dtk {
            self.fail_di(DI_ERROR_NO_AUDIO_BUFFER);
            return;
        }

        let value = match (self.mmio_read_u32_raw(DI_COMMAND_0) >> 16) & 0xff {
            0 => u32::from(self.di.stream),
            1 => ((self.di.audio_position & !0x7fff) >> 2) as u32,
            2 => (self.di.current_start >> 2) as u32,
            3 => self.di.current_length,
            _ => {
                self.fail_di(DI_ERROR_INVALID_AUDIO_COMMAND);
                return;
            }
        };
        self.mmio_write_u32_raw(DI_IMMEDIATE, value);
        self.finish_di(true);
    }

    fn di_stop_motor(&mut self) {
        self.di.stream = false;
        self.di.stop_at_track_end = false;
        self.di.dtk_sample_remainder = 0;
        self.di.drive_state = DiDriveState::MotorStopped;
        self.finish_di(true);
    }

    fn di_audio_buffer_config(&mut self) {
        if !self.check_di_read_preconditions() {
            self.finish_di(false);
            return;
        }
        if self.di.drive_state == DiDriveState::Ready {
            self.fail_di(DI_ERROR_INVALID_PERIOD);
            return;
        }

        let command = self.mmio_read_u32_raw(DI_COMMAND_0);
        self.di.enable_dtk = command & (1 << 16) != 0;
        self.di.dtk_buffer_length = (command & 0x0f) as u8;
        if !self.di.enable_dtk {
            self.di.stream = false;
            self.di.stop_at_track_end = false;
            self.di.dtk_sample_remainder = 0;
        }
        self.finish_di(true);
    }

    fn advance_dtk_stream(&mut self, samples: u32) {
        if !self.di.stream {
            return;
        }

        let total_samples = u32::from(self.di.dtk_sample_remainder).saturating_add(samples);
        let blocks = total_samples / DTK_SAMPLES_PER_BLOCK;
        self.di.dtk_sample_remainder = (total_samples % DTK_SAMPLES_PER_BLOCK) as u8;

        for _ in 0..blocks {
            if self.di.audio_position >= self.di.current_start + u64::from(self.di.current_length) {
                self.di.audio_position = self.di.next_start;
                self.di.current_start = self.di.next_start;
                self.di.current_length = self.di.next_length;
                if self.di.stop_at_track_end {
                    self.di.stop_at_track_end = false;
                    self.di.stream = false;
                    self.di.dtk_sample_remainder = 0;
                    break;
                }
            }
            self.di.audio_position = self.di.audio_position.saturating_add(DTK_BLOCK_BYTES);
        }
    }

    fn service_aram_dma(&mut self) {
        let mm_address = (u32::from(self.mmio_read_u16_raw(AR_DMA_MMADDR_H)) << 16)
            | u32::from(self.mmio_read_u16_raw(AR_DMA_MMADDR_L));
        let ar_address = (u32::from(self.mmio_read_u16_raw(AR_DMA_ARADDR_H)) << 16)
            | u32::from(self.mmio_read_u16_raw(AR_DMA_ARADDR_L));
        let count = (u32::from(self.mmio_read_u16_raw(AR_DMA_CNT_H)) << 16)
            | u32::from(self.mmio_read_u16_raw(AR_DMA_CNT_L));
        let length = (count & 0x7fff_ffff) as usize;
        if length == 0 {
            return;
        }
        let Ok(main) = memory_range(mm_address, length) else {
            return;
        };
        let ar_start = (ar_address & 0x00ff_ffff) as usize;
        let Some(ar_end) = ar_start
            .checked_add(length)
            .filter(|end| *end <= self.aram.len())
        else {
            return;
        };
        if count >> 31 == 0 {
            self.aram[ar_start..ar_end].copy_from_slice(&self.mem1[main]);
        } else {
            self.mem1[main].copy_from_slice(&self.aram[ar_start..ar_end]);
        }
        self.mmio_write_u16_raw(AR_DMA_CNT_H, 0);
        self.mmio_write_u16_raw(AR_DMA_CNT_L, 0);
        let control = self.mmio_read_u16_raw(DSP_CONTROL) | (1 << 5);
        self.mmio_write_u16_raw(DSP_CONTROL, control);
        self.set_pi_cause(PI_INT_DSP, control & (1 << 6) != 0);
    }

    fn configure_audio_dma(&mut self) {
        let source = (u32::from(self.mmio_read_u16_raw(AUDIO_DMA_START_HI)) << 16)
            | u32::from(self.mmio_read_u16_raw(AUDIO_DMA_START_LO));
        let control = self.mmio_read_u16_raw(AUDIO_DMA_CONTROL_LEN);
        self.audio_dma = AudioDma {
            source: source & !31,
            cursor: source & !31,
            blocks: control & 0x7fff,
            remaining: control & 0x7fff,
            enabled: control & 0x8000 != 0,
        };
        self.mmio_write_u16_raw(AUDIO_DMA_BLOCKS_LEFT, self.audio_dma.remaining);
    }

    fn refresh_dsp_interrupt(&mut self) {
        let control = self.mmio_read_u16_raw(DSP_CONTROL);
        let active = ((control >> 1) & control & (0x80 | 0x20 | 0x08)) != 0;
        self.set_pi_cause(PI_INT_DSP, active);
    }

    fn complete_audio_block(&mut self) {
        if self.audio_dma.remaining != 0 {
            self.audio_dma.remaining -= 1;
            self.audio_dma.cursor = self.audio_dma.cursor.wrapping_add(32);
        }
        if self.audio_dma.remaining == 0 {
            self.audio_dma.cursor = self.audio_dma.source;
            self.audio_dma.remaining = self.audio_dma.blocks;
            let control = self.mmio_read_u16_raw(DSP_CONTROL) | (1 << 3);
            self.mmio_write_u16_raw(DSP_CONTROL, control);
            self.refresh_dsp_interrupt();
        }
        self.mmio_write_u16_raw(AUDIO_DMA_BLOCKS_LEFT, self.audio_dma.remaining);
    }
    fn render_audio(&mut self, audio: &mut AudioBuffer) {
        audio.begin_frame();
        for _ in 0..100 {
            let mut block = [0u8; 32];
            if self.audio_dma.enabled {
                if let Ok(range) = memory_range(self.audio_dma.cursor, block.len()) {
                    block.copy_from_slice(&self.mem1[range]);
                }
            }
            for frame in block.as_chunks::<4>().0 {
                let left = i16::from_be_bytes([frame[0], frame[1]]) as f32 / 32768.0;
                let right = i16::from_be_bytes([frame[2], frame[3]]) as f32 / 32768.0;
                audio.push_stereo(left, right);
            }
            if self.audio_dma.enabled {
                self.complete_audio_block();
            }
        }
    }

    fn xfb_address(&self) -> Option<u32> {
        let raw = (u32::from(self.mmio_read_u16_raw(VI_FB_LEFT_TOP_HI)) << 16)
            | u32::from(self.mmio_read_u16_raw(VI_FB_LEFT_TOP_LO));
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
        let Ok(range) = memory_range(address, byte_len) else {
            self.video.clear([0, 0, 0, 255]);
            return;
        };
        let source = &self.mem1[range];
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
            let mut register = self.mmio_read_u32_raw(offset);
            if register & (1 << 28) != 0 {
                register |= 1 << 31;
                self.mmio_write_u32_raw(offset, register);
                active = true;
            }
        }
        self.set_pi_cause(PI_INT_VI, active);
    }

    fn begin_frame(&mut self, input: &InputState) {
        self.input = input.clone();
        self.refresh_si_channels();
        self.service_di();
    }

    fn end_frame(&mut self, audio: &mut AudioBuffer) {
        self.service_di();
        self.advance_dtk_stream(AUDIO_SAMPLES_PER_FRAME);
        self.present_video();
        self.render_audio(audio);
        self.raise_vi_retrace();
        self.tick_exi_rtc();
        self.frame_counter = self.frame_counter.wrapping_add(1);
    }

    fn write_di_status(&mut self, value: u32) {
        let old = self.mmio_read_u32_raw(DI_STATUS);
        let writable = (1 << 0) | (1 << 1) | (1 << 3) | (1 << 5);
        let mut next = (old & !writable) | (value & writable);
        for active in [2u32, 4, 6] {
            if value & (1 << active) != 0 {
                next &= !(1 << active);
            }
        }
        self.mmio_write_u32_raw(DI_STATUS, next);
        let pending = (next & (1 << 2) != 0 && next & (1 << 1) != 0)
            || (next & (1 << 4) != 0 && next & (1 << 3) != 0)
            || (next & (1 << 6) != 0 && next & (1 << 5) != 0);
        self.set_pi_cause(PI_INT_DI, pending);
    }

    fn write_si_csr(&mut self, value: u32) {
        let old = self.mmio_read_u32_raw(SI_COM_CSR);
        let status = old & ((1 << 28) | (1 << 31));
        let mut next = value & !((1 << 28) | (1 << 31));
        next |= status;
        if value & (1 << 28) != 0 {
            next &= !(1 << 28);
        }
        if value & (1 << 31) != 0 {
            next &= !(1 << 31);
        }
        self.mmio_write_u32_raw(SI_COM_CSR, next);
        self.service_si_transfer();
        let csr = self.mmio_read_u32_raw(SI_COM_CSR);
        let pending = (csr & (1 << 28) != 0 && csr & (1 << 27) != 0)
            || (csr & (1 << 31) != 0 && csr & (1 << 30) != 0);
        self.set_pi_cause(PI_INT_SI, pending);
    }

    fn read_mmio8(&mut self, offset: usize) -> u8 {
        if (SI_CHANNEL_0_OUT..SI_IO_BUFFER).contains(&offset) {
            self.refresh_si_channels();
        }
        self.mmio.get(offset).copied().unwrap_or(0)
    }

    fn write_mmio8(&mut self, offset: usize, value: u8) {
        if let Some(byte) = self.mmio.get_mut(offset) {
            *byte = value;
        }
    }

    fn write_mmio16(&mut self, offset: usize, value: u16) {
        self.mmio_write_u16_raw(offset, value);
        match offset {
            DSP_CONTROL => self.refresh_dsp_interrupt(),
            AR_DMA_CNT_L => self.service_aram_dma(),
            AUDIO_DMA_CONTROL_LEN => self.configure_audio_dma(),
            VI_PRERETRACE_HI | VI_POSTRETRACE_HI => {
                let active = [VI_PRERETRACE_HI, VI_POSTRETRACE_HI]
                    .into_iter()
                    .any(|base| {
                        let register = self.mmio_read_u32_raw(base);
                        register & (1 << 31) != 0 && register & (1 << 28) != 0
                    });
                self.set_pi_cause(PI_INT_VI, active);
            }
            _ => {}
        }
    }
    fn read_mmio16(&mut self, offset: usize) -> u16 {
        if (SI_CHANNEL_0_OUT..SI_IO_BUFFER).contains(&offset) {
            self.refresh_si_channels();
        }
        self.mmio_read_u16_raw(offset)
    }

    fn decode_exi_register(offset: usize) -> Option<(usize, usize)> {
        let relative = offset.checked_sub(EXI_STATUS)?;
        let channel = relative / EXI_CHANNEL_STRIDE;
        if channel > 1 {
            return None;
        }
        let register = EXI_STATUS + relative % EXI_CHANNEL_STRIDE;
        matches!(
            register,
            EXI_STATUS | EXI_DMA_ADDRESS | EXI_DMA_LENGTH | EXI_DMA_CONTROL | EXI_IMM_DATA
        )
        .then_some((channel, register))
    }

    fn read_mmio32(&mut self, offset: usize) -> u32 {
        if (SI_CHANNEL_0_OUT..SI_IO_BUFFER).contains(&offset) {
            self.refresh_si_channels();
        }
        if let Some((channel, register)) = Self::decode_exi_register(offset) {
            if register == EXI_STATUS {
                self.exi_channel_mut(channel).status |= EXI_STATUS_EXT;
                self.refresh_exi_interrupt();
            }
            let state = self.exi_channel(channel);
            return match register {
                EXI_STATUS => state.status,
                EXI_DMA_ADDRESS => state.dma_address,
                EXI_DMA_LENGTH => state.dma_length,
                EXI_DMA_CONTROL => state.control,
                EXI_IMM_DATA => state.imm_data,
                _ => unreachable!(),
            };
        }
        self.mmio_read_u32_raw(offset)
    }

    fn write_mmio32(&mut self, offset: usize, value: u32) {
        if let Some((channel, register)) = Self::decode_exi_register(offset) {
            match register {
                EXI_STATUS => self.write_exi_status(channel, value),
                EXI_DMA_ADDRESS => self.exi_channel_mut(channel).dma_address = value & !0x1f,
                EXI_DMA_LENGTH => self.exi_channel_mut(channel).dma_length = value,
                EXI_DMA_CONTROL => self.write_exi_control(channel, value),
                EXI_IMM_DATA => self.exi_channel_mut(channel).imm_data = value,
                _ => unreachable!(),
            }
            return;
        }
        match offset {
            PI_CAUSE => {}
            PI_MASK => self.mmio_write_u32_raw(offset, value),
            DI_STATUS => self.write_di_status(value),
            DI_DMA_CONTROL => {
                self.mmio_write_u32_raw(offset, value);
                self.service_di();
            }
            SI_COM_CSR => self.write_si_csr(value),
            VI_PRERETRACE_HI | VI_POSTRETRACE_HI => {
                self.write_mmio16(offset, (value >> 16) as u16);
                self.write_mmio16(offset + 2, value as u16);
            }
            _ if (0x5000..0x5040).contains(&offset) => {
                self.write_mmio16(offset, (value >> 16) as u16);
                self.write_mmio16(offset + 2, value as u16);
            }
            _ => self.mmio_write_u32_raw(offset, value),
        }
    }

    fn reset(&mut self) -> Result<u32, String> {
        self.mem1.fill(0);
        self.aram.fill(0);
        self.efb.fill(0);
        let entry = load_dol(&self.disc, &mut self.mem1)?;
        self.reset_mmio();
        self.initialize_low_memory();
        self.initialize_hle_boot_state()?;
        self.input = InputState::default();
        self.frame_counter = 0;
        self.video.clear([0, 0, 0, 255]);
        Ok(entry)
    }

    fn save_exi_channel(channel: &ExiChannelState, out: &mut StateWriter) {
        out.u32(channel.status);
        out.u32(channel.dma_address);
        out.u32(channel.dma_length);
        out.u32(channel.control);
        out.u32(channel.imm_data);
        out.u8(channel.card.status);
        out.u8(channel.card.interrupt_switch);
        out.u8(u8::from(channel.card.interrupt_set));
        out.u8(channel.card.command);
        out.u32(channel.card.position);
        out.u32(channel.card.address);
        out.blob(&channel.card.programming_buffer);
    }

    fn load_exi_channel(
        channel: &mut ExiChannelState,
        input: &mut StateReader<'_>,
        label: &str,
    ) -> Result<(), String> {
        channel.status = input.u32()?;
        channel.dma_address = input.u32()?;
        channel.dma_length = input.u32()?;
        channel.control = input.u32()?;
        channel.imm_data = input.u32()?;
        channel.card.status = input.u8()?;
        channel.card.interrupt_switch = input.u8()?;
        channel.card.interrupt_set = input.u8()? != 0;
        channel.card.command = input.u8()?;
        channel.card.position = input.u32()?;
        channel.card.address = input.u32()?;
        Self::load_blob(
            input,
            &mut channel.card.programming_buffer,
            &format!("{label} memory-card programming buffer"),
        )
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.mem1);
        out.blob(&self.aram);
        out.blob(&self.efb);
        out.blob(&self.mmio);
        out.blob(&self.memory_cards[0]);
        out.blob(&self.memory_cards[1]);
        out.u32(self.audio_dma.source);
        out.u32(self.audio_dma.cursor);
        out.u16(self.audio_dma.blocks);
        out.u16(self.audio_dma.remaining);
        out.u8(u8::from(self.audio_dma.enabled));
        out.u8(self.di.drive_state as u8);
        out.u32(self.di.error);
        out.u8(u8::from(self.di.enable_dtk));
        out.u8(self.di.dtk_buffer_length);
        out.u8(self.di.dtk_sample_remainder);
        out.u8(u8::from(self.di.stream));
        out.u8(u8::from(self.di.stop_at_track_end));
        out.u64(self.di.audio_position);
        out.u64(self.di.current_start);
        out.u32(self.di.current_length);
        out.u64(self.di.next_start);
        out.u32(self.di.next_length);
        Self::save_exi_channel(&self.exi.channel0, out);
        Self::save_exi_channel(&self.exi.channel1, out);
        out.u32(self.exi.ipl.command);
        out.u8(self.exi.ipl.command_bytes);
        out.u32(self.exi.ipl.cursor);
        out.u8(self.exi.ipl.rtc_phase);
        out.blob(&self.exi.ipl.sram);
        out.u64(self.frame_counter);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        Self::load_blob(input, &mut self.mem1, "MEM1")?;
        Self::load_blob(input, &mut self.aram, "ARAM")?;
        Self::load_blob(input, &mut self.efb, "EFB")?;
        Self::load_blob(input, &mut self.mmio, "MMIO")?;
        Self::load_blob(input, &mut self.memory_cards[0], "slot-A memory card")?;
        Self::load_blob(input, &mut self.memory_cards[1], "slot-B memory card")?;
        self.audio_dma.source = input.u32()?;
        self.audio_dma.cursor = input.u32()?;
        self.audio_dma.blocks = input.u16()?;
        self.audio_dma.remaining = input.u16()?;
        self.audio_dma.enabled = input.u8()? != 0;
        self.di.drive_state = DiDriveState::from_u8(input.u8()?)?;
        self.di.error = input.u32()?;
        self.di.enable_dtk = input.u8()? != 0;
        self.di.dtk_buffer_length = input.u8()? & 0x0f;
        self.di.dtk_sample_remainder = input.u8()? % DTK_SAMPLES_PER_BLOCK as u8;
        self.di.stream = input.u8()? != 0;
        self.di.stop_at_track_end = input.u8()? != 0;
        self.di.audio_position = input.u64()?;
        self.di.current_start = input.u64()?;
        self.di.current_length = input.u32()?;
        self.di.next_start = input.u64()?;
        self.di.next_length = input.u32()?;
        Self::load_exi_channel(&mut self.exi.channel0, input, "EXI channel 0")?;
        Self::load_exi_channel(&mut self.exi.channel1, input, "EXI channel 1")?;
        self.exi.ipl.command = input.u32()?;
        self.exi.ipl.command_bytes = input.u8()?.min(4);
        self.exi.ipl.cursor = input.u32()?;
        self.exi.ipl.rtc_phase = input.u8()? % FRAME_RATE as u8;
        Self::load_blob(input, &mut self.exi.ipl.sram, "EXI IPL SRAM")?;
        self.frame_counter = input.u64()?;
        self.input = InputState::default();
        self.refresh_exi_interrupt();
        self.present_video();
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
                "GameCube {label} state has {} bytes; expected {}",
                blob.len(),
                target.len()
            ));
        }
        target.copy_from_slice(blob);
        Ok(())
    }
}

impl PowerPcBus for GameCubeBoard {
    fn read8(&mut self, address: u32) -> u8 {
        if let Some(index) = memory_index(address) {
            return self.mem1[index];
        }
        if let Some(offset) = address
            .checked_sub(EFB_BASE)
            .filter(|offset| *offset < EFB_SIZE as u32)
        {
            return self.efb[offset as usize];
        }
        if let Some(offset) = address
            .checked_sub(MMIO_BASE)
            .filter(|offset| *offset < MMIO_SIZE as u32)
        {
            return self.read_mmio8(offset as usize);
        }
        0
    }

    fn write8(&mut self, address: u32, value: u8) {
        if let Some(index) = memory_index(address) {
            self.mem1[index] = value;
            return;
        }
        if let Some(offset) = address
            .checked_sub(EFB_BASE)
            .filter(|offset| *offset < EFB_SIZE as u32)
        {
            self.efb[offset as usize] = value;
            return;
        }
        if let Some(offset) = address
            .checked_sub(MMIO_BASE)
            .filter(|offset| *offset < MMIO_SIZE as u32)
        {
            self.write_mmio8(offset as usize, value);
        }
    }
    fn read16(&mut self, address: u32) -> u16 {
        if let Some(index) = memory_index(address).filter(|index| *index + 2 <= MEM1_SIZE) {
            return u16::from_be_bytes(self.mem1[index..index + 2].try_into().unwrap());
        }
        if let Some(offset) = address
            .checked_sub(EFB_BASE)
            .filter(|offset| (*offset as usize) + 2 <= EFB_SIZE)
        {
            let index = offset as usize;
            return u16::from_be_bytes(self.efb[index..index + 2].try_into().unwrap());
        }
        if let Some(offset) = address
            .checked_sub(MMIO_BASE)
            .filter(|offset| (*offset as usize) + 2 <= MMIO_SIZE)
        {
            return self.read_mmio16(offset as usize);
        }
        u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }

    fn write16(&mut self, address: u32, value: u16) {
        if let Some(index) = memory_index(address).filter(|index| *index + 2 <= MEM1_SIZE) {
            self.mem1[index..index + 2].copy_from_slice(&value.to_be_bytes());
            return;
        }
        if let Some(offset) = address
            .checked_sub(EFB_BASE)
            .filter(|offset| (*offset as usize) + 2 <= EFB_SIZE)
        {
            let index = offset as usize;
            self.efb[index..index + 2].copy_from_slice(&value.to_be_bytes());
            return;
        }
        if let Some(offset) = address
            .checked_sub(MMIO_BASE)
            .filter(|offset| (*offset as usize) + 2 <= MMIO_SIZE)
        {
            self.write_mmio16(offset as usize, value);
            return;
        }
        let bytes = value.to_be_bytes();
        self.write8(address, bytes[0]);
        self.write8(address.wrapping_add(1), bytes[1]);
    }

    fn read32(&mut self, address: u32) -> u32 {
        if let Some(index) = memory_index(address).filter(|index| *index + 4 <= MEM1_SIZE) {
            return u32::from_be_bytes(self.mem1[index..index + 4].try_into().unwrap());
        }
        if let Some(offset) = address
            .checked_sub(EFB_BASE)
            .filter(|offset| (*offset as usize) + 4 <= EFB_SIZE)
        {
            let index = offset as usize;
            return u32::from_be_bytes(self.efb[index..index + 4].try_into().unwrap());
        }
        if let Some(offset) = address
            .checked_sub(MMIO_BASE)
            .filter(|offset| (*offset as usize) + 4 <= MMIO_SIZE)
        {
            return self.read_mmio32(offset as usize);
        }
        let mut bytes = [0; 4];
        for (index, byte) in bytes.iter_mut().enumerate() {
            *byte = self.read8(address.wrapping_add(index as u32));
        }
        u32::from_be_bytes(bytes)
    }

    fn write32(&mut self, address: u32, value: u32) {
        if let Some(index) = memory_index(address).filter(|index| *index + 4 <= MEM1_SIZE) {
            self.mem1[index..index + 4].copy_from_slice(&value.to_be_bytes());
            return;
        }
        if let Some(offset) = address
            .checked_sub(EFB_BASE)
            .filter(|offset| (*offset as usize) + 4 <= EFB_SIZE)
        {
            let index = offset as usize;
            self.efb[index..index + 4].copy_from_slice(&value.to_be_bytes());
            return;
        }
        if let Some(offset) = address
            .checked_sub(MMIO_BASE)
            .filter(|offset| (*offset as usize) + 4 <= MMIO_SIZE)
        {
            self.write_mmio32(offset as usize, value);
            return;
        }
        for (index, byte) in value.to_be_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(index as u32), byte);
        }
    }

    fn read64(&mut self, address: u32) -> u64 {
        let high = u64::from(self.read32(address));
        let low = u64::from(self.read32(address.wrapping_add(4)));
        (high << 32) | low
    }

    fn write64(&mut self, address: u32, value: u64) {
        self.write32(address, (value >> 32) as u32);
        self.write32(address.wrapping_add(4), value as u32);
    }
}

pub struct GameCubeMachine {
    cpu: PowerPc750,
    board: GameCubeBoard,
    audio: AudioBuffer,
    powered: bool,
}

impl GameCubeMachine {
    pub fn from_image(image: ResourceBlob) -> Result<Self, String> {
        Self::from_image_and_ipl(image, None)
    }

    pub fn from_image_and_ipl(image: ResourceBlob, ipl: Option<&[u8]>) -> Result<Self, String> {
        let ipl_rom = ipl.map(prepare_ipl_rom).transpose()?;
        let (board, entry) = GameCubeBoard::new_with_ipl(image, ipl_rom)?;
        let mut cpu = PowerPc750::new();
        cpu.reset_to(entry);
        cpu.msr |= 0x0000_2000;
        cpu.hid2 = 0xe000_0000;
        cpu.gpr[1] = 0x817f_ffc0;
        Ok(Self {
            cpu,
            board,
            audio: AudioBuffer::new(AUDIO_RATE, 2),
            powered: true,
        })
    }

    fn boot_cpu(&mut self, entry: u32) {
        self.cpu.reset_to(entry);
        self.cpu.msr |= 0x0000_2000;
        self.cpu.hid2 = 0xe000_0000;
        self.cpu.gpr[1] = 0x817f_ffc0;
        self.powered = true;
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

impl Machine for GameCubeMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::GameCube
    }

    fn reset(&mut self) {
        match self.board.reset() {
            Ok(entry) => self.boot_cpu(entry),
            Err(_) => self.powered = false,
        }
        self.audio.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        if !self.powered {
            return;
        }
        self.board.begin_frame(input);
        self.run_cpu_frame();
        self.board.end_frame(&mut self.audio);
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE as f64
    }

    fn video(&self) -> &VideoBuffer {
        &self.board.video
    }

    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::GameCube, STATE_VERSION);
        self.cpu.save(&mut out);
        self.board.save(&mut out);
        out.u8(u8::from(self.powered));
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::GameCube, STATE_VERSION)?;
        self.cpu.load(&mut input)?;
        self.board.load(&mut input)?;
        self.powered = input.u8()? != 0;
        input.finish()?;
        self.audio.begin_frame();
        Ok(())
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        match (kind, slot) {
            (ResourceKind::MemoryCard, 0 | 1) => MEMORY_CARD_SIZE,
            (ResourceKind::Storage, 0) => EXI_IPL_SRAM_SIZE,
            _ => 0,
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        match (kind, slot) {
            (ResourceKind::MemoryCard, 0 | 1) if out.len() == MEMORY_CARD_SIZE => {
                out.copy_from_slice(&self.board.memory_cards[slot as usize]);
                Ok(())
            }
            (ResourceKind::Storage, 0) if out.len() == EXI_IPL_SRAM_SIZE => {
                out.copy_from_slice(&self.board.exi.ipl.sram);
                Ok(())
            }
            (ResourceKind::MemoryCard, 0 | 1) => {
                Err("GameCube memory-card output has the wrong size".into())
            }
            (ResourceKind::Storage, 0) => Err("GameCube IPL SRAM output has the wrong size".into()),
            _ => Err(
                "GameCube persistent resources are MemoryCard slots 0-1 and Storage slot 0".into(),
            ),
        }
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        match (kind, slot) {
            (ResourceKind::MemoryCard, 0 | 1) if data.len() == MEMORY_CARD_SIZE => {
                self.board.memory_cards[slot as usize].copy_from_slice(data);
                Ok(())
            }
            (ResourceKind::Storage, 0) if data.len() == EXI_IPL_SRAM_SIZE => {
                self.board.exi.ipl.sram.copy_from_slice(data);
                Ok(())
            }
            (ResourceKind::MemoryCard, 0 | 1) => {
                Err("GameCube memory-card image has the wrong size".into())
            }
            (ResourceKind::Storage, 0) => Err("GameCube IPL SRAM image has the wrong size".into()),
            _ => Err(
                "GameCube persistent resources are MemoryCard slots 0-1 and Storage slot 0".into(),
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dol_bytes(program: &[u32]) -> Vec<u8> {
        let mut image = vec![0; 0x100 + program.len() * 4];
        image[0x00..0x04].copy_from_slice(&0x100u32.to_be_bytes());
        image[0x48..0x4c].copy_from_slice(&0x8000_3100u32.to_be_bytes());
        image[0x90..0x94].copy_from_slice(&((program.len() * 4) as u32).to_be_bytes());
        image[0xd8..0xdc].copy_from_slice(&0x8000_4000u32.to_be_bytes());
        image[0xdc..0xe0].copy_from_slice(&0x100u32.to_be_bytes());
        image[0xe0..0xe4].copy_from_slice(&0x8000_3100u32.to_be_bytes());
        for (index, instruction) in program.iter().enumerate() {
            let start = 0x100 + index * 4;
            image[start..start + 4].copy_from_slice(&instruction.to_be_bytes());
        }
        image
    }

    fn disc_bytes(program: &[u32]) -> Vec<u8> {
        let dol = dol_bytes(program);
        let mut disc = vec![0; 0x2000.max(0x500 + dol.len())];
        disc[0x1c..0x20].copy_from_slice(&DISC_MAGIC.to_be_bytes());
        disc[0x420..0x424].copy_from_slice(&0x500u32.to_be_bytes());
        disc[0x500..0x500 + dol.len()].copy_from_slice(&dol);
        disc
    }

    fn addi(rd: u32, ra: u32, immediate: u16) -> u32 {
        (14 << 26) | (rd << 21) | (ra << 16) | u32::from(immediate)
    }

    fn stw(rs: u32, ra: u32, displacement: u16) -> u32 {
        (36 << 26) | (rs << 21) | (ra << 16) | u32::from(displacement)
    }

    fn idle_board() -> GameCubeBoard {
        let image = ResourceBlob::from_bytes(&dol_bytes(&[0x4800_0000]));
        GameCubeBoard::new(image).unwrap().0
    }

    #[test]
    fn raw_dol_boots_powerpc_and_writes_mem1() {
        let image = ResourceBlob::from_bytes(&dol_bytes(&[
            addi(3, 0, 0x1234),
            stw(3, 0, 0x0200),
            0x4800_0000,
        ]));
        let (mut board, entry) = GameCubeBoard::new(image).unwrap();
        let mut cpu = PowerPc750::new();
        cpu.reset_to(entry);
        for _ in 0..3 {
            cpu.step(&mut board);
        }
        assert_eq!(board.read32(0x8000_0200), 0x0000_1234);
    }

    #[test]
    fn controller_poll_and_si_direct_transfer_match_gc_layout() {
        let mut board = idle_board();
        board.input.buttons[0] = FACE_SOUTH | FACE_WEST | START | L1 | crate::input::R2;
        board.input.axes[0][AXIS_LEFT_X] = 32767;
        board.input.axes[0][AXIS_LEFT_Y] = 0;
        board.input.axes[0][AXIS_RIGHT_X] = -32768;
        board.input.axes[0][AXIS_RIGHT_Y] = 32767;
        board.input.axes[0][AXIS_LEFT_TRIGGER] = 16384;
        let (high, low) = board.controller_words(0);
        assert_eq!(high & 0xff, 128);
        assert_eq!((high >> 8) & 0xff, 255);
        let pad = (high >> 16) as u16;
        assert_eq!(
            pad & (PAD_A | PAD_X | PAD_START | PAD_L | PAD_Z),
            PAD_A | PAD_X | PAD_START | PAD_L | PAD_Z
        );
        assert_eq!((low >> 24) as u8, 0);
        assert_eq!((low >> 16) as u8, 1);
        assert_eq!((low >> 12) & 0xf, 15);

        board.mmio[SI_IO_BUFFER] = 0x40;
        board.write_si_csr(1);
        assert_eq!(
            &board.mmio[SI_IO_BUFFER..SI_IO_BUFFER + 4],
            &high.to_be_bytes()
        );
        assert_eq!(
            &board.mmio[SI_IO_BUFFER + 4..SI_IO_BUFFER + 8],
            &low.to_be_bytes()
        );
        assert_ne!(board.mmio_read_u32_raw(SI_COM_CSR) & (1 << 31), 0);
    }

    fn exi_register(channel: usize, register: usize) -> usize {
        register + channel * EXI_CHANNEL_STRIDE
    }

    fn exi_select_card_channel(board: &mut GameCubeBoard, channel: usize) {
        board.write_mmio32(exi_register(channel, EXI_STATUS), 1 << 7);
    }

    fn exi_reselect_card_channel(board: &mut GameCubeBoard, channel: usize) {
        board.write_mmio32(exi_register(channel, EXI_STATUS), 0);
        exi_select_card_channel(board, channel);
    }

    fn exi_imm_write_channel(board: &mut GameCubeBoard, channel: usize, value: u32, length: u32) {
        board.write_mmio32(exi_register(channel, EXI_IMM_DATA), value);
        board.write_mmio32(
            exi_register(channel, EXI_DMA_CONTROL),
            1 | (1 << 2) | ((length - 1) << 4),
        );
    }

    fn exi_imm_read_channel(board: &mut GameCubeBoard, channel: usize, length: u32) -> u32 {
        board.write_mmio32(
            exi_register(channel, EXI_DMA_CONTROL),
            1 | ((length - 1) << 4),
        );
        board.read_mmio32(exi_register(channel, EXI_IMM_DATA))
    }

    fn exi_select_card(board: &mut GameCubeBoard) {
        exi_select_card_channel(board, 0);
    }

    fn exi_reselect_card(board: &mut GameCubeBoard) {
        exi_reselect_card_channel(board, 0);
    }

    fn exi_imm_write(board: &mut GameCubeBoard, value: u32, length: u32) {
        exi_imm_write_channel(board, 0, value, length);
    }

    fn exi_imm_read(board: &mut GameCubeBoard, length: u32) -> u32 {
        exi_imm_read_channel(board, 0, length)
    }

    fn exi_select_ipl(board: &mut GameCubeBoard) {
        board.write_mmio32(EXI_STATUS, 2 << 7);
    }

    fn exi_ipl_command(address: u32, write: bool) -> u32 {
        ((address & 0x01ff_ffff) << 6) | if write { 1 << 31 } else { 0 }
    }

    fn run_di_command(
        board: &mut GameCubeBoard,
        command0: u32,
        command1: u32,
        command2: u32,
        address: u32,
        length: u32,
    ) {
        board.mmio_write_u32_raw(DI_COMMAND_0, command0);
        board.mmio_write_u32_raw(DI_COMMAND_1, command1);
        board.mmio_write_u32_raw(DI_COMMAND_2, command2);
        board.mmio_write_u32_raw(DI_DMA_ADDRESS, address);
        board.mmio_write_u32_raw(DI_DMA_LENGTH, length);
        board.write_mmio32(DI_DMA_CONTROL, 1);
    }

    fn clear_di_completion(board: &mut GameCubeBoard) {
        board.write_di_status((1 << 2) | (1 << 4));
    }

    #[test]
    fn exi_ipl_reads_descrambled_user_rom_and_font_regions() {
        let mut raw_ipl = vec![0; EXI_IPL_ROM_SIZE];
        raw_ipl[0x20] = 0x5a;
        raw_ipl[0x1aff00] = 0xa5;
        raw_ipl[0x1fcf00] = 0x3c;
        let prepared = prepare_ipl_rom(&raw_ipl).unwrap();
        assert_eq!(prepared[0x20], 0x5a);
        assert_eq!(prepared[0x100], 0x89);
        assert_eq!(prepared[0x1aff00], 0xa5);
        assert_eq!(prepared[0x1fcf00], 0x3c);

        let image = ResourceBlob::from_bytes(&dol_bytes(&[0x6000_0000]));
        let (mut board, _) = GameCubeBoard::new_with_ipl(image, Some(prepared)).unwrap();

        for (address, expected) in [
            (0x20u32, 0x5au8),
            (0x100, 0x89),
            (0x1aff00, 0xa5),
            (0x1fcf00, 0x3c),
        ] {
            board.write_mmio32(EXI_STATUS, 0);
            exi_select_ipl(&mut board);
            exi_imm_write(&mut board, exi_ipl_command(address, false), 4);
            assert_eq!(exi_imm_read(&mut board, 1), u32::from(expected) << 24);
        }
    }

    #[test]
    fn exi_ipl_rejects_nonstandard_dump_sizes() {
        let image = ResourceBlob::from_bytes(&dol_bytes(&[0x6000_0000]));
        let err = GameCubeMachine::from_image_and_ipl(image, Some(&[0; 16]))
            .err()
            .unwrap();
        assert!(err.contains("exactly 2097152 bytes"));
    }

    #[test]
    fn exi_ipl_sram_reads_writes_and_dma_match_channel_two_protocol() {
        let mut board = idle_board();
        exi_select_ipl(&mut board);
        exi_imm_write(&mut board, exi_ipl_command(EXI_IPL_SRAM_BASE, false), 4);
        assert_eq!(exi_imm_read(&mut board, 4), 0);
        assert_eq!(exi_imm_read(&mut board, 4), 0x002c_ffd0);

        board.write_mmio32(EXI_STATUS, 0);
        exi_select_ipl(&mut board);
        exi_imm_write(&mut board, exi_ipl_command(EXI_IPL_SRAM_BASE + 22, true), 4);
        exi_imm_write(&mut board, 0x0300_0000, 1);
        board.write_mmio32(EXI_STATUS, 0);
        exi_select_ipl(&mut board);
        exi_imm_write(
            &mut board,
            exi_ipl_command(EXI_IPL_SRAM_BASE + 22, false),
            4,
        );
        assert_eq!(exi_imm_read(&mut board, 1), 0x0300_0000);

        board.write_mmio32(EXI_STATUS, 0);
        exi_select_ipl(&mut board);
        exi_imm_write(
            &mut board,
            exi_ipl_command(EXI_IPL_SRAM_BASE + 24, false),
            4,
        );
        board.write_mmio32(EXI_DMA_ADDRESS, 0x1000);
        board.write_mmio32(EXI_DMA_LENGTH, 12);
        board.write_mmio32(EXI_DMA_CONTROL, 1 | 2);
        assert_eq!(&board.mem1[0x1000..0x100c], b"DOLPHINSLOTA");
    }

    #[test]
    fn exi_ipl_rtc_advances_deterministically_and_survives_reset() {
        let mut board = idle_board();
        board.exi.ipl.sram[0..4].copy_from_slice(&41u32.to_be_bytes());
        for _ in 0..FRAME_RATE {
            board.tick_exi_rtc();
        }
        assert_eq!(
            u32::from_be_bytes(board.exi.ipl.sram[0..4].try_into().unwrap()),
            42
        );
        let expected = board.exi.ipl.sram;
        board.reset().unwrap();
        assert_eq!(board.exi.ipl.sram, expected);
        assert_eq!(board.exi.ipl.rtc_phase, 0);
    }

    #[test]
    fn exi_memory_card_identifies_reports_status_and_routes_transfer_interrupt() {
        let mut board = idle_board();
        exi_select_card(&mut board);
        exi_imm_write(&mut board, 0, 1);
        assert_eq!(exi_imm_read(&mut board, 4), 0x8000_0000);
        assert_eq!(exi_imm_read(&mut board, 1), 0x0400_0000);

        exi_reselect_card(&mut board);
        board.write_mmio32(
            EXI_STATUS,
            (1 << 7) | EXI_STATUS_TCINTMASK | EXI_STATUS_TCINT,
        );
        assert_eq!(board.read_mmio32(EXI_STATUS) & EXI_STATUS_TCINT, 0);
        exi_imm_write(&mut board, 0x8300_0000, 1);
        assert_ne!(board.read_mmio32(EXI_STATUS) & EXI_STATUS_TCINT, 0);
        assert_ne!(board.mmio_read_u32_raw(PI_CAUSE) & PI_INT_EXI, 0);
        assert_eq!(exi_imm_read(&mut board, 1), 0xc100_0000);
    }

    #[test]
    fn exi_memory_card_dma_reads_programs_and_erases_guest_visible_storage() {
        let mut board = idle_board();
        board.memory_cards[0][0x120..0x128].copy_from_slice(b"OMNICORE");
        exi_select_card(&mut board);
        exi_imm_write(&mut board, 0x5200_0002, 4);
        exi_imm_write(&mut board, 0x2000_0000, 1);
        board.write_mmio32(EXI_DMA_ADDRESS, 0x1000);
        board.write_mmio32(EXI_DMA_LENGTH, 8);
        board.write_mmio32(EXI_DMA_CONTROL, 1 | 2);
        assert_eq!(&board.mem1[0x1000..0x1008], b"OMNICORE");

        board.mem1[0x2000..0x2008].copy_from_slice(b"GAMECUBE");
        exi_reselect_card(&mut board);
        exi_imm_write(&mut board, 0xf200_0003, 4);
        exi_imm_write(&mut board, 0, 1);
        board.write_mmio32(EXI_DMA_ADDRESS, 0x2000);
        board.write_mmio32(EXI_DMA_LENGTH, 8);
        board.write_mmio32(EXI_DMA_CONTROL, 1 | 2 | (1 << 2));
        assert_eq!(&board.memory_cards[0][0x180..0x188], b"GAMECUBE");

        board.memory_cards[0][0x2000..0x4000].fill(0x5a);
        exi_reselect_card(&mut board);
        exi_imm_write(&mut board, 0xf100_1000, 3);
        board.write_mmio32(EXI_STATUS, 0);
        assert!(board.memory_cards[0][0x2000..0x4000]
            .iter()
            .all(|byte| *byte == 0xff));
    }

    #[test]
    fn exi_slot_b_uses_channel_one_and_independent_memory_card() {
        let mut board = idle_board();
        let channel = 1;
        board.memory_cards[0][0x120..0x128].copy_from_slice(b"SLOTA123");
        board.memory_cards[1][0x120..0x128].copy_from_slice(b"SLOTB123");

        exi_reselect_card_channel(&mut board, channel);
        exi_imm_write_channel(&mut board, channel, 0x5200_0002, 4);
        exi_imm_write_channel(&mut board, channel, 0x2000_0000, 1);
        board.write_mmio32(exi_register(channel, EXI_DMA_ADDRESS), 0x1000);
        board.write_mmio32(exi_register(channel, EXI_DMA_LENGTH), 8);
        board.write_mmio32(exi_register(channel, EXI_DMA_CONTROL), 1 | 2);
        assert_eq!(&board.mem1[0x1000..0x1008], b"SLOTB123");
        assert_eq!(&board.memory_cards[0][0x120..0x128], b"SLOTA123");

        board.mem1[0x2000..0x2008].copy_from_slice(b"SECOND!!");
        exi_reselect_card_channel(&mut board, channel);
        exi_imm_write_channel(&mut board, channel, 0xf200_0003, 4);
        exi_imm_write_channel(&mut board, channel, 0, 1);
        board.write_mmio32(exi_register(channel, EXI_DMA_ADDRESS), 0x2000);
        board.write_mmio32(exi_register(channel, EXI_DMA_LENGTH), 8);
        board.write_mmio32(exi_register(channel, EXI_DMA_CONTROL), 1 | 2 | (1 << 2));
        assert_eq!(&board.memory_cards[1][0x180..0x188], b"SECOND!!");
        assert!(board.memory_cards[0][0x180..0x188]
            .iter()
            .all(|byte| *byte == 0xff));

        board.write_mmio32(
            exi_register(channel, EXI_STATUS),
            (1 << 7) | EXI_STATUS_TCINTMASK | EXI_STATUS_TCINT,
        );
        assert_eq!(
            board.read_mmio32(exi_register(channel, EXI_STATUS)) & EXI_STATUS_TCINT,
            0
        );
        exi_imm_write_channel(&mut board, channel, 0x8300_0000, 1);
        assert_ne!(
            board.read_mmio32(exi_register(channel, EXI_STATUS)) & EXI_STATUS_TCINT,
            0
        );
        assert_ne!(board.mmio_read_u32_raw(PI_CAUSE) & PI_INT_EXI, 0);
    }

    #[test]
    fn hle_disc_boot_recreates_bs2_disc_id_and_dtk_state() {
        let mut full = disc_bytes(&[0x4800_0000]);
        full[..6].copy_from_slice(b"GM8E01");
        full[8] = 1;
        full[9] = 0;
        let expected_id = full[..0x20].to_vec();
        let (mut board, _) = GameCubeBoard::new(ResourceBlob::from_bytes(&full)).unwrap();

        assert_eq!(&board.mem1[..0x20], expected_id.as_slice());
        assert_eq!(board.di.drive_state, DiDriveState::Ready);
        assert!(board.di.enable_dtk);
        assert_eq!(board.di.dtk_buffer_length, 10);

        board.reset().unwrap();
        assert_eq!(&board.mem1[..0x20], expected_id.as_slice());
        assert_eq!(board.di.drive_state, DiDriveState::Ready);
        assert!(board.di.enable_dtk);
        assert_eq!(board.di.dtk_buffer_length, 10);

        let (raw_board, _) =
            GameCubeBoard::new(ResourceBlob::from_bytes(&dol_bytes(&[0x4800_0000]))).unwrap();
        assert_eq!(raw_board.di.drive_state, DiDriveState::CoverOpened);
        assert!(!raw_board.di.enable_dtk);
    }

    #[test]
    fn dvd_dtk_position_advances_in_adpcm_blocks_and_promotes_queued_track() {
        let full = disc_bytes(&[0x4800_0000]);
        let (mut board, _) = GameCubeBoard::new(ResourceBlob::from_bytes(&full)).unwrap();
        board.di.enable_dtk = true;
        board.di.stream = true;
        board.di.current_start = 0x1000;
        board.di.current_length = 0x40;
        board.di.next_start = 0x2000;
        board.di.next_length = 0x40;
        board.di.audio_position = 0x1000;

        board.advance_dtk_stream(DTK_SAMPLES_PER_BLOCK - 1);
        assert_eq!(board.di.audio_position, 0x1000);
        assert_eq!(board.di.dtk_sample_remainder, 27);
        board.advance_dtk_stream(1);
        assert_eq!(board.di.audio_position, 0x1020);
        assert_eq!(board.di.dtk_sample_remainder, 0);

        board.advance_dtk_stream(DTK_SAMPLES_PER_BLOCK);
        assert_eq!(board.di.audio_position, 0x1040);
        board.advance_dtk_stream(DTK_SAMPLES_PER_BLOCK);
        assert_eq!(board.di.current_start, 0x2000);
        assert_eq!(board.di.audio_position, 0x2020);

        board.di.audio_position = 0x2040;
        board.di.stop_at_track_end = true;
        board.advance_dtk_stream(DTK_SAMPLES_PER_BLOCK);
        assert!(!board.di.stream);
        assert!(!board.di.stop_at_track_end);
        assert_eq!(board.di.audio_position, 0x2000);
        assert_eq!(board.di.dtk_sample_remainder, 0);
    }

    #[test]
    fn dvd_drive_requires_disc_id_and_reports_packed_error_state() {
        let full = disc_bytes(&[0x4800_0000]);
        let expected_id = full[..0x20].to_vec();
        let (mut board, _) = GameCubeBoard::new(ResourceBlob::from_bytes(&full)).unwrap();
        board.reset_mmio();

        run_di_command(&mut board, 0xa800_0000, 0, 4, 0x1000, 4);
        assert_ne!(board.mmio_read_u32_raw(DI_STATUS) & (1 << 2), 0);
        assert_eq!(board.di.error, DI_ERROR_NO_DISC_ID);
        assert_eq!(board.di.drive_state, DiDriveState::DiscIdNotRead);

        run_di_command(&mut board, 0xe000_0000, 0, 0, 0, 0);
        assert_eq!(board.mmio_read_u32_raw(DI_IMMEDIATE), 0x0502_0401);
        assert_eq!(board.di.error, DI_ERROR_NONE);
        assert_ne!(board.mmio_read_u32_raw(DI_STATUS) & (1 << 2), 0);
        assert_ne!(board.mmio_read_u32_raw(DI_STATUS) & (1 << 4), 0);

        clear_di_completion(&mut board);
        run_di_command(&mut board, 0xa800_0040, 0, 0, 0x1000, 0x20);
        assert_eq!(&board.mem1[0x1000..0x1020], expected_id.as_slice());
        assert_eq!(board.di.drive_state, DiDriveState::ReadyNoReadsMade);
        assert_eq!(board.mmio_read_u32_raw(DI_DMA_LENGTH), 0);
        assert_ne!(board.mmio_read_u32_raw(DI_STATUS) & (1 << 4), 0);

        clear_di_completion(&mut board);
        run_di_command(&mut board, 0xa800_0040, 0, 0, 0x1040, 0x20);
        assert_eq!(board.di.drive_state, DiDriveState::Ready);
    }

    #[test]
    fn dvd_dtk_configuration_stream_status_and_period_rules_are_stateful() {
        let full = disc_bytes(&[0x4800_0000]);
        let (mut board, _) = GameCubeBoard::new(ResourceBlob::from_bytes(&full)).unwrap();
        board.reset_mmio();

        run_di_command(&mut board, 0xe401_000a, 0, 0, 0, 0);
        assert_eq!(board.di.error, DI_ERROR_NO_DISC_ID);
        clear_di_completion(&mut board);

        run_di_command(&mut board, 0xa800_0040, 0, 0, 0x1000, 0x20);
        assert_eq!(board.di.drive_state, DiDriveState::ReadyNoReadsMade);
        clear_di_completion(&mut board);

        run_di_command(&mut board, 0xe401_000a, 0, 0, 0, 0);
        assert!(board.di.enable_dtk);
        assert_eq!(board.di.dtk_buffer_length, 10);
        clear_di_completion(&mut board);

        run_di_command(&mut board, 0xe100_0000, 0x800, 0x4000, 0, 0);
        assert!(board.di.stream);
        assert_eq!(board.di.current_start, 0x2000);
        assert_eq!(board.di.current_length, 0x4000);
        assert_eq!(board.di.audio_position, 0x2000);

        for (command, expected) in [
            (0xe200_0000, 1),
            (0xe201_0000, 0),
            (0xe202_0000, 0x800),
            (0xe203_0000, 0x4000),
        ] {
            clear_di_completion(&mut board);
            run_di_command(&mut board, command, 0, 0, 0, 0);
            assert_eq!(board.mmio_read_u32_raw(DI_IMMEDIATE), expected);
        }

        clear_di_completion(&mut board);
        run_di_command(&mut board, 0xe401_0000, 0, 0, 0, 0);
        assert_eq!(board.di.error, DI_ERROR_INVALID_PERIOD);
        run_di_command(&mut board, 0xe000_0000, 0, 0, 0, 0);
        assert_eq!(
            board.mmio_read_u32_raw(DI_IMMEDIATE),
            DI_ERROR_INVALID_PERIOD
        );

        clear_di_completion(&mut board);
        run_di_command(&mut board, 0xe300_0000, 0, 0, 0, 0);
        assert_eq!(board.di.drive_state, DiDriveState::MotorStopped);
        assert!(!board.di.stream);
        clear_di_completion(&mut board);
        run_di_command(&mut board, 0xe000_0000, 0, 0, 0, 0);
        assert_eq!(board.mmio_read_u32_raw(DI_IMMEDIATE), 0x0400_0000);
    }

    #[test]
    fn dvd_dma_requests_missing_range_then_resumes_after_hydration() {
        let mut full = disc_bytes(&[0x4800_0000]);
        let payload: Vec<u8> = (0..32u8).collect();
        full[0x1000..0x1020].copy_from_slice(&payload);
        let mut sparse = ResourceBlob::streaming(full.len() as u64, 2).unwrap();
        sparse.write(0, &full[..0x700]).unwrap();
        let (mut board, _) = GameCubeBoard::new(sparse).unwrap();
        board.di.drive_state = DiDriveState::Ready;
        board.write_di_status(1 << 3);
        board.mmio_write_u32_raw(DI_COMMAND_0, 0xa800_0000);
        board.mmio_write_u32_raw(DI_COMMAND_1, 0x1000 >> 2);
        board.mmio_write_u32_raw(DI_COMMAND_2, payload.len() as u32);
        board.mmio_write_u32_raw(DI_DMA_ADDRESS, 0x8000_1000);
        board.mmio_write_u32_raw(DI_DMA_LENGTH, payload.len() as u32);
        board.write_mmio32(DI_DMA_CONTROL, 1);
        assert_eq!(board.disc.pending_range(), Some((0x1000, 0x1020)));
        assert_ne!(board.mmio_read_u32_raw(DI_DMA_CONTROL) & 1, 0);

        let mut staged = board.disc.clone();
        staged.write(0x1000, &payload).unwrap();
        board.service_di();
        assert_eq!(&board.mem1[0x1000..0x1020], payload.as_slice());
        assert_eq!(board.mmio_read_u32_raw(DI_DMA_LENGTH), 0);
        assert_ne!(board.mmio_read_u32_raw(DI_STATUS) & (1 << 4), 0);
        assert_ne!(board.mmio_read_u32_raw(PI_CAUSE) & PI_INT_DI, 0);
    }

    #[test]
    fn audio_dma_and_yuyv_xfb_reach_public_outputs() {
        let mut board = idle_board();
        for frame in board.mem1[0x3000..0x3020].as_chunks_mut::<4>().0 {
            frame[..2].copy_from_slice(&0x4000i16.to_be_bytes());
            frame[2..].copy_from_slice(&(-0x4000i16).to_be_bytes());
        }
        board.write_mmio16(AUDIO_DMA_START_HI, 0);
        board.write_mmio16(AUDIO_DMA_START_LO, 0x3000);
        board.write_mmio16(AUDIO_DMA_CONTROL_LEN, 0x8001);
        let mut audio = AudioBuffer::new(AUDIO_RATE, 2);
        board.render_audio(&mut audio);
        assert_eq!(audio.samples().len(), 1600);
        assert_eq!(audio.samples()[0], 0.5);
        assert_eq!(audio.samples()[1], -0.5);
        assert_ne!(board.mmio_read_u16_raw(DSP_CONTROL) & (1 << 3), 0);

        board.mem1[0x4000..0x4004].copy_from_slice(&[235, 128, 235, 128]);
        board.write_mmio16(VI_FB_LEFT_TOP_HI, 0);
        board.write_mmio16(VI_FB_LEFT_TOP_LO, 0x4000);
        board.present_video();
        let pixel = &board.video.pixels()[..4];
        assert!(pixel[0] > 250 && pixel[1] > 250 && pixel[2] > 250);
        assert_eq!(pixel[3], 255);
    }

    #[test]
    fn aram_dma_copies_in_both_directions_and_interrupts() {
        let mut board = idle_board();
        let payload: Vec<u8> = (0..32u8).map(|value| value ^ 0x5a).collect();
        board.mem1[0x5000..0x5020].copy_from_slice(&payload);
        board.write_mmio16(DSP_CONTROL, 1 << 6);
        board.write_mmio16(AR_DMA_MMADDR_H, 0);
        board.write_mmio16(AR_DMA_MMADDR_L, 0x5000);
        board.write_mmio16(AR_DMA_ARADDR_H, 0);
        board.write_mmio16(AR_DMA_ARADDR_L, 0x1000);
        board.write_mmio16(AR_DMA_CNT_H, 0);
        board.write_mmio16(AR_DMA_CNT_L, 32);
        assert_eq!(&board.aram[0x1000..0x1020], payload.as_slice());
        assert_ne!(board.mmio_read_u32_raw(PI_CAUSE) & PI_INT_DSP, 0);

        board.aram[0x1000..0x1020].fill(0xa5);
        board.mem1[0x5000..0x5020].fill(0);
        board.write_mmio16(AR_DMA_CNT_H, 0x8000);
        board.write_mmio16(AR_DMA_CNT_L, 32);
        assert!(board.mem1[0x5000..0x5020].iter().all(|byte| *byte == 0xa5));
    }

    #[test]
    fn machine_state_and_memory_cards_round_trip_are_deterministic() {
        let image = ResourceBlob::from_bytes(&dol_bytes(&[0x4800_0000]));
        let mut machine = GameCubeMachine::from_image(image.clone()).unwrap();
        machine.cpu.gpr[7] = 0xfeed_beef;
        machine.board.mem1[0x6000..0x6004].copy_from_slice(&0x1234_5678u32.to_be_bytes());
        machine.board.exi.channel1.imm_data = 0x89ab_cdef;
        machine.board.exi.channel1.card.address = 0x2468;
        machine.board.di.drive_state = DiDriveState::Ready;
        machine.board.di.enable_dtk = true;
        machine.board.di.dtk_buffer_length = 10;
        machine.board.di.dtk_sample_remainder = 17;
        machine.board.di.stream = true;
        machine.board.di.audio_position = 0x1234_8000;
        machine.board.di.current_start = 0x1234_0000;
        machine.board.di.current_length = 0x4000;

        let card_a = vec![0x3c; MEMORY_CARD_SIZE];
        let card_b = vec![0xa7; MEMORY_CARD_SIZE];
        machine
            .write_persistent(ResourceKind::MemoryCard, 0, &card_a)
            .unwrap();
        machine
            .write_persistent(ResourceKind::MemoryCard, 1, &card_b)
            .unwrap();
        let mut sram = default_exi_sram();
        sram[0..4].copy_from_slice(&1234u32.to_be_bytes());
        sram[22] = 5;
        machine
            .write_persistent(ResourceKind::Storage, 0, &sram)
            .unwrap();
        let saved = machine.save_state().unwrap();

        let mut restored = GameCubeMachine::from_image(image).unwrap();
        restored.load_state(&saved).unwrap();
        assert_eq!(restored.cpu.gpr[7], 0xfeed_beef);
        assert_eq!(restored.board.read32(0x8000_6000), 0x1234_5678);
        assert_eq!(restored.board.exi.channel1.imm_data, 0x89ab_cdef);
        assert_eq!(restored.board.exi.channel1.card.address, 0x2468);
        assert_eq!(restored.board.di.drive_state, DiDriveState::Ready);
        assert!(restored.board.di.enable_dtk);
        assert_eq!(restored.board.di.dtk_buffer_length, 10);
        assert_eq!(restored.board.di.dtk_sample_remainder, 17);
        assert!(restored.board.di.stream);
        assert_eq!(restored.board.di.audio_position, 0x1234_8000);
        assert_eq!(restored.board.di.current_start, 0x1234_0000);
        assert_eq!(restored.board.di.current_length, 0x4000);

        let mut card_a_out = vec![0; MEMORY_CARD_SIZE];
        let mut card_b_out = vec![0; MEMORY_CARD_SIZE];
        restored
            .read_persistent(ResourceKind::MemoryCard, 0, &mut card_a_out)
            .unwrap();
        restored
            .read_persistent(ResourceKind::MemoryCard, 1, &mut card_b_out)
            .unwrap();
        assert_eq!(card_a_out, card_a);
        assert_eq!(card_b_out, card_b);

        let mut sram_out = [0u8; EXI_IPL_SRAM_SIZE];
        restored
            .read_persistent(ResourceKind::Storage, 0, &mut sram_out)
            .unwrap();
        assert_eq!(sram_out, sram);
        assert_eq!(
            restored.persistent_len(ResourceKind::MemoryCard, 1),
            MEMORY_CARD_SIZE
        );
        assert_eq!(
            restored.persistent_len(ResourceKind::Storage, 0),
            EXI_IPL_SRAM_SIZE
        );
        assert_eq!(restored.platform(), PlatformId::GameCube);
        assert_eq!(restored.frame_rate(), 60.0);
    }
}
