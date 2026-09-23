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

const STATE_VERSION: u32 = 12;
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
const GX_FIFO_START: usize = 0x8000;
const GX_FIFO_END: usize = 0x8020;
const GX_FIFO_BUFFER_LIMIT: usize = 16 * 1024 * 1024;
const GX_TMEM_SIZE: usize = 1024 * 1024;
const GX_TMEM_LINE_SIZE: usize = 32;
const GX_XF_REGISTER_COUNT: usize = 0x1058;
const GX_EFB_WIDTH: usize = 640;
const GX_EFB_HEIGHT: usize = 528;
const GX_EFB_COLOR_BYTES: usize = GX_EFB_WIDTH * GX_EFB_HEIGHT * 4;
const GX_EFB_DEPTH_BYTES: usize = GX_EFB_WIDTH * GX_EFB_HEIGHT * 3;
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
const EXI_IPL_UART_BASE: u32 = 0x0080_0400;
const EXI_IPL_UART_SIZE: u32 = 0x50;
const EXI_IPL_EUART_BASE: u32 = 0x00c0_0000;
const EXI_IPL_EUART_SIZE: u32 = 8;

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
    dtk_left_recent: i32,
    dtk_left_older: i32,
    dtk_right_recent: i32,
    dtk_right_older: i32,
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
struct ExiAd16State {
    command: u8,
    position: u32,
    register: [u8; 4],
}

#[derive(Clone, Default)]
struct ExiState {
    channel0: ExiChannelState,
    channel1: ExiChannelState,
    channel2: ExiChannelState,
    ipl: ExiIplState,
    ad16: ExiAd16State,
}

#[derive(Clone, Copy)]
struct GxVertex {
    position: [f32; 3],
    matrix_index: u8,
    color: [u8; 4],
    tex0: [f32; 2],
    has_tex0: bool,
}

#[derive(Clone, Copy)]
struct GxScreenVertex {
    x: f32,
    y: f32,
    z: f32,
    clip_x: f32,
    clip_y: f32,
    clip_w: f32,
    color: [u8; 4],
    tex0: [f32; 2],
    has_tex0: bool,
}

#[derive(Clone)]
struct GxState {
    cp_regs: [u32; 0x100],
    bp_regs: [u32; 0x100],
    xf_regs: Box<[u32]>,
    depth: Box<[u8]>,
    tmem: Box<[u8]>,
    fifo: Vec<u8>,
    commands_processed: u64,
    primitives_processed: u64,
    vertices_processed: u64,
}

impl Default for GxState {
    fn default() -> Self {
        Self {
            cp_regs: [0; 0x100],
            bp_regs: [0; 0x100],
            xf_regs: vec![0; GX_XF_REGISTER_COUNT].into_boxed_slice(),
            depth: vec![0xff; GX_EFB_DEPTH_BYTES].into_boxed_slice(),
            tmem: vec![0; GX_TMEM_SIZE].into_boxed_slice(),
            fifo: Vec::new(),
            commands_processed: 0,
            primitives_processed: 0,
            vertices_processed: 0,
        }
    }
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
    gx: GxState,
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
            gx: GxState::default(),
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
        self.gx = GxState::default();
        let ipl = self.exi.ipl.clone();
        self.exi = ExiState::default();
        self.exi.ipl = ipl;
        self.exi.channel0.status = EXI_STATUS_EXTINT | EXI_STATUS_EXT;
        self.exi.channel1.status = EXI_STATUS_EXTINT | EXI_STATUS_EXT | (1 << 7);
        self.exi.channel2.status = EXI_STATUS_EXTINT | EXI_STATUS_EXT;
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
        } else if (EXI_IPL_UART_BASE..EXI_IPL_UART_BASE + EXI_IPL_UART_SIZE).contains(&address) {
            match address - EXI_IPL_UART_BASE {
                0 => {
                    if write {
                        0xff
                    } else {
                        0
                    }
                }
                0x0c | 0x4c => {
                    if write {
                        0xff
                    } else {
                        0
                    }
                }
                _ => 0xff,
            }
        } else if (EXI_IPL_EUART_BASE..EXI_IPL_EUART_BASE + EXI_IPL_EUART_SIZE).contains(&address) {
            match address - EXI_IPL_EUART_BASE {
                0 => 0xff,
                4 => {
                    if write {
                        0xff
                    } else {
                        0
                    }
                }
                _ => 0xff,
            }
        } else {
            0xff
        }
    }

    fn exi_ad16_select(&mut self) {
        self.exi.ad16.command = 0;
        self.exi.ad16.position = 0;
    }

    fn exi_ad16_transfer_byte(&mut self, input: u8) -> u8 {
        let position = self.exi.ad16.position;
        let mut output = 0xff;
        if position == 0 {
            self.exi.ad16.command = input;
        } else {
            match self.exi.ad16.command {
                0x00 => {
                    self.exi.ad16.register = [0x04, 0x12, 0x00, 0x00];
                    if (2..=5).contains(&position) {
                        output = self.exi.ad16.register[(position - 2) as usize];
                    }
                }
                0xa0 if position <= 4 => {
                    self.exi.ad16.register[(position - 1) as usize] = input;
                }
                0xa2 if position <= 4 => {
                    output = self.exi.ad16.register[(position - 1) as usize];
                }
                _ => {}
            }
        }
        self.exi.ad16.position = position.wrapping_add(1);
        output
    }

    fn exi_channel(&self, channel: usize) -> &ExiChannelState {
        match channel {
            0 => &self.exi.channel0,
            1 => &self.exi.channel1,
            2 => &self.exi.channel2,
            _ => unreachable!("GameCube EXI channel {channel} is not modeled"),
        }
    }

    fn exi_channel_mut(&mut self, channel: usize) -> &mut ExiChannelState {
        match channel {
            0 => &mut self.exi.channel0,
            1 => &mut self.exi.channel1,
            2 => &mut self.exi.channel2,
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
            (2, 1) => self.exi_ad16_transfer_byte(input),
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
        let active = [
            self.exi.channel0.status,
            self.exi.channel1.status,
            self.exi.channel2.status,
        ]
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
        if channel == 2 && previous_select != 1 && selected == 1 {
            self.exi_ad16_select();
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
        if channel < 2 && self.exi_chip_select(channel) == 1 && card_command == 0x52 {
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
        if channel < 2 && self.exi_chip_select(channel) == 1 && card_command == 0xf2 {
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

    fn gx_component_size(format: u32) -> usize {
        match format & 7 {
            0 | 1 => 1,
            2 | 3 => 2,
            _ => 4,
        }
    }

    fn gx_direct_component_size(format: u32, components: usize) -> usize {
        Self::gx_component_size(format) * components
    }

    fn gx_decode_scalar(
        data: &[u8],
        format: u32,
        fraction: u32,
        byte_dequant: bool,
    ) -> Option<(f32, usize)> {
        let scale = 1.0 / ((1u64 << fraction.min(31)) as f32);
        match format & 7 {
            0 => data.first().map(|value| {
                (
                    f32::from(*value) * if byte_dequant { scale } else { 1.0 },
                    1,
                )
            }),
            1 => data.first().map(|value| {
                (
                    f32::from(*value as i8) * if byte_dequant { scale } else { 1.0 },
                    1,
                )
            }),
            2 => data.get(..2).map(|bytes| {
                (
                    f32::from(u16::from_be_bytes(bytes.try_into().unwrap())) * scale,
                    2,
                )
            }),
            3 => data.get(..2).map(|bytes| {
                (
                    f32::from(i16::from_be_bytes(bytes.try_into().unwrap())) * scale,
                    2,
                )
            }),
            _ => data.get(..4).map(|bytes| {
                (
                    f32::from_bits(u32::from_be_bytes(bytes.try_into().unwrap())),
                    4,
                )
            }),
        }
    }

    fn gx_decode_position_data(
        data: &[u8],
        format: u32,
        fraction: u32,
        byte_dequant: bool,
        components: usize,
    ) -> Option<[f32; 3]> {
        let mut offset = 0usize;
        let mut position = [0.0f32; 3];
        for value in position.iter_mut().take(components) {
            let (decoded, bytes) =
                Self::gx_decode_scalar(data.get(offset..)?, format, fraction, byte_dequant)?;
            *value = decoded;
            offset += bytes;
        }
        position
            .iter()
            .all(|value| value.is_finite())
            .then_some(position)
    }

    fn gx_expand_4(value: u8) -> u8 {
        (value << 4) | value
    }

    fn gx_expand_5(value: u8) -> u8 {
        (value << 3) | (value >> 2)
    }

    fn gx_expand_6(value: u8) -> u8 {
        (value << 2) | (value >> 4)
    }

    fn gx_decode_color_data(data: &[u8], format: u32) -> Option<[u8; 4]> {
        match format & 7 {
            0 => {
                let value = u16::from_be_bytes(data.get(..2)?.try_into().unwrap());
                Some([
                    Self::gx_expand_5(((value >> 11) & 0x1f) as u8),
                    Self::gx_expand_6(((value >> 5) & 0x3f) as u8),
                    Self::gx_expand_5((value & 0x1f) as u8),
                    0xff,
                ])
            }
            1 => Some([*data.first()?, *data.get(1)?, *data.get(2)?, 0xff]),
            2 => Some([*data.first()?, *data.get(1)?, *data.get(2)?, 0xff]),
            3 => {
                let value = u16::from_be_bytes(data.get(..2)?.try_into().unwrap());
                Some([
                    Self::gx_expand_4(((value >> 12) & 0xf) as u8),
                    Self::gx_expand_4(((value >> 8) & 0xf) as u8),
                    Self::gx_expand_4(((value >> 4) & 0xf) as u8),
                    Self::gx_expand_4((value & 0xf) as u8),
                ])
            }
            4 => {
                let bytes = data.get(..3)?;
                let value =
                    (u32::from(bytes[0]) << 16) | (u32::from(bytes[1]) << 8) | u32::from(bytes[2]);
                Some([
                    Self::gx_expand_6(((value >> 18) & 0x3f) as u8),
                    Self::gx_expand_6(((value >> 12) & 0x3f) as u8),
                    Self::gx_expand_6(((value >> 6) & 0x3f) as u8),
                    Self::gx_expand_6((value & 0x3f) as u8),
                ])
            }
            _ => Some([*data.first()?, *data.get(1)?, *data.get(2)?, *data.get(3)?]),
        }
    }

    fn gx_stream_index(data: &[u8], offset: &mut usize, descriptor: u32) -> Option<u32> {
        match descriptor & 3 {
            2 => {
                let value = u32::from(*data.get(*offset)?);
                *offset += 1;
                Some(value)
            }
            3 => {
                let end = offset.checked_add(2)?;
                let value = u32::from(u16::from_be_bytes(
                    data.get(*offset..end)?.try_into().unwrap(),
                ));
                *offset = end;
                Some(value)
            }
            _ => None,
        }
    }

    fn gx_array_data(&self, array: usize, index: u32, len: usize) -> Option<&[u8]> {
        let base = *self.gx.cp_regs.get(0xa0 + array)?;
        let stride = *self.gx.cp_regs.get(0xb0 + array)?;
        let address = base.checked_add(index.checked_mul(stride)?)?;
        let range = memory_range(address, len).ok()?;
        self.mem1.get(range)
    }

    fn gx_decode_vertex(&self, vat: usize, data: &[u8]) -> Option<GxVertex> {
        let vcd_lo = self.gx.cp_regs[0x50];
        let vat_a = self.gx.cp_regs[0x70 + (vat & 7)];
        let matrix_index = if vcd_lo & 1 != 0 {
            *data.first()? & 0x3f
        } else {
            (self.gx.cp_regs[0x30] & 0x3f) as u8
        };
        let mut offset = (vcd_lo & 0x1ff).count_ones() as usize;

        let position_descriptor = (vcd_lo >> 9) & 3;
        let position_components = if vat_a & 1 != 0 { 3 } else { 2 };
        let position_format = (vat_a >> 1) & 7;
        let position_fraction = (vat_a >> 4) & 0x1f;
        let byte_dequant = vat_a & (1 << 30) != 0;
        let position_size = Self::gx_direct_component_size(position_format, position_components);
        let position = match position_descriptor {
            1 => {
                let end = offset.checked_add(position_size)?;
                let value = Self::gx_decode_position_data(
                    data.get(offset..end)?,
                    position_format,
                    position_fraction,
                    byte_dequant,
                    position_components,
                )?;
                offset = end;
                value
            }
            2 | 3 => {
                let index = Self::gx_stream_index(data, &mut offset, position_descriptor)?;
                let source = self.gx_array_data(0, index, position_size)?;
                Self::gx_decode_position_data(
                    source,
                    position_format,
                    position_fraction,
                    byte_dequant,
                    position_components,
                )?
            }
            _ => return None,
        };

        let normal_descriptor = (vcd_lo >> 11) & 3;
        let normal_components = if vat_a & (1 << 9) != 0 { 9 } else { 3 };
        let normal_direct = Self::gx_direct_component_size((vat_a >> 10) & 7, normal_components);
        let normal_size = match normal_descriptor {
            0 => 0,
            1 => normal_direct,
            2 | 3 => {
                let index_size = if normal_descriptor == 2 { 1 } else { 2 };
                if normal_components == 9 && vat_a & (1 << 31) != 0 {
                    index_size * 3
                } else {
                    index_size
                }
            }
            _ => unreachable!(),
        };
        offset = offset.checked_add(normal_size)?;
        if offset > data.len() {
            return None;
        }

        let color_descriptor = (vcd_lo >> 13) & 3;
        let color_format = (vat_a >> 14) & 7;
        let color_size = match color_format {
            0 | 3 => 2,
            1 | 4 => 3,
            _ => 4,
        };
        let color = match color_descriptor {
            0 => [0xff; 4],
            1 => {
                let end = offset.checked_add(color_size)?;
                let value = Self::gx_decode_color_data(data.get(offset..end)?, color_format)?;
                offset = end;
                value
            }
            2 | 3 => {
                let index = Self::gx_stream_index(data, &mut offset, color_descriptor)?;
                let source = self.gx_array_data(2, index, color_size)?;
                Self::gx_decode_color_data(source, color_format)?
            }
            _ => unreachable!(),
        };

        let color1_descriptor = (vcd_lo >> 15) & 3;
        let color1_format = (vat_a >> 18) & 7;
        let color1_size = match color1_format {
            0 | 3 => 2,
            1 | 4 => 3,
            _ => 4,
        };
        match color1_descriptor {
            0 => {}
            1 => offset = offset.checked_add(color1_size)?,
            2 | 3 => {
                Self::gx_stream_index(data, &mut offset, color1_descriptor)?;
            }
            _ => unreachable!(),
        }
        if offset > data.len() {
            return None;
        }

        let vcd_hi = self.gx.cp_regs[0x60];
        let tex0_descriptor = vcd_hi & 3;
        let tex0_components = if vat_a & (1 << 21) != 0 { 2 } else { 1 };
        let tex0_format = (vat_a >> 22) & 7;
        let tex0_fraction = (vat_a >> 25) & 0x1f;
        let tex0_size = Self::gx_direct_component_size(tex0_format, tex0_components);
        let tex0 = match tex0_descriptor {
            0 => [0.0, 0.0],
            1 => {
                let end = offset.checked_add(tex0_size)?;
                let decoded = Self::gx_decode_position_data(
                    data.get(offset..end)?,
                    tex0_format,
                    tex0_fraction,
                    byte_dequant,
                    tex0_components,
                )?;
                [decoded[0], decoded[1]]
            }
            2 | 3 => {
                let index = Self::gx_stream_index(data, &mut offset, tex0_descriptor)?;
                let source = self.gx_array_data(4, index, tex0_size)?;
                let decoded = Self::gx_decode_position_data(
                    source,
                    tex0_format,
                    tex0_fraction,
                    byte_dequant,
                    tex0_components,
                )?;
                [decoded[0], decoded[1]]
            }
            _ => unreachable!(),
        };

        Some(GxVertex {
            position,
            matrix_index,
            color,
            tex0,
            has_tex0: tex0_descriptor != 0,
        })
    }

    fn gx_xf_f32(&self, index: usize) -> f32 {
        f32::from_bits(self.gx.xf_regs[index])
    }

    fn gx_transform_vertex(&self, vertex: GxVertex) -> Option<GxScreenVertex> {
        let matrix_base = usize::from(vertex.matrix_index).checked_mul(4)?;
        if matrix_base.checked_add(11)? >= 0x100 {
            return None;
        }
        let [x, y, z] = vertex.position;
        let transform_row = |row: usize| {
            let base = matrix_base + row * 4;
            self.gx_xf_f32(base) * x
                + self.gx_xf_f32(base + 1) * y
                + self.gx_xf_f32(base + 2) * z
                + self.gx_xf_f32(base + 3)
        };
        let view_x = transform_row(0);
        let view_y = transform_row(1);
        let view_z = transform_row(2);

        let projection = [
            self.gx_xf_f32(0x1020),
            self.gx_xf_f32(0x1021),
            self.gx_xf_f32(0x1022),
            self.gx_xf_f32(0x1023),
            self.gx_xf_f32(0x1024),
            self.gx_xf_f32(0x1025),
        ];
        let projection_type = self.gx.xf_regs[0x1026] & 1;
        let (clip_x, clip_y, clip_z, clip_w) = if projection_type == 0 {
            (
                projection[0] * view_x + projection[1] * view_z,
                projection[2] * view_y + projection[3] * view_z,
                projection[4] * view_z + projection[5],
                -view_z,
            )
        } else {
            (
                projection[0] * view_x + projection[1],
                projection[2] * view_y + projection[3],
                projection[4] * view_z + projection[5],
                1.0,
            )
        };
        if ![clip_x, clip_y, clip_z, clip_w]
            .into_iter()
            .all(f32::is_finite)
            || clip_w.abs() < f32::EPSILON
        {
            return None;
        }

        let scissor_offset = self.gx.bp_regs[0x59];
        let viewport_offset_x = ((scissor_offset & 0x1ff) << 1) as f32;
        let viewport_offset_y = (((scissor_offset >> 10) & 0x1ff) << 1) as f32;
        let inv_w = clip_w.recip();
        let screen_x =
            clip_x * inv_w * self.gx_xf_f32(0x101a) + self.gx_xf_f32(0x101d) - viewport_offset_x;
        let screen_y =
            clip_y * inv_w * self.gx_xf_f32(0x101b) + self.gx_xf_f32(0x101e) - viewport_offset_y;
        let screen_z = clip_z * inv_w * self.gx_xf_f32(0x101c) + self.gx_xf_f32(0x101f);
        if !screen_x.is_finite() || !screen_y.is_finite() || !screen_z.is_finite() {
            return None;
        }
        Some(GxScreenVertex {
            x: screen_x,
            y: screen_y,
            z: screen_z,
            clip_x,
            clip_y,
            clip_w,
            color: vertex.color,
            tex0: vertex.tex0,
            has_tex0: vertex.has_tex0,
        })
    }

    fn gx_scissor_bounds(&self) -> (i32, i32, i32, i32) {
        let top_left = self.gx.bp_regs[0x20];
        let bottom_right = self.gx.bp_regs[0x21];
        let offset = self.gx.bp_regs[0x59];
        let offset_x = ((offset & 0x1ff) << 1) as i32;
        let offset_y = (((offset >> 10) & 0x1ff) << 1) as i32;
        let left = (top_left & 0x3ff) as i32 - offset_x;
        let top = ((top_left >> 10) & 0x3ff) as i32 - offset_y;
        let right = (bottom_right & 0x3ff) as i32 - offset_x;
        let bottom = ((bottom_right >> 10) & 0x3ff) as i32 - offset_y;
        (
            left.clamp(0, (GX_EFB_WIDTH - 1) as i32),
            right.clamp(-1, (GX_EFB_WIDTH - 1) as i32),
            top.clamp(0, (GX_EFB_HEIGHT - 1) as i32),
            bottom.clamp(-1, (GX_EFB_HEIGHT - 1) as i32),
        )
    }

    fn gx_efb_pixel(&self, x: usize, y: usize) -> [u8; 4] {
        let offset = (y * GX_EFB_WIDTH + x) * 4;
        self.efb[offset..offset + 4].try_into().unwrap()
    }

    fn gx_write_efb_pixel(&mut self, x: i32, y: i32, color: [u8; 4]) {
        if x < 0 || y < 0 || x >= GX_EFB_WIDTH as i32 || y >= GX_EFB_HEIGHT as i32 {
            return;
        }
        let offset = (y as usize * GX_EFB_WIDTH + x as usize) * 4;
        if offset + 4 <= GX_EFB_COLOR_BYTES {
            self.efb[offset..offset + 4].copy_from_slice(&color);
        }
    }

    fn gx_efb_depth(&self, x: usize, y: usize) -> u32 {
        let offset = (y * GX_EFB_WIDTH + x) * 3;
        (u32::from(self.gx.depth[offset]) << 16)
            | (u32::from(self.gx.depth[offset + 1]) << 8)
            | u32::from(self.gx.depth[offset + 2])
    }

    fn gx_write_efb_depth(&mut self, x: usize, y: usize, depth: u32) {
        let offset = (y * GX_EFB_WIDTH + x) * 3;
        self.gx.depth[offset] = ((depth >> 16) & 0xff) as u8;
        self.gx.depth[offset + 1] = ((depth >> 8) & 0xff) as u8;
        self.gx.depth[offset + 2] = (depth & 0xff) as u8;
    }

    fn gx_depth_compare(function: u32, source: u32, destination: u32) -> bool {
        match function & 7 {
            0 => false,
            1 => source < destination,
            2 => source == destination,
            3 => source <= destination,
            4 => source > destination,
            5 => source != destination,
            6 => source >= destination,
            _ => true,
        }
    }

    fn gx_wrap_texture_texel(mut texel: i32, size: usize, mode: u32) -> usize {
        if size == 0 {
            return 0;
        }
        let size = size as i32;
        match mode & 3 {
            1 => (texel & (size - 1)) as usize,
            2 => {
                if texel & size != 0 {
                    texel = !texel;
                }
                (texel & (size - 1)) as usize
            }
            _ => texel.clamp(0, size - 1) as usize,
        }
    }

    fn gx_texture_bytes(&self, address: u32, offset: usize, len: usize) -> Option<&[u8]> {
        let address = address.checked_add(u32::try_from(offset).ok()?)?;
        let range = memory_range(address, len).ok()?;
        self.mem1.get(range)
    }

    fn gx_decode_rgb565(value: u16) -> [u8; 4] {
        [
            Self::gx_expand_5(((value >> 11) & 0x1f) as u8),
            Self::gx_expand_6(((value >> 5) & 0x3f) as u8),
            Self::gx_expand_5((value & 0x1f) as u8),
            0xff,
        ]
    }

    fn gx_decode_rgb5a3(value: u16) -> [u8; 4] {
        if value & 0x8000 != 0 {
            [
                Self::gx_expand_5(((value >> 10) & 0x1f) as u8),
                Self::gx_expand_5(((value >> 5) & 0x1f) as u8),
                Self::gx_expand_5((value & 0x1f) as u8),
                0xff,
            ]
        } else {
            let expand_3 = |component: u8| (component << 5) | (component << 2) | (component >> 1);
            [
                Self::gx_expand_4(((value >> 8) & 0xf) as u8),
                Self::gx_expand_4(((value >> 4) & 0xf) as u8),
                Self::gx_expand_4((value & 0xf) as u8),
                expand_3(((value >> 12) & 7) as u8),
            ]
        }
    }

    fn gx_cmpr_blend(first: u8, second: u8) -> u8 {
        ((u16::from(first) * 5 + u16::from(second) * 3) >> 3) as u8
    }

    fn gx_tlut_texel(&self, index: usize) -> Option<[u8; 4]> {
        let tlut = self.gx.bp_regs[0x98];
        let base = ((tlut & 0x3ff) as usize) << 9;
        let format = (tlut >> 10) & 3;
        let offset = base.checked_add(index.checked_mul(2)?)?;
        let bytes = self.gx.tmem.get(offset..offset + 2)?;
        match format {
            0 => Some([bytes[1], bytes[1], bytes[1], bytes[0]]),
            1 => Some(Self::gx_decode_rgb565(u16::from_be_bytes([
                bytes[0], bytes[1],
            ]))),
            2 => Some(Self::gx_decode_rgb5a3(u16::from_be_bytes([
                bytes[0], bytes[1],
            ]))),
            _ => None,
        }
    }

    fn gx_texture0_texel(&self, s: usize, t: usize) -> Option<[u8; 4]> {
        let image = self.gx.bp_regs[0x88];
        let width = (image & 0x3ff) as usize + 1;
        let height = ((image >> 10) & 0x3ff) as usize + 1;
        if s >= width || t >= height {
            return None;
        }
        let format = (image >> 20) & 0xf;
        let address = (self.gx.bp_regs[0x94] & 0x00ff_ffff) << 5;
        let (block_width, block_height, block_bytes) = match format {
            0 => (8usize, 8usize, 32usize),
            1 | 2 => (8, 4, 32),
            3..=5 => (4, 4, 32),
            6 => (4, 4, 64),
            8 => (8, 8, 32),
            9 => (8, 4, 32),
            10 => (4, 4, 32),
            14 => (8, 8, 32),
            _ => return None,
        };
        let blocks_per_row = width.div_ceil(block_width);
        let block = (t / block_height)
            .checked_mul(blocks_per_row)?
            .checked_add(s / block_width)?;
        let block_base = block.checked_mul(block_bytes)?;
        let local_x = s % block_width;
        let local_y = t % block_height;

        match format {
            0 => {
                let texel = local_y * 8 + local_x;
                let byte = *self
                    .gx_texture_bytes(address, block_base + texel / 2, 1)?
                    .first()?;
                let intensity = if texel & 1 == 0 {
                    byte >> 4
                } else {
                    byte & 0xf
                };
                let intensity = Self::gx_expand_4(intensity);
                Some([intensity, intensity, intensity, intensity])
            }
            1 => {
                let intensity = *self
                    .gx_texture_bytes(address, block_base + local_y * 8 + local_x, 1)?
                    .first()?;
                Some([intensity, intensity, intensity, intensity])
            }
            2 => {
                let value = *self
                    .gx_texture_bytes(address, block_base + local_y * 8 + local_x, 1)?
                    .first()?;
                let alpha = Self::gx_expand_4(value >> 4);
                let intensity = Self::gx_expand_4(value & 0xf);
                Some([intensity, intensity, intensity, alpha])
            }
            3 => {
                let offset = block_base + (local_y * 4 + local_x) * 2;
                let bytes = self.gx_texture_bytes(address, offset, 2)?;
                let alpha = bytes[0];
                let intensity = bytes[1];
                Some([intensity, intensity, intensity, alpha])
            }
            4 => {
                let offset = block_base + (local_y * 4 + local_x) * 2;
                let value =
                    u16::from_be_bytes(self.gx_texture_bytes(address, offset, 2)?.try_into().ok()?);
                Some(Self::gx_decode_rgb565(value))
            }
            5 => {
                let offset = block_base + (local_y * 4 + local_x) * 2;
                let value =
                    u16::from_be_bytes(self.gx_texture_bytes(address, offset, 2)?.try_into().ok()?);
                Some(Self::gx_decode_rgb5a3(value))
            }
            6 => {
                let texel = (local_y * 4 + local_x) * 2;
                let ar = self.gx_texture_bytes(address, block_base + texel, 2)?;
                let gb = self.gx_texture_bytes(address, block_base + 32 + texel, 2)?;
                Some([ar[1], gb[0], gb[1], ar[0]])
            }
            8 => {
                let texel = local_y * 8 + local_x;
                let byte = *self
                    .gx_texture_bytes(address, block_base + texel / 2, 1)?
                    .first()?;
                let index = if texel & 1 == 0 {
                    byte >> 4
                } else {
                    byte & 0xf
                };
                self.gx_tlut_texel(usize::from(index))
            }
            9 => {
                let index = *self
                    .gx_texture_bytes(address, block_base + local_y * 8 + local_x, 1)?
                    .first()?;
                self.gx_tlut_texel(usize::from(index))
            }
            10 => {
                let offset = block_base + (local_y * 4 + local_x) * 2;
                let index =
                    u16::from_be_bytes(self.gx_texture_bytes(address, offset, 2)?.try_into().ok()?)
                        & 0x3fff;
                self.gx_tlut_texel(usize::from(index))
            }
            14 => {
                let sub_block = (local_y / 4) * 2 + local_x / 4;
                let sub_base = block_base + sub_block * 8;
                let bytes = self.gx_texture_bytes(address, sub_base, 8)?;
                let color0 = u16::from_be_bytes([bytes[0], bytes[1]]);
                let color1 = u16::from_be_bytes([bytes[2], bytes[3]]);
                let first = Self::gx_decode_rgb565(color0);
                let second = Self::gx_decode_rgb565(color1);
                let row = bytes[4 + local_y % 4];
                let selector = (row >> (6 - 2 * (local_x % 4))) & 3;
                match selector {
                    0 => Some(first),
                    1 => Some(second),
                    2 if color0 > color1 => Some([
                        Self::gx_cmpr_blend(first[0], second[0]),
                        Self::gx_cmpr_blend(first[1], second[1]),
                        Self::gx_cmpr_blend(first[2], second[2]),
                        0xff,
                    ]),
                    3 if color0 > color1 => Some([
                        Self::gx_cmpr_blend(second[0], first[0]),
                        Self::gx_cmpr_blend(second[1], first[1]),
                        Self::gx_cmpr_blend(second[2], first[2]),
                        0xff,
                    ]),
                    2 => Some([
                        ((u16::from(first[0]) + u16::from(second[0])) / 2) as u8,
                        ((u16::from(first[1]) + u16::from(second[1])) / 2) as u8,
                        ((u16::from(first[2]) + u16::from(second[2])) / 2) as u8,
                        0xff,
                    ]),
                    _ => Some([
                        ((u16::from(first[0]) + u16::from(second[0])) / 2) as u8,
                        ((u16::from(first[1]) + u16::from(second[1])) / 2) as u8,
                        ((u16::from(first[2]) + u16::from(second[2])) / 2) as u8,
                        0,
                    ]),
                }
            }
            _ => None,
        }
    }

    fn gx_texture0_sample(&self, coordinate: [f32; 2], linear: bool) -> Option<[u8; 4]> {
        if !coordinate.iter().all(|value| value.is_finite()) {
            return None;
        }
        let image = self.gx.bp_regs[0x88];
        let width = (image & 0x3ff) as usize + 1;
        let height = ((image >> 10) & 0x3ff) as usize + 1;
        let mode = self.gx.bp_regs[0x80];
        let fixed_s = (coordinate[0] * width as f32 * 128.0) as i32;
        let fixed_t = (coordinate[1] * height as f32 * 128.0) as i32;

        if !linear {
            let s = Self::gx_wrap_texture_texel(fixed_s >> 7, width, mode & 3);
            let t = Self::gx_wrap_texture_texel(fixed_t >> 7, height, (mode >> 2) & 3);
            return self.gx_texture0_texel(s, t);
        }

        let fixed_s = fixed_s - 64;
        let fixed_t = fixed_t - 64;
        let s0 = fixed_s >> 7;
        let t0 = fixed_t >> 7;
        let s1 = s0 + 1;
        let t1 = t0 + 1;
        let fract_s = (fixed_s & 0x7f) as u32;
        let fract_t = (fixed_t & 0x7f) as u32;
        let s0 = Self::gx_wrap_texture_texel(s0, width, mode & 3);
        let s1 = Self::gx_wrap_texture_texel(s1, width, mode & 3);
        let t0 = Self::gx_wrap_texture_texel(t0, height, (mode >> 2) & 3);
        let t1 = Self::gx_wrap_texture_texel(t1, height, (mode >> 2) & 3);
        let samples = [
            self.gx_texture0_texel(s0, t0)?,
            self.gx_texture0_texel(s1, t0)?,
            self.gx_texture0_texel(s0, t1)?,
            self.gx_texture0_texel(s1, t1)?,
        ];
        let weights = [
            (128 - fract_s) * (128 - fract_t),
            fract_s * (128 - fract_t),
            (128 - fract_s) * fract_t,
            fract_s * fract_t,
        ];
        let mut output = [0u8; 4];
        for channel in 0..4 {
            let value = samples
                .iter()
                .zip(weights)
                .map(|(sample, weight)| u32::from(sample[channel]) * weight)
                .sum::<u32>();
            output[channel] = (value >> 14) as u8;
        }
        Some(output)
    }

    fn gx_modulate_texture(color: [u8; 4], texel: [u8; 4]) -> [u8; 4] {
        let mut output = [0u8; 4];
        for channel in 0..4 {
            output[channel] =
                ((u16::from(color[channel]) * u16::from(texel[channel]) + 127) / 255) as u8;
        }
        output
    }

    fn gx_source_factor(mode: u32, channel: usize, source: [u8; 4], destination: [u8; 4]) -> u16 {
        let factor = match mode & 7 {
            0 => 0,
            1 => 0xff,
            2 => destination[channel],
            3 => 0xff - destination[channel],
            4 => source[3],
            5 => 0xff - source[3],
            6 => destination[3],
            _ => 0xff - destination[3],
        };
        let factor = u16::from(factor);
        factor + (factor >> 7)
    }

    fn gx_destination_factor(
        mode: u32,
        channel: usize,
        source: [u8; 4],
        destination: [u8; 4],
    ) -> u16 {
        let factor = match mode & 7 {
            0 => 0,
            1 => 0xff,
            2 => source[channel],
            3 => 0xff - source[channel],
            4 => source[3],
            5 => 0xff - source[3],
            6 => destination[3],
            _ => 0xff - destination[3],
        };
        let factor = u16::from(factor);
        factor + (factor >> 7)
    }

    fn gx_logic_channel(operation: u32, source: u8, destination: u8) -> u8 {
        match operation & 0xf {
            0 => 0,
            1 => source & destination,
            2 => source & !destination,
            3 => source,
            4 => !source & destination,
            5 => destination,
            6 => source ^ destination,
            7 => source | destination,
            8 => !(source | destination),
            9 => !(source ^ destination),
            10 => !destination,
            11 => source | !destination,
            12 => !source,
            13 => !source | destination,
            14 => !(source & destination),
            _ => 0xff,
        }
    }

    fn gx_write_fragment(&mut self, x: i32, y: i32, z: f32, source: [u8; 4]) {
        if x < 0 || y < 0 || x >= GX_EFB_WIDTH as i32 || y >= GX_EFB_HEIGHT as i32 {
            return;
        }
        let x = x as usize;
        let y = y as usize;
        let depth = z.round().clamp(0.0, 16_777_215.0) as u32;
        let z_mode = self.gx.bp_regs[0x40];
        let destination_depth = self.gx_efb_depth(x, y);
        if z_mode & 1 != 0 && !Self::gx_depth_compare((z_mode >> 1) & 7, depth, destination_depth) {
            return;
        }
        if z_mode & (1 << 4) != 0 {
            self.gx_write_efb_depth(x, y, depth);
        }

        let blend_mode = self.gx.bp_regs[0x41];
        let destination = self.gx_efb_pixel(x, y);
        let mut result = source;
        if blend_mode & 1 != 0 {
            if blend_mode & (1 << 11) != 0 {
                for channel in 0..4 {
                    result[channel] = destination[channel].saturating_sub(source[channel]);
                }
            } else {
                let destination_mode = (blend_mode >> 5) & 7;
                let source_mode = (blend_mode >> 8) & 7;
                for channel in 0..4 {
                    let source_factor =
                        Self::gx_source_factor(source_mode, channel, source, destination);
                    let destination_factor =
                        Self::gx_destination_factor(destination_mode, channel, source, destination);
                    let value = (u32::from(source[channel]) * u32::from(source_factor)
                        + u32::from(destination[channel]) * u32::from(destination_factor))
                        >> 8;
                    result[channel] = value.min(255) as u8;
                }
            }
        } else if blend_mode & (1 << 1) != 0 {
            let operation = (blend_mode >> 12) & 0xf;
            for channel in 0..4 {
                result[channel] =
                    Self::gx_logic_channel(operation, source[channel], destination[channel]);
            }
        }

        let constant_alpha = self.gx.bp_regs[0x42];
        if constant_alpha & (1 << 8) != 0 {
            result[3] = (constant_alpha & 0xff) as u8;
        }
        if blend_mode & (1 << 2) != 0 && self.gx.bp_regs[0x43] & 7 == 1 {
            const DITHER: [[u16; 2]; 2] = [[0, 2], [3, 1]];
            let adjustment = DITHER[y & 1][x & 1];
            for value in &mut result[..3] {
                let value_6bit = u16::from(*value) - (u16::from(*value) >> 6);
                *value = ((value_6bit + adjustment) & 0xfc) as u8;
            }
        }

        let color_update = blend_mode & (1 << 3) != 0;
        let alpha_update = blend_mode & (1 << 4) != 0;
        if color_update || alpha_update {
            let mut output = destination;
            if color_update {
                output[..3].copy_from_slice(&result[..3]);
            }
            if alpha_update {
                output[3] = result[3];
            }
            self.gx_write_efb_pixel(x as i32, y as i32, output);
        }
    }

    fn gx_edge(a: GxScreenVertex, b: GxScreenVertex, x: f32, y: f32) -> f32 {
        (x - a.x) * (b.y - a.y) - (y - a.y) * (b.x - a.x)
    }

    fn gx_perspective_texcoord(
        vertices: [GxScreenVertex; 3],
        weights: [f32; 3],
    ) -> Option<[f32; 2]> {
        let perspective = [
            weights[0] / vertices[0].clip_w,
            weights[1] / vertices[1].clip_w,
            weights[2] / vertices[2].clip_w,
        ];
        let divisor = perspective.iter().sum::<f32>();
        if !divisor.is_finite() || divisor.abs() < f32::EPSILON {
            return None;
        }
        Some([
            (perspective[0] * vertices[0].tex0[0]
                + perspective[1] * vertices[1].tex0[0]
                + perspective[2] * vertices[2].tex0[0])
                / divisor,
            (perspective[0] * vertices[0].tex0[1]
                + perspective[1] * vertices[1].tex0[1]
                + perspective[2] * vertices[2].tex0[1])
                / divisor,
        ])
    }

    fn gx_line_texcoord(a: GxScreenVertex, b: GxScreenVertex, t: f32) -> Option<[f32; 2]> {
        let qa = (1.0 - t) / a.clip_w;
        let qb = t / b.clip_w;
        let divisor = qa + qb;
        if !divisor.is_finite() || divisor.abs() < f32::EPSILON {
            return None;
        }
        Some([
            (qa * a.tex0[0] + qb * b.tex0[0]) / divisor,
            (qa * a.tex0[1] + qb * b.tex0[1]) / divisor,
        ])
    }

    fn gx_triangle_weights(vertices: [GxScreenVertex; 3], area: f32, x: f32, y: f32) -> [f32; 3] {
        [
            Self::gx_edge(vertices[1], vertices[2], x, y) / area,
            Self::gx_edge(vertices[2], vertices[0], x, y) / area,
            Self::gx_edge(vertices[0], vertices[1], x, y) / area,
        ]
    }

    fn gx_texture0_linear_filter(&self, dudx: f32, dvdx: f32, dudy: f32, dvdy: f32) -> bool {
        let mode = self.gx.bp_regs[0x80];
        let (s_delta, t_delta) = if mode & (1 << 8) != 0 {
            (dudx + dudy, dvdx + dvdy)
        } else {
            (dudx.max(dudy), dvdx.max(dvdy))
        };
        let delta = s_delta.max(t_delta);
        let bias = (((mode >> 9) & 0xff) as u8 as i8) as f32 / 32.0;
        let lod = if delta > 0.0 {
            delta.log2() + bias
        } else {
            f32::NEG_INFINITY
        };
        if lod > 0.0 {
            mode & (1 << 7) != 0
        } else {
            mode & (1 << 4) != 0
        }
    }

    fn gx_is_backface(&self, vertices: [GxScreenVertex; 3]) -> bool {
        let [v0, v1, v2] = vertices;
        let normal_z = (v0.clip_x * v2.clip_w - v2.clip_x * v0.clip_w) * v1.clip_y
            + (v2.clip_x * v0.clip_y - v0.clip_x * v2.clip_y) * v1.clip_w
            + (v2.clip_y * v0.clip_w - v0.clip_y * v2.clip_w) * v1.clip_x;
        let mut backface = normal_z <= 0.0;
        if self.gx_xf_f32(0x101b) > 0.0 {
            backface = !backface;
        }
        backface
    }

    fn gx_draw_triangle(&mut self, vertices: [GxScreenVertex; 3]) {
        let cull_mode = (self.gx.bp_regs[0x00] >> 14) & 3;
        let backface = self.gx_is_backface(vertices);
        if (!backface && matches!(cull_mode, 1 | 3)) || (backface && matches!(cull_mode, 2 | 3)) {
            return;
        }
        let area = Self::gx_edge(vertices[0], vertices[1], vertices[2].x, vertices[2].y);
        if !area.is_finite() || area.abs() < f32::EPSILON {
            return;
        }
        let (scissor_left, scissor_right, scissor_top, scissor_bottom) = self.gx_scissor_bounds();
        let min_x = vertices
            .iter()
            .map(|vertex| vertex.x.floor() as i32)
            .min()
            .unwrap_or(0)
            .max(scissor_left);
        let max_x = vertices
            .iter()
            .map(|vertex| vertex.x.ceil() as i32)
            .max()
            .unwrap_or(-1)
            .min(scissor_right);
        let min_y = vertices
            .iter()
            .map(|vertex| vertex.y.floor() as i32)
            .min()
            .unwrap_or(0)
            .max(scissor_top);
        let max_y = vertices
            .iter()
            .map(|vertex| vertex.y.ceil() as i32)
            .max()
            .unwrap_or(-1)
            .min(scissor_bottom);
        if min_x > max_x || min_y > max_y {
            return;
        }

        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let sample_x = x as f32 + 0.5;
                let sample_y = y as f32 + 0.5;
                let w0 = Self::gx_edge(vertices[1], vertices[2], sample_x, sample_y);
                let w1 = Self::gx_edge(vertices[2], vertices[0], sample_x, sample_y);
                let w2 = Self::gx_edge(vertices[0], vertices[1], sample_x, sample_y);
                let inside = if area > 0.0 {
                    w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0
                } else {
                    w0 <= 0.0 && w1 <= 0.0 && w2 <= 0.0
                };
                if !inside {
                    continue;
                }
                let weights = [w0 / area, w1 / area, w2 / area];
                let mut color = [0u8; 4];
                for (channel, value) in color.iter_mut().enumerate() {
                    let interpolated = weights[0] * f32::from(vertices[0].color[channel])
                        + weights[1] * f32::from(vertices[1].color[channel])
                        + weights[2] * f32::from(vertices[2].color[channel]);
                    *value = interpolated.round().clamp(0.0, 255.0) as u8;
                }
                if vertices.iter().all(|vertex| vertex.has_tex0) {
                    if let Some(coordinate) = Self::gx_perspective_texcoord(vertices, weights) {
                        let image = self.gx.bp_regs[0x88];
                        let width = (image & 0x3ff) as f32 + 1.0;
                        let height = ((image >> 10) & 0x3ff) as f32 + 1.0;
                        let weights_x =
                            Self::gx_triangle_weights(vertices, area, sample_x + 1.0, sample_y);
                        let weights_y =
                            Self::gx_triangle_weights(vertices, area, sample_x, sample_y + 1.0);
                        let linear = match (
                            Self::gx_perspective_texcoord(vertices, weights_x),
                            Self::gx_perspective_texcoord(vertices, weights_y),
                        ) {
                            (Some(coordinate_x), Some(coordinate_y)) => self
                                .gx_texture0_linear_filter(
                                    (coordinate_x[0] - coordinate[0]).abs() * width,
                                    (coordinate_x[1] - coordinate[1]).abs() * height,
                                    (coordinate_y[0] - coordinate[0]).abs() * width,
                                    (coordinate_y[1] - coordinate[1]).abs() * height,
                                ),
                            _ => self.gx.bp_regs[0x80] & (1 << 4) != 0,
                        };
                        if let Some(texel) = self.gx_texture0_sample(coordinate, linear) {
                            color = Self::gx_modulate_texture(color, texel);
                        }
                    }
                }
                let depth = weights[0] * vertices[0].z
                    + weights[1] * vertices[1].z
                    + weights[2] * vertices[2].z;
                self.gx_write_fragment(x, y, depth, color);
            }
        }
    }

    fn gx_draw_line(&mut self, a: GxScreenVertex, b: GxScreenVertex) {
        let dx = b.x - a.x;
        let dy = b.y - a.y;
        let steps = dx.abs().max(dy.abs()).ceil().max(1.0) as u32;
        for step in 0..=steps {
            let t = step as f32 / steps as f32;
            let mut color = [0u8; 4];
            for (channel, value) in color.iter_mut().enumerate() {
                *value = (f32::from(a.color[channel])
                    + (f32::from(b.color[channel]) - f32::from(a.color[channel])) * t)
                    .round()
                    .clamp(0.0, 255.0) as u8;
            }
            if a.has_tex0 && b.has_tex0 {
                if let Some(coordinate) = Self::gx_line_texcoord(a, b, t) {
                    let neighbor_t = if step < steps {
                        (step + 1) as f32 / steps as f32
                    } else {
                        step.saturating_sub(1) as f32 / steps as f32
                    };
                    let linear = if let Some(neighbor) = Self::gx_line_texcoord(a, b, neighbor_t) {
                        let image = self.gx.bp_regs[0x88];
                        let width = (image & 0x3ff) as f32 + 1.0;
                        let height = ((image >> 10) & 0x3ff) as f32 + 1.0;
                        self.gx_texture0_linear_filter(
                            (neighbor[0] - coordinate[0]).abs() * width,
                            (neighbor[1] - coordinate[1]).abs() * height,
                            0.0,
                            0.0,
                        )
                    } else {
                        self.gx.bp_regs[0x80] & (1 << 4) != 0
                    };
                    if let Some(texel) = self.gx_texture0_sample(coordinate, linear) {
                        color = Self::gx_modulate_texture(color, texel);
                    }
                }
            }
            self.gx_write_fragment(
                (a.x + dx * t).round() as i32,
                (a.y + dy * t).round() as i32,
                a.z + (b.z - a.z) * t,
                color,
            );
        }
    }

    fn gx_draw_point(&mut self, vertex: GxScreenVertex) {
        let mut color = vertex.color;
        if vertex.has_tex0 {
            let linear = self.gx.bp_regs[0x80] & (1 << 4) != 0;
            if let Some(texel) = self.gx_texture0_sample(vertex.tex0, linear) {
                color = Self::gx_modulate_texture(color, texel);
            }
        }
        self.gx_write_fragment(
            vertex.x.round() as i32,
            vertex.y.round() as i32,
            vertex.z,
            color,
        );
    }

    fn gx_render_primitive(&mut self, command: &[u8]) {
        let opcode = command[0];
        let vat = (opcode & 7) as usize;
        let count = usize::from(u16::from_be_bytes([command[1], command[2]]));
        let vertex_size = self.gx_vertex_size(vat);
        if vertex_size == 0 {
            return;
        }
        let mut vertices = Vec::with_capacity(count);
        let mut offset = 3usize;
        for _ in 0..count {
            let Some(end) = offset.checked_add(vertex_size) else {
                return;
            };
            let vertex = command
                .get(offset..end)
                .and_then(|data| self.gx_decode_vertex(vat, data))
                .and_then(|vertex| self.gx_transform_vertex(vertex));
            vertices.push(vertex);
            offset = end;
        }
        let draw_triangle = |a: usize, b: usize, c: usize, board: &mut Self| {
            if let (Some(a), Some(b), Some(c)) = (vertices[a], vertices[b], vertices[c]) {
                board.gx_draw_triangle([a, b, c]);
            }
        };
        match opcode & 0xf8 {
            0x80 | 0x88 => {
                for base in (0..count.saturating_sub(3)).step_by(4) {
                    draw_triangle(base, base + 1, base + 2, self);
                    draw_triangle(base, base + 2, base + 3, self);
                }
            }
            0x90 => {
                for base in (0..count.saturating_sub(2)).step_by(3) {
                    draw_triangle(base, base + 1, base + 2, self);
                }
            }
            0x98 => {
                for index in 2..count {
                    if index & 1 == 0 {
                        draw_triangle(index - 2, index - 1, index, self);
                    } else {
                        draw_triangle(index - 1, index - 2, index, self);
                    }
                }
            }
            0xa0 => {
                for index in 2..count {
                    draw_triangle(0, index - 1, index, self);
                }
            }
            0xa8 => {
                for base in (0..count.saturating_sub(1)).step_by(2) {
                    if let (Some(a), Some(b)) = (vertices[base], vertices[base + 1]) {
                        self.gx_draw_line(a, b);
                    }
                }
            }
            0xb0 => {
                for index in 1..count {
                    if let (Some(a), Some(b)) = (vertices[index - 1], vertices[index]) {
                        self.gx_draw_line(a, b);
                    }
                }
            }
            0xb8 => {
                for vertex in vertices.into_iter().flatten() {
                    self.gx_draw_point(vertex);
                }
            }
            _ => {}
        }
    }

    fn gx_rgb_to_yuv(color: [u8; 4]) -> (u8, u8, u8) {
        let r = i32::from(color[0]);
        let g = i32::from(color[1]);
        let b = i32::from(color[2]);
        let y = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
        let u = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
        let v = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
        (
            Self::clamp_byte(y),
            Self::clamp_byte(u),
            Self::clamp_byte(v),
        )
    }

    fn gx_clear_efb_rect(&mut self, x: usize, y: usize, width: usize, height: usize) {
        let ar = self.gx.bp_regs[0x4f];
        let gb = self.gx.bp_regs[0x50];
        let clear_color = [
            (ar & 0xff) as u8,
            ((gb >> 8) & 0xff) as u8,
            (gb & 0xff) as u8,
            ((ar >> 8) & 0xff) as u8,
        ];
        let blend_mode = self.gx.bp_regs[0x41];
        let color_update = blend_mode & (1 << 3) != 0;
        let alpha_update = blend_mode & (1 << 4) != 0;
        let depth_update = self.gx.bp_regs[0x40] & (1 << 4) != 0;
        let clear_depth = self.gx.bp_regs[0x51] & 0x00ff_ffff;
        let right = x.saturating_add(width).min(GX_EFB_WIDTH);
        let bottom = y.saturating_add(height).min(GX_EFB_HEIGHT);
        for py in y.min(GX_EFB_HEIGHT)..bottom {
            for px in x.min(GX_EFB_WIDTH)..right {
                if color_update || alpha_update {
                    let mut output = self.gx_efb_pixel(px, py);
                    if color_update {
                        output[..3].copy_from_slice(&clear_color[..3]);
                    }
                    if alpha_update {
                        output[3] = clear_color[3];
                    }
                    self.gx_write_efb_pixel(px as i32, py as i32, output);
                }
                if depth_update {
                    self.gx_write_efb_depth(px, py, clear_depth);
                }
            }
        }
    }

    fn gx_copy_efb(&mut self, trigger: u32) {
        let source_xy = self.gx.bp_regs[0x49];
        let source_wh = self.gx.bp_regs[0x4a];
        let source_x = (source_xy & 0x3ff) as usize;
        let source_y = ((source_xy >> 10) & 0x3ff) as usize;
        let source_width = ((source_wh & 0x3ff) as usize).saturating_add(1);
        let source_height = (((source_wh >> 10) & 0x3ff) as usize).saturating_add(1);
        let copy_width = source_width.min(GX_EFB_WIDTH.saturating_sub(source_x));
        let copy_height = source_height.min(GX_EFB_HEIGHT.saturating_sub(source_y));

        if trigger & (1 << 14) != 0 && copy_width != 0 && copy_height != 0 {
            let destination = self.gx.bp_regs[0x4b] << 5;
            let stride = (self.gx.bp_regs[0x4d] << 5) as usize;
            if stride != 0 {
                let scale_register = self.gx.bp_regs[0x4e] as f32;
                let y_scale = if scale_register == 0.0 {
                    1.0
                } else if trigger & (1 << 10) != 0 {
                    256.0 / scale_register
                } else {
                    scale_register / 256.0
                };
                let output_height = (1.0 + (copy_height.saturating_sub(1) as f32) * y_scale)
                    .floor()
                    .max(1.0) as usize;
                if let Some(destination_base) = memory_index(destination) {
                    let row_pixels = copy_width.min(stride / 2);
                    for output_y in 0..output_height {
                        let source_row =
                            ((output_y as f32 / y_scale).floor() as usize).min(copy_height - 1);
                        for x in (0..row_pixels).step_by(2) {
                            let first = self.gx_efb_pixel(source_x + x, source_y + source_row);
                            let second_x = (x + 1).min(row_pixels - 1);
                            let second =
                                self.gx_efb_pixel(source_x + second_x, source_y + source_row);
                            let (y0, u0, v0) = Self::gx_rgb_to_yuv(first);
                            let (y1, u1, v1) = Self::gx_rgb_to_yuv(second);
                            let destination_offset = destination_base
                                .saturating_add(output_y.saturating_mul(stride))
                                .saturating_add(x.saturating_mul(2));
                            if destination_offset + 4 > MEM1_SIZE {
                                break;
                            }
                            self.mem1[destination_offset..destination_offset + 4].copy_from_slice(
                                &[
                                    y0,
                                    (u16::from(u0) + u16::from(u1)).div_ceil(2) as u8,
                                    y1,
                                    (u16::from(v0) + u16::from(v1)).div_ceil(2) as u8,
                                ],
                            );
                        }
                    }
                }
            }
        }

        if trigger & (1 << 11) != 0 && copy_width != 0 && copy_height != 0 {
            self.gx_clear_efb_rect(source_x, source_y, copy_width, copy_height);
        }
    }

    fn gx_stream_component_size(descriptor: u32, direct_size: usize) -> usize {
        match descriptor & 3 {
            0 => 0,
            1 => direct_size,
            2 => 1,
            3 => 2,
            _ => unreachable!(),
        }
    }

    fn gx_vertex_size(&self, vat: usize) -> usize {
        let vcd_lo = self.gx.cp_regs[0x50];
        let vcd_hi = self.gx.cp_regs[0x60];
        let vat_a = self.gx.cp_regs[0x70 + (vat & 7)];
        let vat_b = self.gx.cp_regs[0x80 + (vat & 7)];
        let vat_c = self.gx.cp_regs[0x90 + (vat & 7)];

        let mut size = (vcd_lo & 0x1ff).count_ones() as usize;

        let position_components = if vat_a & 1 != 0 { 3 } else { 2 };
        let position_direct = Self::gx_direct_component_size((vat_a >> 1) & 7, position_components);
        size += Self::gx_stream_component_size((vcd_lo >> 9) & 3, position_direct);

        let normal_descriptor = (vcd_lo >> 11) & 3;
        let normal_components = if vat_a & (1 << 9) != 0 { 9 } else { 3 };
        let normal_direct = Self::gx_direct_component_size((vat_a >> 10) & 7, normal_components);
        size += match normal_descriptor {
            0 => 0,
            1 => normal_direct,
            2 | 3 => {
                let index_size = if normal_descriptor == 2 { 1 } else { 2 };
                if normal_components == 9 && vat_a & (1 << 31) != 0 {
                    index_size * 3
                } else {
                    index_size
                }
            }
            _ => unreachable!(),
        };

        for color in 0..2 {
            let descriptor = (vcd_lo >> (13 + color * 2)) & 3;
            let format = (vat_a >> (14 + color * 4)) & 7;
            let direct = match format {
                0 | 3 => 2,
                1 | 4 => 3,
                _ => 4,
            };
            size += Self::gx_stream_component_size(descriptor, direct);
        }

        for tex in 0..8 {
            let descriptor = (vcd_hi >> (tex * 2)) & 3;
            let (two_components, format) = match tex {
                0 => (vat_a & (1 << 21) != 0, (vat_a >> 22) & 7),
                1 => (vat_b & 1 != 0, (vat_b >> 1) & 7),
                2 => (vat_b & (1 << 9) != 0, (vat_b >> 10) & 7),
                3 => (vat_b & (1 << 18) != 0, (vat_b >> 19) & 7),
                4 => (vat_b & (1 << 27) != 0, (vat_b >> 28) & 7),
                5 => (vat_c & (1 << 5) != 0, (vat_c >> 6) & 7),
                6 => (vat_c & (1 << 14) != 0, (vat_c >> 15) & 7),
                _ => (vat_c & (1 << 23) != 0, (vat_c >> 24) & 7),
            };
            let direct = Self::gx_direct_component_size(format, if two_components { 2 } else { 1 });
            size += Self::gx_stream_component_size(descriptor, direct);
        }
        size
    }

    fn gx_command_size(&self, data: &[u8]) -> Option<usize> {
        let opcode = *data.first()?;
        let fixed = match opcode {
            0x00 | 0x44 | 0x48 => Some(1usize),
            0x08 => Some(6),
            0x10 => {
                if data.len() < 5 {
                    return None;
                }
                let header = u32::from_be_bytes(data[1..5].try_into().unwrap());
                let words = ((header >> 16) & 0xf) as usize + 1;
                Some(5 + words * 4)
            }
            0x20 | 0x28 | 0x30 | 0x38 | 0x61 => Some(5),
            0x40 => Some(9),
            0x80..=0xbf => {
                if data.len() < 3 {
                    return None;
                }
                let vertices = u16::from_be_bytes([data[1], data[2]]) as usize;
                let vertex_size = self.gx_vertex_size((opcode & 7) as usize);
                3usize.checked_add(vertices.checked_mul(vertex_size)?)
            }
            _ => Some(1),
        }?;
        (data.len() >= fixed).then_some(fixed)
    }

    fn gx_load_cp_reg(&mut self, register: u8, value: u32) {
        let target = match register & 0xf0 {
            0x30 => 0x30,
            0x40 => 0x40,
            0x50 => 0x50,
            0x60 => 0x60,
            0x70 | 0x80 | 0x90 => register & 0xf7,
            0xa0 | 0xb0 => register,
            _ => register,
        } as usize;
        self.gx.cp_regs[target] = match register & 0xf0 {
            0xa0 => value & 0x01ff_ffff,
            0xb0 => value & 0xff,
            _ => value,
        };
    }

    fn gx_load_tlut(&mut self) {
        let source = (self.gx.bp_regs[0x64] << 5) & 0x01ff_ffff;
        let destination = ((self.gx.bp_regs[0x65] & 0x3ff) as usize) << 9;
        let line_count = ((self.gx.bp_regs[0x65] >> 10) & 0x7ff) as usize;
        let Some(length) = line_count.checked_mul(GX_TMEM_LINE_SIZE) else {
            return;
        };
        if length == 0 {
            return;
        }
        let Some(destination_end) = destination.checked_add(length) else {
            return;
        };
        if destination_end > self.gx.tmem.len() {
            return;
        }
        let Ok(source_range) = memory_range(source, length) else {
            return;
        };
        self.gx.tmem[destination..destination_end].copy_from_slice(&self.mem1[source_range]);
    }

    fn gx_load_indexed_xf(&mut self, opcode: u8, value: u32) {
        let array = match opcode {
            0x20 => 12usize,
            0x28 => 13,
            0x30 => 14,
            0x38 => 15,
            _ => return,
        };
        let index = value >> 16;
        let destination = (value & 0xfff) as usize;
        let words = ((value >> 12) & 0xf) as usize + 1;
        let base = self.gx.cp_regs[0xa0 + array];
        let stride = self.gx.cp_regs[0xb0 + array];
        let source = base.wrapping_add(index.wrapping_mul(stride));
        for word in 0..words {
            let Some(target) = destination
                .checked_add(word)
                .filter(|target| *target < GX_XF_REGISTER_COUNT)
            else {
                break;
            };
            let address = source.wrapping_add((word * 4) as u32);
            let Ok(range) = memory_range(address, 4) else {
                break;
            };
            self.gx.xf_regs[target] = u32::from_be_bytes(self.mem1[range].try_into().unwrap());
        }
    }

    fn gx_execute_command(&mut self, command: &[u8], depth: u8) {
        let opcode = command[0];
        match opcode {
            0x08 => {
                let value = u32::from_be_bytes(command[2..6].try_into().unwrap());
                self.gx_load_cp_reg(command[1], value);
            }
            0x10 => {
                let header = u32::from_be_bytes(command[1..5].try_into().unwrap());
                let address = (header & 0xffff) as usize;
                let words = ((header >> 16) & 0xf) as usize + 1;
                for word in 0..words {
                    let target = address + word;
                    if target >= GX_XF_REGISTER_COUNT {
                        break;
                    }
                    let start = 5 + word * 4;
                    self.gx.xf_regs[target] =
                        u32::from_be_bytes(command[start..start + 4].try_into().unwrap());
                }
            }
            0x20 | 0x28 | 0x30 | 0x38 => {
                let value = u32::from_be_bytes(command[1..5].try_into().unwrap());
                self.gx_load_indexed_xf(opcode, value);
            }
            0x40 if depth < 16 => {
                let address = u32::from_be_bytes(command[1..5].try_into().unwrap()) & !31;
                let size = u32::from_be_bytes(command[5..9].try_into().unwrap()) & !31;
                if size != 0 && size as usize <= GX_FIFO_BUFFER_LIMIT {
                    if let Ok(range) = memory_range(address, size as usize) {
                        let display_list = self.mem1[range].to_vec();
                        self.gx_execute_stream(&display_list, depth + 1);
                    }
                }
            }
            0x61 => {
                let register = command[1];
                let value = u32::from_be_bytes([0, command[2], command[3], command[4]]);
                self.gx.bp_regs[register as usize] = value;
                if register == 0x52 {
                    self.gx_copy_efb(value);
                } else if register == 0x65 {
                    self.gx_load_tlut();
                }
            }
            0x80..=0xbf => {
                let vertices = u16::from_be_bytes([command[1], command[2]]) as u64;
                self.gx.primitives_processed = self.gx.primitives_processed.wrapping_add(1);
                self.gx.vertices_processed = self.gx.vertices_processed.wrapping_add(vertices);
                self.gx_render_primitive(command);
            }
            _ => {}
        }
        self.gx.commands_processed = self.gx.commands_processed.wrapping_add(1);
    }

    fn gx_execute_stream(&mut self, data: &[u8], depth: u8) {
        let mut offset = 0usize;
        while offset < data.len() {
            let Some(size) = self.gx_command_size(&data[offset..]) else {
                break;
            };
            let end = offset + size;
            self.gx_execute_command(&data[offset..end], depth);
            offset = end;
        }
    }

    fn gx_push_fifo(&mut self, bytes: &[u8]) {
        let Some(next_len) = self.gx.fifo.len().checked_add(bytes.len()) else {
            self.gx.fifo.clear();
            return;
        };
        if next_len > GX_FIFO_BUFFER_LIMIT {
            self.gx.fifo.clear();
            return;
        }
        self.gx.fifo.extend_from_slice(bytes);
        while let Some(size) = self.gx_command_size(&self.gx.fifo) {
            let command = self.gx.fifo[..size].to_vec();
            self.gx.fifo.drain(..size);
            self.gx_execute_command(&command, 0);
        }
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
                        self.reset_dtk_filter();
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

    fn reset_dtk_filter(&mut self) {
        self.di.dtk_left_recent = 0;
        self.di.dtk_left_older = 0;
        self.di.dtk_right_recent = 0;
        self.di.dtk_right_older = 0;
    }

    fn decode_dtk_nibble(header: u8, nibble: u8, recent: &mut i32, older: &mut i32) -> i16 {
        let predicted = match header >> 4 {
            1 => *recent * 0x3c,
            2 => *recent * 0x73 - *older * 0x34,
            3 => *recent * 0x62 - *older * 0x37,
            _ => 0,
        };
        let predicted = ((predicted + 0x20) >> 6).clamp(-0x20_0000, 0x1f_ffff);
        let signed_nibble = if nibble & 0x08 != 0 {
            i32::from(nibble) - 16
        } else {
            i32::from(nibble)
        };
        let exponent = u32::from(header & 0x0f);
        let residual = ((signed_nibble * 4096) >> exponent) * 64;
        let current = predicted + residual;
        *older = *recent;
        *recent = current;
        (current >> 6).clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
    }

    fn prepare_dtk_sample(&mut self) -> bool {
        if !self.di.stream {
            return false;
        }
        let end = self
            .di
            .current_start
            .saturating_add(u64::from(self.di.current_length));
        if self.di.audio_position < end {
            return true;
        }

        self.di.audio_position = self.di.next_start;
        self.di.current_start = self.di.next_start;
        self.di.current_length = self.di.next_length;
        self.di.dtk_sample_remainder = 0;

        if self.di.stop_at_track_end {
            self.di.stop_at_track_end = false;
            self.di.stream = false;
            return false;
        }
        self.reset_dtk_filter();
        if self.di.current_length == 0 {
            self.di.stream = false;
            return false;
        }
        true
    }

    fn render_dtk_samples(&mut self, sample_count: usize) -> Vec<(f32, f32)> {
        let mut output = vec![(0.0, 0.0); sample_count];
        let mut block = [0u8; DTK_BLOCK_BYTES as usize];
        let mut cached_position = None;
        let mut block_available = false;

        for sample in &mut output {
            if !self.prepare_dtk_sample() {
                continue;
            }

            if cached_position != Some(self.di.audio_position) {
                block_available = self.disc.read(self.di.audio_position, &mut block).is_ok();
                cached_position = Some(self.di.audio_position);
            }
            if !block_available {
                continue;
            }

            let index = usize::from(self.di.dtk_sample_remainder);
            let packed = block[4 + index];
            let left = Self::decode_dtk_nibble(
                block[0],
                packed & 0x0f,
                &mut self.di.dtk_left_recent,
                &mut self.di.dtk_left_older,
            );
            let right = Self::decode_dtk_nibble(
                block[1],
                packed >> 4,
                &mut self.di.dtk_right_recent,
                &mut self.di.dtk_right_older,
            );
            *sample = (f32::from(left) / 32768.0, f32::from(right) / 32768.0);

            self.di.dtk_sample_remainder += 1;
            if u32::from(self.di.dtk_sample_remainder) == DTK_SAMPLES_PER_BLOCK {
                self.di.dtk_sample_remainder = 0;
                self.di.audio_position = self.di.audio_position.saturating_add(DTK_BLOCK_BYTES);
                cached_position = None;
            }
        }

        output
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
        let dtk = self.render_dtk_samples(AUDIO_SAMPLES_PER_FRAME as usize);
        audio.begin_frame();
        let mut sample_index = 0usize;
        for _ in 0..100 {
            let mut block = [0u8; 32];
            if self.audio_dma.enabled {
                if let Ok(range) = memory_range(self.audio_dma.cursor, block.len()) {
                    block.copy_from_slice(&self.mem1[range]);
                }
            }
            for frame in block.as_chunks::<4>().0 {
                let dma_left = i16::from_be_bytes([frame[0], frame[1]]) as f32 / 32768.0;
                let dma_right = i16::from_be_bytes([frame[2], frame[3]]) as f32 / 32768.0;
                let (dtk_left, dtk_right) = dtk[sample_index];
                sample_index += 1;
                audio.push_stereo(
                    (dma_left + dtk_left).clamp(-1.0, 1.0),
                    (dma_right + dtk_right).clamp(-1.0, 1.0),
                );
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
        if (GX_FIFO_START..GX_FIFO_END).contains(&offset) {
            self.gx_push_fifo(&[value]);
            return;
        }
        if let Some(byte) = self.mmio.get_mut(offset) {
            *byte = value;
        }
    }

    fn write_mmio16(&mut self, offset: usize, value: u16) {
        if (GX_FIFO_START..GX_FIFO_END).contains(&offset) {
            self.gx_push_fifo(&value.to_be_bytes());
            return;
        }
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
        if channel > 2 {
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
        if (GX_FIFO_START..GX_FIFO_END).contains(&offset) {
            self.gx_push_fifo(&value.to_be_bytes());
            return;
        }
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
        self.gx = GxState::default();
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
        out.u32(self.di.dtk_left_recent as u32);
        out.u32(self.di.dtk_left_older as u32);
        out.u32(self.di.dtk_right_recent as u32);
        out.u32(self.di.dtk_right_older as u32);
        out.u8(u8::from(self.di.stream));
        out.u8(u8::from(self.di.stop_at_track_end));
        out.u64(self.di.audio_position);
        out.u64(self.di.current_start);
        out.u32(self.di.current_length);
        out.u64(self.di.next_start);
        out.u32(self.di.next_length);
        Self::save_exi_channel(&self.exi.channel0, out);
        Self::save_exi_channel(&self.exi.channel1, out);
        Self::save_exi_channel(&self.exi.channel2, out);
        out.u8(self.exi.ad16.command);
        out.u32(self.exi.ad16.position);
        out.blob(&self.exi.ad16.register);
        out.u32(self.exi.ipl.command);
        out.u8(self.exi.ipl.command_bytes);
        out.u32(self.exi.ipl.cursor);
        out.u8(self.exi.ipl.rtc_phase);
        out.blob(&self.exi.ipl.sram);
        for value in self.gx.cp_regs {
            out.u32(value);
        }
        for value in self.gx.bp_regs {
            out.u32(value);
        }
        for &value in &self.gx.xf_regs {
            out.u32(value);
        }
        out.blob(&self.gx.depth);
        out.blob(&self.gx.tmem);
        out.blob(&self.gx.fifo);
        out.u64(self.gx.commands_processed);
        out.u64(self.gx.primitives_processed);
        out.u64(self.gx.vertices_processed);
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
        self.di.dtk_left_recent = input.u32()? as i32;
        self.di.dtk_left_older = input.u32()? as i32;
        self.di.dtk_right_recent = input.u32()? as i32;
        self.di.dtk_right_older = input.u32()? as i32;
        self.di.stream = input.u8()? != 0;
        self.di.stop_at_track_end = input.u8()? != 0;
        self.di.audio_position = input.u64()?;
        self.di.current_start = input.u64()?;
        self.di.current_length = input.u32()?;
        self.di.next_start = input.u64()?;
        self.di.next_length = input.u32()?;
        Self::load_exi_channel(&mut self.exi.channel0, input, "EXI channel 0")?;
        Self::load_exi_channel(&mut self.exi.channel1, input, "EXI channel 1")?;
        Self::load_exi_channel(&mut self.exi.channel2, input, "EXI channel 2")?;
        self.exi.ad16.command = input.u8()?;
        self.exi.ad16.position = input.u32()?;
        Self::load_blob(input, &mut self.exi.ad16.register, "EXI AD16 register")?;
        self.exi.ipl.command = input.u32()?;
        self.exi.ipl.command_bytes = input.u8()?.min(4);
        self.exi.ipl.cursor = input.u32()?;
        self.exi.ipl.rtc_phase = input.u8()? % FRAME_RATE as u8;
        Self::load_blob(input, &mut self.exi.ipl.sram, "EXI IPL SRAM")?;
        for value in &mut self.gx.cp_regs {
            *value = input.u32()?;
        }
        for value in &mut self.gx.bp_regs {
            *value = input.u32()?;
        }
        for value in &mut self.gx.xf_regs {
            *value = input.u32()?;
        }
        Self::load_blob(input, &mut self.gx.depth, "GX EFB depth")?;
        Self::load_blob(input, &mut self.gx.tmem, "GX TMEM")?;
        let fifo = input.blob()?;
        if fifo.len() > GX_FIFO_BUFFER_LIMIT {
            return Err(format!(
                "GameCube GX FIFO state has {} bytes; limit is {}",
                fifo.len(),
                GX_FIFO_BUFFER_LIMIT
            ));
        }
        self.gx.fifo.clear();
        self.gx.fifo.extend_from_slice(fifo);
        self.gx.commands_processed = input.u64()?;
        self.gx.primitives_processed = input.u64()?;
        self.gx.vertices_processed = input.u64()?;
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

    fn write_gx_bytes(board: &mut GameCubeBoard, bytes: &[u8]) {
        for &byte in bytes {
            board.write_mmio8(GX_FIFO_START, byte);
        }
    }

    fn gx_cp_command(register: u8, value: u32) -> [u8; 6] {
        let mut command = [0u8; 6];
        command[0] = 0x08;
        command[1] = register;
        command[2..].copy_from_slice(&value.to_be_bytes());
        command
    }

    fn gx_bp_command(register: u8, value: u32) -> [u8; 5] {
        let bytes = value.to_be_bytes();
        [0x61, register, bytes[1], bytes[2], bytes[3]]
    }

    fn gx_xf_command(address: u16, values: &[u32]) -> Vec<u8> {
        assert!((1..=16).contains(&values.len()));
        let header = (((values.len() - 1) as u32) << 16) | u32::from(address);
        let mut command = Vec::with_capacity(5 + values.len() * 4);
        command.push(0x10);
        command.extend_from_slice(&header.to_be_bytes());
        for value in values {
            command.extend_from_slice(&value.to_be_bytes());
        }
        command
    }

    fn configure_gx_2d(board: &mut GameCubeBoard, vcd: u32, vat_a: u32) {
        write_gx_bytes(board, &gx_cp_command(0x30, 0));
        write_gx_bytes(board, &gx_cp_command(0x50, vcd));
        write_gx_bytes(board, &gx_cp_command(0x70, vat_a));

        let identity = [
            1.0f32, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        ];
        write_gx_bytes(board, &gx_xf_command(0, &identity.map(f32::to_bits)));
        let viewport = [320.0f32, -240.0, 16_777_215.0, 662.0, 582.0, 0.0];
        write_gx_bytes(board, &gx_xf_command(0x101a, &viewport.map(f32::to_bits)));
        let projection = [1.0f32, 0.0, 1.0, 0.0, 1.0, 0.0];
        let mut projection_words = projection.map(f32::to_bits).to_vec();
        projection_words.push(1);
        write_gx_bytes(board, &gx_xf_command(0x1020, &projection_words));
        write_gx_bytes(board, &gx_bp_command(0x20, 342 | (342 << 10)));
        write_gx_bytes(board, &gx_bp_command(0x21, 981 | (869 << 10)));
        write_gx_bytes(board, &gx_bp_command(0x41, (1 << 3) | (1 << 4)));
        write_gx_bytes(board, &gx_bp_command(0x59, 171 | (171 << 10)));
    }

    fn gx_direct_xy_rgba_vertex(x: f32, y: f32, color: [u8; 4]) -> Vec<u8> {
        let mut vertex = Vec::with_capacity(12);
        vertex.extend_from_slice(&x.to_bits().to_be_bytes());
        vertex.extend_from_slice(&y.to_bits().to_be_bytes());
        vertex.extend_from_slice(&color);
        vertex
    }

    fn gx_direct_xyz_rgba_vertex(x: f32, y: f32, z: f32, color: [u8; 4]) -> Vec<u8> {
        let mut vertex = Vec::with_capacity(16);
        vertex.extend_from_slice(&x.to_bits().to_be_bytes());
        vertex.extend_from_slice(&y.to_bits().to_be_bytes());
        vertex.extend_from_slice(&z.to_bits().to_be_bytes());
        vertex.extend_from_slice(&color);
        vertex
    }

    fn gx_direct_xy_rgba_st_vertex(x: f32, y: f32, color: [u8; 4], s: f32, t: f32) -> Vec<u8> {
        let mut vertex = gx_direct_xy_rgba_vertex(x, y, color);
        vertex.extend_from_slice(&s.to_bits().to_be_bytes());
        vertex.extend_from_slice(&t.to_bits().to_be_bytes());
        vertex
    }

    fn configure_texture0(
        board: &mut GameCubeBoard,
        address: u32,
        width: u32,
        height: u32,
        format: u32,
        mode: u32,
    ) {
        board.gx.bp_regs[0x80] = mode;
        board.gx.bp_regs[0x88] = (width - 1) | ((height - 1) << 10) | (format << 20);
        board.gx.bp_regs[0x94] = address >> 5;
    }

    fn draw_gx_xyz_triangle(board: &mut GameCubeBoard, z: f32, color: [u8; 4]) {
        let mut primitive = vec![0x90, 0, 3];
        for (x, y) in [(-0.5f32, -0.5f32), (0.5, -0.5), (0.0, 0.5)] {
            primitive.extend_from_slice(&gx_direct_xyz_rgba_vertex(x, y, z, color));
        }
        write_gx_bytes(board, &primitive);
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
    fn gx_write_gather_fifo_decodes_cp_vertex_state_and_primitive_payloads() {
        let mut board = idle_board();
        let vcd = (1 << 9) | (1 << 13);
        let vcd_command = gx_cp_command(0x50, vcd);
        board.write_mmio32(
            GX_FIFO_START,
            u32::from_be_bytes(vcd_command[..4].try_into().unwrap()),
        );
        board.write_mmio16(
            GX_FIFO_START,
            u16::from_be_bytes(vcd_command[4..].try_into().unwrap()),
        );

        let vat_a = 1 | (4 << 1) | (1 << 13) | (5 << 14);
        write_gx_bytes(&mut board, &gx_cp_command(0x70, vat_a));
        assert_eq!(board.gx_vertex_size(0), 16);

        let mut primitive = vec![0x90, 0, 3];
        primitive.resize(3 + 3 * 16, 0);
        write_gx_bytes(&mut board, &primitive);

        assert!(board.gx.fifo.is_empty());
        assert_eq!(board.gx.cp_regs[0x50], vcd);
        assert_eq!(board.gx.cp_regs[0x70], vat_a);
        assert_eq!(board.gx.commands_processed, 3);
        assert_eq!(board.gx.primitives_processed, 1);
        assert_eq!(board.gx.vertices_processed, 3);
    }

    #[test]
    fn gx_direct_position_color_triangle_rasterizes_into_efb() {
        let mut board = idle_board();
        let vcd = (1 << 9) | (1 << 13);
        let vat_a = (4 << 1) | (1 << 13) | (5 << 14);
        configure_gx_2d(&mut board, vcd, vat_a);

        let mut primitive = vec![0x90, 0, 3];
        for (x, y) in [(-0.5f32, -0.5f32), (0.5, -0.5), (0.0, 0.5)] {
            primitive.extend_from_slice(&gx_direct_xy_rgba_vertex(x, y, [255, 0, 0, 255]));
        }
        write_gx_bytes(&mut board, &primitive);

        assert_eq!(board.gx_efb_pixel(320, 240), [255, 0, 0, 255]);
        assert_eq!(board.gx_efb_pixel(0, 0), [0, 0, 0, 0]);
        assert_eq!(board.gx.primitives_processed, 1);
        assert_eq!(board.gx.vertices_processed, 3);
    }

    #[test]
    fn gx_texcoord_zero_decodes_direct_and_indexed_streams() {
        let mut board = idle_board();
        let vcd = (1 << 9) | (1 << 13);
        let vat_a = (4 << 1) | (1 << 13) | (5 << 14) | (1 << 21) | (4 << 22);
        configure_gx_2d(&mut board, vcd, vat_a);
        write_gx_bytes(&mut board, &gx_cp_command(0x60, 1));

        let direct = gx_direct_xy_rgba_st_vertex(-0.25, 0.5, [1, 2, 3, 4], 0.125, 0.75);
        assert_eq!(board.gx_vertex_size(0), 20);
        let vertex = board.gx_decode_vertex(0, &direct).unwrap();
        assert!(vertex.has_tex0);
        assert_eq!(vertex.tex0, [0.125, 0.75]);

        write_gx_bytes(&mut board, &gx_cp_command(0x60, 2));
        write_gx_bytes(&mut board, &gx_cp_command(0xa4, 0x5200));
        write_gx_bytes(&mut board, &gx_cp_command(0xb4, 8));
        board.mem1[0x5208..0x520c].copy_from_slice(&0.375f32.to_bits().to_be_bytes());
        board.mem1[0x520c..0x5210].copy_from_slice(&0.625f32.to_bits().to_be_bytes());
        let mut indexed = gx_direct_xy_rgba_vertex(0.0, 0.0, [0xff; 4]);
        indexed.push(1);
        assert_eq!(board.gx_vertex_size(0), 13);
        let vertex = board.gx_decode_vertex(0, &indexed).unwrap();
        assert!(vertex.has_tex0);
        assert_eq!(vertex.tex0, [0.375, 0.625]);
    }

    #[test]
    fn gx_texture_unit_zero_decodes_native_tiles_and_wrap_modes() {
        let mut board = idle_board();
        let base = 0x6000u32;

        configure_texture0(&mut board, base, 8, 8, 0, 0);
        board.mem1[base as usize] = 0xa0;
        assert_eq!(board.gx_texture0_texel(0, 0), Some([0xaa; 4]));

        board.mem1[base as usize..base as usize + 64].fill(0);
        configure_texture0(&mut board, base, 8, 4, 1, 0);
        board.mem1[base as usize + 1] = 0x35;
        board.mem1[base as usize + 6] = 0x66;
        board.mem1[base as usize + 7] = 0x77;
        assert_eq!(board.gx_texture0_texel(1, 0), Some([0x35; 4]));
        board.gx.bp_regs[0x80] = 1;
        assert_eq!(
            board.gx_texture0_sample([1.125, 0.0], false),
            Some([0x35; 4])
        );
        board.gx.bp_regs[0x80] = 2;
        assert_eq!(
            board.gx_texture0_sample([1.125, 0.0], false),
            Some([0x66; 4])
        );
        board.gx.bp_regs[0x80] = 0;
        assert_eq!(
            board.gx_texture0_sample([1.125, 0.0], false),
            Some([0x77; 4])
        );

        board.mem1[base as usize..base as usize + 64].fill(0);
        configure_texture0(&mut board, base, 6, 4, 1, 1);
        board.mem1[base as usize + 1] = 0x11;
        board.mem1[base as usize + 4] = 0x44;
        board.mem1[base as usize + 5] = 0x55;
        assert_eq!(
            board.gx_texture0_sample([1.125, 0.0], false),
            Some([0x44; 4])
        );
        board.gx.bp_regs[0x80] = 2;
        assert_eq!(
            board.gx_texture0_sample([1.125, 0.0], false),
            Some([0x11; 4])
        );
        board.gx.bp_regs[0x80] = 0;
        assert_eq!(
            board.gx_texture0_sample([1.125, 0.0], false),
            Some([0x55; 4])
        );

        board.mem1[base as usize..base as usize + 64].fill(0);
        configure_texture0(&mut board, base, 8, 4, 2, 0);
        board.mem1[base as usize] = 0xb4;
        assert_eq!(
            board.gx_texture0_texel(0, 0),
            Some([0x44, 0x44, 0x44, 0xbb])
        );

        board.mem1[base as usize..base as usize + 64].fill(0);
        configure_texture0(&mut board, base, 4, 4, 3, 0);
        board.mem1[base as usize..base as usize + 2].copy_from_slice(&[0x7f, 0x40]);
        assert_eq!(
            board.gx_texture0_texel(0, 0),
            Some([0x40, 0x40, 0x40, 0x7f])
        );

        board.mem1[base as usize..base as usize + 64].fill(0);
        configure_texture0(&mut board, base, 4, 4, 4, 0);
        board.mem1[base as usize..base as usize + 2].copy_from_slice(&0xf800u16.to_be_bytes());
        assert_eq!(board.gx_texture0_texel(0, 0), Some([0xff, 0, 0, 0xff]));

        board.mem1[base as usize..base as usize + 64].fill(0);
        configure_texture0(&mut board, base, 4, 4, 5, 0);
        board.mem1[base as usize..base as usize + 2].copy_from_slice(&0x801fu16.to_be_bytes());
        assert_eq!(board.gx_texture0_texel(0, 0), Some([0, 0, 0xff, 0xff]));

        board.mem1[base as usize..base as usize + 64].fill(0);
        configure_texture0(&mut board, base, 4, 4, 6, 0);
        board.mem1[base as usize..base as usize + 2].copy_from_slice(&[0x80, 0x11]);
        board.mem1[base as usize + 32..base as usize + 34].copy_from_slice(&[0x22, 0x33]);
        assert_eq!(
            board.gx_texture0_texel(0, 0),
            Some([0x11, 0x22, 0x33, 0x80])
        );

        board.mem1[base as usize..base as usize + 64].fill(0);
        configure_texture0(&mut board, base, 8, 8, 14, 0);
        board.mem1[base as usize..base as usize + 2].copy_from_slice(&0xf800u16.to_be_bytes());
        board.mem1[base as usize + 2..base as usize + 4].copy_from_slice(&0x001fu16.to_be_bytes());
        board.mem1[base as usize + 4] = 0x1b;
        assert_eq!(board.gx_texture0_texel(0, 0), Some([0xff, 0, 0, 0xff]));
        assert_eq!(board.gx_texture0_texel(1, 0), Some([0, 0, 0xff, 0xff]));
        assert_eq!(board.gx_texture0_texel(2, 0), Some([159, 0, 95, 0xff]));
        assert_eq!(board.gx_texture0_texel(3, 0), Some([95, 0, 159, 0xff]));
    }

    #[test]
    fn gx_linear_texture_filter_uses_texel_centers_and_lod_filter_selection() {
        let mut board = idle_board();
        let base = 0x6800u32;
        configure_texture0(&mut board, base, 2, 2, 4, 1 << 4);
        let base = base as usize;
        board.mem1[base..base + 2].copy_from_slice(&0xf800u16.to_be_bytes());
        board.mem1[base + 2..base + 4].copy_from_slice(&0x07e0u16.to_be_bytes());
        board.mem1[base + 8..base + 10].copy_from_slice(&0x001fu16.to_be_bytes());
        board.mem1[base + 10..base + 12].copy_from_slice(&0xffffu16.to_be_bytes());

        assert_eq!(
            board.gx_texture0_sample([0.25, 0.25], true),
            Some([0xff, 0, 0, 0xff])
        );
        assert_eq!(
            board.gx_texture0_sample([0.5, 0.5], true),
            Some([0x7f, 0x7f, 0x7f, 0xff])
        );
        assert_eq!(
            board.gx_texture0_sample([0.5, 0.5], false),
            Some([0xff, 0xff, 0xff, 0xff])
        );

        assert!(board.gx_texture0_linear_filter(0.5, 0.0, 0.0, 0.0));
        assert!(!board.gx_texture0_linear_filter(2.0, 0.0, 0.0, 0.0));
        board.gx.bp_regs[0x80] |= 1 << 7;
        assert!(board.gx_texture0_linear_filter(2.0, 0.0, 0.0, 0.0));
        board.gx.bp_regs[0x80] = (1 << 4) | (64 << 9);
        assert!(!board.gx_texture0_linear_filter(0.5, 0.0, 0.0, 0.0));
        board.gx.bp_regs[0x80] = (1 << 7) | (64 << 9);
        assert!(board.gx_texture0_linear_filter(0.5, 0.0, 0.0, 0.0));
    }

    #[test]
    fn gx_tlut_load_and_paletted_formats_use_tmem_state() {
        let mut board = idle_board();
        let palette_source = 0x7000usize;
        let palette_tmem = 1u32;
        board.mem1[palette_source + 2..palette_source + 4]
            .copy_from_slice(&0xf800u16.to_be_bytes());
        write_gx_bytes(
            &mut board,
            &gx_bp_command(0x64, (palette_source as u32) >> 5),
        );
        write_gx_bytes(&mut board, &gx_bp_command(0x65, palette_tmem | (1 << 10)));
        assert_eq!(
            &board.gx.tmem[0x200..0x220],
            &board.mem1[palette_source..palette_source + 32]
        );

        board.gx.bp_regs[0x98] = palette_tmem | (1 << 10);
        let texture_base = 0x7200u32;
        board.mem1[texture_base as usize] = 0x10;
        configure_texture0(&mut board, texture_base, 8, 8, 8, 0);
        assert_eq!(board.gx_texture0_texel(0, 0), Some([0xff, 0, 0, 0xff]));

        board.mem1[texture_base as usize] = 1;
        configure_texture0(&mut board, texture_base, 8, 4, 9, 0);
        assert_eq!(board.gx_texture0_texel(0, 0), Some([0xff, 0, 0, 0xff]));

        board.mem1[texture_base as usize..texture_base as usize + 2]
            .copy_from_slice(&1u16.to_be_bytes());
        configure_texture0(&mut board, texture_base, 4, 4, 10, 0);
        assert_eq!(board.gx_texture0_texel(0, 0), Some([0xff, 0, 0, 0xff]));

        board.gx.tmem[0x200..0x202].copy_from_slice(&[0x40, 0x80]);
        board.gx.bp_regs[0x98] = palette_tmem;
        assert_eq!(board.gx_tlut_texel(0), Some([0x80, 0x80, 0x80, 0x40]));

        board.gx.tmem[0x200..0x202].copy_from_slice(&0x801fu16.to_be_bytes());
        board.gx.bp_regs[0x98] = palette_tmem | (2 << 10);
        assert_eq!(board.gx_tlut_texel(0), Some([0, 0, 0xff, 0xff]));
    }

    #[test]
    fn gx_textured_triangle_modulates_vertex_color_into_efb() {
        let mut board = idle_board();
        let vcd = (1 << 9) | (1 << 13);
        let vat_a = (4 << 1) | (1 << 13) | (5 << 14) | (1 << 21) | (4 << 22);
        configure_gx_2d(&mut board, vcd, vat_a);
        write_gx_bytes(&mut board, &gx_cp_command(0x60, 1));

        let texture_base = 0x6200u32;
        for texel in board.mem1[texture_base as usize..texture_base as usize + 32]
            .as_chunks_mut::<2>()
            .0
        {
            *texel = 0xf800u16.to_be_bytes();
        }
        write_gx_bytes(&mut board, &gx_bp_command(0x80, 0));
        write_gx_bytes(&mut board, &gx_bp_command(0x88, 1 | (1 << 10) | (4 << 20)));
        write_gx_bytes(&mut board, &gx_bp_command(0x94, texture_base >> 5));

        let mut primitive = vec![0x90, 0, 3];
        for (x, y, s, t) in [
            (-0.5f32, -0.5f32, 0.0f32, 0.0f32),
            (0.5, -0.5, 1.0, 0.0),
            (0.0, 0.5, 0.5, 1.0),
        ] {
            primitive.extend_from_slice(&gx_direct_xy_rgba_st_vertex(
                x,
                y,
                [0x80, 0xff, 0xff, 0xff],
                s,
                t,
            ));
        }
        write_gx_bytes(&mut board, &primitive);
        assert_eq!(board.gx_efb_pixel(320, 240), [0x80, 0, 0, 0xff]);
    }

    #[test]
    fn gx_depth_test_and_write_order_overlapping_primitives() {
        let mut board = idle_board();
        let vcd = (1 << 9) | (1 << 13);
        let vat_a = 1 | (4 << 1) | (1 << 13) | (5 << 14);
        configure_gx_2d(&mut board, vcd, vat_a);
        write_gx_bytes(&mut board, &gx_bp_command(0x40, 1 | (1 << 1) | (1 << 4)));

        draw_gx_xyz_triangle(&mut board, 0.75, [255, 0, 0, 255]);
        let first_depth = board.gx_efb_depth(320, 240);
        assert_eq!(board.gx_efb_pixel(320, 240), [255, 0, 0, 255]);
        assert!(first_depth < 0x00ff_ffff);

        draw_gx_xyz_triangle(&mut board, 0.9, [0, 255, 0, 255]);
        assert_eq!(board.gx_efb_pixel(320, 240), [255, 0, 0, 255]);
        assert_eq!(board.gx_efb_depth(320, 240), first_depth);

        draw_gx_xyz_triangle(&mut board, 0.5, [0, 0, 255, 255]);
        assert_eq!(board.gx_efb_pixel(320, 240), [0, 0, 255, 255]);
        assert!(board.gx_efb_depth(320, 240) < first_depth);
    }

    #[test]
    fn gx_cull_modes_follow_predivide_projected_facing() {
        let mut board = idle_board();
        let vcd = (1 << 9) | (1 << 13);
        let vat_a = (4 << 1) | (1 << 13) | (5 << 14);
        configure_gx_2d(&mut board, vcd, vat_a);

        let draw = |board: &mut GameCubeBoard, reversed: bool, color: [u8; 4]| {
            let points = if reversed {
                [(-0.5f32, -0.5f32), (0.0, 0.5), (0.5, -0.5)]
            } else {
                [(-0.5f32, -0.5f32), (0.5, -0.5), (0.0, 0.5)]
            };
            let mut primitive = vec![0x90, 0, 3];
            for (x, y) in points {
                primitive.extend_from_slice(&gx_direct_xy_rgba_vertex(x, y, color));
            }
            write_gx_bytes(board, &primitive);
        };

        write_gx_bytes(&mut board, &gx_bp_command(0x00, 1 << 14));
        draw(&mut board, false, [255, 0, 0, 255]);
        assert_eq!(board.gx_efb_pixel(320, 240), [0, 0, 0, 0]);
        draw(&mut board, true, [0, 255, 0, 255]);
        assert_eq!(board.gx_efb_pixel(320, 240), [0, 255, 0, 255]);

        board.efb[..GX_EFB_COLOR_BYTES].fill(0);
        write_gx_bytes(&mut board, &gx_bp_command(0x00, 2 << 14));
        draw(&mut board, false, [0, 0, 255, 255]);
        assert_eq!(board.gx_efb_pixel(320, 240), [0, 0, 255, 255]);
        board.efb[..GX_EFB_COLOR_BYTES].fill(0);
        draw(&mut board, true, [255, 255, 0, 255]);
        assert_eq!(board.gx_efb_pixel(320, 240), [0, 0, 0, 0]);

        write_gx_bytes(&mut board, &gx_bp_command(0x00, 3 << 14));
        draw(&mut board, false, [255, 0, 255, 255]);
        assert_eq!(board.gx_efb_pixel(320, 240), [0, 0, 0, 0]);
    }

    #[test]
    fn gx_blend_logic_write_masks_and_constant_alpha_follow_bp_state() {
        let mut board = idle_board();
        let vcd = (1 << 9) | (1 << 13);
        let vat_a = (4 << 1) | (1 << 13) | (5 << 14);
        configure_gx_2d(&mut board, vcd, vat_a);

        let draw = |board: &mut GameCubeBoard, color: [u8; 4]| {
            let mut primitive = vec![0x90, 0, 3];
            for (x, y) in [(-0.5f32, -0.5f32), (0.5, -0.5), (0.0, 0.5)] {
                primitive.extend_from_slice(&gx_direct_xy_rgba_vertex(x, y, color));
            }
            write_gx_bytes(board, &primitive);
        };

        draw(&mut board, [255, 0, 0, 255]);
        let alpha_blend = 1 | (1 << 3) | (1 << 4) | (5 << 5) | (4 << 8);
        write_gx_bytes(&mut board, &gx_bp_command(0x41, alpha_blend));
        draw(&mut board, [0, 255, 0, 128]);
        assert_eq!(board.gx_efb_pixel(320, 240), [126, 128, 0, 191]);

        write_gx_bytes(&mut board, &gx_bp_command(0x42, (1 << 8) | 77));
        write_gx_bytes(&mut board, &gx_bp_command(0x41, 1 << 4));
        draw(&mut board, [0, 0, 255, 200]);
        assert_eq!(board.gx_efb_pixel(320, 240), [126, 128, 0, 77]);

        write_gx_bytes(&mut board, &gx_bp_command(0x42, 0));
        write_gx_bytes(
            &mut board,
            &gx_bp_command(0x41, (1 << 1) | (1 << 3) | (1 << 4) | (6 << 12)),
        );
        draw(&mut board, [255, 15, 240, 170]);
        assert_eq!(board.gx_efb_pixel(320, 240), [129, 143, 240, 231]);
    }

    #[test]
    fn gx_indexed_position_and_color_arrays_feed_rasterizer() {
        let mut board = idle_board();
        let vcd = (2 << 9) | (2 << 13);
        let vat_a = (4 << 1) | (1 << 13) | (5 << 14);
        configure_gx_2d(&mut board, vcd, vat_a);
        write_gx_bytes(&mut board, &gx_cp_command(0xa0, 0x5000));
        write_gx_bytes(&mut board, &gx_cp_command(0xb0, 8));
        write_gx_bytes(&mut board, &gx_cp_command(0xa2, 0x5100));
        write_gx_bytes(&mut board, &gx_cp_command(0xb2, 4));

        for (index, (x, y)) in [(-0.5f32, -0.5f32), (0.5, -0.5), (0.0, 0.5)]
            .into_iter()
            .enumerate()
        {
            let offset = 0x5000 + index * 8;
            board.mem1[offset..offset + 4].copy_from_slice(&x.to_bits().to_be_bytes());
            board.mem1[offset + 4..offset + 8].copy_from_slice(&y.to_bits().to_be_bytes());
            board.mem1[0x5100 + index * 4..0x5104 + index * 4].copy_from_slice(&[0, 255, 0, 255]);
        }

        write_gx_bytes(&mut board, &[0x90, 0, 3, 0, 0, 1, 1, 2, 2]);
        assert_eq!(board.gx_efb_pixel(320, 240), [0, 255, 0, 255]);
    }

    #[test]
    fn gx_efb_copy_reaches_xfb_scanout_and_clears_after_copy() {
        let mut board = idle_board();
        let vcd = (1 << 9) | (1 << 13);
        let vat_a = (4 << 1) | (1 << 13) | (5 << 14);
        configure_gx_2d(&mut board, vcd, vat_a);

        let mut primitive = vec![0x90, 0, 3];
        for (x, y) in [(-0.5f32, -0.5f32), (0.5, -0.5), (0.0, 0.5)] {
            primitive.extend_from_slice(&gx_direct_xy_rgba_vertex(x, y, [255, 0, 0, 255]));
        }
        write_gx_bytes(&mut board, &primitive);
        assert_eq!(board.gx_efb_pixel(320, 240), [255, 0, 0, 255]);

        write_gx_bytes(&mut board, &gx_bp_command(0x49, 319 | (239 << 10)));
        write_gx_bytes(&mut board, &gx_bp_command(0x4a, 1 | (1 << 10)));
        write_gx_bytes(&mut board, &gx_bp_command(0x4b, 0x4000 >> 5));
        write_gx_bytes(&mut board, &gx_bp_command(0x4d, 1));
        write_gx_bytes(&mut board, &gx_bp_command(0x4e, 256));
        write_gx_bytes(&mut board, &gx_bp_command(0x4f, 0xff00));
        write_gx_bytes(&mut board, &gx_bp_command(0x50, 0));
        write_gx_bytes(&mut board, &gx_bp_command(0x40, 1 << 4));
        write_gx_bytes(&mut board, &gx_bp_command(0x51, 0x12_3456));
        write_gx_bytes(&mut board, &gx_bp_command(0x52, (1 << 14) | (1 << 11)));

        assert_eq!(board.gx_efb_pixel(320, 240), [0, 0, 0, 255]);
        assert_eq!(board.gx_efb_depth(320, 240), 0x12_3456);
        assert_ne!(&board.mem1[0x4000..0x4004], &[0, 0, 0, 0]);
        board.write_mmio16(VI_FB_LEFT_TOP_HI, 0);
        board.write_mmio16(VI_FB_LEFT_TOP_LO, 0x4000);
        board.present_video();
        let pixel = &board.video.pixels()[..4];
        assert!(pixel[0] > 240 && pixel[1] < 20 && pixel[2] < 20);
        assert_eq!(pixel[3], 255);
    }

    #[test]
    fn gx_fifo_loads_xf_bp_indexed_state_and_nested_display_lists() {
        let mut board = idle_board();
        board.mem1[0x5008..0x500c].copy_from_slice(&0x1122_3344u32.to_be_bytes());
        board.mem1[0x500c..0x5010].copy_from_slice(&0x5566_7788u32.to_be_bytes());
        write_gx_bytes(&mut board, &gx_cp_command(0xac, 0x5000));
        write_gx_bytes(&mut board, &gx_cp_command(0xbc, 8));

        let mut indexed = vec![0x20];
        indexed.extend_from_slice(&0x0001_1120u32.to_be_bytes());
        write_gx_bytes(&mut board, &indexed);
        assert_eq!(board.gx.xf_regs[0x120], 0x1122_3344);
        assert_eq!(board.gx.xf_regs[0x121], 0x5566_7788);

        let mut xf = vec![0x10];
        xf.extend_from_slice(&0x0001_0100u32.to_be_bytes());
        xf.extend_from_slice(&0xaabb_ccddu32.to_be_bytes());
        xf.extend_from_slice(&0x0102_0304u32.to_be_bytes());
        write_gx_bytes(&mut board, &xf);
        assert_eq!(board.gx.xf_regs[0x100], 0xaabb_ccdd);
        assert_eq!(board.gx.xf_regs[0x101], 0x0102_0304);

        let mut viewport = vec![0x10];
        viewport.extend_from_slice(&0x0005_101au32.to_be_bytes());
        for value in [320.0f32, -240.0, 1.0, 342.0, 342.0, 0.0] {
            viewport.extend_from_slice(&value.to_bits().to_be_bytes());
        }
        write_gx_bytes(&mut board, &viewport);
        assert_eq!(board.gx.xf_regs[0x101a], 320.0f32.to_bits());
        assert_eq!(board.gx.xf_regs[0x101f], 0.0f32.to_bits());

        let mut projection = vec![0x10];
        projection.extend_from_slice(&0x0006_1020u32.to_be_bytes());
        for value in [1.0f32, 0.0, 1.0, 0.0, 1.0, 0.0] {
            projection.extend_from_slice(&value.to_bits().to_be_bytes());
        }
        projection.extend_from_slice(&0u32.to_be_bytes());
        write_gx_bytes(&mut board, &projection);
        assert_eq!(board.gx.xf_regs[0x1020], 1.0f32.to_bits());
        assert_eq!(board.gx.xf_regs[0x1026], 0);

        write_gx_bytes(&mut board, &[0x61, 0x42, 0x11, 0x22, 0x33]);
        assert_eq!(board.gx.bp_regs[0x42], 0x0011_2233);

        let mut display_list = [0u8; 32];
        display_list[..5].copy_from_slice(&[0x61, 0x43, 0x44, 0x55, 0x66]);
        board.mem1[0x6000..0x6020].copy_from_slice(&display_list);
        let mut call = vec![0x40];
        call.extend_from_slice(&0x0000_6000u32.to_be_bytes());
        call.extend_from_slice(&0x0000_0020u32.to_be_bytes());
        write_gx_bytes(&mut board, &call);
        assert_eq!(board.gx.bp_regs[0x43], 0x0044_5566);
        assert!(board.gx.commands_processed >= 9);
    }

    #[test]
    fn gx_partial_fifo_and_register_state_round_trip_deterministically() {
        let image = dol_bytes(&[0x4800_0000]);
        let mut machine = GameCubeMachine::from_image(ResourceBlob::from_bytes(&image)).unwrap();
        machine.board.gx.cp_regs[0x50] = 0x1234_5678;
        machine.board.gx.bp_regs[0x61] = 0x00ab_cdef;
        machine.board.gx.xf_regs[0x234] = 0xfeed_beef;
        machine.board.gx.xf_regs[0x1026] = 1;
        machine.board.gx.depth[..3].copy_from_slice(&[0x12, 0x34, 0x56]);
        machine.board.gx.tmem[0x200..0x204].copy_from_slice(&[0xde, 0xad, 0xbe, 0xef]);
        machine.board.gx.fifo = vec![0x08, 0x50, 0x12];
        machine.board.gx.commands_processed = 41;
        machine.board.gx.primitives_processed = 7;
        machine.board.gx.vertices_processed = 29;
        let state = machine.save_state().unwrap();

        let mut restored = GameCubeMachine::from_image(ResourceBlob::from_bytes(&image)).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.board.gx.cp_regs[0x50], 0x1234_5678);
        assert_eq!(restored.board.gx.bp_regs[0x61], 0x00ab_cdef);
        assert_eq!(restored.board.gx.xf_regs[0x234], 0xfeed_beef);
        assert_eq!(restored.board.gx.xf_regs[0x1026], 1);
        assert_eq!(&restored.board.gx.depth[..3], &[0x12, 0x34, 0x56]);
        assert_eq!(
            &restored.board.gx.tmem[0x200..0x204],
            &[0xde, 0xad, 0xbe, 0xef]
        );
        assert_eq!(restored.board.gx.fifo, vec![0x08, 0x50, 0x12]);
        assert_eq!(restored.board.gx.commands_processed, 41);
        assert_eq!(restored.board.gx.primitives_processed, 7);
        assert_eq!(restored.board.gx.vertices_processed, 29);
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
    fn exi_ipl_uart_accepts_debug_writes_and_reports_empty_fifo() {
        let mut board = idle_board();

        exi_select_ipl(&mut board);
        exi_imm_write(&mut board, exi_ipl_command(EXI_IPL_UART_BASE, false), 4);
        assert_eq!(exi_imm_read(&mut board, 1), 0);

        board.write_mmio32(EXI_STATUS, 0);
        exi_select_ipl(&mut board);
        exi_imm_write(&mut board, exi_ipl_command(EXI_IPL_UART_BASE, true), 4);
        exi_imm_write(&mut board, u32::from(b'X') << 24, 1);
        exi_imm_write(&mut board, u32::from(b'\r') << 24, 1);

        board.write_mmio32(EXI_STATUS, 0);
        exi_select_ipl(&mut board);
        exi_imm_write(
            &mut board,
            exi_ipl_command(EXI_IPL_UART_BASE + 0x0c, false),
            4,
        );
        assert_eq!(exi_imm_read(&mut board, 1), 0);

        board.write_mmio32(EXI_STATUS, 0);
        exi_select_ipl(&mut board);
        exi_imm_write(
            &mut board,
            exi_ipl_command(EXI_IPL_UART_BASE + 0x4c, true),
            4,
        );
        exi_imm_write(&mut board, 0x5a00_0000, 1);
        assert_ne!(board.read_mmio32(EXI_STATUS) & EXI_STATUS_TCINT, 0);

        board.write_mmio32(EXI_STATUS, 0);
        exi_select_ipl(&mut board);
        exi_imm_write(&mut board, exi_ipl_command(EXI_IPL_EUART_BASE, false), 4);
        assert_eq!(exi_imm_read(&mut board, 1), 0xff00_0000);

        board.write_mmio32(EXI_STATUS, 0);
        exi_select_ipl(&mut board);
        exi_imm_write(
            &mut board,
            exi_ipl_command(EXI_IPL_EUART_BASE + 4, false),
            4,
        );
        assert_eq!(exi_imm_read(&mut board, 1), 0);

        board.write_mmio32(EXI_STATUS, 0);
        exi_select_ipl(&mut board);
        exi_imm_write(&mut board, exi_ipl_command(EXI_IPL_EUART_BASE + 4, true), 4);
        exi_imm_write(&mut board, u32::from(b'Y') << 24, 1);
        assert_ne!(board.read_mmio32(EXI_STATUS) & EXI_STATUS_TCINT, 0);
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
    fn exi_ad16_channel_two_reports_id_round_trips_register_and_interrupts() {
        let mut board = idle_board();
        let channel = 2;
        board.write_mmio32(
            exi_register(channel, EXI_STATUS),
            (1 << 7) | EXI_STATUS_TCINTMASK,
        );
        assert_ne!(
            board.read_mmio32(exi_register(channel, EXI_STATUS)) & EXI_STATUS_EXT,
            0
        );

        exi_imm_write_channel(&mut board, channel, 0, 2);
        assert_eq!(exi_imm_read_channel(&mut board, channel, 4), 0x0412_0000);
        assert_ne!(
            board.read_mmio32(exi_register(channel, EXI_STATUS)) & EXI_STATUS_TCINT,
            0
        );
        assert_ne!(board.mmio_read_u32_raw(PI_CAUSE) & PI_INT_EXI, 0);

        board.write_mmio32(exi_register(channel, EXI_STATUS), EXI_STATUS_TCINT);
        board.write_mmio32(
            exi_register(channel, EXI_STATUS),
            (1 << 7) | EXI_STATUS_TCINTMASK,
        );
        exi_imm_write_channel(&mut board, channel, 0xa000_0000, 1);
        exi_imm_write_channel(&mut board, channel, 0x1234_5678, 4);

        board.write_mmio32(exi_register(channel, EXI_STATUS), 0);
        board.write_mmio32(
            exi_register(channel, EXI_STATUS),
            (1 << 7) | EXI_STATUS_TCINTMASK,
        );
        exi_imm_write_channel(&mut board, channel, 0xa200_0000, 1);
        assert_eq!(exi_imm_read_channel(&mut board, channel, 4), 0x1234_5678);
        assert_eq!(board.exi.ad16.register, 0x1234_5678u32.to_be_bytes());
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
        let mut full = disc_bytes(&[0x4800_0000]);
        full.resize(0x3000, 0);
        let (mut board, _) = GameCubeBoard::new(ResourceBlob::from_bytes(&full)).unwrap();
        board.di.enable_dtk = true;
        board.di.stream = true;
        board.di.current_start = 0x1000;
        board.di.current_length = 0x40;
        board.di.next_start = 0x2000;
        board.di.next_length = 0x40;
        board.di.audio_position = 0x1000;

        board.render_dtk_samples((DTK_SAMPLES_PER_BLOCK - 1) as usize);
        assert_eq!(board.di.audio_position, 0x1000);
        assert_eq!(board.di.dtk_sample_remainder, 27);
        board.render_dtk_samples(1);
        assert_eq!(board.di.audio_position, 0x1020);
        assert_eq!(board.di.dtk_sample_remainder, 0);

        board.render_dtk_samples(DTK_SAMPLES_PER_BLOCK as usize);
        assert_eq!(board.di.audio_position, 0x1040);
        board.render_dtk_samples(DTK_SAMPLES_PER_BLOCK as usize);
        assert_eq!(board.di.current_start, 0x2000);
        assert_eq!(board.di.audio_position, 0x2020);

        board.di.audio_position = 0x2040;
        board.di.stop_at_track_end = true;
        board.render_dtk_samples(1);
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
    fn dvd_dtk_streaming_requests_missing_block_before_decoding() {
        let mut full = disc_bytes(&[0x4800_0000]);
        full[0x1000] = 0x0c;
        full[0x1001] = 0x0c;
        full[0x1004..0x1020].fill(0x1f);
        let mut sparse = ResourceBlob::streaming(full.len() as u64, 2).unwrap();
        sparse.write(0, &full[..0x700]).unwrap();
        let (mut board, _) = GameCubeBoard::new(sparse).unwrap();
        board.di.enable_dtk = true;
        board.di.stream = true;
        board.di.current_start = 0x1000;
        board.di.current_length = 0x20;
        board.di.audio_position = 0x1000;

        let missing = board.render_dtk_samples(1);
        assert_eq!(missing, vec![(0.0, 0.0)]);
        assert_eq!(board.disc.pending_range(), Some((0x1000, 0x1020)));
        assert_eq!(board.di.audio_position, 0x1000);
        assert_eq!(board.di.dtk_sample_remainder, 0);

        let mut staged = board.disc.clone();
        staged.write(0x1000, &full[0x1000..0x1020]).unwrap();
        let decoded = board.render_dtk_samples(1);
        assert_eq!(decoded[0].0, -(1.0 / 32768.0));
        assert_eq!(decoded[0].1, 1.0 / 32768.0);
        assert_eq!(board.di.dtk_sample_remainder, 1);
        assert_eq!(board.di.dtk_left_recent, -64);
        assert_eq!(board.di.dtk_right_recent, 64);
    }

    #[test]
    fn dvd_dtk_adpcm_decodes_stereo_and_mixes_with_audio_dma() {
        let mut full = disc_bytes(&[0x4800_0000]);
        full[0x1000] = 0x0c;
        full[0x1001] = 0x0c;
        full[0x1004..0x1020].fill(0x1f);
        let (mut board, _) = GameCubeBoard::new(ResourceBlob::from_bytes(&full)).unwrap();
        board.di.enable_dtk = true;
        board.di.stream = true;
        board.di.current_start = 0x1000;
        board.di.current_length = 0x20;
        board.di.audio_position = 0x1000;

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
        assert_eq!(audio.samples()[0], 0.5 - (1.0 / 32768.0));
        assert_eq!(audio.samples()[1], -0.5 + (1.0 / 32768.0));
        assert!(!board.di.stream);
        assert_eq!(board.di.dtk_sample_remainder, 0);
        assert_eq!(board.di.dtk_left_recent, 0);
        assert_eq!(board.di.dtk_right_recent, 0);
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
        machine.board.exi.channel2.imm_data = 0x0bad_f00d;
        machine.board.exi.ad16.command = 0xa2;
        machine.board.exi.ad16.position = 3;
        machine.board.exi.ad16.register = 0x1357_9bdfu32.to_be_bytes();
        machine.board.di.drive_state = DiDriveState::Ready;
        machine.board.di.enable_dtk = true;
        machine.board.di.dtk_buffer_length = 10;
        machine.board.di.dtk_sample_remainder = 17;
        machine.board.di.dtk_left_recent = -12345;
        machine.board.di.dtk_left_older = 6789;
        machine.board.di.dtk_right_recent = 23456;
        machine.board.di.dtk_right_older = -7890;
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
        assert_eq!(restored.board.exi.channel2.imm_data, 0x0bad_f00d);
        assert_eq!(restored.board.exi.ad16.command, 0xa2);
        assert_eq!(restored.board.exi.ad16.position, 3);
        assert_eq!(
            restored.board.exi.ad16.register,
            0x1357_9bdfu32.to_be_bytes()
        );
        assert_eq!(restored.board.di.drive_state, DiDriveState::Ready);
        assert!(restored.board.di.enable_dtk);
        assert_eq!(restored.board.di.dtk_buffer_length, 10);
        assert_eq!(restored.board.di.dtk_sample_remainder, 17);
        assert_eq!(restored.board.di.dtk_left_recent, -12345);
        assert_eq!(restored.board.di.dtk_left_older, 6789);
        assert_eq!(restored.board.di.dtk_right_recent, 23456);
        assert_eq!(restored.board.di.dtk_right_older, -7890);
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
