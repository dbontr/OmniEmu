use crate::cpu_mips_r4300::{Mips64Bus, MipsR4300};
use crate::cpu_rsp::{Rsp, RspBus};
use crate::input::{
    AXIS_LEFT_X, AXIS_LEFT_Y, DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, L1, LEFT, R1,
    RIGHT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;
use crate::state::{StateReader, StateWriter};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const CPU_HZ: u64 = 93_750_000;
const RSP_HZ: u64 = 62_500_000;
const FRAME_HZ: u64 = 60;
const AUDIO_RATE: u32 = 48_000;
const NTSC_VIDEO_CLOCK_HZ: u64 = 48_681_818;
const RDRAM_SIZE: usize = 8 * 1024 * 1024;
const RDRAM_HIDDEN_SIZE: usize = RDRAM_SIZE / 8;
const SP_MEM_SIZE: usize = 8 * 1024;
const SAVE_SIZE: usize = 128 * 1024;
const EEPROM_WRITE_CYCLES: u32 = (CPU_HZ as u32 / 1_000) * 6;
const CONTROLLER_PAK_SIZE: usize = 32 * 1024;
const RTC_SIZE: usize = 32;
const STATE_VERSION: u32 = 19;
const DEFAULT_CONVERT_K: [i16; 6] = [175, -43, -89, 222, 114, 42];
const RDP_PERSPECTIVE_TABLE: [(i16, i16); 64] = [
    (0x4000, -252 * 4),
    (0x3f04, -244 * 4),
    (0x3e10, -238 * 4),
    (0x3d22, -230 * 4),
    (0x3c3c, -223 * 4),
    (0x3b5d, -218 * 4),
    (0x3a83, -210 * 4),
    (0x39b1, -205 * 4),
    (0x38e4, -200 * 4),
    (0x381c, -194 * 4),
    (0x375a, -189 * 4),
    (0x369d, -184 * 4),
    (0x35e5, -179 * 4),
    (0x3532, -175 * 4),
    (0x3483, -170 * 4),
    (0x33d9, -166 * 4),
    (0x3333, -162 * 4),
    (0x3291, -157 * 4),
    (0x31f4, -155 * 4),
    (0x3159, -150 * 4),
    (0x30c3, -147 * 4),
    (0x3030, -143 * 4),
    (0x2fa1, -140 * 4),
    (0x2f15, -137 * 4),
    (0x2e8c, -134 * 4),
    (0x2e06, -131 * 4),
    (0x2d83, -128 * 4),
    (0x2d03, -125 * 4),
    (0x2c86, -123 * 4),
    (0x2c0b, -120 * 4),
    (0x2b93, -117 * 4),
    (0x2b1e, -115 * 4),
    (0x2aab, -113 * 4),
    (0x2a3a, -110 * 4),
    (0x29cc, -108 * 4),
    (0x2960, -106 * 4),
    (0x28f6, -104 * 4),
    (0x288e, -102 * 4),
    (0x2828, -100 * 4),
    (0x27c4, -98 * 4),
    (0x2762, -96 * 4),
    (0x2702, -94 * 4),
    (0x26a4, -92 * 4),
    (0x2648, -91 * 4),
    (0x25ed, -89 * 4),
    (0x2594, -87 * 4),
    (0x253d, -86 * 4),
    (0x24e7, -85 * 4),
    (0x2492, -83 * 4),
    (0x243f, -81 * 4),
    (0x23ee, -80 * 4),
    (0x239e, -79 * 4),
    (0x234f, -77 * 4),
    (0x2302, -76 * 4),
    (0x22b6, -74 * 4),
    (0x226c, -74 * 4),
    (0x2222, -72 * 4),
    (0x21da, -71 * 4),
    (0x2193, -70 * 4),
    (0x214d, -69 * 4),
    (0x2108, -67 * 4),
    (0x20c5, -67 * 4),
    (0x2082, -65 * 4),
    (0x2041, -65 * 4),
];

const RDP_DITHER_MAGIC: [u8; 16] = [0, 6, 1, 7, 4, 2, 5, 3, 3, 5, 2, 4, 7, 1, 6, 0];
const RDP_DITHER_BAYER: [u8; 16] = [0, 4, 1, 5, 4, 0, 5, 1, 3, 7, 2, 6, 7, 3, 6, 2];

const FLASH_READ_ARRAY: u8 = 0;
const FLASH_STATUS: u8 = 1;
const FLASH_SILICON_ID: u8 = 2;
const FLASH_PAGE_PROGRAM: u8 = 3;
const FLASH_SECTOR_ERASE: u8 = 4;
const FLASH_CHIP_ERASE: u8 = 5;
const FLASH_TYPE_ID: u32 = 0x1111_8001;
const FLASH_DEVICE_ID: u32 = 0x00c2_001e;

const MI_SP: u32 = 1 << 0;
const MI_SI: u32 = 1 << 1;
const MI_AI: u32 = 1 << 2;
const MI_VI: u32 = 1 << 3;
const MI_PI: u32 = 1 << 4;
const MI_DP: u32 = 1 << 5;

const DP_STATUS_XBUS: u32 = 1 << 0;
const DP_STATUS_FREEZE: u32 = 1 << 1;
const DP_STATUS_FLUSH: u32 = 1 << 2;
const DP_STATUS_CMD_BUSY: u32 = 1 << 6;
const DP_STATUS_START_VALID: u32 = 1 << 10;

fn eeprom_size_for_rom(rom: &[u8]) -> usize {
    if rom.len() <= 0x3d {
        return 0;
    }
    let id = [rom[0x3b], rom[0x3c], rom[0x3d]];
    const EEPROM_16K_IDS: &[[u8; 3]] = &[
        *b"NB7", *b"NGT", *b"NFU", *b"NCW", *b"NCZ", *b"ND6", *b"NDO", *b"ND2", *b"N3D", *b"NMX",
        *b"NGC", *b"NIM", *b"NNB", *b"NMV", *b"NM8", *b"NEV", *b"NPP", *b"NUB", *b"NPD", *b"NRZ",
        *b"NR7", *b"NEP", *b"NYS",
    ];
    if EEPROM_16K_IDS.contains(&id) {
        return 2 * 1024;
    }
    const EEPROM_4K_IDS: &[[u8; 3]] = &[
        *b"NTW", *b"NHF", *b"NOS", *b"NTC", *b"NER", *b"NAG", *b"NAB", *b"NS3", *b"NTN", *b"NBN",
        *b"NBK", *b"NFH", *b"NMU", *b"NBC", *b"NBH", *b"NHA", *b"NBM", *b"NBV", *b"NBD", *b"NCT",
        *b"NCH", *b"NCG", *b"NP2", *b"NXO", *b"NCU", *b"NCX", *b"NDY", *b"NDQ", *b"NDR", *b"NN6",
        *b"NDU", *b"NJM", *b"NFW", *b"NF2", *b"NKA", *b"NFG", *b"NGL", *b"NGV", *b"NGE", *b"NHP",
        *b"NPG", *b"NIJ", *b"NIC", *b"NFY", *b"NKI", *b"NLL", *b"NLR", *b"NKT", *b"CLB", *b"NLB",
        *b"NMW", *b"NML", *b"NTM", *b"NMI", *b"NMG", *b"NMO", *b"NMS", *b"NMR", *b"NCR", *b"NEA",
        *b"NPW", *b"NPY", *b"NPT", *b"NRA", *b"NWQ", *b"NSU", *b"NSN", *b"NK2", *b"NSV", *b"NFX",
        *b"NS6", *b"NNA", *b"NRS", *b"NSW", *b"NSC", *b"NSA", *b"NB6", *b"NSS", *b"NTX", *b"NT6",
        *b"NTP", *b"NTJ", *b"NRC", *b"NTR", *b"NTB", *b"NGU", *b"NIR", *b"NVL", *b"NVY", *b"NJK",
        *b"NWC", *b"NAD", *b"NWU", *b"NYK", *b"NMZ",
    ];
    if EEPROM_4K_IDS.contains(&id) {
        512
    } else {
        0
    }
}

fn flash_present_for_rom(rom: &[u8]) -> bool {
    if rom.len() <= 0x3d {
        return false;
    }
    let id = [rom[0x3b], rom[0x3c], rom[0x3d]];
    const FLASH_IDS: &[[u8; 3]] = &[
        *b"NCC", *b"NCV", *b"NDA", *b"NAF", *b"NJF", *b"NKJ", *b"NZS", *b"NM6", *b"NCK", *b"NMQ",
        *b"NPN", *b"NPF", *b"NPO", *b"CP2", *b"NP3", *b"NRH", *b"NSQ", *b"NT9", *b"NW4", *b"NDP",
    ];
    FLASH_IDS.contains(&id)
}

fn rtc_present_for_rom(rom: &[u8]) -> bool {
    rom.get(0x3b..0x3e) == Some(b"NAF")
}

fn default_rtc_ram() -> [u8; RTC_SIZE] {
    let mut ram = [0; RTC_SIZE];
    ram[16..24].copy_from_slice(&[0x00, 0x00, 0x80, 0x01, 0x06, 0x01, 0x00, 0x20]);
    ram
}

fn cic_seed_for_crc(crc: u32) -> u8 {
    match crc {
        0x6170_a4a1 | 0x90bb_6cb5 => 0x3f,
        0x0b05_0ee0 => 0x78,
        0x98bc_2c86 => 0x91,
        0xacc8_580a => 0x85,
        _ => 0x3f,
    }
}

fn cic_seed_for_rom(rom: &[u8]) -> u8 {
    if rom.len() < 0x1000 {
        return 0x3f;
    }
    let mut crc = 0xffff_ffffu32;
    for &byte in &rom[0x40..0x1000] {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    cic_seed_for_crc(!crc)
}

fn tv_type_for_rom(rom: &[u8]) -> u64 {
    match rom.get(0x3e).copied().unwrap_or(b'E') {
        b'B' => 2,
        b'D' | b'F' | b'H' | b'I' | b'L' | b'P' | b'S' | b'U' | b'W' | b'X' | b'Y' => 0,
        _ => 1,
    }
}

fn normalize_rom(image: &[u8]) -> Result<Vec<u8>, String> {
    if image.len() < 0x1000 {
        return Err("Nintendo 64 cartridge must contain at least 4 KiB".into());
    }
    let mut rom = image.to_vec();
    match u32::from_be_bytes([rom[0], rom[1], rom[2], rom[3]]) {
        0x8037_1240 => {}
        0x3780_4012 => {
            for pair in rom.as_chunks_mut::<2>().0 {
                pair.swap(0, 1);
            }
        }
        0x4012_3780 => {
            for word in rom.as_chunks_mut::<4>().0 {
                word.reverse();
            }
        }
        magic => {
            return Err(format!(
                "unsupported Nintendo 64 ROM byte order {magic:#010x}"
            ))
        }
    }
    Ok(rom)
}

#[derive(Clone, Copy, Default)]
struct SpDmaJob {
    mem_addr: u32,
    dram_addr: u32,
    length: u32,
    count: u32,
    skip: u32,
    direction: u8,
}

#[derive(Clone)]
struct SpState {
    mem_addr: u32,
    dram_addr: u32,
    rd_len: u32,
    wr_len: u32,
    status: u32,
    semaphore: bool,
    pc: u32,
    dma_current: SpDmaJob,
    dma_pending: SpDmaJob,
    dma_busy: bool,
    dma_full: bool,
    dma_rsp_cycles: u32,
    dma_phase: u64,
}
impl Default for SpState {
    fn default() -> Self {
        Self {
            mem_addr: 0,
            dram_addr: 0,
            rd_len: 0xff8,
            wr_len: 0xff8,
            status: 1,
            semaphore: false,
            pc: 0,
            dma_current: SpDmaJob::default(),
            dma_pending: SpDmaJob::default(),
            dma_busy: false,
            dma_full: false,
            dma_rsp_cycles: 0,
            dma_phase: 0,
        }
    }
}

#[derive(Clone, Default)]
struct DpState {
    start: u32,
    end: u32,
    current: u32,
    status: u32,
    clock: u32,
}

#[derive(Clone, Copy, Default)]
struct RdpTile {
    format: u8,
    size: u8,
    line: u16,
    tmem: u16,
    palette: u8,
    clamp_t: bool,
    mirror_t: bool,
    mask_t: u8,
    shift_t: u8,
    clamp_s: bool,
    mirror_s: bool,
    mask_s: u8,
    shift_s: u8,
    sl: u16,
    tl: u16,
    sh: u16,
    th: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct RdpLodSignals {
    frac: i16,
    level: u32,
    magnify: bool,
    distant: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct RdpDepthInputs {
    current_depth: u16,
    current_dz: u8,
    current_coverage: i32,
    z_compare: bool,
    z_mode: u8,
    force_blend: bool,
    aa_enable: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct RdpDepthResult {
    depth_pass: bool,
    blend_en: bool,
    coverage_wrap: bool,
    blend_shift: [u8; 2],
    coverage_count: i32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct RdpDepthSample {
    depth: i32,
    delta_z: i32,
    compressed_delta_z: i32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct RdpTrianglePixel {
    y: usize,
    x: usize,
    major_x: i64,
    yh: i32,
    coverage_mask: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct RdpPixelState {
    depth: RdpDepthResult,
    memory_coverage: u8,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct RdpPixelInputs {
    texels: [[u8; 4]; 2],
    shade: [u8; 4],
    legacy: [u8; 4],
    lod_frac: i16,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct RdpDitherSignals {
    rgb: [i32; 3],
    alpha: i32,
    noise: i32,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct RdpPreparedPixel {
    color: [u8; 4],
    shade: [u8; 4],
    rgb_dither: [i32; 3],
    coverage_count: i32,
}

#[derive(Clone, Copy, Debug)]
struct RdpTriangleSpan {
    y: usize,
    x0: usize,
    x1: usize,
    major_x: i64,
    coverage_left: [i32; 4],
    coverage_right: [i32; 4],
    subpixel: bool,
}

#[derive(Clone)]
struct ViState {
    regs: [u32; 14],
    current: u32,
}
impl Default for ViState {
    fn default() -> Self {
        let mut regs = [0; 14];
        regs[6] = 0x20d;
        regs[7] = 0x0c15;
        Self { regs, current: 0 }
    }
}

#[derive(Clone)]
struct AiState {
    dram_addr: u32,
    len: u32,
    next_dram_addr: u32,
    next_len: u32,
    dma_count: u8,
    control: u32,
    status: u32,
    dac_rate: u32,
    bit_rate: u32,
    sample_phase: u64,
    output_phase: u64,
    sample_left: i16,
    sample_right: i16,
}

impl Default for AiState {
    fn default() -> Self {
        Self {
            dram_addr: 0,
            len: 0,
            next_dram_addr: 0,
            next_len: 0,
            dma_count: 0,
            control: 0,
            status: (1 << 20) | (1 << 24),
            dac_rate: 0,
            bit_rate: 0,
            sample_phase: 0,
            output_phase: 0,
            sample_left: 0,
            sample_right: 0,
        }
    }
}

#[derive(Clone, Default)]
struct PiState {
    dram_addr: u32,
    cart_addr: u32,
    rd_len: u32,
    wr_len: u32,
    status: u32,
    timing: [u32; 8],
    dma_cycles: u32,
}

#[derive(Clone, Default)]
struct SiState {
    dram_addr: u32,
    status: u32,
    dma_cycles: u32,
    dma_direction: u8,
}
struct N64Board {
    rdram: Box<[u8]>,
    rdram_hidden: Box<[u8]>,
    sp_mem: Box<[u8]>,
    rom: Vec<u8>,
    save: Box<[u8]>,
    eeprom_size: usize,
    eeprom_busy_cycles: u32,
    rtc_present: bool,
    rtc_ram: [u8; RTC_SIZE],
    rtc_status: u8,
    rtc_write_lock: u8,
    rtc_cycle_phase: u64,
    flash_present: bool,
    flash_mode: u8,
    flash_status: u32,
    flash_erase_page: u16,
    flash_page_buffer: [u8; 128],
    controller_pak: Box<[u8]>,
    pif_ram: [u8; 64],
    sp: SpState,
    dp: DpState,
    vi: ViState,
    ai: AiState,
    pi: PiState,
    si: SiState,
    mi_mode: u32,
    mi_intr: u32,
    mi_mask: u32,
    controllers: [u64; 4],
    controller_axes: [[i16; 2]; 4],
    audio_samples: Vec<(f32, f32)>,
    fill_color: u32,
    color_image: u32,
    color_width: u32,
    color_size: u8,
    color_format: u8,
    scissor: [u16; 4],
    scissor_fixed: [u16; 4],
    scissor_field: bool,
    scissor_keep_odd: bool,
    other_modes: u64,
    primitive_color: u32,
    primitive_lod_frac: u8,
    primitive_min_level: u8,
    environment_color: u32,
    blend_color: u32,
    fog_color: u32,
    texture_image: u32,
    texture_width: u32,
    texture_size: u8,
    texture_format: u8,
    depth_image: u32,
    primitive_depth: u16,
    primitive_delta_z: u16,
    combine_mode: u64,
    convert_k: [i16; 6],
    key_center: [u8; 3],
    key_scale: [u8; 3],
    key_width: [u16; 3],
    tmem: Box<[u8; 4096]>,
    tlut: [u16; 256],
    tiles: [RdpTile; 8],
    rdp_noise_seed: u32,
    rdp_pipeline_crashed: bool,
    frame: u64,
}

impl N64Board {
    fn new(image: &[u8]) -> Result<Self, String> {
        let rom = normalize_rom(image)?;
        let eeprom_size = eeprom_size_for_rom(&rom);
        let rtc_present = rtc_present_for_rom(&rom);
        let flash_present = flash_present_for_rom(&rom);
        let mut sp_mem = vec![0; SP_MEM_SIZE].into_boxed_slice();
        sp_mem[..0x1000].copy_from_slice(&rom[..0x1000]);
        Ok(Self {
            rdram: vec![0; RDRAM_SIZE].into_boxed_slice(),
            rdram_hidden: vec![0; RDRAM_HIDDEN_SIZE].into_boxed_slice(),
            sp_mem,
            rom,
            save: vec![0xff; SAVE_SIZE].into_boxed_slice(),
            eeprom_size,
            eeprom_busy_cycles: 0,
            rtc_present,
            rtc_ram: default_rtc_ram(),
            rtc_status: 0,
            rtc_write_lock: 0,
            rtc_cycle_phase: 0,
            flash_present,
            flash_mode: FLASH_READ_ARRAY,
            flash_status: 0,
            flash_erase_page: 0,
            flash_page_buffer: [0xff; 128],
            controller_pak: vec![0; CONTROLLER_PAK_SIZE].into_boxed_slice(),
            pif_ram: [0; 64],
            sp: SpState::default(),
            dp: DpState::default(),
            vi: ViState::default(),
            ai: AiState::default(),
            pi: PiState::default(),
            si: SiState::default(),
            mi_mode: 0,
            mi_intr: 0,
            mi_mask: 0,
            controllers: [0; 4],
            controller_axes: [[0; 2]; 4],
            audio_samples: Vec::with_capacity(2048),
            fill_color: 0,
            color_image: 0,
            color_width: 320,
            color_size: 2,
            color_format: 0,
            scissor: [0, 0, 1024, 1024],
            scissor_fixed: [0, 0, 4096, 4096],
            scissor_field: false,
            scissor_keep_odd: false,
            other_modes: 0,
            primitive_color: 0,
            primitive_lod_frac: 0,
            primitive_min_level: 0,
            environment_color: 0,
            blend_color: 0,
            fog_color: 0,
            texture_image: 0,
            texture_width: 1,
            texture_size: 0,
            texture_format: 0,
            depth_image: 0,
            primitive_depth: 0,
            primitive_delta_z: 0,
            combine_mode: 0,
            convert_k: DEFAULT_CONVERT_K,
            key_center: [0; 3],
            key_scale: [0; 3],
            key_width: [0; 3],
            tmem: Box::new([0; 4096]),
            tlut: [0; 256],
            tiles: [RdpTile::default(); 8],
            rdp_noise_seed: 1,
            rdp_pipeline_crashed: false,
            frame: 0,
        })
    }

    fn reset(&mut self) {
        self.rdram.fill(0);
        self.rdram_hidden.fill(0);
        self.sp_mem.fill(0);
        self.sp_mem[..0x1000].copy_from_slice(&self.rom[..0x1000]);
        self.pif_ram.fill(0);
        self.eeprom_busy_cycles = 0;
        self.flash_mode = FLASH_READ_ARRAY;
        self.flash_status = 0;
        self.flash_erase_page = 0;
        self.flash_page_buffer.fill(0xff);
        self.sp = SpState::default();
        self.dp = DpState::default();
        self.vi = ViState::default();
        self.ai = AiState::default();
        self.pi = PiState::default();
        self.si = SiState::default();
        self.mi_mode = 0;
        self.mi_intr = 0;
        self.mi_mask = 0;
        self.controllers = [0; 4];
        self.controller_axes = [[0; 2]; 4];
        self.audio_samples.clear();
        self.fill_color = 0;
        self.color_image = 0;
        self.color_width = 320;
        self.color_size = 2;
        self.color_format = 0;
        self.scissor = [0, 0, 1024, 1024];
        self.scissor_fixed = [0, 0, 4096, 4096];
        self.scissor_field = false;
        self.scissor_keep_odd = false;
        self.other_modes = 0;
        self.primitive_color = 0;
        self.primitive_lod_frac = 0;
        self.primitive_min_level = 0;
        self.environment_color = 0;
        self.blend_color = 0;
        self.fog_color = 0;
        self.texture_image = 0;
        self.texture_width = 1;
        self.texture_size = 0;
        self.texture_format = 0;
        self.depth_image = 0;
        self.primitive_depth = 0;
        self.primitive_delta_z = 0;
        self.combine_mode = 0;
        self.convert_k = DEFAULT_CONVERT_K;
        self.key_center = [0; 3];
        self.key_scale = [0; 3];
        self.key_width = [0; 3];
        self.tmem.fill(0);
        self.tlut.fill(0);
        self.tiles = [RdpTile::default(); 8];
        self.rdp_noise_seed = 1;
        self.rdp_pipeline_crashed = false;
        self.frame = 0;
    }

    fn set_input(&mut self, input: &InputState) {
        self.controllers.copy_from_slice(&input.buttons);
        for port in 0..4 {
            self.controller_axes[port] =
                [input.axes[port][AXIS_LEFT_X], input.axes[port][AXIS_LEFT_Y]];
        }
    }
    fn cpu_irq(&self) -> bool {
        self.mi_intr & self.mi_mask != 0
    }
    fn raise(&mut self, source: u32) {
        self.mi_intr |= source;
    }
    fn clear(&mut self, source: u32) {
        self.mi_intr &= !source;
    }
    fn read_bytes(&self, address: u32, width: usize) -> u32 {
        let mut value = 0u32;
        for offset in 0..width {
            value = (value << 8) | u32::from(self.read8(address.wrapping_add(offset as u32)));
        }
        value
    }

    fn write_bytes(&mut self, address: u32, value: u32, width: usize) {
        for offset in 0..width {
            let shift = (width - 1 - offset) * 8;
            self.write8(address.wrapping_add(offset as u32), (value >> shift) as u8);
        }
    }

    fn read8(&self, address: u32) -> u8 {
        match address {
            0x0000_0000..=0x007f_ffff => self.rdram[address as usize & (RDRAM_SIZE - 1)],
            0x0400_0000..=0x0403_ffff => {
                self.sp_mem[(address - 0x0400_0000) as usize & (SP_MEM_SIZE - 1)]
            }
            0x0800_0000..=0x0801_ffff => self.save[(address - 0x0800_0000) as usize],
            0x1000_0000..=0x1fbf_ffff => self
                .rom
                .get((address - 0x1000_0000) as usize)
                .copied()
                .unwrap_or(0xff),
            0x1fc0_07c0..=0x1fc0_07ff => self.pif_ram[(address - 0x1fc0_07c0) as usize],
            _ => 0,
        }
    }
    fn write8(&mut self, address: u32, value: u8) {
        match address {
            0x0000_0000..=0x007f_ffff => self.rdram[address as usize & (RDRAM_SIZE - 1)] = value,
            0x0400_0000..=0x0403_ffff => {
                self.sp_mem[(address - 0x0400_0000) as usize & (SP_MEM_SIZE - 1)] = value
            }
            0x0800_0000..=0x0801_ffff if !self.flash_present => {
                self.save[(address - 0x0800_0000) as usize] = value
            }
            0x1fc0_07c0..=0x1fc0_07ff => self.pif_ram[(address - 0x1fc0_07c0) as usize] = value,
            _ => {}
        }
    }

    fn read32(&mut self, address: u32) -> u32 {
        match address & !3 {
            0x0404_0000 => self.sp_mem_addr_read(),
            0x0404_0004 => self.sp_dram_addr_read(),
            0x0404_0008 | 0x0404_000c => self.sp_dma_len_read(),
            0x0404_0010 => self.sp_status(),
            0x0404_0014 => u32::from(self.sp.dma_full),
            0x0404_0018 => u32::from(self.sp.dma_busy),
            0x0404_001c => {
                let value = u32::from(self.sp.semaphore);
                self.sp.semaphore = true;
                value
            }
            0x0408_0000 => self.sp.pc,
            0x0410_0000 => self.dp.start,
            0x0410_0004 => self.dp.end,
            0x0410_0008 => self.dp.current,
            0x0410_000c => self.dp.status,
            0x0410_0010 => self.dp.clock,
            0x0430_0000 => self.mi_mode,
            0x0430_0004 => 0x0202_0102,
            0x0430_0008 => self.mi_intr,
            0x0430_000c => self.mi_mask,
            0x0440_0010 => self.vi.current,
            0x0440_0000..=0x0440_0034 => self.vi.regs[((address - 0x0440_0000) >> 2) as usize],
            0x0450_0000 => self.ai.dram_addr,
            0x0450_0004 => self.ai.len,
            0x0450_0008 => self.ai.control,
            0x0450_000c => self.ai.status,
            0x0450_0010 => self.ai.dac_rate,
            0x0450_0014 => self.ai.bit_rate,
            0x0460_0000 => self.pi.dram_addr,
            0x0460_0004 => self.pi.cart_addr,
            0x0460_0008 => self.pi.rd_len,
            0x0460_000c => self.pi.wr_len,
            0x0460_0010 => self.pi.status,
            0x0460_0014..=0x0460_0030 => self.pi.timing[((address - 0x0460_0014) >> 2) as usize],
            0x0480_0000 => self.si.dram_addr,
            0x0480_0018 => self.si.status,
            0x0800_0000 if self.flash_present => {
                if self.flash_mode == FLASH_STATUS {
                    self.flash_status
                } else {
                    0
                }
            }
            _ => self.read_bytes(address, 4),
        }
    }
    fn write32(&mut self, address: u32, value: u32) {
        match address & !3 {
            0x0404_0000 => self.sp.mem_addr = value & 0x1ff8,
            0x0404_0004 => self.sp.dram_addr = value & 0x00ff_fff8,
            0x0404_0008 => {
                self.sp.rd_len = value;
                self.sp_dma(value, true);
            }
            0x0404_000c => {
                self.sp.wr_len = value;
                self.sp_dma(value, false);
            }
            0x0404_0010 => self.write_sp_status(value),
            0x0404_001c => self.sp.semaphore = false,
            0x0408_0000 => self.sp.pc = value & 0x0ffc,
            0x0410_0000 => self.write_dp_start(value),
            0x0410_0004 => self.write_dp_end(value),
            0x0410_000c => self.write_dp_status(value),
            0x0430_0000 => self.write_mi_mode(value),
            0x0430_000c => self.write_mi_mask(value),
            0x0440_0010 => self.clear(MI_VI),
            0x0440_0000..=0x0440_0034 => {
                self.vi.regs[((address - 0x0440_0000) >> 2) as usize] = value
            }
            0x0450_0000 => self.write_ai_dram_addr(value),
            0x0450_0004 => self.queue_ai_dma(value),
            0x0450_0008 => {
                self.ai.control = value & 1;
                self.update_ai_status();
            }
            0x0450_000c => self.clear(MI_AI),
            0x0450_0010 => self.ai.dac_rate = value & 0x3fff,
            0x0450_0014 => self.ai.bit_rate = value & 0xf,
            0x0460_0000 => {
                if self.pi_write_allowed() {
                    self.pi.dram_addr = value & 0x00ff_fffe;
                }
            }
            0x0460_0004 => {
                if self.pi_write_allowed() {
                    self.pi.cart_addr = value & !1;
                }
            }
            0x0460_0008 => {
                if self.pi_write_allowed() {
                    self.pi.rd_len = value & 0x00ff_ffff;
                    self.pi_dma(value, false);
                }
            }
            0x0460_000c => {
                if self.pi_write_allowed() {
                    self.pi.wr_len = value & 0x00ff_ffff;
                    self.pi_dma(value, true);
                }
            }
            0x0460_0010 => self.write_pi_status(value),
            0x0460_0014..=0x0460_0030 => {
                if self.pi_write_allowed() {
                    let index = ((address - 0x0460_0014) >> 2) as usize;
                    let mask = match index & 3 {
                        0 | 1 => 0xff,
                        2 => 0x0f,
                        _ => 0x03,
                    };
                    self.pi.timing[index] = value & mask;
                }
            }
            0x0480_0000 => self.si.dram_addr = value & 0x00ff_fff8,
            0x0480_0004 => self.si_dma(false),
            0x0480_0010 => self.si_dma(true),
            0x0480_0018 => {
                self.si.status &= !0x1000;
                self.clear(MI_SI);
            }
            0x0800_0000 if self.flash_present && self.flash_mode == FLASH_STATUS => {
                self.flash_status = value & 0xff;
            }
            0x0801_0000 if self.flash_present => self.flash_command(value),
            _ => self.write_bytes(address, value, 4),
        }
    }

    fn sp_status(&self) -> u32 {
        (self.sp.status & !0x0c)
            | (u32::from(self.sp.dma_busy) << 2)
            | (u32::from(self.sp.dma_full) << 3)
    }

    fn write_sp_status(&mut self, value: u32) {
        let apply_pair = |status: &mut u32, clear_bit: u32, set_bit: u32, status_bit: u32| {
            let clear = value & (1 << clear_bit) != 0;
            let set = value & (1 << set_bit) != 0;
            if clear == set {
                return;
            }
            if clear {
                *status &= !(1 << status_bit);
            } else {
                *status |= 1 << status_bit;
            }
        };

        apply_pair(&mut self.sp.status, 0, 1, 0);
        if value & (1 << 2) != 0 {
            self.sp.status &= !2;
        }

        let clear_interrupt = value & (1 << 3) != 0;
        let set_interrupt = value & (1 << 4) != 0;
        if clear_interrupt != set_interrupt {
            if clear_interrupt {
                self.clear(MI_SP);
            } else {
                self.raise(MI_SP);
            }
        }

        apply_pair(&mut self.sp.status, 5, 6, 5);
        apply_pair(&mut self.sp.status, 7, 8, 6);
        for signal in 0..8 {
            let clear_bit = 9 + signal * 2;
            apply_pair(&mut self.sp.status, clear_bit, clear_bit + 1, 7 + signal);
        }
    }

    fn sp_dma_job_value(job: SpDmaJob) -> u32 {
        (job.length & 0x0ff8) | ((job.count & 0xff) << 12) | ((job.skip & 0x0ff8) << 20)
    }

    fn sp_mem_addr_read(&self) -> u32 {
        if self.sp.dma_busy {
            self.sp.dma_current.mem_addr
        } else {
            self.sp.mem_addr
        }
    }

    fn sp_dram_addr_read(&self) -> u32 {
        if self.sp.dma_busy {
            self.sp.dma_current.dram_addr
        } else {
            self.sp.dram_addr
        }
    }

    fn sp_dma_len_read(&self) -> u32 {
        if self.sp.dma_busy {
            Self::sp_dma_job_value(self.sp.dma_current)
        } else {
            0x0ff8
        }
    }

    fn sp_start_dma_job(&mut self, job: SpDmaJob) {
        self.sp.dma_current = job;
        self.sp.dma_busy = true;
        self.sp.dma_rsp_cycles = ((job.length + 8) / 8).saturating_mul(3);
    }

    fn sp_dma(&mut self, descriptor: u32, rdram_to_sp: bool) {
        let job = SpDmaJob {
            mem_addr: (self.sp.mem_addr & 0x1000) | (self.sp.mem_addr & 0x0ff8),
            dram_addr: self.sp.dram_addr & 0x00ff_fff8,
            length: descriptor & 0x0ff8,
            count: (descriptor >> 12) & 0xff,
            skip: (descriptor >> 20) & 0x0ff8,
            direction: if rdram_to_sp { 1 } else { 2 },
        };

        if self.sp.dma_busy {
            self.sp.dma_pending = job;
            self.sp.dma_full = true;
            return;
        }
        self.sp_start_dma_job(job);
    }

    fn sp_finish_dma_block(&mut self) {
        if !self.sp.dma_busy {
            return;
        }
        let mut job = self.sp.dma_current;
        let bytes = (job.length + 8) as usize;
        let mem_bank = job.mem_addr as usize & 0x1000;
        let mem = job.mem_addr as usize & 0x0ff8;
        let dram = job.dram_addr as usize & 0x00ff_fff8;

        for offset in 0..bytes {
            let sp_index = mem_bank | (mem.wrapping_add(offset) & 0x0fff);
            let ram_index = (dram + offset) & (RDRAM_SIZE - 1);
            if job.direction == 1 {
                self.sp_mem[sp_index] = self.rdram[ram_index];
            } else {
                self.rdram[ram_index] = self.sp_mem[sp_index];
            }
        }

        job.mem_addr = (mem_bank | (mem.wrapping_add(bytes) & 0x0fff)) as u32;
        job.dram_addr = job.dram_addr.wrapping_add(bytes as u32) & 0x00ff_ffff;
        if job.count != 0 {
            job.count -= 1;
            job.dram_addr = job.dram_addr.wrapping_add(job.skip) & 0x00ff_ffff;
            self.sp.dma_current = job;
            self.sp.dma_rsp_cycles = ((job.length + 8) / 8).saturating_mul(3);
            return;
        }

        self.sp.mem_addr = job.mem_addr;
        self.sp.dram_addr = job.dram_addr;
        self.sp.rd_len = 0x0ff8;
        self.sp.wr_len = 0x0ff8;
        self.sp.dma_current = SpDmaJob {
            mem_addr: job.mem_addr,
            dram_addr: job.dram_addr,
            length: 0x0ff8,
            ..SpDmaJob::default()
        };
        self.sp.dma_busy = false;
        self.sp.dma_rsp_cycles = 0;

        if self.sp.dma_full {
            let pending = self.sp.dma_pending;
            self.sp.dma_pending = SpDmaJob::default();
            self.sp.dma_full = false;
            self.sp_start_dma_job(pending);
        }
    }

    fn tick_sp_dma(&mut self, cpu_cycles: u32) {
        self.sp.dma_phase = self
            .sp
            .dma_phase
            .wrapping_add(u64::from(cpu_cycles).saturating_mul(RSP_HZ));
        let mut rsp_cycles = self.sp.dma_phase / CPU_HZ;
        self.sp.dma_phase %= CPU_HZ;

        while self.sp.dma_busy && rsp_cycles != 0 {
            let remaining = u64::from(self.sp.dma_rsp_cycles);
            if rsp_cycles < remaining {
                self.sp.dma_rsp_cycles -= rsp_cycles as u32;
                break;
            }
            rsp_cycles -= remaining;
            self.sp.dma_rsp_cycles = 0;
            self.sp_finish_dma_block();
        }
    }

    fn write_mi_mode(&mut self, value: u32) {
        self.mi_mode = (self.mi_mode & !0x7f) | (value & 0x7f);
        if value & (1 << 11) != 0 {
            self.mi_mode &= !(1 << 7);
        }
        if value & (1 << 12) != 0 {
            self.mi_mode |= 1 << 7;
        }
        if value & (1 << 13) != 0 {
            self.mi_mode &= !(1 << 8);
        }
        if value & (1 << 14) != 0 {
            self.mi_mode |= 1 << 8;
        }
        if value & (1 << 15) != 0 {
            self.clear(MI_DP);
        }
    }

    fn write_mi_mask(&mut self, value: u32) {
        let sources = [MI_SP, MI_SI, MI_AI, MI_VI, MI_PI, MI_DP];
        for (index, source) in sources.into_iter().enumerate() {
            if value & (1 << (index * 2)) != 0 {
                self.mi_mask &= !source;
            }
            if value & (1 << (index * 2 + 1)) != 0 {
                self.mi_mask |= source;
            }
        }
    }

    fn write_dp_start(&mut self, value: u32) {
        if self.dp.status & DP_STATUS_START_VALID != 0 {
            return;
        }
        self.dp.start = value & 0x00ff_fff8;
        self.dp.status |= DP_STATUS_START_VALID;
    }

    fn write_dp_end(&mut self, value: u32) {
        self.dp.end = value & 0x00ff_fff8;
        if self.dp.status & DP_STATUS_START_VALID != 0 {
            self.dp.current = self.dp.start;
            self.dp.status &= !DP_STATUS_START_VALID;
        }
        if self.dp.status & DP_STATUS_FREEZE == 0 {
            self.run_rdp();
        }
    }

    fn write_dp_status(&mut self, value: u32) {
        let previous_freeze = self.dp.status & DP_STATUS_FREEZE != 0;
        let apply_pair = |status: &mut u32, clear_bit: u32, set_bit: u32, status_mask: u32| {
            let clear = value & (1 << clear_bit) != 0;
            let set = value & (1 << set_bit) != 0;
            if clear == set {
                return;
            }
            if clear {
                *status &= !status_mask;
            } else {
                *status |= status_mask;
            }
        };

        apply_pair(&mut self.dp.status, 0, 1, DP_STATUS_XBUS);
        apply_pair(&mut self.dp.status, 2, 3, DP_STATUS_FREEZE);
        apply_pair(&mut self.dp.status, 4, 5, DP_STATUS_FLUSH);
        if value & (1 << 9) != 0 {
            self.dp.clock = 0;
        }

        let resumed = previous_freeze && self.dp.status & DP_STATUS_FREEZE == 0;
        if resumed && self.dp.current < self.dp.end {
            self.run_rdp();
        }
    }
    fn update_ai_status(&mut self) {
        let mut status = (1 << 20) | (1 << 24);
        if self.ai.control & 1 != 0 {
            status |= 1 << 25;
        }
        if self.ai.dma_count != 0 {
            status |= 1 << 30;
        }
        if self.ai.dma_count > 1 {
            status |= 1 | (1 << 31);
        }
        self.ai.status = status;
    }

    fn write_ai_dram_addr(&mut self, value: u32) {
        if self.ai.dma_count >= 2 {
            return;
        }
        let address = value & 0x00ff_fff8;
        if self.ai.dma_count == 0 {
            self.ai.dram_addr = address;
        } else {
            self.ai.next_dram_addr = address;
        }
    }

    fn queue_ai_dma(&mut self, value: u32) {
        if self.ai.dma_count >= 2 {
            self.update_ai_status();
            return;
        }
        let length = value & 0x0003_fff8;
        if self.ai.dma_count == 0 {
            self.ai.len = length;
            self.ai.dma_count = 1;
            self.raise(MI_AI);
        } else {
            self.ai.next_len = length;
            self.ai.dma_count = 2;
        }
        self.update_ai_status();
    }

    fn ai_dac_sample(&mut self) {
        if self.ai.control & 1 != 0 && self.ai.dma_count != 0 && self.ai.len != 0 {
            let base = self.ai.dram_addr as usize & (RDRAM_SIZE - 1);
            self.ai.sample_left =
                i16::from_be_bytes([self.rdram[base], self.rdram[(base + 1) & (RDRAM_SIZE - 1)]]);
            self.ai.sample_right = i16::from_be_bytes([
                self.rdram[(base + 2) & (RDRAM_SIZE - 1)],
                self.rdram[(base + 3) & (RDRAM_SIZE - 1)],
            ]);
            self.ai.dram_addr = self.ai.dram_addr.wrapping_add(4) & 0x00ff_ffff;
            self.ai.len = self.ai.len.saturating_sub(4);
        } else {
            self.ai.sample_left = 0;
            self.ai.sample_right = 0;
        }

        if self.ai.dma_count != 0 && self.ai.len == 0 {
            self.ai.dma_count -= 1;
            if self.ai.dma_count != 0 {
                self.ai.dram_addr = self.ai.next_dram_addr;
                self.ai.len = self.ai.next_len;
                self.ai.next_dram_addr = 0;
                self.ai.next_len = 0;
                self.raise(MI_AI);
            }
        }
        self.update_ai_status();
    }

    fn tick_ai(&mut self, cycles: u32) {
        let dac_period = CPU_HZ.saturating_mul(u64::from(self.ai.dac_rate) + 1);
        self.ai.sample_phase = self
            .ai
            .sample_phase
            .wrapping_add(u64::from(cycles).saturating_mul(NTSC_VIDEO_CLOCK_HZ));

        if self.ai.dma_count != 0 {
            while self.ai.sample_phase >= dac_period {
                self.ai.sample_phase -= dac_period;
                self.ai_dac_sample();
            }
        } else if dac_period != 0 {
            self.ai.sample_phase %= dac_period;
            self.ai.sample_left = 0;
            self.ai.sample_right = 0;
        }

        self.ai.output_phase = self
            .ai
            .output_phase
            .wrapping_add(u64::from(cycles).saturating_mul(u64::from(AUDIO_RATE)));
        while self.ai.output_phase >= CPU_HZ {
            self.ai.output_phase -= CPU_HZ;
            self.audio_samples.push((
                f32::from(self.ai.sample_left) / 32768.0,
                f32::from(self.ai.sample_right) / 32768.0,
            ));
        }
    }

    fn flash_command(&mut self, command: u32) {
        if !self.flash_present {
            return;
        }
        match command & 0xff00_0000 {
            0x3c00_0000 => {
                self.flash_mode = FLASH_CHIP_ERASE;
            }
            0x4b00_0000 => {
                self.flash_mode = FLASH_SECTOR_ERASE;
                self.flash_erase_page = (command & 0xffff) as u16;
            }
            0x7800_0000 => {
                self.flash_status |= 0x02;
                match self.flash_mode {
                    FLASH_SECTOR_ERASE => {
                        let start = (usize::from(self.flash_erase_page & 0xff80) * 128)
                            .min(self.save.len());
                        let end = start.saturating_add(16 * 1024).min(self.save.len());
                        self.save[start..end].fill(0xff);
                    }
                    FLASH_CHIP_ERASE => self.save.fill(0xff),
                    _ => {}
                }
                self.flash_status = (self.flash_status & !0x02) | 0x08;
                self.flash_mode = FLASH_STATUS;
            }
            0xa500_0000 => {
                self.flash_status |= 0x01;
                let start = usize::try_from(command & 0xffff)
                    .unwrap_or_default()
                    .saturating_mul(128);
                if start < self.save.len() {
                    let count = 128.min(self.save.len() - start);
                    for index in 0..count {
                        self.save[start + index] &= self.flash_page_buffer[index];
                    }
                }
                self.flash_status = (self.flash_status & !0x01) | 0x04;
                self.flash_mode = FLASH_STATUS;
            }
            0xb400_0000 => {
                self.flash_mode = FLASH_PAGE_PROGRAM;
                self.flash_page_buffer.fill(0xff);
            }
            0xd200_0000 => self.flash_mode = FLASH_STATUS,
            0xe100_0000 => {
                self.flash_mode = FLASH_SILICON_ID;
                self.flash_status |= 0x01;
            }
            0xf000_0000 => self.flash_mode = FLASH_READ_ARRAY,
            _ => {}
        }
    }

    fn flash_dma_to_rdram(&mut self, length: usize) -> bool {
        if !self.flash_present {
            return false;
        }
        let cart_offset = self.pi.cart_addr & 0x1ffff;
        let dram_start = self.pi.dram_addr as usize & (RDRAM_SIZE - 1);
        match self.flash_mode {
            FLASH_SILICON_ID if cart_offset == 0 && length == 8 => {
                let mut id = [0u8; 8];
                id[..4].copy_from_slice(&FLASH_TYPE_ID.to_be_bytes());
                id[4..].copy_from_slice(&FLASH_DEVICE_ID.to_be_bytes());
                for (offset, value) in id.into_iter().enumerate() {
                    self.rdram[(dram_start + offset) & (RDRAM_SIZE - 1)] = value;
                }
                true
            }
            FLASH_READ_ARRAY if cart_offset < 0x1_0000 => {
                let source = usize::try_from(cart_offset)
                    .unwrap_or_default()
                    .saturating_mul(2);
                for offset in 0..length {
                    self.rdram[(dram_start + offset) & (RDRAM_SIZE - 1)] =
                        self.save[(source + offset) & (SAVE_SIZE - 1)];
                }
                true
            }
            _ => false,
        }
    }

    fn flash_dma_from_rdram(&mut self, length: usize) -> bool {
        if !self.flash_present
            || self.flash_mode != FLASH_PAGE_PROGRAM
            || self.pi.cart_addr & 0x1ffff != 0
            || length != 128
        {
            return false;
        }
        let dram_start = self.pi.dram_addr as usize & (RDRAM_SIZE - 1);
        for offset in 0..128 {
            self.flash_page_buffer[offset] = self.rdram[(dram_start + offset) & (RDRAM_SIZE - 1)];
        }
        true
    }

    fn cart_read8(&self, address: u32) -> u8 {
        match address {
            0x0800_0000..=0x0801_ffff if !self.flash_present => {
                self.save[(address - 0x0800_0000) as usize]
            }
            0x0800_0000..=0x0801_ffff => 0xff,
            0x1000_0000..=0x1fbf_ffff => self
                .rom
                .get((address - 0x1000_0000) as usize)
                .copied()
                .unwrap_or(0xff),
            _ => 0xff,
        }
    }
    fn cart_write8(&mut self, address: u32, value: u8) {
        if !self.flash_present && (0x0800_0000..=0x0801_ffff).contains(&address) {
            self.save[(address - 0x0800_0000) as usize] = value;
        }
    }

    fn pi_write_allowed(&mut self) -> bool {
        if self.pi.status & 3 == 0 {
            true
        } else {
            self.pi.status |= 1 << 2;
            false
        }
    }

    fn pi_bus_timing(&self, address: u32) -> [u32; 4] {
        let domain_two = matches!(address >> 24, 0x05 | 0x08..=0x0f);
        let base = if domain_two { 4 } else { 0 };
        [
            self.pi.timing[base] & 0xff,
            self.pi.timing[base + 1] & 0xff,
            self.pi.timing[base + 2] & 0x0f,
            self.pi.timing[base + 3] & 0x03,
        ]
    }

    fn pi_dma_duration(&self, descriptor: u32) -> u32 {
        let length = u64::from(((descriptor & 0x00ff_ffff) | 1).wrapping_add(1));
        let [latency, pulse_width, page_size_field, release_duration] =
            self.pi_bus_timing(self.pi.cart_addr);
        let page_shift = page_size_field + 2;
        let page_size = 1u64 << page_shift;
        let page_mask = page_size - 1;
        let first = u64::from(self.pi.cart_addr);
        let last = first.saturating_add(length.saturating_sub(2));
        let first_page = first >> page_shift;
        let last_page = last >> page_shift;
        let pages = last_page.saturating_sub(first_page).saturating_add(1);

        let mut buffers = 0u64;
        let mut partial_bytes = 0u64;
        if first_page == last_page {
            if length == 128 {
                buffers = 1;
            } else {
                partial_bytes = length;
            }
        } else {
            let full_first = first & page_mask == 0;
            let full_last = (last + 2) & page_mask == 0;
            if full_first {
                buffers += 1;
            } else {
                partial_bytes += page_size - (first & page_mask);
            }
            if full_last {
                buffers += 1;
            } else {
                partial_bytes += (last & page_mask) + 2;
            }
            if first_page + 1 < last_page {
                buffers += (pages - 2) * page_size / 128;
            }
        }

        let mut cycles = (u64::from(latency) + 15) * pages;
        cycles += (u64::from(pulse_width) + u64::from(release_duration) + 2) * length / 2;
        cycles += buffers * 28;
        cycles += partial_bytes;
        cycles.saturating_mul(3).clamp(1, u64::from(u32::MAX)) as u32
    }

    fn pi_dma(&mut self, descriptor: u32, cart_to_rdram: bool) {
        if !self.pi_write_allowed() {
            return;
        }
        let dma_cycles = self.pi_dma_duration(descriptor);
        let raw_length = (descriptor & 0x00ff_ffff).wrapping_add(1);
        let length = if cart_to_rdram {
            raw_length
        } else {
            ((descriptor & 0x00ff_ffff) | 1).wrapping_add(1)
        } as usize;

        self.pi.status |= 1;
        self.pi.dma_cycles = dma_cycles;
        let flash_window =
            self.flash_present && (0x0800_0000..=0x0801_ffff).contains(&self.pi.cart_addr);
        if flash_window {
            if cart_to_rdram {
                let _ = self.flash_dma_to_rdram(length);
            } else {
                let _ = self.flash_dma_from_rdram(length);
            }
        } else {
            for offset in 0..length {
                let dram = (self.pi.dram_addr as usize + offset) & (RDRAM_SIZE - 1);
                let cart = self.pi.cart_addr.wrapping_add(offset as u32);
                if cart_to_rdram {
                    self.rdram[dram] = self.cart_read8(cart);
                } else {
                    self.cart_write8(cart, self.rdram[dram]);
                }
            }
        }
        self.pi.dram_addr = self.pi.dram_addr.wrapping_add(length as u32) & 0x00ff_ffff;
        self.pi.cart_addr = self.pi.cart_addr.wrapping_add(length as u32);
    }

    fn tick_pi(&mut self, cycles: u32) {
        if self.pi.dma_cycles == 0 {
            return;
        }
        self.pi.dma_cycles = self.pi.dma_cycles.saturating_sub(cycles);
        if self.pi.dma_cycles == 0 {
            self.pi.status &= !1;
            self.pi.status |= 1 << 3;
            self.raise(MI_PI);
        }
    }

    fn write_pi_status(&mut self, value: u32) {
        if value & 1 != 0 {
            self.pi.dma_cycles = 0;
            self.pi.status &= !((1 << 0) | (1 << 2));
        }
        if value & 2 != 0 {
            self.pi.status &= !(1 << 3);
            self.clear(MI_PI);
        }
    }
    fn estimate_si_read_cycles(&self) -> u32 {
        let mut cycles = 13_600u32;
        let mut short_commands = 0u32;
        let mut offset = 0usize;
        let mut channel = 0usize;

        while offset < self.pif_ram.len() && channel < 5 {
            let send = self.pif_ram[offset];
            offset += 1;
            match send {
                0xfe => {
                    short_commands += 1;
                    break;
                }
                0x00 | 0xfd => {
                    short_commands += 1;
                    channel += 1;
                    continue;
                }
                0xff => {
                    short_commands += 1;
                    continue;
                }
                _ => {}
            }
            if offset >= self.pif_ram.len() {
                break;
            }
            let recv = self.pif_ram[offset];
            offset += 1;
            let send = usize::from(send & 0x3f);
            let recv = usize::from(recv & 0x3f);
            offset = offset.saturating_add(send).saturating_add(recv);
            cycles = cycles.saturating_add(if channel < 4 { 22_000 } else { 20_000 });
            channel += 1;
        }

        cycles
            .saturating_add(short_commands.saturating_mul(1_420))
            .saturating_mul(3)
    }

    fn perform_si_dma(&mut self, rdram_to_pif: bool) {
        let base = self.si.dram_addr as usize & (RDRAM_SIZE - 1);
        if rdram_to_pif {
            for index in 0..64 {
                self.pif_ram[index] = self.rdram[(base + index) & (RDRAM_SIZE - 1)];
            }
            self.process_pif();
        } else {
            for index in 0..64 {
                self.rdram[(base + index) & (RDRAM_SIZE - 1)] = self.pif_ram[index];
            }
        }
    }

    fn si_dma(&mut self, rdram_to_pif: bool) {
        if self.si.status & 1 != 0 {
            return;
        }

        self.si.dma_direction = if rdram_to_pif { 2 } else { 1 };
        self.si.dma_cycles = if rdram_to_pif {
            4_065 * 3
        } else {
            self.estimate_si_read_cycles()
        };
        let pch_state = if rdram_to_pif { 1 } else { 4 };
        let dma_state = if rdram_to_pif { 4 } else { 1 };
        self.si.status &= 1 << 12;
        self.si.status |= 1 | (pch_state << 4) | (dma_state << 8);
    }

    fn tick_si(&mut self, cycles: u32) {
        if self.si.dma_cycles == 0 {
            return;
        }
        self.si.dma_cycles = self.si.dma_cycles.saturating_sub(cycles);
        if self.si.dma_cycles != 0 {
            return;
        }

        let rdram_to_pif = self.si.dma_direction == 2;
        self.perform_si_dma(rdram_to_pif);
        self.si.dma_direction = 0;
        self.si.status &= !0x0fff;
        self.si.status |= 1 << 12;
        self.raise(MI_SI);
    }

    fn controller_word(&self, port: usize) -> u16 {
        let state = self.controllers[port];
        let mut value = 0u16;
        if state & FACE_SOUTH != 0 {
            value |= 0x8000;
        }
        if state & FACE_EAST != 0 {
            value |= 0x4000;
        }
        if state & FACE_NORTH != 0 {
            value |= 0x2000;
        }
        if state & START != 0 {
            value |= 0x1000;
        }
        if state & UP != 0 {
            value |= 0x0800;
        }
        if state & DOWN != 0 {
            value |= 0x0400;
        }
        if state & LEFT != 0 {
            value |= 0x0200;
        }
        if state & RIGHT != 0 {
            value |= 0x0100;
        }
        if state & L1 != 0 {
            value |= 0x0020;
        }
        if state & R1 != 0 {
            value |= 0x0010;
        }
        if state & FACE_WEST != 0 {
            value |= 0x0008;
        }
        value
    }
    fn pak_crc(data: &[u8]) -> u8 {
        let mut crc = 0u8;
        for bit in 0..=data.len() * 8 {
            let feedback = crc & 0x80 != 0;
            crc <<= 1;
            if bit < data.len() * 8 && data[bit >> 3] & (0x80 >> (bit & 7)) != 0 {
                crc |= 1;
            }
            if feedback {
                crc ^= 0x85;
            }
        }
        crc
    }

    fn stick_axis(value: i16) -> u8 {
        let scaled = (i32::from(value) * 80 / 32767).clamp(-80, 80) as i8;
        scaled as u8
    }

    fn bcd_decode(value: u8) -> u16 {
        u16::from(value >> 4) * 10 + u16::from(value & 0x0f)
    }

    fn bcd_encode(value: u16) -> u8 {
        (((value / 10) % 10) as u8) << 4 | (value % 10) as u8
    }

    fn rtc_days_in_month(month: u16, year: u16) -> u16 {
        match month {
            4 | 6 | 9 | 11 => 30,
            2 if year.is_multiple_of(400)
                || (year.is_multiple_of(4) && !year.is_multiple_of(100)) =>
            {
                29
            }
            2 => 28,
            _ => 31,
        }
    }

    fn sync_rtc_control(&mut self) {
        let control = u16::from_le_bytes([self.rtc_ram[0], self.rtc_ram[1]]);
        self.rtc_write_lock = (self.rtc_write_lock & 1) | ((((control >> 8) & 3) as u8) << 1);
        if control & 0x0004 != 0 {
            self.rtc_status |= 0x80;
        } else {
            self.rtc_status &= !0x80;
        }
    }

    fn advance_rtc_second(&mut self) {
        let mut second = Self::bcd_decode(self.rtc_ram[16]).min(59);
        let mut minute = Self::bcd_decode(self.rtc_ram[17]).min(59);
        let mut hour = Self::bcd_decode(self.rtc_ram[18] & 0x7f).min(23);
        let mut day = Self::bcd_decode(self.rtc_ram[19]).max(1);
        let mut weekday = Self::bcd_decode(self.rtc_ram[20]) % 7;
        let mut month = Self::bcd_decode(self.rtc_ram[21]).clamp(1, 12);
        let mut year =
            Self::bcd_decode(self.rtc_ram[22]) + 100 * Self::bcd_decode(self.rtc_ram[23]);
        day = day.min(Self::rtc_days_in_month(month, year));

        second += 1;
        if second == 60 {
            second = 0;
            minute += 1;
            if minute == 60 {
                minute = 0;
                hour += 1;
                if hour == 24 {
                    hour = 0;
                    weekday = (weekday + 1) % 7;
                    day += 1;
                    if day > Self::rtc_days_in_month(month, year) {
                        day = 1;
                        month += 1;
                        if month == 13 {
                            month = 1;
                            year = (year + 1) % 10_000;
                        }
                    }
                }
            }
        }

        self.rtc_ram[16] = Self::bcd_encode(second);
        self.rtc_ram[17] = Self::bcd_encode(minute);
        self.rtc_ram[18] = Self::bcd_encode(hour) | 0x80;
        self.rtc_ram[19] = Self::bcd_encode(day);
        self.rtc_ram[20] = Self::bcd_encode(weekday);
        self.rtc_ram[21] = Self::bcd_encode(month);
        self.rtc_ram[22] = Self::bcd_encode(year % 100);
        self.rtc_ram[23] = Self::bcd_encode(year / 100);
    }

    fn tick_cartridge(&mut self, cycles: u32) {
        self.eeprom_busy_cycles = self.eeprom_busy_cycles.saturating_sub(cycles);
        if !self.rtc_present || self.rtc_status & 0x80 != 0 {
            return;
        }
        self.rtc_cycle_phase = self.rtc_cycle_phase.wrapping_add(u64::from(cycles));
        while self.rtc_cycle_phase >= CPU_HZ {
            self.rtc_cycle_phase -= CPU_HZ;
            self.advance_rtc_second();
        }
    }

    fn process_eeprom_joybus(
        &mut self,
        cursor: usize,
        tx: usize,
        rx: usize,
        response: usize,
    ) -> bool {
        if self.eeprom_size == 0 {
            return false;
        }
        let command = self.pif_ram[cursor + 2];
        match command {
            0x00 | 0xff if tx >= 1 && rx >= 3 => {
                self.pif_ram[response] = 0x00;
                self.pif_ram[response + 1] = if self.eeprom_size > 512 { 0xc0 } else { 0x80 };
                self.pif_ram[response + 2] = if self.eeprom_busy_cycles != 0 {
                    0x80
                } else {
                    0x00
                };
                true
            }
            0x04 if tx >= 2 => {
                let base = usize::from(self.pif_ram[cursor + 3]).wrapping_mul(8) % self.eeprom_size;
                for index in 0..rx {
                    self.pif_ram[response + index] = if self.eeprom_busy_cycles != 0 {
                        0xff
                    } else {
                        self.save[(base + index) % self.eeprom_size]
                    };
                }
                true
            }
            0x05 if tx >= 10 && rx >= 1 => {
                let busy = self.eeprom_busy_cycles != 0;
                self.pif_ram[response] = if busy { 0x80 } else { 0x00 };
                if !busy {
                    let base =
                        usize::from(self.pif_ram[cursor + 3]).wrapping_mul(8) % self.eeprom_size;
                    for index in 0..8 {
                        self.save[(base + index) % self.eeprom_size] =
                            self.pif_ram[cursor + 4 + index];
                    }
                    self.eeprom_busy_cycles = EEPROM_WRITE_CYCLES;
                }
                true
            }
            _ => false,
        }
    }

    fn process_rtc_joybus(&mut self, cursor: usize, tx: usize, rx: usize, response: usize) -> bool {
        if !self.rtc_present {
            return false;
        }
        let command = self.pif_ram[cursor + 2];
        match command {
            0x06 if tx >= 1 && rx >= 3 => {
                self.pif_ram[response] = 0x00;
                self.pif_ram[response + 1] = 0x10;
                self.pif_ram[response + 2] = self.rtc_status;
                true
            }
            0x07 if tx >= 2 && rx >= 9 => {
                let block = usize::from(self.pif_ram[cursor + 3] & 3);
                let start = block * 8;
                self.pif_ram[response..response + 8]
                    .copy_from_slice(&self.rtc_ram[start..start + 8]);
                self.pif_ram[response + 8] = self.rtc_status;
                true
            }
            0x08 if tx >= 10 && rx >= 1 => {
                let block = usize::from(self.pif_ram[cursor + 3] & 3);
                if self.rtc_write_lock & (1 << block) == 0 {
                    let start = block * 8;
                    self.rtc_ram[start..start + 8]
                        .copy_from_slice(&self.pif_ram[cursor + 4..cursor + 12]);
                    if block == 0 {
                        self.sync_rtc_control();
                    }
                }
                self.pif_ram[response] = self.rtc_status;
                true
            }
            _ => false,
        }
    }

    fn process_pif(&mut self) {
        let mut cursor = 0usize;
        let mut channel = 0usize;
        while cursor < 63 && channel < 6 {
            let tx = self.pif_ram[cursor];
            if tx == 0xfe {
                break;
            }
            if tx == 0xff {
                cursor += 1;
                continue;
            }
            if tx == 0 {
                cursor += 1;
                channel += 1;
                continue;
            }
            if cursor + 2 >= 64 {
                break;
            }
            let rx = usize::from(self.pif_ram[cursor + 1] & 0x3f);
            let command = self.pif_ram[cursor + 2];
            let response = cursor + 2 + usize::from(tx & 0x3f);
            if response + rx > 64 {
                break;
            }
            if channel < 4 {
                match command {
                    0x00 | 0xff if rx >= 3 => {
                        self.pif_ram[response] = 0x05;
                        self.pif_ram[response + 1] = 0x00;
                        self.pif_ram[response + 2] = 0x01;
                    }
                    0x01 if rx >= 4 => {
                        let buttons = self.controller_word(channel).to_be_bytes();
                        self.pif_ram[response] = buttons[0];
                        self.pif_ram[response + 1] = buttons[1];
                        self.pif_ram[response + 2] =
                            Self::stick_axis(self.controller_axes[channel][0]);
                        self.pif_ram[response + 3] =
                            Self::stick_axis(-self.controller_axes[channel][1]);
                    }
                    0x02 if rx >= 33 && cursor + 4 < 64 => {
                        let address = ((usize::from(self.pif_ram[cursor + 3]) << 8)
                            | usize::from(self.pif_ram[cursor + 4]))
                            & 0x7fe0;
                        if address + 32 <= self.controller_pak.len() {
                            let block = &self.controller_pak[address..address + 32];
                            self.pif_ram[response..response + 32].copy_from_slice(block);
                            self.pif_ram[response + 32] = Self::pak_crc(block);
                        }
                    }
                    0x03 if rx >= 1 && cursor + 36 < 64 => {
                        let address = ((usize::from(self.pif_ram[cursor + 3]) << 8)
                            | usize::from(self.pif_ram[cursor + 4]))
                            & 0x7fe0;
                        if address + 32 <= self.controller_pak.len() {
                            let start = cursor + 5;
                            let mut block = [0u8; 32];
                            block.copy_from_slice(&self.pif_ram[start..start + 32]);
                            self.controller_pak[address..address + 32].copy_from_slice(&block);
                            self.pif_ram[response] = Self::pak_crc(&block);
                        }
                    }
                    _ => {}
                }
            } else if channel == 4 {
                let tx = usize::from(tx & 0x3f);
                let valid = self.process_eeprom_joybus(cursor, tx, rx, response)
                    || self.process_rtc_joybus(cursor, tx, rx, response);
                if valid {
                    self.pif_ram[cursor + 1] &= 0x3f;
                } else {
                    self.pif_ram[cursor + 1] |= 0x80;
                }
            }
            cursor = response + rx;
            channel += 1;
        }
        self.pif_ram[63] &= !1;
    }

    fn read_rdp_word(&self, address: u32) -> Option<(u32, u32)> {
        if self.dp.status & DP_STATUS_XBUS != 0 {
            let start = address as usize & 0x0fff;
            let mut bytes = [0u8; 8];
            for (offset, byte) in bytes.iter_mut().enumerate() {
                *byte = self.sp_mem[start.wrapping_add(offset) & 0x0fff];
            }
            return Some((
                u32::from_be_bytes(bytes[..4].try_into().unwrap()),
                u32::from_be_bytes(bytes[4..].try_into().unwrap()),
            ));
        }

        let start = address as usize;
        if start + 8 > self.rdram.len() {
            return None;
        }
        Some((
            u32::from_be_bytes(self.rdram[start..start + 4].try_into().unwrap()),
            u32::from_be_bytes(self.rdram[start + 4..start + 8].try_into().unwrap()),
        ))
    }
    fn rdp_command_length(opcode: u8) -> u32 {
        match opcode {
            0x08 => 32,
            0x09 => 48,
            0x0a | 0x0c => 96,
            0x0b | 0x0d => 112,
            0x0e => 160,
            0x0f => 176,
            0x24 | 0x25 => 16,
            _ => 8,
        }
    }

    fn rdp_cycle_type(&self) -> u8 {
        ((self.other_modes >> 52) & 3) as u8
    }

    fn set_rdp_color_image(&mut self, high: u32, low: u32) {
        self.color_format = ((high >> 21) & 7) as u8;
        self.color_size = ((high >> 19) & 3) as u8;
        self.color_width = (high & 0x03ff).wrapping_add(1);
        self.color_image = low & 0x00ff_ffff;
    }

    fn set_rdp_texture_image(&mut self, high: u32, low: u32) {
        self.texture_format = ((high >> 21) & 7) as u8;
        self.texture_size = ((high >> 19) & 3) as u8;
        self.texture_width = (high & 0x03ff).wrapping_add(1);
        self.texture_image = low & 0x00ff_ffff;
    }

    fn set_rdp_scissor(&mut self, high: u32, low: u32) {
        self.scissor_fixed = [
            ((high >> 12) & 0x0fff) as u16,
            (high & 0x0fff) as u16,
            ((low >> 12) & 0x0fff) as u16,
            (low & 0x0fff) as u16,
        ];
        self.scissor = self.scissor_fixed.map(|value| value >> 2);
        self.scissor_field = low & (1 << 25) != 0;
        self.scissor_keep_odd = low & (1 << 24) != 0;
    }

    fn rdp_scissor_accepts_line(&self, line: usize) -> bool {
        !self.scissor_field || ((line & 1) != 0) == self.scissor_keep_odd
    }

    fn set_rdp_key_gb(&mut self, high: u32, low: u32) {
        self.key_width[1] = ((high >> 12) & 0x0fff) as u16;
        self.key_width[2] = (high & 0x0fff) as u16;
        self.key_center[1] = (low >> 24) as u8;
        self.key_scale[1] = (low >> 16) as u8;
        self.key_center[2] = (low >> 8) as u8;
        self.key_scale[2] = low as u8;
    }

    fn set_rdp_key_r(&mut self, low: u32) {
        self.key_width[0] = ((low >> 16) & 0x0fff) as u16;
        self.key_center[0] = (low >> 8) as u8;
        self.key_scale[0] = low as u8;
    }

    fn set_rdp_convert(&mut self, high: u32, low: u32) {
        let raw = [
            (high >> 13) & 0x01ff,
            (high >> 4) & 0x01ff,
            ((high & 0x0f) << 5) | ((low >> 27) & 0x1f),
            (low >> 18) & 0x01ff,
            (low >> 9) & 0x01ff,
            low & 0x01ff,
        ];
        for (coefficient, value) in self.convert_k[..4].iter_mut().zip(raw.iter().copied()) {
            *coefficient = Self::sign_extend(value, 9) as i16;
        }
        self.convert_k[4] = raw[4] as i16;
        self.convert_k[5] = raw[5] as i16;
    }

    fn rdp_key_enabled(&self) -> bool {
        self.other_modes & (1u64 << 40) != 0
    }

    fn rdp_key_alpha(&self, color: [u8; 4]) -> u8 {
        let mut alpha = 255i32;
        for (component, &channel) in color.iter().take(3).enumerate() {
            let delta = i32::from(channel) - i32::from(self.key_center[component]);
            let product = delta.saturating_mul(i32::from(self.key_scale[component]));
            let key = (i32::from(self.key_width[component]) << 4)
                .saturating_sub(product.saturating_abs())
                .clamp(0, 255);
            alpha = alpha.min(key);
        }
        alpha as u8
    }

    fn set_rdp_tile(&mut self, high: u32, low: u32) {
        let index = ((low >> 24) & 7) as usize;
        self.tiles[index] = RdpTile {
            format: ((high >> 21) & 7) as u8,
            size: ((high >> 19) & 3) as u8,
            line: ((high >> 9) & 0x01ff) as u16,
            tmem: (high & 0x01ff) as u16,
            palette: ((low >> 20) & 0x0f) as u8,
            clamp_t: low & (1 << 19) != 0,
            mirror_t: low & (1 << 18) != 0,
            mask_t: ((low >> 14) & 0x0f) as u8,
            shift_t: ((low >> 10) & 0x0f) as u8,
            clamp_s: low & (1 << 9) != 0,
            mirror_s: low & (1 << 8) != 0,
            mask_s: ((low >> 4) & 0x0f) as u8,
            shift_s: (low & 0x0f) as u8,
            ..self.tiles[index]
        };
    }

    fn set_rdp_tile_size(&mut self, high: u32, low: u32) {
        let tile = &mut self.tiles[((low >> 24) & 7) as usize];
        tile.sl = ((high >> 12) & 0x0fff) as u16;
        tile.tl = (high & 0x0fff) as u16;
        tile.sh = ((low >> 12) & 0x0fff) as u16;
        tile.th = (low & 0x0fff) as u16;
    }

    fn texture_bits(size: u8) -> usize {
        match size & 3 {
            0 => 4,
            1 => 8,
            2 => 16,
            _ => 32,
        }
    }

    fn read_rdram_texture_raw(&self, texel: usize, size: u8) -> Option<u32> {
        let bits = Self::texture_bits(size);
        let bit = texel.checked_mul(bits)?;
        let address = (self.texture_image as usize).checked_add(bit >> 3)?;
        match bits {
            4 => {
                let byte = *self.rdram.get(address)?;
                Some(u32::from(if bit & 4 == 0 {
                    byte >> 4
                } else {
                    byte & 0x0f
                }))
            }
            8 => Some(u32::from(*self.rdram.get(address)?)),
            16 => {
                let bytes = self.rdram.get(address..address + 2)?;
                Some(u32::from(u16::from_be_bytes([bytes[0], bytes[1]])))
            }
            32 => {
                let bytes = self.rdram.get(address..address + 4)?;
                Some(u32::from_be_bytes(bytes.try_into().ok()?))
            }
            _ => None,
        }
    }

    fn rdp_tmem_byte(&self, address: usize) -> u8 {
        self.tmem[address & 0x0fff]
    }

    fn rdp_tmem_write_byte(&mut self, address: usize, value: u8) {
        self.tmem[address & 0x0fff] = value;
    }

    fn rdp_tmem_row_base(tile: RdpTile, t: usize) -> usize {
        usize::from(tile.tmem)
            .wrapping_mul(8)
            .wrapping_add(usize::from(tile.line).wrapping_mul(8).wrapping_mul(t))
    }

    fn write_tmem_tile_texel(&mut self, tile: RdpTile, s: usize, t: usize, value: u32) {
        let base = Self::rdp_tmem_row_base(tile, t);
        let swap = (t & 1) << 2;
        match tile.size & 3 {
            0 => {
                let address = (base.wrapping_add(s >> 1) ^ swap) & 0x0fff;
                let nibble = value as u8 & 0x0f;
                if s & 1 == 0 {
                    self.tmem[address] = (self.tmem[address] & 0x0f) | (nibble << 4);
                } else {
                    self.tmem[address] = (self.tmem[address] & 0xf0) | nibble;
                }
            }
            1 => {
                self.rdp_tmem_write_byte((base.wrapping_add(s)) ^ swap, value as u8);
            }
            2 if tile.format == 1 => {
                let address = ((base.wrapping_add(s)) & 0x07ff) ^ swap;
                let bytes = (value as u16).to_be_bytes();
                self.tmem[address & 0x07ff] = bytes[0];
                self.tmem[(address & 0x07ff) | 0x0800] = bytes[1];
            }
            2 => {
                let address = base.wrapping_add(s.wrapping_mul(2)) ^ swap;
                let bytes = (value as u16).to_be_bytes();
                self.rdp_tmem_write_byte(address, bytes[0]);
                self.rdp_tmem_write_byte(address.wrapping_add(1), bytes[1]);
            }
            _ => {
                let address = ((base.wrapping_add(s.wrapping_mul(2))) & 0x07ff) ^ swap;
                let bytes = value.to_be_bytes();
                self.rdp_tmem_write_byte(address, bytes[0]);
                self.rdp_tmem_write_byte(address.wrapping_add(1), bytes[1]);
                self.rdp_tmem_write_byte(address.wrapping_add(0x0800), bytes[2]);
                self.rdp_tmem_write_byte(address.wrapping_add(0x0801), bytes[3]);
            }
        }
    }

    fn read_tmem_tile_raw(&self, tile: RdpTile, s: usize, t: usize) -> u32 {
        let base = Self::rdp_tmem_row_base(tile, t);
        let swap = (t & 1) << 2;
        match tile.size & 3 {
            0 => {
                let address = (base.wrapping_add(s >> 1) ^ swap) & 0x0fff;
                let byte = self.tmem[address];
                u32::from(if s & 1 == 0 { byte >> 4 } else { byte & 0x0f })
            }
            1 => u32::from(self.rdp_tmem_byte(base.wrapping_add(s) ^ swap)),
            2 => {
                let address = base.wrapping_add(s.wrapping_mul(2)) ^ swap;
                u32::from(u16::from_be_bytes([
                    self.rdp_tmem_byte(address),
                    self.rdp_tmem_byte(address.wrapping_add(1)),
                ]))
            }
            _ => {
                let address = ((base.wrapping_add(s.wrapping_mul(2))) & 0x07ff) ^ swap;
                u32::from_be_bytes([
                    self.rdp_tmem_byte(address),
                    self.rdp_tmem_byte(address.wrapping_add(1)),
                    self.rdp_tmem_byte(address.wrapping_add(0x0800)),
                    self.rdp_tmem_byte(address.wrapping_add(0x0801)),
                ])
            }
        }
    }

    fn write_tmem_block_texel(&mut self, tile: RdpTile, index: usize, dxt: u32, value: u32) {
        let bits = Self::texture_bits(tile.size);
        let bit_offset = index.wrapping_mul(bits);
        let byte_offset = bit_offset >> 3;
        let word = byte_offset >> 3;
        let line = (u32::try_from(word).unwrap_or(u32::MAX).wrapping_mul(dxt)) >> 11;
        let swap = usize::try_from((line & 1) << 2).unwrap_or(0);
        let base = usize::from(tile.tmem).wrapping_mul(8);

        match tile.size & 3 {
            0 => {
                let address = (base.wrapping_add(byte_offset) ^ swap) & 0x0fff;
                let nibble = value as u8 & 0x0f;
                if bit_offset & 4 == 0 {
                    self.tmem[address] = (self.tmem[address] & 0x0f) | (nibble << 4);
                } else {
                    self.tmem[address] = (self.tmem[address] & 0xf0) | nibble;
                }
            }
            1 => self.rdp_tmem_write_byte(base.wrapping_add(byte_offset) ^ swap, value as u8),
            2 if tile.format == 1 => {
                let address = ((base.wrapping_add(index)) & 0x07ff) ^ swap;
                let bytes = (value as u16).to_be_bytes();
                self.tmem[address & 0x07ff] = bytes[0];
                self.tmem[(address & 0x07ff) | 0x0800] = bytes[1];
            }
            2 => {
                let address = base.wrapping_add(byte_offset) ^ swap;
                let bytes = (value as u16).to_be_bytes();
                self.rdp_tmem_write_byte(address, bytes[0]);
                self.rdp_tmem_write_byte(address.wrapping_add(1), bytes[1]);
            }
            _ => {
                let bank_offset = index.wrapping_mul(2);
                let address = ((base.wrapping_add(bank_offset)) & 0x07ff) ^ swap;
                let bytes = value.to_be_bytes();
                self.rdp_tmem_write_byte(address, bytes[0]);
                self.rdp_tmem_write_byte(address.wrapping_add(1), bytes[1]);
                self.rdp_tmem_write_byte(address.wrapping_add(0x0800), bytes[2]);
                self.rdp_tmem_write_byte(address.wrapping_add(0x0801), bytes[3]);
            }
        }
    }

    fn load_rdp_tile(&mut self, high: u32, low: u32, block: bool) {
        let tile_index = ((low >> 24) & 7) as usize;
        let tile = self.tiles[tile_index];

        if block {
            let start = (high >> 12) & 0x0fff;
            let row = high & 0x0fff;
            let end = (low >> 12) & 0x0fff;
            let dxt = low & 0x0fff;
            {
                let tile = &mut self.tiles[tile_index];
                tile.sl = start as u16;
                tile.tl = row as u16;
                tile.sh = end as u16;
                tile.th = dxt as u16;
            }
            if end < start {
                return;
            }
            let count = end - start + 1;
            if count > 2048 {
                return;
            }
            let source_start =
                usize::try_from(start.wrapping_add(row.wrapping_mul(self.texture_width)))
                    .unwrap_or(usize::MAX);
            for index in 0..usize::try_from(count).unwrap_or(0) {
                let Some(value) = self
                    .read_rdram_texture_raw(source_start.saturating_add(index), self.texture_size)
                else {
                    break;
                };
                self.write_tmem_block_texel(tile, index, dxt, value);
            }
            return;
        }

        let sx0 = ((high >> 12) & 0x0fff) >> 2;
        let sy0 = (high & 0x0fff) >> 2;
        let sx1 = ((low >> 12) & 0x0fff) >> 2;
        let sy1 = (low & 0x0fff) >> 2;
        if sx1 < sx0 || sy1 < sy0 {
            return;
        }

        for (destination_t, y) in (sy0..=sy1).enumerate() {
            for (destination_s, x) in (sx0..=sx1).enumerate() {
                let source = usize::try_from(y.wrapping_mul(self.texture_width).wrapping_add(x))
                    .unwrap_or(usize::MAX);
                let Some(value) = self.read_rdram_texture_raw(source, self.texture_size) else {
                    return;
                };
                self.write_tmem_tile_texel(tile, destination_s, destination_t, value);
            }
        }

        let tile = &mut self.tiles[tile_index];
        tile.sl = ((high >> 12) & 0x0fff) as u16;
        tile.tl = (high & 0x0fff) as u16;
        tile.sh = ((low >> 12) & 0x0fff) as u16;
        tile.th = (low & 0x0fff) as u16;
    }

    fn load_rdp_tlut(&mut self, high: u32, low: u32) {
        let tile_index = ((low >> 24) & 7) as usize;
        let tile = self.tiles[tile_index];
        let first = ((high >> 12) & 0x0fff) >> 2;
        let last = ((low >> 12) & 0x0fff) >> 2;
        if last < first {
            return;
        }
        let source_start =
            first.wrapping_add(((high & 0x0fff) >> 2).wrapping_mul(self.texture_width));
        let palette_start = usize::from(tile.tmem.saturating_sub(256)).min(255);
        let tmem_base = usize::from(tile.tmem).wrapping_mul(8);
        for index in 0..=usize::try_from(last - first).unwrap_or(0) {
            let source = usize::try_from(source_start)
                .unwrap_or(0)
                .saturating_add(index);
            let Some(value) = self.read_rdram_texture_raw(source, 2) else {
                break;
            };
            let value = value as u16;
            let destination = palette_start + index;
            if destination < self.tlut.len() {
                self.tlut[destination] = value;
            }
            let bytes = value.to_be_bytes();
            let entry = tmem_base.wrapping_add(index.wrapping_mul(8));
            for copy in 0..4usize {
                let address = entry.wrapping_add(copy * 2);
                self.rdp_tmem_write_byte(address, bytes[0]);
                self.rdp_tmem_write_byte(address.wrapping_add(1), bytes[1]);
            }
        }

        let tile = &mut self.tiles[tile_index];
        tile.sl = ((high >> 12) & 0x0fff) as u16;
        tile.tl = (high & 0x0fff) as u16;
        tile.sh = ((low >> 12) & 0x0fff) as u16;
        tile.th = (low & 0x0fff) as u16;
    }

    fn shifted_texture_coordinate(value: i32, shift: u8) -> i32 {
        if shift <= 10 {
            value >> shift
        } else {
            value.wrapping_shl(u32::from(16 - shift))
        }
    }

    fn tile_coordinate(
        value: i32,
        low: u16,
        high: u16,
        mask: u8,
        mirror: bool,
        clamp: bool,
    ) -> usize {
        let low = i32::from(low >> 2);
        let high = i32::from(high >> 2).max(low);
        let mut local = value - low;
        if clamp || mask == 0 {
            return local.clamp(0, high - low) as usize;
        }
        let span = 1i32 << mask.min(10);
        if mirror {
            let period = span * 2;
            local = local.rem_euclid(period);
            if local >= span {
                local = period - 1 - local;
            }
        } else {
            local = local.rem_euclid(span);
        }
        local as usize
    }

    fn expand_4(value: u32) -> u8 {
        ((value & 0x0f) * 17) as u8
    }

    fn sample_rdp_yuv(&self, tile: RdpTile, s: usize, t: usize) -> [u8; 4] {
        let base = Self::rdp_tmem_row_base(tile, t);
        let swap = (t & 1) << 2;
        let address = ((base.wrapping_add(s)) & 0x07ff) ^ swap;
        let pair = address & !1;
        let u = i32::from(self.tmem[pair]) - 0x80;
        let v = i32::from(self.tmem[(pair + 1) & 0x07ff]) - 0x80;
        let y = i32::from(self.tmem[address | 0x0800]);
        let k0 = i32::from(self.convert_k[0]) * 2 + 1;
        let k1 = i32::from(self.convert_k[1]) * 2 + 1;
        let k2 = i32::from(self.convert_k[2]) * 2 + 1;
        let k3 = i32::from(self.convert_k[3]) * 2 + 1;
        let r = y + ((k0 * v + 0x80) >> 8);
        let g = y + ((k1 * u + k2 * v + 0x80) >> 8);
        let b = y + ((k3 * u + 0x80) >> 8);
        [
            r.clamp(0, 255) as u8,
            g.clamp(0, 255) as u8,
            b.clamp(0, 255) as u8,
            y as u8,
        ]
    }

    fn rdp_resolve_texture_texel(
        &self,
        tile_index: usize,
        s: i32,
        t: i32,
    ) -> Option<(RdpTile, usize, usize)> {
        let tile = *self.tiles.get(tile_index)?;
        let s = Self::shifted_texture_coordinate(s, tile.shift_s);
        let t = Self::shifted_texture_coordinate(t, tile.shift_t);
        let s = Self::tile_coordinate(
            s,
            tile.sl,
            tile.sh,
            tile.mask_s,
            tile.mirror_s,
            tile.clamp_s,
        );
        let t = Self::tile_coordinate(
            t,
            tile.tl,
            tile.th,
            tile.mask_t,
            tile.mirror_t,
            tile.clamp_t,
        );
        Some((tile, s, t))
    }

    fn sample_rdp_texture(&self, tile_index: usize, s: i32, t: i32) -> Option<[u8; 4]> {
        let (_, s, t) = self.rdp_resolve_texture_texel(tile_index, s, t)?;
        self.sample_rdp_texel(tile_index, s, t)
    }

    fn rdp_copy_u16_texel(&self, tile_index: usize, s: i32, t: i32) -> Option<u16> {
        let (tile, s, t) = self.rdp_resolve_texture_texel(tile_index, s, t)?;
        let raw = self.read_tmem_tile_raw(tile, s, t);

        if self.other_modes & (1u64 << 47) != 0 {
            let index = match (tile.format, tile.size) {
                (2, 0) => usize::from(tile.palette) * 16 + raw as usize,
                (2, 1) => raw as usize,
                _ => return None,
            };
            return self.tlut.get(index).copied();
        }

        if tile.size == 2 && tile.format != 1 {
            return Some(raw as u16);
        }
        None
    }

    fn rdp_copy_shift_coordinate(value: i32, shift: u8) -> i32 {
        let value = value as i16;
        if shift < 11 {
            i32::from(value) >> shift
        } else {
            i32::from(value.wrapping_shl(u32::from(16 - shift.min(15))))
        }
    }

    fn rdp_copy_mask_coordinate(mut value: i32, mask: u8, mirror: bool) -> usize {
        if mask != 0 {
            let mask = mask.min(10);
            if mirror {
                let wrap = (value >> mask) & 1;
                value ^= -wrap;
            }
            value &= (1i32 << mask) - 1;
        }
        value as usize
    }

    fn rdp_copy_u16_resolved(&self, tile: RdpTile, s: usize, t: usize) -> Option<u16> {
        let raw = self.read_tmem_tile_raw(tile, s, t);
        if self.other_modes & (1u64 << 47) != 0 {
            let index = match (tile.format, tile.size) {
                (2, 0) => usize::from(tile.palette) * 16 + raw as usize,
                (2, 1) => raw as usize,
                _ => return None,
            };
            return self.tlut.get(index).copied();
        }
        (tile.size == 2 && tile.format != 1).then_some(raw as u16)
    }

    fn rdp_copy_u16_qword(&self, tile_index: usize, s: i32, t: i32) -> Option<[u16; 4]> {
        let tile = *self.tiles.get(tile_index)?;
        let s = Self::rdp_copy_shift_coordinate(s, tile.shift_s)
            .wrapping_sub(i32::from(tile.sl) << 3)
            >> 5;
        let t = Self::rdp_copy_shift_coordinate(t, tile.shift_t)
            .wrapping_sub(i32::from(tile.tl) << 3)
            >> 5;
        let t = Self::rdp_copy_mask_coordinate(t, tile.mask_t, tile.mirror_t);
        let mut words = [0u16; 4];
        for (index, word) in words.iter_mut().enumerate() {
            let s = Self::rdp_copy_mask_coordinate(
                s.wrapping_add(index as i32),
                tile.mask_s,
                tile.mirror_s,
            );
            *word = self.rdp_copy_u16_resolved(tile, s, t)?;
        }
        Some(words)
    }

    fn rdp_copy_u16_span(&mut self, y: usize, x0: usize, x1: usize, flip: bool) -> bool {
        if x0 > x1 {
            return true;
        }
        let Some(words) = self.rdp_copy_u16_qword(0, 0, 0) else {
            return false;
        };
        let mut qword = 0u64;
        for word in words {
            qword = (qword << 16) | u64::from(word);
        }

        let width = self.color_width.max(1) as usize;
        let start_pixel = y
            .saturating_mul(width)
            .saturating_add(if flip { x0 } else { x1 });
        let mut address = (self.color_image as usize).saturating_add(start_pixel.saturating_mul(2));
        let direction = if flip { 1isize } else { -1isize };
        let mut remaining = x1.saturating_sub(x0).saturating_add(1).saturating_mul(2);

        while remaining != 0 {
            let count = remaining.min(8);
            for byte_index in 0..count {
                let word_index = byte_index >> 1;
                if self.other_modes as u32 & 1 != 0 && words[word_index] & 1 == 0 {
                    continue;
                }
                let shift = (7 - byte_index) * 8;
                let byte = (qword >> shift) as u8;
                let target = if direction > 0 {
                    address.checked_add(byte_index)
                } else {
                    address.checked_sub(byte_index)
                };
                if let Some(target) = target.and_then(|target| self.rdram.get_mut(target)) {
                    *target = byte;
                }
            }

            remaining -= count;
            if remaining == 0 {
                break;
            }
            address = if direction > 0 {
                address.saturating_add(8)
            } else {
                address.saturating_sub(8)
            };
        }

        for x in x0..=x1 {
            let pixel = y.saturating_mul(width).saturating_add(x);
            let address = (self.color_image as usize).saturating_add(pixel.saturating_mul(2));
            if let Some(bytes) = self.rdram.get(address..address.saturating_add(2)) {
                let word = u16::from_be_bytes([bytes[0], bytes[1]]);
                self.rdp_hidden_write(address, if word & 1 != 0 { 3 } else { 0 });
            }
        }
        true
    }

    fn shifted_texture_coordinate_fixed(value: i32, shift: u8) -> i32 {
        let value = value as i16;
        if shift <= 10 {
            i32::from(value) >> shift
        } else {
            i32::from(value.wrapping_shl(u32::from(16 - shift.min(15))))
        }
    }

    fn rdp_texture_axis(value: i32, tile: RdpTile, is_t: bool) -> (i32, u32, i32) {
        let (shift, low, high, mask, mirror, clamp) = if is_t {
            (
                tile.shift_t,
                tile.tl,
                tile.th,
                tile.mask_t,
                tile.mirror_t,
                tile.clamp_t,
            )
        } else {
            (
                tile.shift_s,
                tile.sl,
                tile.sh,
                tile.mask_s,
                tile.mirror_s,
                tile.clamp_s,
            )
        };
        let shifted = Self::shifted_texture_coordinate_fixed(value, shift);
        let over_max = (shifted >> 3) >= i32::from(high);
        let relative = shifted - (i32::from(low) << 3);
        let mut fraction = (relative & 0x1f) as u32;
        let mut base = if clamp || mask == 0 {
            if over_max {
                fraction = 0;
                ((i32::from(high) >> 2) - (i32::from(low) >> 2)).max(0)
            } else if relative >= 0 {
                relative >> 5
            } else {
                fraction = 0;
                0
            }
        } else {
            relative >> 5
        };

        let mask = mask.min(10);
        if mask == 0 {
            return (base, fraction, 1);
        }

        let mask_bits = (1i32 << mask) - 1;
        let neighbor = if mirror {
            let wrap = (base >> mask) & 1;
            base = (base ^ -wrap) & mask_bits;
            if (base - wrap) & mask_bits == mask_bits {
                0
            } else {
                1 - (wrap << 1)
            }
        } else {
            base &= mask_bits;
            if base == mask_bits {
                if is_t {
                    -(base & 0xff)
                } else {
                    -base
                }
            } else {
                1
            }
        };
        (base, fraction, neighbor)
    }

    fn rdp_filter_3point(
        samples: [[u8; 4]; 4],
        s_fraction: u32,
        t_fraction: u32,
        mid_texel: bool,
    ) -> [u8; 4] {
        let [t0, t1, t2, t3] = samples;
        let sf = s_fraction as i32;
        let tf = t_fraction as i32;
        let upper = (s_fraction + t_fraction) & 0x20 != 0;
        let center = mid_texel && s_fraction == 0x10 && t_fraction == 0x10;
        let mut output = [0u8; 4];

        for component in 0..4 {
            let c0 = i32::from(t0[component]);
            let c1 = i32::from(t1[component]);
            let c2 = i32::from(t2[component]);
            let c3 = i32::from(t3[component]);
            let value = if center {
                c3 + ((((c1 + c2) << 6) - (c3 << 7) + ((!c3 + c0) << 6) + 0xc0) >> 8)
            } else if upper {
                c3 + (((0x20 - sf) * (c2 - c3) + (0x20 - tf) * (c1 - c3) + 0x10) >> 5)
            } else {
                c0 + ((sf * (c1 - c0) + tf * (c2 - c0) + 0x10) >> 5)
            };
            output[component] = value.clamp(0, 255) as u8;
        }
        output
    }

    fn sample_rdp_texture_fixed(&self, tile_index: usize, s: i32, t: i32) -> Option<[u8; 4]> {
        if self.other_modes & (1u64 << 45) == 0 {
            return self.sample_rdp_texture(tile_index, s >> 5, t >> 5);
        }

        let tile = *self.tiles.get(tile_index)?;
        let (s0, sf, sd) = Self::rdp_texture_axis(s, tile, false);
        let (t0, tf, td) = Self::rdp_texture_axis(t, tile, true);
        let s1 = s0.checked_add(sd)?;
        let t1 = t0.checked_add(td)?;
        let s0 = usize::try_from(s0).ok()?;
        let s1 = usize::try_from(s1).ok()?;
        let t0 = usize::try_from(t0).ok()?;
        let t1 = usize::try_from(t1).ok()?;
        let samples = [
            self.sample_rdp_texel(tile_index, s0, t0)?,
            self.sample_rdp_texel(tile_index, s1, t0)?,
            self.sample_rdp_texel(tile_index, s0, t1)?,
            self.sample_rdp_texel(tile_index, s1, t1)?,
        ];
        Some(Self::rdp_filter_3point(
            samples,
            sf,
            tf,
            self.other_modes & (1u64 << 44) != 0,
        ))
    }

    fn rdp_find_msb(value: i32) -> i32 {
        if value <= 0 {
            -1
        } else {
            31 - value.leading_zeros() as i32
        }
    }

    fn rdp_perspective_get_lut(w: i32) -> (i32, i32) {
        let shift = (14 - Self::rdp_find_msb(w)).min(14);
        let normalized = (w << shift) & 0x3fff;
        let fraction = normalized & 0xff;
        let (base, slope) = RDP_PERSPECTIVE_TABLE[(normalized >> 8) as usize];
        let reciprocal = ((i32::from(slope) * fraction) >> 10) + i32::from(base);
        (reciprocal, shift)
    }

    fn rdp_perspective_divide(s: i32, t: i32, w: i32) -> [i32; 2] {
        let w_carry = w <= 0;
        let w = w & 0x7fff;
        let (reciprocal, shift) = Self::rdp_perspective_get_lut(w);
        let products = [s.wrapping_mul(reciprocal), t.wrapping_mul(reciprocal)];
        let mask = ((1 << 30) - 1) & -((1 << 29) >> shift);
        let out_of_bounds = [products[0] & mask, products[1] & mask];
        let (mut result, saturation_source) = if shift == 14 {
            ([products[0] << 1, products[1] << 1], products)
        } else {
            let shifted = [products[0] >> (13 - shift), products[1] >> (13 - shift)];
            (shifted, shifted)
        };
        if out_of_bounds != [0, 0] {
            for component in 0..2 {
                if out_of_bounds[component] != mask && out_of_bounds[component] != 0 {
                    result[component] = if saturation_source[component] & (1 << 29) == 0 {
                        0x7fff
                    } else {
                        -0x8000
                    };
                }
            }
        }
        if w_carry {
            result = [0x7fff; 2];
        }
        [
            result[0].clamp(-0x1_0000, 0xffff),
            result[1].clamp(-0x1_0000, 0xffff),
        ]
    }

    fn rdp_texture_coordinates(texture: [i64; 4], perspective: bool) -> [i32; 2] {
        if perspective {
            return Self::rdp_perspective_divide(
                (texture[0] >> 16) as i32,
                (texture[1] >> 16) as i32,
                (texture[2] >> 16) as i32,
            );
        }
        [(texture[0] >> 11) as i32, (texture[1] >> 11) as i32]
    }

    fn rdp_lod_delta(
        s_current: i32,
        s_next: i32,
        t_current: i32,
        t_next: i32,
        previous: i32,
    ) -> i32 {
        let fold = |next: i32, current: i32| {
            let next = Self::sign_extend((next & 0x1_ffff) as u32, 17);
            let current = Self::sign_extend((current & 0x1_ffff) as u32, 17);
            let delta = next - current;
            if delta & 0x2_0000 != 0 {
                !delta & 0x1_ffff
            } else {
                delta
            }
        };
        let delta = fold(s_next, s_current)
            .max(fold(t_next, t_current))
            .max(previous);
        let mut lod = delta & 0x7fff;
        if delta & 0x1_c000 != 0 {
            lod |= 0x4000;
        }
        lod
    }

    fn rdp_lod_signals(&self, lod_clamp: bool, lod: i32, max_level: u8) -> RdpLodSignals {
        let sharpen = self.other_modes & (1u64 << 49) != 0;
        let detail = self.other_modes & (1u64 << 50) != 0;
        let plain = !sharpen && !detail;
        let min_level = i32::from(self.primitive_min_level);

        if lod & 0x4000 != 0 || lod_clamp {
            return RdpLodSignals {
                frac: 0xff,
                level: 0,
                magnify: false,
                distant: true,
            };
        }

        if lod < min_level || lod < 32 {
            let distant = max_level == 0;
            let frac = if plain {
                if distant {
                    0xff
                } else {
                    0
                }
            } else {
                let base = if lod < min_level { min_level } else { lod };
                let value = base << 3;
                if sharpen {
                    value | 0x100
                } else {
                    value
                }
            };
            return RdpLodSignals {
                frac: frac as i16,
                level: 0,
                magnify: true,
                distant,
            };
        }

        let magnitude = ((lod >> 5) as u32) & 0xff;
        let level = if magnitude < 2 { 0 } else { magnitude.ilog2() };
        let distant = if max_level == 0 {
            true
        } else {
            lod & 0x6000 != 0 || level >= u32::from(max_level)
        };
        let frac = if plain && distant {
            0xff
        } else {
            ((lod << 3) >> level) & 0xff
        };
        RdpLodSignals {
            frac: frac as i16,
            level,
            magnify: false,
            distant,
        }
    }

    fn rdp_lod_tiles(
        &self,
        base_tile: usize,
        signals: RdpLodSignals,
        max_level: u8,
    ) -> (usize, usize) {
        let sharpen = self.other_modes & (1u64 << 49) != 0;
        let detail = self.other_modes & (1u64 << 50) != 0;
        let level = if signals.distant {
            usize::from(max_level)
        } else {
            signals.level as usize
        };

        if detail {
            let finer = usize::from(!signals.magnify);
            let first = (base_tile + level + finer) & 7;
            let second = if !signals.distant && !signals.magnify {
                (base_tile + level + 2) & 7
            } else {
                (base_tile + level + 1) & 7
            };
            return (first, second);
        }

        let first = (base_tile + level) & 7;
        let second = if signals.distant || (!sharpen && signals.magnify) {
            first
        } else {
            (first + 1) & 7
        };
        (first, second)
    }

    fn sample_rdp_texel(&self, tile_index: usize, s: usize, t: usize) -> Option<[u8; 4]> {
        let tile = *self.tiles.get(tile_index)?;
        let raw = self.read_tmem_tile_raw(tile, s, t);

        match (tile.format, tile.size) {
            (0, 2) => Some(Self::rgba5551(raw as u16)),
            (1, 2) => Some(self.sample_rdp_yuv(tile, s, t)),
            (0, 3) => Some(raw.to_be_bytes()),
            (2, _) if self.other_modes & (1u64 << 47) == 0 => Some([0; 4]),
            (2, 0) => {
                let index = usize::from(tile.palette) * 16 + raw as usize;
                self.rdp_tlut_color(index)
            }
            (2, 1) => self.rdp_tlut_color(raw as usize),
            (3, 0) => {
                let intensity = ((raw >> 1) & 7) * 255 / 7;
                let alpha = if raw & 1 != 0 { 255 } else { 0 };
                Some([intensity as u8, intensity as u8, intensity as u8, alpha])
            }
            (3, 1) => {
                let intensity = Self::expand_4(raw >> 4);
                let alpha = Self::expand_4(raw);
                Some([intensity, intensity, intensity, alpha])
            }
            (3, 2) => {
                let intensity = (raw >> 8) as u8;
                let alpha = raw as u8;
                Some([intensity, intensity, intensity, alpha])
            }
            (4, 0) => {
                let intensity = Self::expand_4(raw);
                Some([intensity, intensity, intensity, intensity])
            }
            (4, 1) => {
                let intensity = raw as u8;
                Some([intensity, intensity, intensity, intensity])
            }
            _ => None,
        }
    }

    fn rdp_tlut_color(&self, index: usize) -> Option<[u8; 4]> {
        let value = *self.tlut.get(index)?;
        if self.other_modes & (1u64 << 46) != 0 {
            let intensity = (value >> 8) as u8;
            Some([intensity, intensity, intensity, value as u8])
        } else {
            Some(Self::rgba5551(value))
        }
    }

    fn rdp_register_color(value: u32) -> [u8; 4] {
        value.to_be_bytes()
    }

    fn rdp_rgb_mux(
        &self,
        selector: u8,
        role: u8,
        component: usize,
        sources: &[[u8; 4]; 4],
        noise: i32,
        lod_frac: i16,
    ) -> i32 {
        let [combined, texel0, texel1, shade] = *sources;
        let primitive = Self::rdp_register_color(self.primitive_color);
        let environment = Self::rdp_register_color(self.environment_color);
        match role {
            0 => match selector {
                0 => i32::from(combined[component]),
                1 => i32::from(texel0[component]),
                2 => i32::from(texel1[component]),
                3 => i32::from(primitive[component]),
                4 => i32::from(shade[component]),
                5 => i32::from(environment[component]),
                6 => 0x100,
                7 => noise,
                _ => 0,
            },
            1 => match selector {
                0 => i32::from(combined[component]),
                1 => i32::from(texel0[component]),
                2 => i32::from(texel1[component]),
                3 => i32::from(primitive[component]),
                4 => i32::from(shade[component]),
                5 => i32::from(environment[component]),
                6 => i32::from(self.key_center[component]),
                7 => i32::from(self.convert_k[4]),
                _ => 0,
            },
            2 => match selector {
                0 => i32::from(combined[component]),
                1 => i32::from(texel0[component]),
                2 => i32::from(texel1[component]),
                3 => i32::from(primitive[component]),
                4 => i32::from(shade[component]),
                5 => i32::from(environment[component]),
                6 => i32::from(self.key_scale[component]),
                7 => i32::from(combined[3]),
                8 => i32::from(texel0[3]),
                9 => i32::from(texel1[3]),
                10 => i32::from(primitive[3]),
                11 => i32::from(shade[3]),
                12 => i32::from(environment[3]),
                13 => i32::from(lod_frac),
                14 => i32::from(self.primitive_lod_frac),
                15 => i32::from(self.convert_k[5]),
                _ => 0,
            },
            _ => match selector {
                0 => i32::from(combined[component]),
                1 => i32::from(texel0[component]),
                2 => i32::from(texel1[component]),
                3 => i32::from(primitive[component]),
                4 => i32::from(shade[component]),
                5 => i32::from(environment[component]),
                6 => 0x100,
                _ => 0,
            },
        }
    }

    fn rdp_alpha_mux(
        &self,
        selector: u8,
        role_c: bool,
        combined: [u8; 4],
        texels: [[u8; 4]; 2],
        shade: [u8; 4],
        lod_frac: i16,
    ) -> i32 {
        let [texel0, texel1] = texels;
        let primitive = Self::rdp_register_color(self.primitive_color);
        let environment = Self::rdp_register_color(self.environment_color);
        if role_c {
            return match selector {
                0 => i32::from(lod_frac),
                1 => i32::from(texel0[3]),
                2 => i32::from(texel1[3]),
                3 => i32::from(primitive[3]),
                4 => i32::from(shade[3]),
                5 => i32::from(environment[3]),
                6 => i32::from(self.primitive_lod_frac),
                _ => 0,
            };
        }
        match selector {
            0 => i32::from(combined[3]),
            1 => i32::from(texel0[3]),
            2 => i32::from(texel1[3]),
            3 => i32::from(primitive[3]),
            4 => i32::from(shade[3]),
            5 => i32::from(environment[3]),
            6 => 0x100,
            _ => 0,
        }
    }

    fn rdp_sext9(value: i32) -> i32 {
        (value << 23) >> 23
    }

    fn rdp_special_expand(value: i32) -> i32 {
        Self::rdp_sext9(value - 0x80) + 0x80
    }

    fn rdp_combiner_value(a: i32, b: i32, c: i32, d: i32) -> i32 {
        let color =
            (Self::rdp_special_expand(a) - Self::rdp_special_expand(b)) * Self::rdp_sext9(c) + 0x80;
        (color >> 8) + Self::rdp_special_expand(d)
    }

    fn rdp_clamp_9bit(value: i32) -> u8 {
        Self::rdp_special_expand(value).clamp(0, 0xff) as u8
    }

    fn rdp_combiner_term(a: i32, b: i32, c: i32, d: i32) -> u8 {
        Self::rdp_clamp_9bit(Self::rdp_combiner_value(a, b, c, d))
    }

    fn rdp_combine_cycle(
        &self,
        cycle: usize,
        texels: [[u8; 4]; 2],
        shade: [u8; 4],
        combined: [u8; 4],
        noise: i32,
        lod_frac: i16,
    ) -> [u8; 4] {
        let [texel0, texel1] = texels;
        let mode = self.combine_mode;
        let (a, b, c, d, aa, ab, ac, ad) = if cycle == 0 {
            (
                ((mode >> 52) & 0x0f) as u8,
                ((mode >> 28) & 0x0f) as u8,
                ((mode >> 47) & 0x1f) as u8,
                ((mode >> 15) & 0x07) as u8,
                ((mode >> 44) & 0x07) as u8,
                ((mode >> 12) & 0x07) as u8,
                ((mode >> 41) & 0x07) as u8,
                ((mode >> 9) & 0x07) as u8,
            )
        } else {
            (
                ((mode >> 37) & 0x0f) as u8,
                ((mode >> 24) & 0x0f) as u8,
                ((mode >> 32) & 0x1f) as u8,
                ((mode >> 6) & 0x07) as u8,
                ((mode >> 21) & 0x07) as u8,
                ((mode >> 3) & 0x07) as u8,
                ((mode >> 18) & 0x07) as u8,
                (mode & 0x07) as u8,
            )
        };

        let sources = [combined, texel0, texel1, shade];
        let mut output = [0u8; 4];
        for (component, channel) in output.iter_mut().take(3).enumerate() {
            *channel = Self::rdp_combiner_term(
                self.rdp_rgb_mux(a, 0, component, &sources, noise, lod_frac),
                self.rdp_rgb_mux(b, 1, component, &sources, noise, lod_frac),
                self.rdp_rgb_mux(c, 2, component, &sources, noise, lod_frac),
                self.rdp_rgb_mux(d, 3, component, &sources, noise, lod_frac),
            );
        }
        output[3] = Self::rdp_combiner_term(
            self.rdp_alpha_mux(aa, false, combined, [texel0, texel1], shade, lod_frac),
            self.rdp_alpha_mux(ab, false, combined, [texel0, texel1], shade, lod_frac),
            self.rdp_alpha_mux(ac, true, combined, [texel0, texel1], shade, lod_frac),
            self.rdp_alpha_mux(ad, false, combined, [texel0, texel1], shade, lod_frac),
        );
        output
    }

    fn read_rdp_rgba(&self, x: usize, y: usize) -> [u8; 4] {
        if x >= self.color_width.max(1) as usize {
            return [0; 4];
        }
        let pixel = y
            .saturating_mul(self.color_width.max(1) as usize)
            .saturating_add(x);
        match self.color_size {
            0 => [0, 0, 0, 0xe0],
            1 => {
                let address = self.color_image as usize + pixel;
                self.rdram
                    .get(address)
                    .map_or([0; 4], |value| [*value, *value, *value, 0xe0])
            }
            2 => {
                let address = self.color_image as usize + pixel * 2;
                if address + 2 > self.rdram.len() {
                    return [0; 4];
                }
                let word = u16::from_be_bytes([self.rdram[address], self.rdram[address + 1]]);
                if self.color_format == 0 {
                    Self::rgba5551(word)
                } else {
                    let intensity = (word >> 8) as u8;
                    let coverage = ((word >> 5) & 7) as u8;
                    [intensity, intensity, intensity, coverage << 5]
                }
            }
            3 => {
                let address = self.color_image as usize + pixel * 4;
                if address + 4 > self.rdram.len() {
                    return [0; 4];
                }
                self.rdram[address..address + 4]
                    .try_into()
                    .unwrap_or([0; 4])
            }
            _ => [0; 4],
        }
    }

    fn rdp_framebuffer_coverage_read(&self, x: usize, y: usize) -> u8 {
        if self.other_modes as u32 & 0x0040 == 0 {
            return 7;
        }
        let width = self.color_width.max(1) as usize;
        let Some(pixel) = y.checked_mul(width).and_then(|value| value.checked_add(x)) else {
            return 7;
        };
        match self.color_size {
            1 => 7,
            2 => {
                let Some(address) =
                    (self.color_image as usize).checked_add(pixel.saturating_mul(2))
                else {
                    return 7;
                };
                let Some(bytes) = self.rdram.get(address..address.saturating_add(2)) else {
                    return 7;
                };
                let word = u16::from_be_bytes([bytes[0], bytes[1]]);
                if self.color_format == 0 {
                    ((((word & 1) as u8) << 2) | self.rdp_hidden_read(address)) & 7
                } else {
                    ((word >> 5) & 7) as u8
                }
            }
            3 => {
                let Some(address) =
                    (self.color_image as usize).checked_add(pixel.saturating_mul(4))
                else {
                    return 7;
                };
                self.rdram
                    .get(address.saturating_add(3))
                    .map_or(7, |value| (value >> 5) & 7)
            }
            _ => 7,
        }
    }

    fn rdp_finalize_coverage(
        &self,
        blend_en: bool,
        coverage_count: i32,
        memory_coverage: u8,
    ) -> u8 {
        let coverage_count = coverage_count.clamp(0, 8);
        let memory_coverage = i32::from(memory_coverage & 7);
        let destination = ((self.other_modes as u32 >> 8) & 3) as u8;
        let final_coverage = match destination {
            0 => {
                let value = if blend_en {
                    coverage_count + memory_coverage
                } else {
                    coverage_count - 1
                };
                if value & 8 != 0 {
                    7
                } else {
                    value & 7
                }
            }
            1 => (coverage_count + memory_coverage) & 7,
            2 => 7,
            _ => memory_coverage,
        };
        final_coverage as u8
    }

    fn rdp_memory_color(&self, x: usize, y: usize, memory_coverage: u8) -> [u8; 4] {
        let mut memory = self.read_rdp_rgba(x, y);
        memory[3] = (memory_coverage & 7) << 5;
        memory
    }

    fn rdp_blend_color_source(&self, selector: u8, input: [u8; 4], memory: [u8; 4]) -> [u8; 4] {
        match selector {
            0 => input,
            1 => memory,
            2 => Self::rdp_register_color(self.blend_color),
            _ => Self::rdp_register_color(self.fog_color),
        }
    }

    fn rdp_blend_alpha_source(&self, selector: u8, input: [u8; 4], shade: [u8; 4]) -> u16 {
        match selector {
            0 => u16::from(input[3]),
            1 => u16::from(Self::rdp_register_color(self.fog_color)[3]),
            2 => u16::from(shade[3]),
            _ => 0,
        }
    }

    fn rdp_blend_divider(index: u32) -> u8 {
        let d = (index >> 11) & 0x0f;
        let n = index & 0x07ff;
        let inverse_d = (!d) & 0x0f;
        let mut partial = [0u32; 9];
        let mut result = 0u32;
        let mut temp = inverse_d + (n >> 8) + 1;
        partial[0] = temp & 7;
        for k in 0..8 {
            let nbit = (n >> (7 - k)) & 1;
            temp = if result & (0x100 >> k) != 0 {
                inverse_d + (partial[k] << 1) + nbit + 1
            } else {
                d + (partial[k] << 1) + nbit
            };
            partial[k + 1] = temp & 7;
            if temp & 0x10 != 0 {
                result |= 1 << (7 - k);
            }
        }
        result as u8
    }

    fn rdp_blend_cycle(
        &self,
        selectors: [u8; 4],
        input: [u8; 4],
        shade: [u8; 4],
        memory: [u8; 4],
        fixed_weight: bool,
        blend_shift: [u8; 2],
    ) -> [u8; 3] {
        let [p_sel, a_sel, m_sel, b_sel] = selectors;
        let p = self.rdp_blend_color_source(p_sel, input, memory);
        let m = self.rdp_blend_color_source(m_sel, input, memory);
        let alpha_full = self.rdp_blend_alpha_source(a_sel, input, shade) as u8;
        let beta_full = match b_sel {
            0 => !alpha_full,
            1 => memory[3],
            2 => 0xff,
            _ => 0,
        };
        let mut alpha = u32::from(alpha_full >> 3);
        let mut beta = u32::from(beta_full >> 3);
        if b_sel == 1 {
            alpha = (alpha >> blend_shift[0]) & 0x3c;
            beta = (beta >> blend_shift[1]) | 3;
        }

        let mut output = [0u8; 3];
        for component in 0..3 {
            let blended = u32::from(p[component]) * alpha + u32::from(m[component]) * (beta + 1);
            output[component] = if fixed_weight {
                ((blended >> 5) & 0xff) as u8
            } else {
                let denominator = ((alpha & !3) + (beta & !3) + 4) << 9;
                Self::rdp_blend_divider(denominator | ((blended >> 2) & 0x07ff))
            };
        }
        output
    }

    fn rdp_blend(
        &self,
        x: usize,
        y: usize,
        input: [u8; 4],
        shade: [u8; 4],
        state: RdpPixelState,
    ) -> [u8; 4] {
        let modes = self.other_modes as u32;
        let cycle_type = self.rdp_cycle_type();
        if cycle_type >= 2 {
            return input;
        }
        let force_blend = modes & 0x4000 != 0;
        let color_on_coverage = modes & 0x0080 != 0 && !state.depth.coverage_wrap;
        if !color_on_coverage && !state.depth.blend_en && !force_blend {
            return input;
        }
        let memory = self.rdp_memory_color(x, y, state.memory_coverage);
        let cycle0 = [
            ((modes >> 30) & 3) as u8,
            ((modes >> 26) & 3) as u8,
            ((modes >> 22) & 3) as u8,
            ((modes >> 18) & 3) as u8,
        ];
        let first = self.rdp_blend_cycle(
            cycle0,
            input,
            shade,
            memory,
            force_blend || cycle_type == 1,
            state.depth.blend_shift,
        );
        let mut output = input;
        if cycle_type == 0 && color_on_coverage {
            let source = self.rdp_blend_color_source(cycle0[2], input, memory);
            output[..3].copy_from_slice(&source[..3]);
            return output;
        }
        output[..3].copy_from_slice(&first);

        if cycle_type == 1 {
            let cycle1 = [
                ((modes >> 28) & 3) as u8,
                ((modes >> 24) & 3) as u8,
                ((modes >> 20) & 3) as u8,
                ((modes >> 16) & 3) as u8,
            ];
            let mut chained = input;
            chained[..3].copy_from_slice(&first);
            if color_on_coverage {
                let source = self.rdp_blend_color_source(cycle1[2], chained, memory);
                output[..3].copy_from_slice(&source[..3]);
            } else {
                let second = self.rdp_blend_cycle(
                    cycle1,
                    chained,
                    shade,
                    memory,
                    force_blend,
                    state.depth.blend_shift,
                );
                output[..3].copy_from_slice(&second);
            }
        }
        output
    }

    fn rdp_next_random(&mut self) -> u32 {
        self.rdp_noise_seed = self
            .rdp_noise_seed
            .wrapping_mul(0x343fd)
            .wrapping_add(0x269ec3);
        (self.rdp_noise_seed >> 16) & 0x7fff
    }

    fn rdp_combiner_uses_noise(&self, cycle_type: u8) -> bool {
        if self.combine_mode == 0 || cycle_type >= 2 {
            return false;
        }
        let cycle0 = ((self.combine_mode >> 52) & 0x0f) == 7;
        let cycle1 = ((self.combine_mode >> 37) & 0x0f) == 7;
        cycle0 || (cycle_type == 1 && cycle1)
    }

    fn rdp_dither_signals(
        &mut self,
        x: usize,
        y: usize,
        combiner_uses_noise: bool,
    ) -> RdpDitherSignals {
        let rgb_mode = ((self.other_modes >> 38) & 3) as u8;
        let alpha_mode = ((self.other_modes >> 36) & 3) as u8;
        let index = ((y & 3) << 2) | (x & 3);
        let matrix_value = if rgb_mode & 1 == 0 {
            RDP_DITHER_MAGIC[index]
        } else {
            RDP_DITHER_BAYER[index]
        };

        let noise = if combiner_uses_noise || alpha_mode == 2 {
            (((self.rdp_next_random() & 7) << 6) | 0x20) as i32
        } else {
            0x20
        };
        let rgb = match rgb_mode {
            0 | 1 => [i32::from(matrix_value); 3],
            2 => {
                let random = self.rdp_next_random();
                [
                    (random & 7) as i32,
                    ((random >> 3) & 7) as i32,
                    ((random >> 6) & 7) as i32,
                ]
            }
            _ => [7; 3],
        };
        let matrix_value = i32::from(matrix_value);
        let alpha = match alpha_mode {
            0 => matrix_value,
            1 => (!matrix_value) & 7,
            2 => (noise >> 6) & 7,
            _ => 0,
        };

        RdpDitherSignals { rgb, alpha, noise }
    }

    fn rdp_apply_rgb_dither(mut color: [u8; 4], dither: [i32; 3]) -> [u8; 4] {
        for component in 0..3 {
            let value = i32::from(color[component]);
            let rounded = if value > 247 { 255 } else { (value & 0xf8) + 8 };
            if dither[component] < (value & 7) {
                color[component] = rounded as u8;
            }
        }
        color
    }

    fn rdp_process_pixel(
        &mut self,
        x: usize,
        y: usize,
        texels: [[u8; 4]; 2],
        shade: [u8; 4],
        legacy: [u8; 4],
        lod_frac: i16,
    ) -> Option<[u8; 4]> {
        let prepared = self.rdp_prepare_pixel(
            x,
            y,
            RdpPixelInputs {
                texels,
                shade,
                legacy,
                lod_frac,
            },
            8,
        );
        let memory_coverage = self.rdp_framebuffer_coverage_read(x, y);
        let state = RdpPixelState {
            depth: self.rdp_blend_state_without_depth(prepared.coverage_count, memory_coverage),
            memory_coverage,
        };
        self.rdp_finish_prepared_pixel(x, y, prepared, state)
    }

    fn rdp_prepare_pixel(
        &mut self,
        x: usize,
        y: usize,
        input: RdpPixelInputs,
        coverage_count: i32,
    ) -> RdpPreparedPixel {
        let RdpPixelInputs {
            texels,
            shade,
            legacy,
            lod_frac,
        } = input;
        let [texel0, texel1] = texels;
        let cycle_type = self.rdp_cycle_type();
        let combiner_uses_noise = self.rdp_combiner_uses_noise(cycle_type);
        let dither = if cycle_type < 2 {
            self.rdp_dither_signals(x, y, combiner_uses_noise)
        } else {
            RdpDitherSignals {
                rgb: [7; 3],
                alpha: 0,
                noise: 0x20,
            }
        };

        let mut key_source = texel0;
        let mut rgba = if self.combine_mode == 0 || cycle_type >= 2 {
            legacy
        } else {
            let first =
                self.rdp_combine_cycle(0, [texel0, texel1], shade, [0; 4], dither.noise, lod_frac);
            if self.rdp_key_enabled() {
                let cycle = usize::from(cycle_type == 1);
                let selector = if cycle == 0 {
                    ((self.combine_mode >> 52) & 0x0f) as u8
                } else {
                    ((self.combine_mode >> 37) & 0x0f) as u8
                };
                let combined = if cycle == 0 { [0; 4] } else { first };
                let (cycle_texel0, cycle_texel1) = if cycle == 0 {
                    (texel0, texel1)
                } else {
                    (texel1, texel0)
                };
                let sources = [combined, cycle_texel0, cycle_texel1, shade];
                for (component, channel) in key_source.iter_mut().take(3).enumerate() {
                    *channel = self
                        .rdp_rgb_mux(selector, 0, component, &sources, dither.noise, lod_frac)
                        .clamp(0, 255) as u8;
                }
            }
            if cycle_type == 1 {
                self.rdp_combine_cycle(1, [texel1, texel0], shade, first, dither.noise, lod_frac)
            } else {
                first
            }
        };

        let combined_alpha = rgba[3];
        let key_alpha =
            (self.rdp_key_enabled() && self.combine_mode != 0 && cycle_type < 2).then(|| {
                rgba[..3].copy_from_slice(&key_source[..3]);
                self.rdp_key_alpha(key_source)
            });

        let modes = self.other_modes as u32;
        let mut coverage_count = coverage_count.clamp(0, 8);
        let alpha_internal = if combined_alpha == 0xff {
            0x100
        } else {
            i32::from(combined_alpha)
        };
        let coverage_alpha = if modes & 0x1000 != 0 {
            let value = (alpha_internal * coverage_count + 4) >> 3;
            coverage_count = (value >> 5) & 0x0f;
            Some(value)
        } else {
            None
        };

        if modes & 0x2000 != 0 {
            let alpha = coverage_alpha.unwrap_or(coverage_count << 5);
            rgba[3] = alpha.min(0xff) as u8;
        } else if let Some(alpha) = key_alpha {
            rgba[3] = alpha;
        } else {
            let alpha = alpha_internal + dither.alpha;
            rgba[3] = if alpha & 0x100 != 0 {
                0xff
            } else {
                alpha.clamp(0, 0xff) as u8
            };
        }

        let mut blender_shade = shade;
        let shade_alpha = i32::from(shade[3]) + dither.alpha;
        blender_shade[3] = if shade_alpha & 0x100 != 0 {
            0xff
        } else {
            shade_alpha.clamp(0, 0xff) as u8
        };

        RdpPreparedPixel {
            color: rgba,
            shade: blender_shade,
            rgb_dither: dither.rgb,
            coverage_count,
        }
    }

    fn rdp_finish_prepared_pixel(
        &mut self,
        x: usize,
        y: usize,
        prepared: RdpPreparedPixel,
        state: RdpPixelState,
    ) -> Option<[u8; 4]> {
        let modes = self.other_modes as u32;
        let cycle_type = self.rdp_cycle_type();
        if modes & 0x0008 != 0 && state.depth.coverage_count <= 0 {
            return None;
        }

        if modes & 1 != 0 {
            if cycle_type == 2 && self.color_size == 2 {
                if prepared.color[3] == 0 {
                    return None;
                }
            } else {
                let threshold = if modes & 2 != 0 {
                    (self.rdp_next_random() & 0xff) as u8
                } else {
                    Self::rdp_register_color(self.blend_color)[3]
                };
                if prepared.color[3] < threshold {
                    return None;
                }
            }
        }

        let mut rgba = self.rdp_blend(x, y, prepared.color, prepared.shade, state);
        if cycle_type < 2 {
            rgba = Self::rdp_apply_rgb_dither(rgba, prepared.rgb_dither);
        }
        Some(rgba)
    }

    fn write_rdp_rgba(&mut self, x: usize, y: usize, rgba: [u8; 4]) {
        if x >= self.color_width.max(1) as usize {
            return;
        }
        let pixel = y
            .saturating_mul(self.color_width.max(1) as usize)
            .saturating_add(x);
        match self.color_size {
            0 => {
                let address = self.color_image as usize + pixel;
                if let Some(target) = self.rdram.get_mut(address) {
                    *target = 0;
                }
            }
            1 => {
                let address = self.color_image as usize + pixel;
                if let Some(target) = self.rdram.get_mut(address) {
                    *target = if address & 1 == 0 { rgba[0] } else { rgba[1] };
                }
            }
            2 => {
                let address = self.color_image as usize + pixel * 2;
                if address + 2 > self.rdram.len() {
                    return;
                }
                let color = if self.color_format == 0 {
                    (u16::from(rgba[0] >> 3) << 11)
                        | (u16::from(rgba[1] >> 3) << 6)
                        | (u16::from(rgba[2] >> 3) << 1)
                        | u16::from(rgba[3] >= 128)
                } else {
                    (u16::from(rgba[0]) << 8) | (7 << 5)
                };
                self.rdram[address..address + 2].copy_from_slice(&color.to_be_bytes());
                if self.color_format != 0 {
                    self.rdp_hidden_write(address, 0);
                }
            }
            3 => {
                let address = self.color_image as usize + pixel * 4;
                if address + 4 > self.rdram.len() {
                    return;
                }
                self.rdram[address..address + 4].copy_from_slice(&rgba);
            }
            _ => {}
        }
    }

    fn write_rdp_coverage_pixel(&mut self, x: usize, y: usize, rgba: [u8; 4], coverage: u8) {
        let width = self.color_width.max(1) as usize;
        if x >= width {
            return;
        }
        let Some(pixel) = y.checked_mul(width).and_then(|value| value.checked_add(x)) else {
            return;
        };
        let coverage = coverage & 7;
        match self.color_size {
            0 => {
                let address = self.color_image as usize + pixel;
                if let Some(target) = self.rdram.get_mut(address) {
                    *target = 0;
                }
            }
            1 => {
                let address = self.color_image as usize + pixel;
                if let Some(target) = self.rdram.get_mut(address) {
                    *target = if address & 1 == 0 { rgba[0] } else { rgba[1] };
                }
            }
            2 => {
                let Some(address) =
                    (self.color_image as usize).checked_add(pixel.saturating_mul(2))
                else {
                    return;
                };
                if address + 2 > self.rdram.len() {
                    return;
                }
                let color = if self.color_format == 0 {
                    (u16::from(rgba[0] >> 3) << 11)
                        | (u16::from(rgba[1] >> 3) << 6)
                        | (u16::from(rgba[2] >> 3) << 1)
                        | u16::from(coverage >> 2)
                } else {
                    (u16::from(rgba[0]) << 8) | (u16::from(coverage) << 5)
                };
                self.rdram[address..address + 2].copy_from_slice(&color.to_be_bytes());
                self.rdp_hidden_write(
                    address,
                    if self.color_format == 0 {
                        coverage & 3
                    } else {
                        0
                    },
                );
            }
            3 => {
                let Some(address) =
                    (self.color_image as usize).checked_add(pixel.saturating_mul(4))
                else {
                    return;
                };
                if address + 4 > self.rdram.len() {
                    return;
                }
                self.rdram[address..address + 4].copy_from_slice(&[
                    rgba[0],
                    rgba[1],
                    rgba[2],
                    coverage << 5,
                ]);
                self.rdp_hidden_write(address, if rgba[1] & 1 != 0 { 3 } else { 0 });
                self.rdp_hidden_write(address + 2, 0);
            }
            _ => {}
        }
    }

    fn rdp_texture_rectangle(&mut self, high: u32, low: u32, high2: u32, low2: u32, flip: bool) {
        let cycle_type = self.rdp_cycle_type();
        if cycle_type == 2 && self.color_size == 3 {
            self.rdp_pipeline_crashed = true;
            return;
        }
        if cycle_type == 3 {
            self.rdp_fill_rectangle(high, low);
            return;
        }

        let start_x = ((low >> 12) & 0x0fff) as i32;
        let start_y = (low & 0x0fff) as i32;
        let mut end_x = ((high >> 12) & 0x0fff) as i32;
        let mut end_y = (high & 0x0fff) as i32;
        if matches!(self.rdp_cycle_type(), 2 | 3) {
            end_y |= 3;
        }
        if end_x < start_x || end_y < start_y {
            return;
        }

        let tile = ((low >> 24) & 7) as usize;
        let s = i32::from((high2 >> 16) as i16);
        let t = i32::from(high2 as i16);
        let mut dsdx = i32::from((low2 >> 16) as i16);
        let dtdy = i32::from(low2 as i16);
        if cycle_type == 2 {
            dsdx >>= 2;
        }

        let x0 = (start_x + 3) >> 2;
        let y0 = (start_y + 3) >> 2;
        end_x >>= 2;
        end_y >>= 2;
        let clip_x0 = x0.max(i32::from(self.scissor[0])).max(0);
        let clip_y0 = y0.max(i32::from(self.scissor[1])).max(0);
        let clip_x1 = end_x.min(i32::from(self.scissor[2].saturating_sub(1)));
        let clip_y1 = end_y.min(i32::from(self.scissor[3].saturating_sub(1)));
        if clip_x0 > clip_x1 || clip_y0 > clip_y1 {
            return;
        }

        for y in clip_y0..=clip_y1 {
            if !self.rdp_scissor_accepts_line(y as usize) {
                continue;
            }
            for x in clip_x0..=clip_x1 {
                let coverage_count = if cycle_type < 2 {
                    let Some(coverage) = self.rdp_rectangle_coverage(
                        (low >> 12) & 0x0fff,
                        low & 0x0fff,
                        (high >> 12) & 0x0fff,
                        high & 0x0fff,
                        x as usize,
                        y as usize,
                    ) else {
                        continue;
                    };
                    coverage
                } else {
                    8
                };
                let dx = x - x0;
                let dy = y - y0;
                let (sample_s, sample_t) = if flip {
                    (s + ((dy * dsdx) >> 5), t + ((dx * dtdy) >> 5))
                } else {
                    (s + ((dx * dsdx) >> 5), t + ((dy * dtdy) >> 5))
                };
                if cycle_type == 2 && self.color_size == 2 {
                    if let Some(word) = self.rdp_copy_u16_texel(tile, sample_s >> 5, sample_t >> 5)
                    {
                        if self.other_modes as u32 & 1 != 0 && word & 1 == 0 {
                            continue;
                        }
                        let pixel = (y as usize)
                            .saturating_mul(self.color_width.max(1) as usize)
                            .saturating_add(x as usize);
                        let address =
                            (self.color_image as usize).saturating_add(pixel.saturating_mul(2));
                        if address + 2 <= self.rdram.len() {
                            self.rdram[address..address + 2].copy_from_slice(&word.to_be_bytes());
                            self.rdp_hidden_write(address, if word & 1 != 0 { 3 } else { 0 });
                        }
                        continue;
                    }
                }

                let texel0 = if cycle_type == 2 {
                    self.sample_rdp_texture(tile, sample_s >> 5, sample_t >> 5)
                } else {
                    self.sample_rdp_texture_fixed(tile, sample_s, sample_t)
                };
                let Some(texel0) = texel0 else {
                    continue;
                };
                let texel1 = if cycle_type == 1 {
                    self.sample_rdp_texture_fixed((tile + 1) & 7, sample_s, sample_t)
                        .unwrap_or([0; 4])
                } else {
                    texel0
                };
                let shade = [255; 4];
                if cycle_type < 2 {
                    let px = x as usize;
                    let py = y as usize;
                    let prepared = self.rdp_prepare_pixel(
                        px,
                        py,
                        RdpPixelInputs {
                            texels: [texel0, texel1],
                            shade,
                            legacy: texel0,
                            lod_frac: 0,
                        },
                        coverage_count,
                    );
                    let memory_coverage = self.rdp_framebuffer_coverage_read(px, py);
                    let depth = self
                        .rdp_blend_state_without_depth(prepared.coverage_count, memory_coverage);
                    let Some(rgba) = self.rdp_finish_prepared_pixel(
                        px,
                        py,
                        prepared,
                        RdpPixelState {
                            depth,
                            memory_coverage,
                        },
                    ) else {
                        continue;
                    };
                    let final_coverage = self.rdp_finalize_coverage(
                        depth.blend_en,
                        depth.coverage_count,
                        memory_coverage,
                    );
                    self.write_rdp_coverage_pixel(px, py, rgba, final_coverage);
                } else {
                    let Some(rgba) = self.rdp_process_pixel(
                        x as usize,
                        y as usize,
                        [texel0, texel1],
                        shade,
                        texel0,
                        0,
                    ) else {
                        continue;
                    };
                    self.write_rdp_rgba(x as usize, y as usize, rgba);
                }
            }
        }
    }

    fn run_rdp(&mut self) {
        if self.dp.status & DP_STATUS_FREEZE != 0 {
            return;
        }
        let end = self.dp.end & 0x007f_fff8;
        if self.rdp_pipeline_crashed {
            self.dp.current = end;
            self.dp.status &= !DP_STATUS_CMD_BUSY;
            return;
        }
        self.dp.status |= DP_STATUS_CMD_BUSY;
        let mut cursor = self.dp.current & 0x007f_fff8;
        let mut commands = 0usize;
        while cursor < end && commands < 4096 {
            let Some((high, low)) = self.read_rdp_word(cursor) else {
                break;
            };
            let opcode = ((high >> 24) & 0x3f) as u8;
            let length = Self::rdp_command_length(opcode);
            if cursor.saturating_add(length) > end {
                break;
            }

            match opcode {
                0x08 => self.rdp_fill_triangle(cursor, high, low),
                0x09 => self.rdp_fill_triangle_z(cursor, high, low),
                0x0a => self.rdp_texture_triangle(cursor, high, low, 32, None, false),
                0x0b => self.rdp_texture_triangle(cursor, high, low, 32, Some(96), false),
                0x0c => self.rdp_shade_triangle(cursor, high, low, None),
                0x0d => self.rdp_shade_triangle(cursor, high, low, Some(96)),
                0x0e => self.rdp_texture_triangle(cursor, high, low, 96, None, true),
                0x0f => self.rdp_texture_triangle(cursor, high, low, 96, Some(160), true),
                0x24 | 0x25 => {
                    if let Some((high2, low2)) = self.read_rdp_word(cursor + 8) {
                        self.rdp_texture_rectangle(high, low, high2, low2, opcode == 0x25);
                    }
                }
                0x26..=0x28 => {}
                0x29 => self.raise(MI_DP),
                0x2a => self.set_rdp_key_gb(high, low),
                0x2b => self.set_rdp_key_r(low),
                0x2c => self.set_rdp_convert(high, low),
                0x2d => self.set_rdp_scissor(high, low),
                0x2e => {
                    self.primitive_depth = ((low >> 16) & 0x7fff) as u16;
                    self.primitive_delta_z = (low & 0xffff) as u16;
                }
                0x2f => {
                    self.other_modes = (u64::from(high) << 32) | u64::from(low);
                }
                0x30 => self.load_rdp_tlut(high, low),
                0x32 => self.set_rdp_tile_size(high, low),
                0x33 => self.load_rdp_tile(high, low, true),
                0x34 => self.load_rdp_tile(high, low, false),
                0x35 => self.set_rdp_tile(high, low),
                0x36 => self.rdp_fill_rectangle(high, low),
                0x37 => self.fill_color = low,
                0x38 => self.fog_color = low,
                0x39 => self.blend_color = low,
                0x3a => {
                    self.primitive_color = low;
                    self.primitive_lod_frac = (high & 0xff) as u8;
                    self.primitive_min_level = ((high >> 8) & 0x1f) as u8;
                }
                0x3b => self.environment_color = low,
                0x3c => {
                    self.combine_mode = (u64::from(high) << 32) | u64::from(low);
                }
                0x3d => self.set_rdp_texture_image(high, low),
                0x3e => self.depth_image = low & 0x00ff_ffff,
                0x3f => self.set_rdp_color_image(high, low),
                _ => {}
            }

            cursor = cursor.wrapping_add(length);
            commands += 1;
            if self.rdp_pipeline_crashed {
                cursor = end;
                break;
            }
        }
        self.dp.current = cursor;
        self.dp.clock = self.dp.clock.wrapping_add(commands as u32);
        self.dp.status &= !DP_STATUS_CMD_BUSY;
    }

    fn sign_extend(value: u32, bits: u32) -> i32 {
        let shift = 32 - bits;
        ((value << shift) as i32) >> shift
    }

    fn rdp_quantize_coverage_x(value: i64) -> i32 {
        let sticky = i32::from(value & 0x1fff != 0);
        ((value >> 13) as i32) | sticky
    }

    fn rdp_coverage_mask(coverage_left: [i32; 4], coverage_right: [i32; 4], x: i32) -> u8 {
        let base = x << 3;
        let sample_x = [base, base + 4, base + 2, base + 6];
        let low_y = [0usize, 0, 1, 1];
        let high_y = [2usize, 2, 3, 3];
        let mut clipped = 0u8;
        for lane in 0..4 {
            let low = low_y[lane];
            if sample_x[lane] < coverage_left[low] || sample_x[lane] >= coverage_right[low] {
                clipped |= 1 << lane;
            }
            let high = high_y[lane];
            if sample_x[lane] < coverage_left[high] || sample_x[lane] >= coverage_right[high] {
                clipped |= 1 << (lane + 4);
            }
        }
        !clipped
    }

    fn rdp_pixel_coverage_mask(&self, span: RdpTriangleSpan, x: usize) -> Option<u8> {
        if !span.subpixel {
            return Some(0xff);
        }
        let x = i32::try_from(x).ok()?;
        let mask = Self::rdp_coverage_mask(span.coverage_left, span.coverage_right, x);
        if mask == 0 {
            return None;
        }
        let aa_enable = self.other_modes as u32 & 0x0008 != 0;
        if !aa_enable && mask & 1 == 0 {
            return None;
        }
        Some(mask)
    }

    fn rdp_pixel_coverage(&self, span: RdpTriangleSpan, x: usize) -> Option<i32> {
        self.rdp_pixel_coverage_mask(span, x)
            .map(|mask| mask.count_ones() as i32)
    }

    fn rdp_coverage_offsets(mask: u8) -> [i32; 2] {
        const Y_OFFSET: [u8; 16] = [0, 0, 1, 0, 2, 0, 1, 0, 3, 0, 1, 0, 2, 0, 1, 0];
        const X_OFFSET: [u8; 16] = [0, 3, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0];

        // Angrylion stores the coverage byte in the opposite bit order from
        // rdp_coverage_mask. Reverse it before expanding the four sample rows.
        let mask = mask.reverse_bits();
        let expanded =
            u16::from(mask & 0x05) | (u16::from(mask & 0x5a) << 4) | (u16::from(mask & 0xa0) << 8);

        let mut rows = 0usize;
        for row in 0..4usize {
            if expanded & (0xf000u16 >> (row * 4)) != 0 {
                rows |= 1 << row;
            }
        }
        let off_y = usize::from(Y_OFFSET[rows]);
        let row_mask = ((expanded & (0xf000u16 >> (off_y * 4))) >> ((off_y ^ 3) * 4)) as usize;
        [i32::from(X_OFFSET[row_mask]), off_y as i32]
    }

    fn rdp_correct_shade_component(value: i64, dx: i64, dy: i64, mask: u8) -> u8 {
        let base = (value >> 14) as i32;
        let corrected = if mask.count_ones() == 8 {
            base >> 2
        } else {
            let [off_x, off_y] = Self::rdp_coverage_offsets(mask);
            let dx = Self::sign_extend((((dx & !0x1f) >> 14) as u32) & 0x1fff, 13);
            let dy = Self::sign_extend(((dy >> 14) as u32) & 0x1fff, 13);
            ((base << 2) + off_x * dx + off_y * dy) >> 4
        };
        Self::rdp_clamp_9bit(corrected)
    }

    fn rdp_rectangle_coverage(
        &self,
        start_x: u32,
        start_y: u32,
        end_x: u32,
        end_y: u32,
        x: usize,
        y: usize,
    ) -> Option<i32> {
        let left = start_x.max(u32::from(self.scissor_fixed[0]));
        let right = end_x.min(u32::from(self.scissor_fixed[2]));
        let top = start_y.max(u32::from(self.scissor_fixed[1]));
        let bottom = end_y.min(u32::from(self.scissor_fixed[3]));
        if right <= left || bottom <= top {
            return None;
        }

        let mut coverage_left = [i32::MAX; 4];
        let mut coverage_right = [i32::MIN; 4];
        for sub in 0..4usize {
            let sample_y = u32::try_from(y.checked_mul(4)?.checked_add(sub)?).ok()?;
            if sample_y >= top && sample_y < bottom {
                coverage_left[sub] = i32::try_from(left.saturating_mul(2)).ok()?;
                coverage_right[sub] = i32::try_from(right.saturating_mul(2)).ok()?;
            }
        }

        self.rdp_pixel_coverage(
            RdpTriangleSpan {
                y,
                x0: x,
                x1: x,
                major_x: 0,
                coverage_left,
                coverage_right,
                subpixel: true,
            },
            x,
        )
    }

    fn rdp_triangle_spans(&self, cursor: u32, high: u32, low: u32) -> Vec<RdpTriangleSpan> {
        let Some((word2, word3)) = self.read_rdp_word(cursor.wrapping_add(8)) else {
            return Vec::new();
        };
        let Some((word4, word5)) = self.read_rdp_word(cursor.wrapping_add(16)) else {
            return Vec::new();
        };
        let Some((word6, word7)) = self.read_rdp_word(cursor.wrapping_add(24)) else {
            return Vec::new();
        };

        let flip = high & 0x0080_0000 != 0;
        let yl = Self::sign_extend(high & 0x3fff, 14);
        let mut ym = Self::sign_extend((low >> 16) & 0x3fff, 14);
        let yh = Self::sign_extend(low & 0x3fff, 14);
        if yl <= yh {
            return Vec::new();
        }
        ym = ym.clamp(yh, yl);

        let xl = Self::sign_extend(word2 & 0x0fff_ffff, 28);
        let dxldy = Self::sign_extend(word3 & 0x3fff_ffff, 30) >> 2;
        let xh = Self::sign_extend(word4 & 0x0fff_ffff, 28);
        let dxhdy = Self::sign_extend(word5 & 0x3fff_ffff, 30) >> 2;
        let xm = Self::sign_extend(word6 & 0x0fff_ffff, 28);
        let dxmdy = Self::sign_extend(word7 & 0x3fff_ffff, 30) >> 2;

        let width = self.color_width.max(1) as i32;
        let scissor_top = i32::from(self.scissor_fixed[1]);
        let scissor_bottom = i32::from(self.scissor_fixed[3]);
        let clipped_top = yh.max(scissor_top);
        let clipped_bottom = yl.min(scissor_bottom);
        if clipped_bottom <= clipped_top {
            return Vec::new();
        }
        let start_line = (clipped_top >> 2).max(0);
        let end_line = ((clipped_bottom - 1) >> 2).max(0);
        if start_line > end_line {
            return Vec::new();
        }
        let yh_base = yh & !3;
        let subpixel = self.rdp_cycle_type() < 2;
        let scissor_left = i32::from(self.scissor_fixed[0]) << 1;
        let scissor_right = i32::from(self.scissor_fixed[2]) << 1;
        let mut spans = Vec::with_capacity((end_line - start_line + 1) as usize);
        for line in start_line..=end_line {
            if !self.rdp_scissor_accepts_line(line as usize) {
                continue;
            }
            let mut span_left = i32::MAX;
            let mut span_right = i32::MIN;
            let mut coverage_left = [i32::MAX; 4];
            let mut coverage_right = [i32::MIN; 4];
            for sub in 0..4 {
                let y = line * 4 + sub;
                if y < yh || y >= yl || y < scissor_top || y >= scissor_bottom {
                    continue;
                }
                let major = i64::from(xh) + i64::from(y - yh_base) * i64::from(dxhdy);
                let minor = if y < ym {
                    i64::from(xm) + i64::from(y - yh_base) * i64::from(dxmdy)
                } else {
                    i64::from(xl) + i64::from(y - ym) * i64::from(dxldy)
                };
                let major_x = (major >> 16) as i32;
                let minor_x = (minor >> 16) as i32;
                let (left, right, raw_left, raw_right) = if flip {
                    (major_x, minor_x, major, minor)
                } else {
                    (minor_x, major_x, minor, major)
                };
                if left > right {
                    continue;
                }
                span_left = span_left.min(left);
                span_right = span_right.max(right);

                let quantized_left =
                    Self::rdp_quantize_coverage_x(raw_left).clamp(scissor_left, scissor_right);
                let quantized_right =
                    Self::rdp_quantize_coverage_x(raw_right).clamp(scissor_left, scissor_right);
                if (quantized_left >> 1) <= (quantized_right >> 1) {
                    coverage_left[sub as usize] = quantized_left;
                    coverage_right[sub as usize] = quantized_right;
                }
            }
            if span_left > span_right {
                continue;
            }

            let (x0, x1) = if subpixel {
                let left = coverage_left.into_iter().min().unwrap_or(i32::MAX);
                let right = coverage_right.into_iter().max().unwrap_or(i32::MIN);
                ((left >> 3).max(0), (right >> 3).min(width - 1))
            } else {
                (
                    span_left.max(i32::from(self.scissor[0])).max(0),
                    span_right
                        .min(i32::from(self.scissor[2].saturating_sub(1)))
                        .min(width - 1),
                )
            };
            if x0 > x1 {
                continue;
            }
            let major_x = i64::from(xh) + i64::from(line * 4 - yh_base) * i64::from(dxhdy);
            spans.push(RdpTriangleSpan {
                y: line as usize,
                x0: x0 as usize,
                x1: x1 as usize,
                major_x,
                coverage_left,
                coverage_right,
                subpixel,
            });
        }
        spans
    }

    fn write_rdp_fill_pixel(&mut self, x: usize, y: usize) {
        let width = self.color_width.max(1) as usize;
        if x >= width {
            return;
        }
        let pixel = y.saturating_mul(width).saturating_add(x);
        match self.color_size {
            0 => {
                self.rdp_pipeline_crashed = true;
            }
            1 => {
                let address = self.color_image as usize + pixel;
                let shift = (((address & 3) ^ 3) * 8) as u32;
                if let Some(target) = self.rdram.get_mut(address) {
                    *target = (self.fill_color >> shift) as u8;
                }
            }
            2 => {
                let address = self.color_image as usize + pixel * 2;
                if address + 2 <= self.rdram.len() {
                    let color = if pixel & 1 == 0 {
                        (self.fill_color >> 16) as u16
                    } else {
                        self.fill_color as u16
                    };
                    self.rdram[address..address + 2].copy_from_slice(&color.to_be_bytes());
                    self.rdp_hidden_write(address, if color & 1 != 0 { 3 } else { 0 });
                }
            }
            3 => {
                let address = self.color_image as usize + pixel * 4;
                if address + 4 <= self.rdram.len() {
                    self.rdram[address..address + 4]
                        .copy_from_slice(&self.fill_color.to_be_bytes());
                    self.rdp_hidden_write(
                        address,
                        if self.fill_color & 0x0001_0000 != 0 {
                            3
                        } else {
                            0
                        },
                    );
                    self.rdp_hidden_write(
                        address + 2,
                        if self.fill_color & 1 != 0 { 3 } else { 0 },
                    );
                }
            }
            _ => {}
        }
    }

    fn rdp_fill_fast_crash(&self) -> bool {
        self.other_modes as u32 & 0x0050 != 0
    }

    fn rdp_fill_slow_crash(&self) -> bool {
        let modes = self.other_modes as u32;
        modes & 0x0050 == 0 && modes & 0x0020 != 0 && modes & 0x0004 == 0
    }

    fn rdp_zero_depth_sample(&self) -> RdpDepthSample {
        if self.other_modes as u32 & 0x0004 != 0 {
            let delta_z = i32::from(self.primitive_delta_z);
            RdpDepthSample {
                depth: i32::from(self.primitive_depth) << 3,
                delta_z,
                compressed_delta_z: Self::rdp_dz_compress(delta_z),
            }
        } else {
            RdpDepthSample {
                depth: 0,
                delta_z: 1,
                compressed_delta_z: 0,
            }
        }
    }

    fn rdp_render_unshaded_pixel(
        &mut self,
        x: usize,
        y: usize,
        coverage_count: i32,
        depth_sample: Option<RdpDepthSample>,
    ) {
        let prepared = self.rdp_prepare_pixel(
            x,
            y,
            RdpPixelInputs {
                texels: [[0; 4]; 2],
                shade: [0; 4],
                legacy: [0; 4],
                lod_frac: 0,
            },
            coverage_count,
        );
        let memory_coverage = self.rdp_framebuffer_coverage_read(x, y);
        let modes = self.other_modes as u32;
        let mut depth_write = None;
        let depth = if depth_sample.is_some() || modes & 0x0030 != 0 {
            let sample = depth_sample.unwrap_or_else(|| self.rdp_zero_depth_sample());
            let result =
                self.rdp_depth_test(x, y, sample, prepared.coverage_count, memory_coverage);
            if !result.depth_pass {
                return;
            }
            depth_write = Some(sample);
            result
        } else {
            self.rdp_blend_state_without_depth(prepared.coverage_count, memory_coverage)
        };

        let Some(rgba) = self.rdp_finish_prepared_pixel(
            x,
            y,
            prepared,
            RdpPixelState {
                depth,
                memory_coverage,
            },
        ) else {
            return;
        };
        let final_coverage =
            self.rdp_finalize_coverage(depth.blend_en, depth.coverage_count, memory_coverage);
        self.write_rdp_coverage_pixel(x, y, rgba, final_coverage);

        if modes & 0x0020 != 0 {
            if let Some(sample) = depth_write {
                self.rdp_depth_write(x, y, sample.depth, sample.compressed_delta_z as u8);
            }
        }
    }

    fn rdp_fill_triangle(&mut self, cursor: u32, high: u32, low: u32) {
        let cycle_type = self.rdp_cycle_type();
        if cycle_type == 3 && self.color_size == 0 {
            self.rdp_pipeline_crashed = true;
            return;
        }
        if cycle_type == 2 && self.color_size == 3 {
            self.rdp_pipeline_crashed = true;
            return;
        }
        let flip = high & 0x0080_0000 != 0;

        for span in self.rdp_triangle_spans(cursor, high, low) {
            if cycle_type == 2 {
                match self.color_size {
                    0 => {
                        for x in span.x0..=span.x1 {
                            let pixel = span
                                .y
                                .saturating_mul(self.color_width.max(1) as usize)
                                .saturating_add(x);
                            if let Some(target) =
                                self.rdram.get_mut(self.color_image as usize + pixel)
                            {
                                *target = 0;
                            }
                        }
                        continue;
                    }
                    2 => {
                        if !self.rdp_copy_u16_span(span.y, span.x0, span.x1, flip) {
                            return;
                        }
                        continue;
                    }
                    _ => {}
                }
            }

            if cycle_type == 3 {
                if self.rdp_fill_fast_crash() {
                    self.rdp_pipeline_crashed = true;
                    return;
                }
                for x in span.x0..=span.x1 {
                    self.write_rdp_fill_pixel(x, span.y);
                }
                if self.rdp_fill_slow_crash() {
                    self.rdp_pipeline_crashed = true;
                    return;
                }
                continue;
            }

            if cycle_type < 2 {
                for x in span.x0..=span.x1 {
                    let Some(mask) = self.rdp_pixel_coverage_mask(span, x) else {
                        continue;
                    };
                    self.rdp_render_unshaded_pixel(x, span.y, mask.count_ones() as i32, None);
                }
                continue;
            }

            for x in span.x0..=span.x1 {
                self.write_rdp_fill_pixel(x, span.y);
            }
        }
    }

    fn rdp_hidden_read(&self, address: usize) -> u8 {
        let halfword = address >> 1;
        let byte = halfword >> 2;
        let shift = (halfword & 3) * 2;
        self.rdram_hidden
            .get(byte)
            .map_or(0, |value| (value >> shift) & 3)
    }

    fn rdp_hidden_write(&mut self, address: usize, value: u8) {
        let halfword = address >> 1;
        let byte = halfword >> 2;
        let shift = (halfword & 3) * 2;
        if let Some(target) = self.rdram_hidden.get_mut(byte) {
            let mask = 3u8 << shift;
            *target = (*target & !mask) | ((value & 3) << shift);
        }
    }

    fn rdp_z_decompress(value: u16) -> i32 {
        let value = i32::from(value & 0x3fff);
        let exponent = value >> 11;
        let mantissa = value & 0x7ff;
        let shift = (6 - exponent).max(0);
        let base = 0x4_0000 - (0x4_0000 >> exponent);
        (mantissa << shift) + base
    }

    fn rdp_z_compress(value: i32) -> u16 {
        let value = value.clamp(0, 0x3_ffff);
        let inverse = (0x3_ffff - value).max(1);
        let exponent = (17 - Self::rdp_find_msb(inverse)).clamp(0, 7);
        let shift = (6 - exponent).max(0);
        let mantissa = (value >> shift) & 0x7ff;
        ((exponent << 11) + mantissa) as u16
    }

    fn rdp_dz_decompress(value: i32) -> i32 {
        1 << value.clamp(0, 15)
    }

    fn rdp_dz_compress(value: i32) -> i32 {
        Self::rdp_find_msb(value).clamp(0, 15)
    }

    fn rdp_combine_dz(value: i32) -> i32 {
        let msb = Self::rdp_find_msb(value);
        if msb >= 0 {
            1 << msb
        } else {
            0
        }
    }

    fn rdp_depth_decision(
        depth: i32,
        delta_z: i32,
        compressed_delta_z: i32,
        mut coverage_count: i32,
        input: RdpDepthInputs,
    ) -> RdpDepthResult {
        let depth = depth.clamp(0, 0x3_ffff);
        let delta_z = delta_z.clamp(0, 0x3_ffff);
        let compressed_delta_z = compressed_delta_z.clamp(0, 15);
        let current_depth = input.current_depth & 0x3fff;
        let current_delta_z = i32::from(input.current_dz & 15);
        let mut blend_shift = [0u8; 2];

        let (depth_pass, blend_en, coverage_wrap) = if input.z_compare {
            let memory_depth = Self::rdp_z_decompress(current_depth);
            let mut memory_delta_z = Self::rdp_dz_decompress(current_delta_z);
            let precision_factor = (i32::from(current_depth) >> 11) & 15;
            let mut coplanar = false;

            blend_shift[0] = (compressed_delta_z - current_delta_z).clamp(0, 4) as u8;
            blend_shift[1] = (current_delta_z - compressed_delta_z).clamp(0, 4) as u8;

            if precision_factor < 3 {
                if memory_delta_z == 0x8000 {
                    coplanar = true;
                    memory_delta_z = 0xffff;
                } else {
                    memory_delta_z = (memory_delta_z << 1).max(16 >> precision_factor);
                }
            }

            let mut combined_delta_z = Self::rdp_combine_dz(delta_z | memory_delta_z);
            let interpenetrating_delta_z = combined_delta_z;
            combined_delta_z <<= 3;

            let farther = coplanar || depth + combined_delta_z >= memory_depth;
            let overflow = coverage_count + input.current_coverage >= 8;
            let blend_en = input.force_blend || (!overflow && input.aa_enable && farther);
            let max_depth = memory_depth == 0x3_ffff;
            let front = depth < memory_depth;
            let nearer = coplanar || depth - combined_delta_z <= memory_depth;
            let opaque_pass = max_depth || if overflow { front } else { nearer };

            let depth_pass = match input.z_mode {
                1 if front && farther && overflow => {
                    let shift = Self::rdp_dz_compress(interpenetrating_delta_z & 0xffff);
                    let coefficient = ((memory_depth >> shift) - (depth >> shift)) & 15;
                    coverage_count = ((coefficient * coverage_count) >> 3).min(8);
                    true
                }
                0 | 1 => opaque_pass,
                2 => front || max_depth,
                _ => farther && nearer && !max_depth,
            };
            (depth_pass, blend_en, overflow)
        } else {
            blend_shift[1] = (15 - compressed_delta_z).min(4) as u8;
            let overflow = coverage_count + input.current_coverage >= 8;
            let blend_en = input.force_blend || (!overflow && input.aa_enable);
            (true, blend_en, overflow)
        };

        RdpDepthResult {
            depth_pass,
            blend_en,
            coverage_wrap,
            blend_shift,
            coverage_count,
        }
    }

    fn rdp_depth_read(&self, x: usize, y: usize) -> Option<(u16, u8)> {
        let pixel = y
            .checked_mul(self.color_width.max(1) as usize)?
            .checked_add(x)?;
        let address = (self.depth_image as usize).checked_add(pixel.checked_mul(2)?)?;
        let bytes = self.rdram.get(address..address.checked_add(2)?)?;
        let word = u16::from_be_bytes([bytes[0], bytes[1]]);
        let hidden = self.rdp_hidden_read(address);
        Some((word >> 2, (((word & 3) as u8) << 2) | hidden))
    }

    fn rdp_depth_write(&mut self, x: usize, y: usize, depth: i32, compressed_delta_z: u8) {
        let Some(pixel) = y
            .checked_mul(self.color_width.max(1) as usize)
            .and_then(|value| value.checked_add(x))
        else {
            return;
        };
        let Some(address) = (self.depth_image as usize).checked_add(pixel.saturating_mul(2)) else {
            return;
        };
        if address + 2 > self.rdram.len() {
            return;
        }

        let compressed_delta_z = compressed_delta_z & 15;
        let word = (Self::rdp_z_compress(depth) << 2) | u16::from(compressed_delta_z >> 2);
        self.rdram[address..address + 2].copy_from_slice(&word.to_be_bytes());
        self.rdp_hidden_write(address, compressed_delta_z & 3);
    }

    fn rdp_depth_test(
        &self,
        x: usize,
        y: usize,
        sample: RdpDepthSample,
        coverage_count: i32,
        memory_coverage: u8,
    ) -> RdpDepthResult {
        let modes = self.other_modes as u32;
        let (current_depth, current_delta_z) = self.rdp_depth_read(x, y).unwrap_or((0x3fff, 0));
        Self::rdp_depth_decision(
            sample.depth,
            sample.delta_z,
            sample.compressed_delta_z,
            coverage_count,
            RdpDepthInputs {
                current_depth,
                current_dz: current_delta_z,
                current_coverage: i32::from(memory_coverage & 7),
                z_compare: modes & 0x0010 != 0,
                z_mode: ((modes >> 10) & 3) as u8,
                force_blend: modes & 0x4000 != 0,
                aa_enable: modes & 0x0008 != 0,
            },
        )
    }

    fn rdp_blend_state_without_depth(
        &self,
        coverage_count: i32,
        memory_coverage: u8,
    ) -> RdpDepthResult {
        let modes = self.other_modes as u32;
        Self::rdp_depth_decision(
            0,
            0,
            0,
            coverage_count,
            RdpDepthInputs {
                current_depth: 0,
                current_dz: 0,
                current_coverage: i32::from(memory_coverage & 7),
                z_compare: false,
                z_mode: ((modes >> 10) & 3) as u8,
                force_blend: modes & 0x4000 != 0,
                aa_enable: modes & 0x0008 != 0,
            },
        )
    }

    fn rdp_fill_triangle_z(&mut self, cursor: u32, high: u32, low: u32) {
        let cycle_type = self.rdp_cycle_type();
        if cycle_type == 3 && self.color_size == 0 {
            self.rdp_pipeline_crashed = true;
            return;
        }
        if cycle_type == 2 {
            self.rdp_fill_triangle(cursor, high, low);
            return;
        }

        let yh = Self::sign_extend(low & 0x3fff, 14);
        for span in self.rdp_triangle_spans(cursor, high, low) {
            if cycle_type == 3 {
                if self.rdp_fill_fast_crash() {
                    self.rdp_pipeline_crashed = true;
                    return;
                }
                for x in span.x0..=span.x1 {
                    self.write_rdp_fill_pixel(x, span.y);
                }
                if self.rdp_fill_slow_crash() {
                    self.rdp_pipeline_crashed = true;
                    return;
                }
                continue;
            }

            for x in span.x0..=span.x1 {
                let Some(coverage_mask) = self.rdp_pixel_coverage_mask(span, x) else {
                    continue;
                };
                let Some(sample) = self.rdp_triangle_depth(
                    cursor,
                    32,
                    RdpTrianglePixel {
                        y: span.y,
                        x,
                        major_x: span.major_x,
                        yh,
                        coverage_mask,
                    },
                ) else {
                    continue;
                };
                self.rdp_render_unshaded_pixel(
                    x,
                    span.y,
                    coverage_mask.count_ones() as i32,
                    Some(sample),
                );
            }
        }
    }

    fn rdp_fixed_component(high: u16, low: u16) -> i64 {
        (i64::from(high as i16) << 16) | i64::from(low)
    }

    fn rdp_component_group(first: u32, second: u32) -> [u16; 4] {
        [
            (first >> 16) as u16,
            first as u16,
            (second >> 16) as u16,
            second as u16,
        ]
    }

    fn rdp_attribute_coefficients(&self, cursor: u32, offset: u32) -> Option<[[i64; 4]; 4]> {
        let mut words = [0u32; 16];
        for pair in 0..8u32 {
            let (first, second) = self.read_rdp_word(cursor.wrapping_add(offset + pair * 8))?;
            words[pair as usize * 2] = first;
            words[pair as usize * 2 + 1] = second;
        }

        let base_high = Self::rdp_component_group(words[0], words[1]);
        let dx_high = Self::rdp_component_group(words[2], words[3]);
        let base_low = Self::rdp_component_group(words[4], words[5]);
        let dx_low = Self::rdp_component_group(words[6], words[7]);
        let de_high = Self::rdp_component_group(words[8], words[9]);
        let dy_high = Self::rdp_component_group(words[10], words[11]);
        let de_low = Self::rdp_component_group(words[12], words[13]);
        let dy_low = Self::rdp_component_group(words[14], words[15]);

        let mut result = [[0i64; 4]; 4];
        for component in 0..4 {
            result[0][component] =
                Self::rdp_fixed_component(base_high[component], base_low[component]);
            result[1][component] = Self::rdp_fixed_component(dx_high[component], dx_low[component]);
            result[2][component] = Self::rdp_fixed_component(de_high[component], de_low[component]);
            result[3][component] = Self::rdp_fixed_component(dy_high[component], dy_low[component]);
        }
        Some(result)
    }

    fn rdp_correct_depth(depth: i32, dzdx: i32, dzdy: i32, mask: u8) -> i32 {
        let raw = (depth >> 10) & 0x3f_ffff;
        let corrected = if mask.count_ones() == 8 {
            raw >> 3
        } else {
            let [off_x, off_y] = Self::rdp_coverage_offsets(mask);
            let dx = Self::sign_extend(((dzdx >> 10) as u32) & 0x3f_ffff, 22);
            let dy = Self::sign_extend(((dzdy >> 10) as u32) & 0x3f_ffff, 22);
            ((raw << 2) + off_x * dx + off_y * dy) >> 5
        };

        match (corrected & 0x6_0000) >> 17 {
            0 | 1 => corrected & 0x3_ffff,
            2 => 0x3_ffff,
            _ => 0,
        }
    }

    fn rdp_interpolate_depth(
        z_base: i32,
        dzdx: i32,
        dzde: i32,
        dzdy: i32,
        pixel: RdpTrianglePixel,
    ) -> i32 {
        let base_x = (pixel.major_x >> 16) as i32;
        let x_fraction = ((pixel.major_x >> 8) & 0xff) as i32;
        let y = i32::try_from(pixel.y).unwrap_or_default();
        let x = i32::try_from(pixel.x).unwrap_or_default();
        let mut depth = z_base.wrapping_add(dzde.wrapping_mul(y.wrapping_sub(pixel.yh >> 2)));
        depth = ((depth & !0x1ff).wrapping_sub(x_fraction.wrapping_mul((dzdx >> 8) & !1))) & !0x3ff;
        depth = depth.wrapping_add(dzdx.wrapping_mul(x.wrapping_sub(base_x)));
        Self::rdp_correct_depth(depth, dzdx, dzdy, pixel.coverage_mask)
    }

    fn rdp_normalize_delta_z(sum: i32) -> i32 {
        let sum = sum & 0xffff;
        if sum & 0xc000 != 0 {
            return 0x8000;
        }
        if sum == 0 {
            return 1;
        }
        if sum == 1 {
            return 3;
        }
        let mut bit = 0x2000;
        while bit > 0 {
            if sum & bit != 0 {
                return bit << 1;
            }
            bit >>= 1;
        }
        1
    }

    fn rdp_triangle_depth(
        &self,
        cursor: u32,
        offset: u32,
        pixel: RdpTrianglePixel,
    ) -> Option<RdpDepthSample> {
        if self.other_modes as u32 & 0x0004 != 0 {
            let delta_z = i32::from(self.primitive_delta_z);
            return Some(RdpDepthSample {
                depth: i32::from(self.primitive_depth) << 3,
                delta_z,
                compressed_delta_z: Self::rdp_dz_compress(delta_z),
            });
        }
        let (z_word, dzdx_word) = self.read_rdp_word(cursor.wrapping_add(offset))?;
        let (dzde_word, dzdy_word) = self.read_rdp_word(cursor.wrapping_add(offset + 8))?;
        let z_base = z_word as i32;
        let dzdx = dzdx_word as i32;
        let dzde = dzde_word as i32;
        let dzdy = dzdy_word as i32;
        let depth = Self::rdp_interpolate_depth(z_base, dzdx, dzde, dzdy, pixel);

        let magnitude = |value: i32| {
            let high = (value >> 16) & 0xffff;
            if high & 0x8000 != 0 {
                (!high) & 0x7fff
            } else {
                high
            }
        };
        let delta_z =
            Self::rdp_normalize_delta_z(magnitude(dzdx).wrapping_add(magnitude(dzdy)) & 0xffff);
        Some(RdpDepthSample {
            depth,
            delta_z,
            compressed_delta_z: Self::rdp_dz_compress(delta_z),
        })
    }

    fn rdp_shade_triangle(&mut self, cursor: u32, high: u32, low: u32, z_offset: Option<u32>) {
        let Some(coefficients) = self.rdp_attribute_coefficients(cursor, 32) else {
            return;
        };
        let yh = Self::sign_extend(low & 0x3fff, 14);
        let base_line = i64::from(yh >> 2);

        for span in self.rdp_triangle_spans(cursor, high, low) {
            let y = span.y;
            let major_x = span.major_x;
            let dy = i64::try_from(y).unwrap_or_default().wrapping_sub(base_line);
            let mut edge = [0i64; 4];
            for component in 0..4 {
                edge[component] = coefficients[0][component]
                    .wrapping_add(coefficients[2][component].wrapping_mul(dy));
            }

            for x in span.x0..=span.x1 {
                let Some(coverage_mask) = self.rdp_pixel_coverage_mask(span, x) else {
                    continue;
                };
                let coverage_count = coverage_mask.count_ones() as i32;
                let x_fixed = i64::try_from(x).unwrap_or_default() << 16;
                let dx = x_fixed.wrapping_sub(major_x);
                let mut rgba = [0u8; 4];
                for component in 0..4 {
                    let value = edge[component]
                        .wrapping_add((coefficients[1][component].wrapping_mul(dx)) >> 16);
                    rgba[component] = Self::rdp_correct_shade_component(
                        value,
                        coefficients[1][component],
                        coefficients[3][component],
                        coverage_mask,
                    );
                }

                let shade = rgba;
                let prepared = self.rdp_prepare_pixel(
                    x,
                    y,
                    RdpPixelInputs {
                        texels: [[255; 4]; 2],
                        shade,
                        legacy: shade,
                        lod_frac: 0,
                    },
                    coverage_count,
                );
                let memory_coverage = self.rdp_framebuffer_coverage_read(x, y);
                let mut depth_write = None;
                let pixel_state = if let Some(offset) = z_offset {
                    let Some(sample) = self.rdp_triangle_depth(
                        cursor,
                        offset,
                        RdpTrianglePixel {
                            y,
                            x,
                            major_x,
                            yh,
                            coverage_mask,
                        },
                    ) else {
                        continue;
                    };
                    let result =
                        self.rdp_depth_test(x, y, sample, prepared.coverage_count, memory_coverage);
                    if !result.depth_pass {
                        continue;
                    }
                    depth_write = Some(sample);
                    result
                } else {
                    self.rdp_blend_state_without_depth(prepared.coverage_count, memory_coverage)
                };

                let Some(rgba) = self.rdp_finish_prepared_pixel(
                    x,
                    y,
                    prepared,
                    RdpPixelState {
                        depth: pixel_state,
                        memory_coverage,
                    },
                ) else {
                    continue;
                };
                let final_coverage = self.rdp_finalize_coverage(
                    pixel_state.blend_en,
                    pixel_state.coverage_count,
                    memory_coverage,
                );
                self.write_rdp_coverage_pixel(x, y, rgba, final_coverage);
                if self.other_modes as u32 & 0x0020 != 0 {
                    if let Some(sample) = depth_write {
                        self.rdp_depth_write(x, y, sample.depth, sample.compressed_delta_z as u8);
                    }
                }
            }
        }
    }

    fn rdp_texture_triangle(
        &mut self,
        cursor: u32,
        high: u32,
        low: u32,
        texture_offset: u32,
        z_offset: Option<u32>,
        shade: bool,
    ) {
        let Some(texture_coefficients) = self.rdp_attribute_coefficients(cursor, texture_offset)
        else {
            return;
        };
        let shade_coefficients = if shade {
            self.rdp_attribute_coefficients(cursor, 32)
        } else {
            None
        };
        if shade && shade_coefficients.is_none() {
            return;
        }

        let tile = ((high >> 16) & 7) as usize;
        let max_level = ((high >> 19) & 7) as u8;
        let perspective = ((self.other_modes >> 32) as u32) & (1 << 19) != 0;
        let yh = Self::sign_extend(low & 0x3fff, 14);
        let base_line = i64::from(yh >> 2);

        for span in self.rdp_triangle_spans(cursor, high, low) {
            let y = span.y;
            let major_x = span.major_x;
            let dy = i64::try_from(y).unwrap_or_default().wrapping_sub(base_line);
            let mut texture_edge = [0i64; 4];
            for component in 0..4 {
                texture_edge[component] = texture_coefficients[0][component]
                    .wrapping_add(texture_coefficients[2][component].wrapping_mul(dy));
            }
            let mut shade_edge = [0i64; 4];
            if let Some(coefficients) = shade_coefficients.as_ref() {
                for component in 0..4 {
                    shade_edge[component] = coefficients[0][component]
                        .wrapping_add(coefficients[2][component].wrapping_mul(dy));
                }
            }

            for x in span.x0..=span.x1 {
                let Some(coverage_mask) = self.rdp_pixel_coverage_mask(span, x) else {
                    continue;
                };
                let coverage_count = coverage_mask.count_ones() as i32;
                let x_fixed = i64::try_from(x).unwrap_or_default() << 16;
                let dx = x_fixed.wrapping_sub(major_x);
                let mut texture = [0i64; 4];
                for component in 0..4 {
                    texture[component] = texture_edge[component]
                        .wrapping_add((texture_coefficients[1][component].wrapping_mul(dx)) >> 16);
                }

                let [sample_s, sample_t] = Self::rdp_texture_coordinates(texture, perspective);
                let mut lod_frac = 0i16;
                let mut tile0 = tile;
                let mut tile1 = (tile + 1) & 7;
                if self.rdp_cycle_type() == 1 {
                    let mut next_x = texture;
                    let mut next_y = texture;
                    for component in 0..3 {
                        next_x[component] = next_x[component]
                            .wrapping_add(texture_coefficients[1][component] & !0x1f_i64);
                        next_y[component] = next_y[component]
                            .wrapping_add(texture_coefficients[3][component] & !0x7fff_i64);
                    }
                    let [next_s, next_t] = Self::rdp_texture_coordinates(next_x, perspective);
                    let [next_y_s, next_y_t] = Self::rdp_texture_coordinates(next_y, perspective);
                    let lod_clamp = (sample_s | sample_t | next_s | next_t | next_y_s | next_y_t)
                        & 0x6_0000
                        != 0;
                    let mut lod = 0;
                    if !lod_clamp {
                        lod = Self::rdp_lod_delta(sample_s, next_s, sample_t, next_t, 0);
                        lod = Self::rdp_lod_delta(sample_s, next_y_s, sample_t, next_y_t, lod);
                    }
                    let signals = self.rdp_lod_signals(lod_clamp, lod, max_level);
                    lod_frac = signals.frac;
                    if self.other_modes & (1u64 << 48) != 0 {
                        (tile0, tile1) = self.rdp_lod_tiles(tile, signals, max_level);
                    }
                }
                let Some(texel0) = self.sample_rdp_texture_fixed(tile0, sample_s, sample_t) else {
                    continue;
                };
                let texel1 = if self.rdp_cycle_type() == 1 {
                    self.sample_rdp_texture_fixed(tile1, sample_s, sample_t)
                        .unwrap_or([0; 4])
                } else {
                    texel0
                };

                let mut shade_rgba = [255; 4];
                if let Some(coefficients) = shade_coefficients.as_ref() {
                    for component in 0..4 {
                        let value = shade_edge[component]
                            .wrapping_add((coefficients[1][component].wrapping_mul(dx)) >> 16);
                        shade_rgba[component] = Self::rdp_correct_shade_component(
                            value,
                            coefficients[1][component],
                            coefficients[3][component],
                            coverage_mask,
                        );
                    }
                }
                let mut legacy = texel0;
                if shade {
                    for component in 0..4 {
                        legacy[component] = ((u16::from(texel0[component])
                            * u16::from(shade_rgba[component])
                            + 127)
                            / 255) as u8;
                    }
                }

                let prepared = self.rdp_prepare_pixel(
                    x,
                    y,
                    RdpPixelInputs {
                        texels: [texel0, texel1],
                        shade: shade_rgba,
                        legacy,
                        lod_frac,
                    },
                    coverage_count,
                );
                let memory_coverage = self.rdp_framebuffer_coverage_read(x, y);
                let mut depth_write = None;
                let pixel_state = if let Some(offset) = z_offset {
                    let Some(sample) = self.rdp_triangle_depth(
                        cursor,
                        offset,
                        RdpTrianglePixel {
                            y,
                            x,
                            major_x,
                            yh,
                            coverage_mask,
                        },
                    ) else {
                        continue;
                    };
                    let result =
                        self.rdp_depth_test(x, y, sample, prepared.coverage_count, memory_coverage);
                    if !result.depth_pass {
                        continue;
                    }
                    depth_write = Some(sample);
                    result
                } else {
                    self.rdp_blend_state_without_depth(prepared.coverage_count, memory_coverage)
                };
                let Some(rgba) = self.rdp_finish_prepared_pixel(
                    x,
                    y,
                    prepared,
                    RdpPixelState {
                        depth: pixel_state,
                        memory_coverage,
                    },
                ) else {
                    continue;
                };
                let final_coverage = self.rdp_finalize_coverage(
                    pixel_state.blend_en,
                    pixel_state.coverage_count,
                    memory_coverage,
                );
                self.write_rdp_coverage_pixel(x, y, rgba, final_coverage);
                if self.other_modes as u32 & 0x0020 != 0 {
                    if let Some(sample) = depth_write {
                        self.rdp_depth_write(x, y, sample.depth, sample.compressed_delta_z as u8);
                    }
                }
            }
        }
    }

    fn rdp_fill_rectangle(&mut self, high: u32, low: u32) {
        let start_x = (low >> 12) & 0x0fff;
        let start_y = low & 0x0fff;
        let end_x = (high >> 12) & 0x0fff;
        let mut end_y = high & 0x0fff;
        let cycle_type = self.rdp_cycle_type();
        if matches!(cycle_type, 2 | 3) {
            end_y |= 3;
        }
        if end_x < start_x || end_y < start_y {
            return;
        }

        let scissor_left = u32::from(self.scissor_fixed[0]);
        let scissor_top = u32::from(self.scissor_fixed[1]);
        let scissor_right = u32::from(self.scissor_fixed[2]);
        let scissor_bottom = u32::from(self.scissor_fixed[3]);
        if start_x >= scissor_right || start_y >= scissor_bottom {
            return;
        }

        let min_x = (start_x >> 2).max(scissor_left >> 2) as usize;
        let min_y = (start_y >> 2).max(scissor_top >> 2) as usize;
        let max_x = (end_x >> 2)
            .min(scissor_right >> 2)
            .min(self.color_width.max(1).saturating_sub(1)) as usize;
        let max_y = (end_y >> 2).min(((scissor_bottom + 3) >> 2).saturating_sub(1)) as usize;
        if min_x > max_x || min_y > max_y {
            return;
        }

        let modes = self.other_modes as u32;
        if cycle_type == 2 {
            match self.color_size {
                0 => {
                    for y in min_y..=max_y {
                        if !self.rdp_scissor_accepts_line(y) {
                            continue;
                        }
                        for x in min_x..=max_x {
                            let pixel = y
                                .saturating_mul(self.color_width.max(1) as usize)
                                .saturating_add(x);
                            if let Some(target) =
                                self.rdram.get_mut(self.color_image as usize + pixel)
                            {
                                *target = 0;
                            }
                        }
                    }
                    return;
                }
                2 => {
                    for y in min_y..=max_y {
                        if !self.rdp_scissor_accepts_line(y) {
                            continue;
                        }
                        if !self.rdp_copy_u16_span(y, min_x, max_x, true) {
                            return;
                        }
                    }
                    return;
                }
                3 => {
                    self.rdp_pipeline_crashed = true;
                    return;
                }
                _ => {}
            }
        }

        if cycle_type == 3 && modes & 0x0050 != 0 {
            self.rdp_pipeline_crashed = true;
            return;
        }

        if cycle_type >= 2 {
            for y in min_y..=max_y {
                if !self.rdp_scissor_accepts_line(y) {
                    continue;
                }
                for x in min_x..=max_x {
                    self.write_rdp_fill_pixel(x, y);
                }
            }
            if cycle_type == 3 && modes & 0x0020 != 0 && modes & 0x0004 == 0 {
                self.rdp_pipeline_crashed = true;
            }
            return;
        }

        for y in min_y..=max_y {
            if !self.rdp_scissor_accepts_line(y) {
                continue;
            }
            for x in min_x..=max_x {
                let Some(coverage_count) =
                    self.rdp_rectangle_coverage(start_x, start_y, end_x, end_y, x, y)
                else {
                    continue;
                };
                let prepared = self.rdp_prepare_pixel(
                    x,
                    y,
                    RdpPixelInputs {
                        texels: [[0; 4]; 2],
                        shade: [0; 4],
                        legacy: [0; 4],
                        lod_frac: 0,
                    },
                    coverage_count,
                );
                let memory_coverage = self.rdp_framebuffer_coverage_read(x, y);
                let mut depth_write = None;
                let pixel_state = if modes & 0x0030 != 0 {
                    let sample = if modes & 0x0004 != 0 {
                        let delta_z = i32::from(self.primitive_delta_z);
                        RdpDepthSample {
                            depth: i32::from(self.primitive_depth) << 3,
                            delta_z,
                            compressed_delta_z: Self::rdp_dz_compress(delta_z),
                        }
                    } else {
                        RdpDepthSample::default()
                    };
                    let result =
                        self.rdp_depth_test(x, y, sample, prepared.coverage_count, memory_coverage);
                    if !result.depth_pass {
                        continue;
                    }
                    depth_write = Some(sample);
                    result
                } else {
                    self.rdp_blend_state_without_depth(prepared.coverage_count, memory_coverage)
                };
                let Some(rgba) = self.rdp_finish_prepared_pixel(
                    x,
                    y,
                    prepared,
                    RdpPixelState {
                        depth: pixel_state,
                        memory_coverage,
                    },
                ) else {
                    continue;
                };
                let final_coverage = self.rdp_finalize_coverage(
                    pixel_state.blend_en,
                    pixel_state.coverage_count,
                    memory_coverage,
                );
                self.write_rdp_coverage_pixel(x, y, rgba, final_coverage);
                if modes & 0x0020 != 0 {
                    if let Some(sample) = depth_write {
                        self.rdp_depth_write(x, y, sample.depth, sample.compressed_delta_z as u8);
                    }
                }
            }
        }
    }
    fn vi_read_rgba16_sample(
        &self,
        origin: usize,
        width: usize,
        x: usize,
        y: usize,
    ) -> Option<([u8; 4], u8)> {
        let pixel = y.checked_mul(width)?.checked_add(x)?;
        let address = origin.checked_add(pixel.checked_mul(2)?)?;
        let bytes = self.rdram.get(address..address.checked_add(2)?)?;
        let value = u16::from_be_bytes([bytes[0], bytes[1]]);
        let mut color = Self::rgba5551(value);
        color[3] = 255;
        let coverage = (((value & 1) as u8) << 2) | self.rdp_hidden_read(address);
        Some((color, coverage))
    }

    fn vi_read_rgba32_sample(
        &self,
        origin: usize,
        width: usize,
        x: usize,
        y: usize,
    ) -> Option<([u8; 4], u8)> {
        let pixel = y.checked_mul(width)?.checked_add(x)?;
        let address = origin.checked_add(pixel.checked_mul(4)?)?;
        let bytes = self.rdram.get(address..address.checked_add(4)?)?;
        Some(([bytes[0], bytes[1], bytes[2], 255], (bytes[3] >> 5) & 7))
    }

    fn vi_restore_rgba16(
        &self,
        origin: usize,
        width: usize,
        x: usize,
        y: usize,
        mut center: [u8; 4],
    ) -> [u8; 4] {
        let neighbors = [
            (x.checked_sub(1), y.checked_sub(1)),
            (Some(x), y.checked_sub(1)),
            (x.checked_add(1), y.checked_sub(1)),
            (x.checked_sub(1), Some(y)),
            (x.checked_add(1), Some(y)),
            (x.checked_sub(1), y.checked_add(1)),
            (Some(x), y.checked_add(1)),
            (x.checked_add(1), y.checked_add(1)),
        ];

        let mut restored = [
            i16::from(center[0]),
            i16::from(center[1]),
            i16::from(center[2]),
        ];
        for (neighbor_x, neighbor_y) in neighbors {
            let (Some(neighbor_x), Some(neighbor_y)) = (neighbor_x, neighbor_y) else {
                continue;
            };
            let Some((neighbor, _)) =
                self.vi_read_rgba16_sample(origin, width, neighbor_x, neighbor_y)
            else {
                continue;
            };
            for channel in 0..3 {
                restored[channel] += match (center[channel] >> 3).cmp(&(neighbor[channel] >> 3)) {
                    std::cmp::Ordering::Less => 1,
                    std::cmp::Ordering::Equal => 0,
                    std::cmp::Ordering::Greater => -1,
                };
            }
        }
        for channel in 0..3 {
            center[channel] = restored[channel].clamp(0, 255) as u8;
        }
        center
    }

    fn vi_restore_rgba32(
        &self,
        origin: usize,
        width: usize,
        x: usize,
        y: usize,
        mut center: [u8; 4],
    ) -> [u8; 4] {
        let neighbors = [
            (x.checked_sub(1), y.checked_sub(1)),
            (Some(x), y.checked_sub(1)),
            (x.checked_add(1), y.checked_sub(1)),
            (x.checked_sub(1), Some(y)),
            (x.checked_add(1), Some(y)),
            (x.checked_sub(1), y.checked_add(1)),
            (Some(x), y.checked_add(1)),
            (x.checked_add(1), y.checked_add(1)),
        ];

        let mut restored = [
            i16::from(center[0]),
            i16::from(center[1]),
            i16::from(center[2]),
        ];
        for (neighbor_x, neighbor_y) in neighbors {
            let (Some(neighbor_x), Some(neighbor_y)) = (neighbor_x, neighbor_y) else {
                continue;
            };
            let Some((neighbor, _)) =
                self.vi_read_rgba32_sample(origin, width, neighbor_x, neighbor_y)
            else {
                continue;
            };
            for channel in 0..3 {
                restored[channel] += match (center[channel] >> 3).cmp(&(neighbor[channel] >> 3)) {
                    std::cmp::Ordering::Less => 1,
                    std::cmp::Ordering::Equal => 0,
                    std::cmp::Ordering::Greater => -1,
                };
            }
        }
        for channel in 0..3 {
            center[channel] = restored[channel].clamp(0, 255) as u8;
        }
        center
    }

    fn vi_filter_rgba32(&self, origin: usize, width: usize, x: usize, y: usize) -> Option<[u8; 4]> {
        let (mut center, coverage) = self.vi_read_rgba32_sample(origin, width, x, y)?;
        let aa_control = (self.vi.regs[0] >> 8) & 3;
        if coverage == 7 {
            if self.vi.regs[0] & (1 << 16) != 0 {
                center = self.vi_restore_rgba32(origin, width, x, y, center);
            }
            return Some(center);
        }
        if aa_control >= 2 {
            return Some(center);
        }

        let mut channels = [[0u8; 7]; 3];
        let mut count = 1usize;
        for channel in 0..3 {
            channels[channel][0] = center[channel];
        }

        let neighbors = [
            (x.checked_sub(1), y.checked_sub(1)),
            (x.checked_add(1), y.checked_sub(1)),
            (x.checked_sub(2), Some(y)),
            (x.checked_add(2), Some(y)),
            (x.checked_sub(1), y.checked_add(1)),
            (x.checked_add(1), y.checked_add(1)),
        ];
        for (neighbor_x, neighbor_y) in neighbors {
            let (Some(neighbor_x), Some(neighbor_y)) = (neighbor_x, neighbor_y) else {
                continue;
            };
            let Some((color, neighbor_coverage)) =
                self.vi_read_rgba32_sample(origin, width, neighbor_x, neighbor_y)
            else {
                continue;
            };
            if neighbor_coverage != 7 {
                continue;
            }
            for channel in 0..3 {
                channels[channel][count] = color[channel];
            }
            count += 1;
        }

        let coefficient = i32::from(7u8.saturating_sub(coverage));
        for channel in 0..3 {
            let values = &channels[channel][..count];
            let (penultimate_min, penultimate_max) = Self::vi_second_extrema(values);
            let base = i32::from(center[channel]);
            let correction = i32::from(penultimate_min) + i32::from(penultimate_max) - (base << 1);
            center[channel] = (((correction * coefficient + 4) >> 3) + base) as u8;
        }
        Some(center)
    }

    fn vi_second_extrema(values: &[u8]) -> (u8, u8) {
        let mut pos_max = 0usize;
        let mut pos_min = 0usize;
        let mut penultimate_max = values[0];
        let mut penultimate_min = values[0];

        for index in 1..values.len() {
            if values[index] > values[pos_max] {
                penultimate_max = values[pos_max];
                pos_max = index;
            } else if values[index] < values[pos_min] {
                penultimate_min = values[pos_min];
                pos_min = index;
            }
        }

        let maximum = values[pos_max];
        let minimum = values[pos_min];
        if penultimate_max != maximum {
            for &value in &values[pos_max + 1..] {
                penultimate_max = penultimate_max.max(value);
            }
        }
        if penultimate_min != minimum {
            for &value in &values[pos_min + 1..] {
                penultimate_min = penultimate_min.min(value);
            }
        }
        (penultimate_min, penultimate_max)
    }

    fn vi_filter_rgba16(&self, origin: usize, width: usize, x: usize, y: usize) -> Option<[u8; 4]> {
        let (mut center, coverage) = self.vi_read_rgba16_sample(origin, width, x, y)?;
        let aa_control = (self.vi.regs[0] >> 8) & 3;
        if coverage == 7 {
            if self.vi.regs[0] & (1 << 16) != 0 {
                center = self.vi_restore_rgba16(origin, width, x, y, center);
            }
            return Some(center);
        }
        if aa_control >= 2 {
            return Some(center);
        }

        let mut channels = [[0u8; 7]; 3];
        let mut count = 1usize;
        for channel in 0..3 {
            channels[channel][0] = center[channel];
        }

        let neighbors = [
            (x.checked_sub(1), y.checked_sub(1)),
            (x.checked_add(1), y.checked_sub(1)),
            (x.checked_sub(2), Some(y)),
            (x.checked_add(2), Some(y)),
            (x.checked_sub(1), y.checked_add(1)),
            (x.checked_add(1), y.checked_add(1)),
        ];
        for (neighbor_x, neighbor_y) in neighbors {
            let (Some(neighbor_x), Some(neighbor_y)) = (neighbor_x, neighbor_y) else {
                continue;
            };
            let Some((color, neighbor_coverage)) =
                self.vi_read_rgba16_sample(origin, width, neighbor_x, neighbor_y)
            else {
                continue;
            };
            if neighbor_coverage != 7 {
                continue;
            }
            for channel in 0..3 {
                channels[channel][count] = color[channel];
            }
            count += 1;
        }

        let coefficient = i32::from(7u8.saturating_sub(coverage));
        for channel in 0..3 {
            let values = &channels[channel][..count];
            let (penultimate_min, penultimate_max) = Self::vi_second_extrema(values);
            let base = i32::from(center[channel]);
            let correction = i32::from(penultimate_min) + i32::from(penultimate_max) - (base << 1);
            center[channel] = (((correction * coefficient + 4) >> 3) + base) as u8;
        }
        Some(center)
    }

    fn vi_divot_color(
        center: [u8; 4],
        left: [u8; 4],
        right: [u8; 4],
        coverages: [u8; 3],
    ) -> [u8; 4] {
        if coverages[0] & coverages[1] & coverages[2] == 7 {
            return center;
        }

        let mut result = center;
        for channel in 0..3 {
            let left_value = left[channel];
            let center_value = center[channel];
            let right_value = right[channel];
            if (left_value >= center_value && right_value >= left_value)
                || (left_value >= right_value && center_value >= left_value)
            {
                result[channel] = left_value;
            } else if (right_value >= center_value && left_value >= right_value)
                || (right_value >= left_value && center_value >= right_value)
            {
                result[channel] = right_value;
            }
        }
        result
    }

    fn vi_apply_divot16(
        &self,
        origin: usize,
        width: usize,
        x: usize,
        y: usize,
        center: [u8; 4],
    ) -> [u8; 4] {
        if self.vi.regs[0] & (1 << 4) == 0 {
            return center;
        }
        let Some(left_x) = x.checked_sub(1) else {
            return center;
        };
        let Some(right_x) = x.checked_add(1) else {
            return center;
        };
        let Some(left) = self.vi_filter_rgba16(origin, width, left_x, y) else {
            return center;
        };
        let Some(right) = self.vi_filter_rgba16(origin, width, right_x, y) else {
            return center;
        };
        let Some((_, left_coverage)) = self.vi_read_rgba16_sample(origin, width, left_x, y) else {
            return center;
        };
        let Some((_, center_coverage)) = self.vi_read_rgba16_sample(origin, width, x, y) else {
            return center;
        };
        let Some((_, right_coverage)) = self.vi_read_rgba16_sample(origin, width, right_x, y)
        else {
            return center;
        };
        Self::vi_divot_color(
            center,
            left,
            right,
            [center_coverage, left_coverage, right_coverage],
        )
    }

    fn vi_apply_divot32(
        &self,
        origin: usize,
        width: usize,
        x: usize,
        y: usize,
        center: [u8; 4],
    ) -> [u8; 4] {
        if self.vi.regs[0] & (1 << 4) == 0 {
            return center;
        }
        let Some(left_x) = x.checked_sub(1) else {
            return center;
        };
        let Some(right_x) = x.checked_add(1) else {
            return center;
        };
        let Some(left) = self.vi_filter_rgba32(origin, width, left_x, y) else {
            return center;
        };
        let Some(right) = self.vi_filter_rgba32(origin, width, right_x, y) else {
            return center;
        };
        let Some((_, left_coverage)) = self.vi_read_rgba32_sample(origin, width, left_x, y) else {
            return center;
        };
        let Some((_, center_coverage)) = self.vi_read_rgba32_sample(origin, width, x, y) else {
            return center;
        };
        let Some((_, right_coverage)) = self.vi_read_rgba32_sample(origin, width, right_x, y)
        else {
            return center;
        };
        Self::vi_divot_color(
            center,
            left,
            right,
            [center_coverage, left_coverage, right_coverage],
        )
    }

    fn vi_lerp(mut upper: [u8; 4], lower: [u8; 4], fraction: u32) -> [u8; 4] {
        if fraction == 0 {
            return upper;
        }
        for channel in 0..3 {
            let start = i32::from(upper[channel]);
            let delta = i32::from(lower[channel]) - start;
            upper[channel] = (((delta * fraction as i32 + 16) >> 5) + start) as u8;
        }
        upper[3] = 255;
        upper
    }

    fn vi_fetch_pre_gamma(
        &self,
        origin: usize,
        width: usize,
        x: usize,
        y: usize,
        pixel_type: u32,
    ) -> Option<[u8; 4]> {
        if pixel_type == 2 {
            let color = self.vi_filter_rgba16(origin, width, x, y)?;
            Some(self.vi_apply_divot16(origin, width, x, y, color))
        } else {
            let color = self.vi_filter_rgba32(origin, width, x, y)?;
            Some(self.vi_apply_divot32(origin, width, x, y, color))
        }
    }

    fn vi_integer_sqrt(value: u32) -> u32 {
        let mut operand = value;
        let mut result = 0u32;
        let mut bit = 1u32 << 30;
        while bit > operand {
            bit >>= 2;
        }
        while bit != 0 {
            if operand >= result + bit {
                operand -= result + bit;
                result += bit << 1;
            }
            result >>= 1;
            bit >>= 2;
        }
        result
    }

    fn vi_apply_gamma(&mut self, mut color: [u8; 4]) -> [u8; 4] {
        let mode = ((self.vi.regs[0] >> 2) & 3) as u8;
        match mode {
            0 => {}
            1 => {
                let random = self.rdp_next_random();
                for (component, bit) in color[..3].iter_mut().zip([0, 1, 2]) {
                    if *component < 255 && random >> bit & 1 != 0 {
                        *component += 1;
                    }
                }
            }
            2 => {
                for component in &mut color[..3] {
                    *component = (Self::vi_integer_sqrt(u32::from(*component) << 6) << 1) as u8;
                }
            }
            _ => {
                let random = self.rdp_next_random();
                let dithers = [
                    random & 0x3f,
                    (random >> 6) & 0x3f,
                    ((random >> 9) & 0x38) | (random & 7),
                ];
                for (component, dither) in color[..3].iter_mut().zip(dithers) {
                    let index = (u32::from(*component) << 6) | dither;
                    *component = (Self::vi_integer_sqrt(index) << 1) as u8;
                }
            }
        }
        color[3] = 255;
        color
    }

    fn render_vi(&mut self, video: &mut VideoBuffer) {
        let serrated = self.vi.regs[0] & (1 << 6) != 0;
        if !serrated {
            video.clear([0, 0, 0, 255]);
        }
        let pixel_type = self.vi.regs[0] & 3;
        if pixel_type < 2 {
            return;
        }

        let origin = (self.vi.regs[1] & 0x00ff_ffff) as usize;
        let source_width = (self.vi.regs[2] & 0x0fff).max(1) as usize;
        let h_video = self.vi.regs[9];
        let v_video = self.vi.regs[10];
        let x_scale = self.vi.regs[12];
        let y_scale = self.vi.regs[13];

        let programmed_geometry = h_video != 0 && x_scale & 0x0fff != 0 && y_scale & 0x0fff != 0;
        if programmed_geometry {
            const HSCAN_START: i32 = 108;
            const HSCAN_STOP: i32 = HSCAN_START + WIDTH as i32;
            const VSCAN_START: i32 = 34;
            const VSCAN_STOP: i32 = VSCAN_START + HEIGHT as i32;

            let h_start = ((h_video >> 16) & 0x03ff) as i32;
            let h_end = (h_video & 0x03ff) as i32;
            let v_start = ((v_video >> 16) & 0x03ff) as i32;
            let mut v_end = (v_video & 0x03ff) as i32;
            if v_end < v_start {
                v_end = VSCAN_STOP;
            }

            let mut dx0 = h_start.max(HSCAN_START);
            let mut dx1 = h_end.min(HSCAN_STOP);
            let dy0 = v_start.max(VSCAN_START);
            let dy1 = v_end.min(VSCAN_STOP);

            // The VI trims an asymmetric horizontal guard band before scanout.
            if dx0 >= HSCAN_START {
                dx0 += 8;
            }
            if dx1 < HSCAN_STOP {
                dx1 -= 7;
            }
            if dx1 <= dx0 || dy1 <= dy0 {
                return;
            }

            let x_step = x_scale & 0x0fff;
            let x_subpixel = (x_scale >> 16) & 0x0fff;
            let y_step = y_scale & 0x0fff;
            let y_subpixel = (y_scale >> 16) & 0x0fff;
            let mut y_fixed = y_subpixel.wrapping_add(y_step.wrapping_mul((dy0 - v_start) as u32));

            let field = self.vi.current & 1 != 0;
            for dy in dy0..dy1 {
                let source_y = (y_fixed >> 11) as usize;
                if serrated && ((dy & 1 != 0) == field) {
                    y_fixed = y_fixed.wrapping_add(y_step);
                    continue;
                }
                let target_y = (dy - VSCAN_START) as usize;
                let mut x_fixed =
                    x_subpixel.wrapping_add(x_step.wrapping_mul((dx0 - h_start) as u32));

                for dx in dx0..dx1 {
                    let source_x = (x_fixed >> 10) as usize;
                    let Some(mut color) = self.vi_fetch_pre_gamma(
                        origin,
                        source_width,
                        source_x,
                        source_y,
                        pixel_type,
                    ) else {
                        x_fixed = x_fixed.wrapping_add(x_step);
                        continue;
                    };

                    if (self.vi.regs[0] >> 8) & 3 != 3 {
                        let x_fraction = (x_fixed >> 5) & 0x1f;
                        let y_fraction = (y_fixed >> 6) & 0x1f;
                        if x_fraction != 0 || y_fraction != 0 {
                            let black = [0, 0, 0, 255];
                            let mut right = self
                                .vi_fetch_pre_gamma(
                                    origin,
                                    source_width,
                                    source_x.saturating_add(1),
                                    source_y,
                                    pixel_type,
                                )
                                .unwrap_or(black);
                            let down = self
                                .vi_fetch_pre_gamma(
                                    origin,
                                    source_width,
                                    source_x,
                                    source_y.saturating_add(1),
                                    pixel_type,
                                )
                                .unwrap_or(black);
                            let down_right = self
                                .vi_fetch_pre_gamma(
                                    origin,
                                    source_width,
                                    source_x.saturating_add(1),
                                    source_y.saturating_add(1),
                                    pixel_type,
                                )
                                .unwrap_or(black);
                            color = Self::vi_lerp(color, down, y_fraction);
                            right = Self::vi_lerp(right, down_right, y_fraction);
                            color = Self::vi_lerp(color, right, x_fraction);
                        }
                    }
                    let color = self.vi_apply_gamma(color);

                    let target_x = (dx - HSCAN_START) as usize;
                    let target = (target_y * WIDTH as usize + target_x) * 4;
                    if let Some(pixel) = video.pixels_mut().get_mut(target..target + 4) {
                        pixel.copy_from_slice(&color);
                    }
                    x_fixed = x_fixed.wrapping_add(x_step);
                }
                y_fixed = y_fixed.wrapping_add(y_step);
            }
            return;
        }

        let v_begin = ((v_video >> 16) & 0x03ff) as usize;
        let v_end = (v_video & 0x03ff) as usize;
        let source_height = if v_end > v_begin {
            ((v_end - v_begin) / 2).clamp(1, HEIGHT as usize)
        } else {
            240
        };
        let output_width = WIDTH as usize;
        let output_height = HEIGHT as usize;
        for target_y in 0..output_height {
            let source_y = (target_y * source_height / output_height).min(source_height - 1);
            for target_x in 0..output_width {
                let source_x = (target_x * source_width / output_width).min(source_width - 1);
                let color = if pixel_type == 2 {
                    let Some(color) =
                        self.vi_filter_rgba16(origin, source_width, source_x, source_y)
                    else {
                        continue;
                    };
                    color
                } else {
                    let Some(color) =
                        self.vi_filter_rgba32(origin, source_width, source_x, source_y)
                    else {
                        continue;
                    };
                    color
                };
                let color = if pixel_type == 2 {
                    self.vi_apply_divot16(origin, source_width, source_x, source_y, color)
                } else {
                    self.vi_apply_divot32(origin, source_width, source_x, source_y, color)
                };
                let color = self.vi_apply_gamma(color);
                let target = (target_y * output_width + target_x) * 4;
                video.pixels_mut()[target..target + 4].copy_from_slice(&color);
            }
        }
    }

    fn rgba5551(value: u16) -> [u8; 4] {
        let r = u32::from((value >> 11) & 0x1f);
        let g = u32::from((value >> 6) & 0x1f);
        let b = u32::from((value >> 1) & 0x1f);
        [
            (r * 255 / 31) as u8,
            (g * 255 / 31) as u8,
            (b * 255 / 31) as u8,
            if value & 1 != 0 { 255 } else { 0 },
        ]
    }

    fn end_frame(&mut self, video: &mut VideoBuffer) {
        self.render_vi(video);
        if self.vi.regs[0] & (1 << 6) != 0 {
            self.vi.current = (self.vi.current ^ 1) & 1;
        } else {
            self.vi.current = 0;
        }
        self.raise(MI_VI);
        self.frame = self.frame.wrapping_add(1);
    }
    fn save(&self, out: &mut StateWriter) {
        out.blob(self.rdram.as_ref());
        out.blob(self.rdram_hidden.as_ref());
        out.blob(self.sp_mem.as_ref());
        out.blob(self.save.as_ref());
        out.blob(self.controller_pak.as_ref());
        out.blob(&self.pif_ram);
        out.u16(self.eeprom_size as u16);
        out.u32(self.eeprom_busy_cycles);
        out.u8(u8::from(self.rtc_present));
        out.blob(&self.rtc_ram);
        out.u8(self.rtc_status);
        out.u8(self.rtc_write_lock);
        out.u64(self.rtc_cycle_phase);
        out.u8(u8::from(self.flash_present));
        out.u8(self.flash_mode);
        out.u32(self.flash_status);
        out.u16(self.flash_erase_page);
        out.blob(&self.flash_page_buffer);
        out.u32(self.sp.mem_addr);
        out.u32(self.sp.dram_addr);
        out.u32(self.sp.rd_len);
        out.u32(self.sp.wr_len);
        out.u32(self.sp.status);
        out.u8(u8::from(self.sp.semaphore));
        out.u32(self.sp.pc);
        for job in [self.sp.dma_current, self.sp.dma_pending] {
            out.u32(job.mem_addr);
            out.u32(job.dram_addr);
            out.u32(job.length);
            out.u32(job.count);
            out.u32(job.skip);
            out.u8(job.direction);
        }
        out.u8(u8::from(self.sp.dma_busy));
        out.u8(u8::from(self.sp.dma_full));
        out.u32(self.sp.dma_rsp_cycles);
        out.u64(self.sp.dma_phase);
        out.u32(self.dp.start);
        out.u32(self.dp.end);
        out.u32(self.dp.current);
        out.u32(self.dp.status);
        out.u32(self.dp.clock);
        for value in self.vi.regs {
            out.u32(value);
        }
        out.u32(self.vi.current);
        out.u32(self.ai.dram_addr);
        out.u32(self.ai.len);
        out.u32(self.ai.next_dram_addr);
        out.u32(self.ai.next_len);
        out.u8(self.ai.dma_count);
        out.u32(self.ai.control);
        out.u32(self.ai.status);
        out.u32(self.ai.dac_rate);
        out.u32(self.ai.bit_rate);
        out.u64(self.ai.sample_phase);
        out.u64(self.ai.output_phase);
        out.u16(self.ai.sample_left as u16);
        out.u16(self.ai.sample_right as u16);
        out.u32(self.pi.dram_addr);
        out.u32(self.pi.cart_addr);
        out.u32(self.pi.rd_len);
        out.u32(self.pi.wr_len);
        out.u32(self.pi.status);
        out.u32(self.pi.dma_cycles);
        for value in self.pi.timing {
            out.u32(value);
        }
        out.u32(self.si.dram_addr);
        out.u32(self.si.status);
        out.u32(self.si.dma_cycles);
        out.u8(self.si.dma_direction);
        out.u32(self.mi_mode);
        out.u32(self.mi_intr);
        out.u32(self.mi_mask);
        out.u32(self.fill_color);
        out.u32(self.color_image);
        out.u32(self.color_width);
        out.u8(self.color_size);
        out.u8(self.color_format);
        for value in self.scissor {
            out.u16(value);
        }
        for value in self.scissor_fixed {
            out.u16(value);
        }
        out.u8(u8::from(self.scissor_field));
        out.u8(u8::from(self.scissor_keep_odd));
        out.u64(self.other_modes);
        out.u32(self.primitive_color);
        out.u8(self.primitive_lod_frac);
        out.u8(self.primitive_min_level);
        out.u32(self.environment_color);
        out.u32(self.blend_color);
        out.u32(self.fog_color);
        out.u32(self.texture_image);
        out.u32(self.texture_width);
        out.u8(self.texture_size);
        out.u8(self.texture_format);
        out.u32(self.depth_image);
        out.u16(self.primitive_depth);
        out.u16(self.primitive_delta_z);
        out.u64(self.combine_mode);
        for value in self.convert_k {
            out.u16(value as u16);
        }
        for value in self.key_center {
            out.u8(value);
        }
        for value in self.key_scale {
            out.u8(value);
        }
        for value in self.key_width {
            out.u16(value);
        }
        out.blob(self.tmem.as_ref());
        for value in self.tlut {
            out.u16(value);
        }
        for tile in self.tiles {
            out.u8(tile.format);
            out.u8(tile.size);
            out.u16(tile.line);
            out.u16(tile.tmem);
            out.u8(tile.palette);
            out.u8(u8::from(tile.clamp_t));
            out.u8(u8::from(tile.mirror_t));
            out.u8(tile.mask_t);
            out.u8(tile.shift_t);
            out.u8(u8::from(tile.clamp_s));
            out.u8(u8::from(tile.mirror_s));
            out.u8(tile.mask_s);
            out.u8(tile.shift_s);
            out.u16(tile.sl);
            out.u16(tile.tl);
            out.u16(tile.sh);
            out.u16(tile.th);
        }
        out.u32(self.rdp_noise_seed);
        out.u8(u8::from(self.rdp_pipeline_crashed));
        out.u64(self.frame);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for (name, target) in [
            ("RDRAM", self.rdram.as_mut()),
            ("RDRAM hidden", self.rdram_hidden.as_mut()),
            ("SP memory", self.sp_mem.as_mut()),
            ("save memory", self.save.as_mut()),
            ("Controller Pak", self.controller_pak.as_mut()),
        ] {
            let data = input.blob()?;
            if data.len() != target.len() {
                return Err(format!("N64 {name} state size mismatch"));
            }
            target.copy_from_slice(data);
        }
        let pif = input.blob()?;
        if pif.len() != self.pif_ram.len() {
            return Err("N64 PIF RAM state size mismatch".into());
        }
        self.pif_ram.copy_from_slice(pif);
        let eeprom_size = usize::from(input.u16()?);
        if eeprom_size != self.eeprom_size {
            return Err("N64 EEPROM profile differs from cartridge".into());
        }
        self.eeprom_busy_cycles = input.u32()?.min(EEPROM_WRITE_CYCLES);
        let rtc_present = input.u8()? != 0;
        if rtc_present != self.rtc_present {
            return Err("N64 RTC profile differs from cartridge".into());
        }
        let rtc = input.blob()?;
        if rtc.len() != self.rtc_ram.len() {
            return Err("N64 RTC state size mismatch".into());
        }
        self.rtc_ram.copy_from_slice(rtc);
        self.rtc_status = input.u8()?;
        self.rtc_write_lock = input.u8()? & 0x07;
        self.rtc_cycle_phase = input.u64()? % CPU_HZ;
        let flash_present = input.u8()? != 0;
        if flash_present != self.flash_present {
            return Err("N64 FlashRAM profile differs from cartridge".into());
        }
        self.flash_mode = input.u8()?.min(FLASH_CHIP_ERASE);
        self.flash_status = input.u32()?;
        self.flash_erase_page = input.u16()?;
        let flash_page = input.blob()?;
        if flash_page.len() != self.flash_page_buffer.len() {
            return Err("N64 FlashRAM page-buffer state size mismatch".into());
        }
        self.flash_page_buffer.copy_from_slice(flash_page);
        self.sp.mem_addr = input.u32()?;
        self.sp.dram_addr = input.u32()?;
        self.sp.rd_len = input.u32()?;
        self.sp.wr_len = input.u32()?;
        self.sp.status = input.u32()?;
        self.sp.semaphore = input.u8()? != 0;
        self.sp.pc = input.u32()? & 0x0ffc;
        for job in [&mut self.sp.dma_current, &mut self.sp.dma_pending] {
            job.mem_addr = input.u32()? & 0x1ff8;
            job.dram_addr = input.u32()? & 0x00ff_fff8;
            job.length = input.u32()? & 0x0ff8;
            job.count = input.u32()? & 0xff;
            job.skip = input.u32()? & 0x0ff8;
            job.direction = input.u8()?.min(2);
        }
        self.sp.dma_busy = input.u8()? != 0;
        self.sp.dma_full = input.u8()? != 0;
        self.sp.dma_rsp_cycles = input.u32()?;
        self.sp.dma_phase = input.u64()? % CPU_HZ;
        self.dp.start = input.u32()?;
        self.dp.end = input.u32()?;
        self.dp.current = input.u32()?;
        self.dp.status = input.u32()?;
        self.dp.clock = input.u32()?;
        for value in &mut self.vi.regs {
            *value = input.u32()?;
        }
        self.vi.current = input.u32()?;
        self.ai.dram_addr = input.u32()?;
        self.ai.len = input.u32()?;
        self.ai.next_dram_addr = input.u32()?;
        self.ai.next_len = input.u32()?;
        self.ai.dma_count = input.u8()?.min(2);
        self.ai.control = input.u32()? & 1;
        self.ai.status = input.u32()?;
        self.ai.dac_rate = input.u32()? & 0x3fff;
        self.ai.bit_rate = input.u32()? & 0xf;
        self.ai.sample_phase = input.u64()?;
        self.ai.output_phase = input.u64()?;
        self.ai.sample_left = input.u16()? as i16;
        self.ai.sample_right = input.u16()? as i16;
        self.update_ai_status();
        self.pi.dram_addr = input.u32()?;
        self.pi.cart_addr = input.u32()?;
        self.pi.rd_len = input.u32()?;
        self.pi.wr_len = input.u32()?;
        self.pi.status = input.u32()?;
        self.pi.dma_cycles = input.u32()?;
        for value in &mut self.pi.timing {
            *value = input.u32()?;
        }
        self.si.dram_addr = input.u32()?;
        self.si.status = input.u32()?;
        self.si.dma_cycles = input.u32()?;
        self.si.dma_direction = input.u8()?.min(2);
        self.mi_mode = input.u32()?;
        self.mi_intr = input.u32()?;
        self.mi_mask = input.u32()?;
        self.fill_color = input.u32()?;
        self.color_image = input.u32()? & 0x00ff_ffff;
        self.color_width = input.u32()?;
        self.color_size = input.u8()?;
        self.color_format = input.u8()? & 7;
        for value in &mut self.scissor {
            *value = input.u16()?;
        }
        for value in &mut self.scissor_fixed {
            *value = input.u16()?;
        }
        self.scissor_field = input.u8()? != 0;
        self.scissor_keep_odd = input.u8()? != 0;
        self.other_modes = input.u64()?;
        self.primitive_color = input.u32()?;
        self.primitive_lod_frac = input.u8()?;
        self.primitive_min_level = input.u8()? & 0x1f;
        self.environment_color = input.u32()?;
        self.blend_color = input.u32()?;
        self.fog_color = input.u32()?;
        self.texture_image = input.u32()? & 0x00ff_ffff;
        self.texture_width = input.u32()?.max(1);
        self.texture_size = input.u8()? & 3;
        self.texture_format = input.u8()? & 7;
        self.depth_image = input.u32()? & 0x00ff_ffff;
        self.primitive_depth = input.u16()? & 0x7fff;
        self.primitive_delta_z = input.u16()?;
        self.combine_mode = input.u64()?;
        for value in &mut self.convert_k {
            *value = input.u16()? as i16;
        }
        for value in &mut self.key_center {
            *value = input.u8()?;
        }
        for value in &mut self.key_scale {
            *value = input.u8()?;
        }
        for value in &mut self.key_width {
            *value = input.u16()? & 0x0fff;
        }
        let tmem = input.blob()?;
        if tmem.len() != self.tmem.len() {
            return Err("N64 TMEM state size mismatch".into());
        }
        self.tmem.copy_from_slice(tmem);
        for value in &mut self.tlut {
            *value = input.u16()?;
        }
        for tile in &mut self.tiles {
            tile.format = input.u8()? & 7;
            tile.size = input.u8()? & 3;
            tile.line = input.u16()? & 0x01ff;
            tile.tmem = input.u16()? & 0x01ff;
            tile.palette = input.u8()? & 0x0f;
            tile.clamp_t = input.u8()? != 0;
            tile.mirror_t = input.u8()? != 0;
            tile.mask_t = input.u8()? & 0x0f;
            tile.shift_t = input.u8()? & 0x0f;
            tile.clamp_s = input.u8()? != 0;
            tile.mirror_s = input.u8()? != 0;
            tile.mask_s = input.u8()? & 0x0f;
            tile.shift_s = input.u8()? & 0x0f;
            tile.sl = input.u16()? & 0x0fff;
            tile.tl = input.u16()? & 0x0fff;
            tile.sh = input.u16()? & 0x0fff;
            tile.th = input.u16()? & 0x0fff;
        }
        self.rdp_noise_seed = input.u32()?;
        self.rdp_pipeline_crashed = input.u8()? != 0;
        self.frame = input.u64()?;
        self.audio_samples.clear();
        Ok(())
    }
}

struct CpuBus<'a> {
    board: &'a mut N64Board,
}
impl Mips64Bus for CpuBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        self.board.read8(address)
    }
    fn write8(&mut self, address: u32, value: u8) {
        self.board.write8(address, value);
    }
    fn read32(&mut self, address: u32) -> u32 {
        self.board.read32(address)
    }
    fn write32(&mut self, address: u32, value: u32) {
        self.board.write32(address, value);
    }

    fn write_sized(&mut self, address: u32, width: u8, value: u64) {
        if (0x0400_0000..=0x04ff_ffff).contains(&address) {
            let word = match width {
                8 => (value >> 32) as u32,
                4 => value as u32,
                1 | 2 => {
                    let lane = 4u32
                        .saturating_sub(u32::from(width))
                        .saturating_sub(address & 3);
                    (value as u32) << (lane * 8)
                }
                _ => return,
            };
            self.board.write32(address & !3, word);
            return;
        }

        match width {
            1 => self.board.write8(address, value as u8),
            2 => {
                let bytes = (value as u16).to_be_bytes();
                self.board.write8(address, bytes[0]);
                self.board.write8(address.wrapping_add(1), bytes[1]);
            }
            4 => self.board.write32(address, value as u32),
            8 => {
                self.board.write32(address, (value >> 32) as u32);
                self.board.write32(address.wrapping_add(4), value as u32);
            }
            _ => {}
        }
    }
}
struct SignalBus<'a> {
    board: &'a mut N64Board,
}
impl RspBus for SignalBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        self.board.sp_mem[address as usize & 0x1fff]
    }
    fn write8(&mut self, address: u32, value: u8) {
        self.board.sp_mem[address as usize & 0x1fff] = value;
    }
    fn read_cop0(&mut self, index: u8) -> u32 {
        match index {
            0 => self.board.sp_mem_addr_read(),
            1 => self.board.sp_dram_addr_read(),
            2 | 3 => self.board.sp_dma_len_read(),
            4 => self.board.sp_status(),
            5 => u32::from(self.board.sp.dma_full),
            6 => u32::from(self.board.sp.dma_busy),
            7 => {
                let value = u32::from(self.board.sp.semaphore);
                self.board.sp.semaphore = true;
                value
            }
            8 => self.board.dp.start,
            9 => self.board.dp.end,
            10 => self.board.dp.current,
            11 => self.board.dp.status,
            12 => self.board.dp.clock,
            _ => 0,
        }
    }
    fn write_cop0(&mut self, index: u8, value: u32) {
        match index {
            0 => self.board.sp.mem_addr = value & 0x1ff8,
            1 => self.board.sp.dram_addr = value & 0x00ff_fff8,
            2 => {
                self.board.sp.rd_len = value;
                self.board.sp_dma(value, true);
            }
            3 => {
                self.board.sp.wr_len = value;
                self.board.sp_dma(value, false);
            }
            4 => self.board.write_sp_status(value),
            7 => self.board.sp.semaphore = false,
            8 => self.board.write_dp_start(value),
            9 => self.board.write_dp_end(value),
            11 => self.board.write_dp_status(value),
            _ => {}
        }
    }
}

pub struct Nintendo64Machine {
    cpu: MipsR4300,
    rsp: Rsp,
    board: N64Board,
    video: VideoBuffer,
    audio: AudioBuffer,
    frame_phase: u64,
    rsp_phase: u64,
}

impl Nintendo64Machine {
    pub fn from_rom(image: &[u8]) -> Result<Self, String> {
        let board = N64Board::new(image)?;
        let mut machine = Self {
            cpu: MipsR4300::default(),
            rsp: Rsp::new(),
            board,
            video: VideoBuffer::new(WIDTH, HEIGHT),
            audio: AudioBuffer::new(AUDIO_RATE, 2),
            frame_phase: 0,
            rsp_phase: 0,
        };
        machine.boot_cpu();
        Ok(machine)
    }

    fn boot_cpu(&mut self) {
        let cic_seed = cic_seed_for_rom(&self.board.rom);
        let tv_type = tv_type_for_rom(&self.board.rom);
        self.boot_cpu_with_profile(cic_seed, tv_type);
    }

    fn boot_cpu_with_profile(&mut self, cic_seed: u8, tv_type: u64) {
        self.cpu.reset_to(0xffff_ffff_a400_0040);
        self.cpu.regs.fill(0);
        self.cpu.regs[2] = 0xffff_ffff_d173_1be9;
        self.cpu.regs[3] = 0xffff_ffff_d173_1be9;
        self.cpu.regs[4] = 0x0000_0000_0000_1be9;
        self.cpu.regs[5] = 0xffff_ffff_f452_31e5;
        self.cpu.regs[6] = 0xffff_ffff_a400_1f0c;
        self.cpu.regs[7] = 0xffff_ffff_a400_1f08;
        self.cpu.regs[8] = 0x0000_0000_0000_00c0;
        self.cpu.regs[10] = 0x0000_0000_0000_0040;
        self.cpu.regs[11] = 0xffff_ffff_a400_0040;
        self.cpu.regs[12] = 0xffff_ffff_d133_0bc3;
        self.cpu.regs[13] = 0xffff_ffff_d133_0bc3;
        self.cpu.regs[14] = 0x0000_0000_2561_3a26;
        self.cpu.regs[15] = 0x0000_0000_2ea0_4317;
        self.cpu.regs[20] = tv_type.min(2);
        self.cpu.regs[22] = u64::from(cic_seed);
        self.cpu.regs[23] = 6;
        self.cpu.regs[25] = 0xffff_ffff_d73f_2993;
        self.cpu.regs[29] = 0xffff_ffff_a400_1ff0;
        self.cpu.regs[31] = 0xffff_ffff_a400_1554;
        self.cpu.cop0[1] = 31;
        self.cpu.cop0[12] = 0x3400_0000;
        self.cpu.cop0[15] = 0x0000_0b00;
        self.cpu.cop0[16] = 0x0006_e463;

        if cic_seed == 0x91 {
            const CIC_6105_IMEM: [u32; 8] = [
                0x3c0d_bfc0,
                0x8da8_07fc,
                0x25ad_07c0,
                0x3108_0080,
                0x5500_fffc,
                0x3c0d_bfc0,
                0x8da8_0024,
                0x3c0b_b000,
            ];
            for (index, word) in CIC_6105_IMEM.into_iter().enumerate() {
                let offset = 0x1000 + index * 4;
                self.board.sp_mem[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
            }
        }

        self.rsp.reset();
        self.board.sp.status = 1;
        self.board.sp.pc = 0;
    }

    fn run_rsp_credit(&mut self) {
        while self.rsp_phase >= CPU_HZ {
            self.rsp_phase -= CPU_HZ;
            if self.board.sp.status & 1 == 0 {
                if self.rsp.broke {
                    self.rsp.broke = false;
                }
                if !self.rsp.running {
                    self.rsp.set_pc(self.board.sp.pc);
                    self.rsp.running = true;
                }
                let used = {
                    let mut bus = SignalBus {
                        board: &mut self.board,
                    };
                    self.rsp.step(&mut bus)
                };
                self.board.sp.pc = self.rsp.pc;
                if used == 0 {
                    break;
                }
                if self.rsp.broke {
                    self.board.sp.status |= 3;
                    if self.board.sp.status & (1 << 6) != 0 {
                        self.board.raise(MI_SP);
                    }
                }
            }
        }
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        if self.board.audio_samples.is_empty() {
            for _ in 0..AUDIO_RATE as usize / FRAME_HZ as usize {
                self.audio.push_stereo(0.0, 0.0);
            }
        } else {
            for &(left, right) in &self.board.audio_samples {
                self.audio.push_stereo(left, right);
            }
        }
    }
}

impl Machine for Nintendo64Machine {
    fn platform(&self) -> PlatformId {
        PlatformId::Nintendo64
    }

    fn reset(&mut self) {
        self.board.reset();
        self.boot_cpu();
        self.frame_phase = 0;
        self.rsp_phase = 0;
        self.video.clear([0, 0, 0, 255]);
        self.flush_audio();
    }

    fn run_frame(&mut self, input: &InputState) {
        self.board.set_input(input);
        self.board.audio_samples.clear();
        self.frame_phase = self.frame_phase.wrapping_add(CPU_HZ);
        let budget = self.frame_phase / FRAME_HZ;
        self.frame_phase %= FRAME_HZ;
        for _ in 0..budget {
            self.cpu.set_irq_line(2, self.board.cpu_irq());
            let used = {
                let mut bus = CpuBus {
                    board: &mut self.board,
                };
                self.cpu.step(&mut bus)
            };
            self.board.tick_cartridge(used);
            self.board.tick_ai(used);
            self.board.tick_pi(used);
            self.board.tick_si(used);
            self.board.tick_sp_dma(used);
            self.rsp_phase = self.rsp_phase.wrapping_add(RSP_HZ);
            self.run_rsp_credit();
        }
        self.board.end_frame(&mut self.video);
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_HZ as f64
    }
    fn video(&self) -> &VideoBuffer {
        &self.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Nintendo64, STATE_VERSION);
        self.cpu.save(&mut out);
        self.rsp.save(&mut out);
        self.board.save(&mut out);
        out.u64(self.frame_phase);
        out.u64(self.rsp_phase);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Nintendo64, STATE_VERSION)?;
        self.cpu.load_state(&mut input)?;
        self.rsp.load_state(&mut input)?;
        self.board.load(&mut input)?;
        self.frame_phase = input.u64()?;
        self.rsp_phase = input.u64()?;
        input.finish()?;
        self.board.render_vi(&mut self.video);
        self.flush_audio();
        Ok(())
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        match (kind, slot) {
            (ResourceKind::Storage, 0) => SAVE_SIZE,
            (ResourceKind::Storage, 1) if self.board.rtc_present => RTC_SIZE,
            (ResourceKind::MemoryCard, 0) => CONTROLLER_PAK_SIZE,
            _ => 0,
        }
    }
    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        let source: &[u8] = match (kind, slot) {
            (ResourceKind::Storage, 0) => self.board.save.as_ref(),
            (ResourceKind::Storage, 1) if self.board.rtc_present => &self.board.rtc_ram,
            (ResourceKind::MemoryCard, 0) => self.board.controller_pak.as_ref(),
            _ => return Err("N64 persistent resource slot is unavailable".into()),
        };
        if out.len() != source.len() {
            return Err("N64 persistent resource size mismatch".into());
        }
        out.copy_from_slice(source);
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        let target: &mut [u8] = match (kind, slot) {
            (ResourceKind::Storage, 0) => self.board.save.as_mut(),
            (ResourceKind::Storage, 1) if self.board.rtc_present => &mut self.board.rtc_ram,
            (ResourceKind::MemoryCard, 0) => self.board.controller_pak.as_mut(),
            _ => return Err("N64 persistent resource slot is unavailable".into()),
        };
        if data.len() != target.len() {
            return Err("N64 persistent resource size mismatch".into());
        }
        target.copy_from_slice(data);
        if kind == ResourceKind::Storage && slot == 1 {
            self.board.sync_rtc_control();
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rom_with_loop() -> Vec<u8> {
        let mut rom = vec![0; 0x4000];
        rom[0..4].copy_from_slice(&0x8037_1240u32.to_be_bytes());
        let loop_branch = 0x1000_ffffu32;
        rom[0x40..0x44].copy_from_slice(&loop_branch.to_be_bytes());
        rom[0x44..0x48].copy_from_slice(&0u32.to_be_bytes());
        rom
    }

    fn rom_with_id(id: &[u8; 3]) -> Vec<u8> {
        let mut rom = rom_with_loop();
        rom[0x3b..0x3e].copy_from_slice(id);
        rom
    }

    fn rom_with_region(region: u8) -> Vec<u8> {
        let mut rom = rom_with_loop();
        rom[0x3e] = region;
        rom
    }

    fn simple_triangle_edges(opcode: u8) -> [u32; 8] {
        [
            (u32::from(opcode) << 24) | 16,
            16u32 << 16,
            0,
            0,
            0,
            1u32 << 16,
            0,
            0,
        ]
    }

    fn constant_red_shade() -> [u32; 16] {
        [
            0x00ff_0000,
            0x0000_00ff,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]
    }

    fn constant_texture(s: u16, t: u16, w: u16) -> [u32; 16] {
        [
            (u32::from(s) << 16) | u32::from(t),
            u32::from(w) << 16,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ]
    }

    fn write_rdp_words(board: &mut N64Board, offset: usize, words: &[u32]) {
        for (index, word) in words.iter().copied().enumerate() {
            let address = offset + index * 4;
            board.rdram[address..address + 4].copy_from_slice(&word.to_be_bytes());
        }
    }

    fn combine_one_cycle(rgb: [u8; 4], alpha: [u8; 4]) -> u64 {
        let [a, b, c, d] = rgb;
        let [aa, ab, ac, ad] = alpha;
        (u64::from(a) << 52)
            | (u64::from(c) << 47)
            | (u64::from(aa) << 44)
            | (u64::from(ac) << 41)
            | (u64::from(a) << 37)
            | (u64::from(c) << 32)
            | (u64::from(b) << 28)
            | (u64::from(b) << 24)
            | (u64::from(aa) << 21)
            | (u64::from(ac) << 18)
            | (u64::from(d) << 15)
            | (u64::from(ab) << 12)
            | (u64::from(ad) << 9)
            | (u64::from(d) << 6)
            | (u64::from(ab) << 3)
            | u64::from(ad)
    }

    #[test]
    fn rdp_one_cycle_combiner_modulates_texel_and_shade() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.other_modes = 3u64 << 38;
        board.combine_mode = combine_one_cycle([1, 15, 4, 7], [1, 7, 4, 7]);
        let rgba = board
            .rdp_process_pixel(
                0,
                0,
                [[200, 100, 50, 128]; 2],
                [128, 255, 64, 128],
                [0; 4],
                0,
            )
            .unwrap();
        assert_eq!(rgba, [100, 100, 13, 64]);
    }

    #[test]
    fn rdp_combiner_uses_hardware_nine_bit_fixed_point_math() {
        assert_eq!(N64Board::rdp_combiner_value(0x100, 0, 128, 0), 128);
        assert_eq!(N64Board::rdp_combiner_value(0x100, 0, 255, 0), 255);
        assert_eq!(N64Board::rdp_combiner_value(200, 100, 128, 100), 150);
        assert_eq!(N64Board::rdp_clamp_9bit(300), 0xff);
        assert_eq!(N64Board::rdp_clamp_9bit(-10), 0);

        let board = N64Board::new(&rom_with_loop()).unwrap();
        let sources = [[0; 4]; 4];
        assert_eq!(board.rdp_rgb_mux(6, 0, 0, &sources, 0, 0), 0x100);
        assert_eq!(board.rdp_rgb_mux(6, 3, 0, &sources, 0, 0), 0x100);
        assert_eq!(
            board.rdp_alpha_mux(6, false, [0; 4], [[0; 4]; 2], [0; 4], 0),
            0x100
        );
    }

    #[test]
    fn rdp_combiner_distinguishes_texel_inputs_and_swaps_them_for_cycle_one() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let texel0 = [255, 0, 0, 255];
        let texel1 = [0, 255, 0, 128];

        board.combine_mode = (2u64 << 15) | (2u64 << 9);
        assert_eq!(
            board
                .rdp_process_pixel(0, 0, [texel0, texel1], [255; 4], [0; 4], 0)
                .unwrap(),
            texel1
        );

        board.other_modes = 1u64 << 52;
        board.combine_mode = (1u64 << 6) | 1;
        assert_eq!(
            board
                .rdp_process_pixel(0, 0, [texel0, texel1], [255; 4], [0; 4], 0)
                .unwrap(),
            texel1
        );
    }

    #[test]
    fn rdp_alpha_threshold_uses_blend_color_alpha() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.combine_mode = combine_one_cycle([15, 15, 31, 1], [7, 7, 7, 1]);
        board.blend_color = 0x0000_0080;
        board.other_modes = 1;
        assert!(board
            .rdp_process_pixel(0, 0, [[255, 0, 0, 64]; 2], [255; 4], [255, 0, 0, 64], 0,)
            .is_none());
        assert_eq!(
            board
                .rdp_process_pixel(0, 0, [[255, 0, 0, 200]; 2], [255; 4], [255, 0, 0, 200], 0,)
                .unwrap(),
            [255, 0, 0, 200]
        );
    }

    #[test]
    fn rdp_alpha_coverage_modes_feed_adjusted_count_into_depth() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.combine_mode = 0;
        board.other_modes = (3u64 << 38) | (3u64 << 36) | (1u64 << 12);

        let input = RdpPixelInputs {
            texels: [[0; 4]; 2],
            shade: [255; 4],
            legacy: [10, 20, 30, 128],
            lod_frac: 0,
        };
        let prepared = board.rdp_prepare_pixel(0, 0, input, 4);
        assert_eq!(prepared.color[3], 128);
        assert_eq!(prepared.coverage_count, 2);

        let depth = board.rdp_blend_state_without_depth(prepared.coverage_count, 5);
        assert!(!depth.coverage_wrap);

        board.other_modes |= 1u64 << 13;
        let prepared = board.rdp_prepare_pixel(0, 0, input, 4);
        assert_eq!(prepared.coverage_count, 2);
        assert_eq!(prepared.color[3], 64);

        board.other_modes &= !(1u64 << 12);
        let prepared = board.rdp_prepare_pixel(0, 0, input, 3);
        assert_eq!(prepared.coverage_count, 3);
        assert_eq!(prepared.color[3], 96);

        let prepared = board.rdp_prepare_pixel(0, 0, input, 8);
        assert_eq!(prepared.color[3], 255);
    }

    #[test]
    fn rdp_alpha_dither_modes_and_rng_order_match_angrylion() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();

        board.other_modes = 0;
        assert_eq!(board.rdp_dither_signals(1, 2, false).alpha, 5);

        board.other_modes = 1u64 << 36;
        assert_eq!(board.rdp_dither_signals(1, 2, false).alpha, 2);

        board.rdp_noise_seed = 1;
        board.other_modes = (3u64 << 38) | (2u64 << 36);
        let signals = board.rdp_dither_signals(0, 0, false);
        assert_eq!(signals.rgb, [7; 3]);
        assert_eq!(signals.noise, 0x60);
        assert_eq!(signals.alpha, 1);
        assert_eq!(board.rdp_noise_seed, 0x0029_e2c0);

        board.rdp_noise_seed = 1;
        board.other_modes = (2u64 << 38) | (2u64 << 36);
        let signals = board.rdp_dither_signals(0, 0, false);
        assert_eq!(signals.noise, 0x60);
        assert_eq!(signals.alpha, 1);
        assert_eq!(signals.rgb, [3, 4, 0]);

        board.other_modes = (3u64 << 38) | (3u64 << 36);
        let input = RdpPixelInputs {
            texels: [[0; 4]; 2],
            shade: [0, 0, 0, 0x22],
            legacy: [1, 2, 3, 0x22],
            lod_frac: 0,
        };
        let prepared = board.rdp_prepare_pixel(1, 2, input, 8);
        assert_eq!(prepared.color[3], 0x22);
        assert_eq!(prepared.shade[3], 0x22);

        board.other_modes = 0;
        let prepared = board.rdp_prepare_pixel(1, 2, input, 8);
        assert_eq!(prepared.color[3], 0x27);
        assert_eq!(prepared.shade[3], 0x27);
    }

    #[test]
    fn rdp_noise_lcg_and_rgb_dither_match_angrylion() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        assert_eq!(board.rdp_next_random(), 41);
        assert_eq!(board.rdp_next_random(), 18_467);
        assert_eq!(board.rdp_next_random(), 6_334);

        board.rdp_noise_seed = 1;
        board.other_modes = 0;
        assert_eq!(board.rdp_dither_signals(1, 2, false).rgb, [5; 3]);
        assert_eq!(
            N64Board::rdp_apply_rgb_dither([0x11, 0x22, 0x33, 0xff], [5; 3]),
            [0x11, 0x22, 0x33, 0xff]
        );
        assert_eq!(
            N64Board::rdp_apply_rgb_dither([0x11, 0x22, 0x33, 0xff], [2; 3]),
            [0x11, 0x22, 0x38, 0xff]
        );
        assert_eq!(
            N64Board::rdp_apply_rgb_dither([0x11, 0x22, 0x33, 0xff], [0; 3]),
            [0x18, 0x28, 0x38, 0xff]
        );

        board.other_modes = 1u64 << 38;
        assert_eq!(board.rdp_dither_signals(1, 0, false).rgb, [4; 3]);

        board.rdp_noise_seed = 1;
        board.other_modes = 2u64 << 38;
        assert_eq!(board.rdp_dither_signals(0, 0, false).rgb, [1, 5, 0]);
        assert_eq!(board.rdp_noise_seed, 0x0029_e2c0);

        board.other_modes = 3u64 << 38;
        assert_eq!(board.rdp_dither_signals(0, 0, false).rgb, [7; 3]);
        assert_eq!(
            N64Board::rdp_apply_rgb_dither([0xff, 0xf8, 0, 0], [0; 3]),
            [0xff, 0xf8, 0, 0]
        );
    }

    #[test]
    fn rdp_color_on_coverage_selects_final_memory_side_until_wrap() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 1;
        board.color_size = 3;
        board.blend_color = 0x1234_5678;
        board.rdram[0x6000..0x6004].copy_from_slice(&[1, 2, 3, 0xe0]);

        let input = [200, 10, 20, 128];
        let mut state = RdpPixelState {
            depth: RdpDepthResult::default(),
            memory_coverage: 7,
        };

        board.other_modes = u64::from((2u32 << 22) | (1 << 7));
        assert_eq!(
            board.rdp_blend(0, 0, input, [255; 4], state),
            [0x12, 0x34, 0x56, 128]
        );

        state.depth.coverage_wrap = true;
        assert_eq!(board.rdp_blend(0, 0, input, [255; 4], state), input);

        state.depth.coverage_wrap = false;
        board.other_modes = (1u64 << 52) | u64::from((2u32 << 20) | (1 << 7));
        assert_eq!(
            board.rdp_blend(0, 0, input, [255; 4], state),
            [0x12, 0x34, 0x56, 128]
        );
    }

    #[test]
    fn rdp_fill_rectangle_copy_cycle_writes_raw_u16_qwords() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 8;
        board.color_size = 2;
        board.color_format = 0;
        board.other_modes = 2u64 << 52;
        board.tiles[0] = RdpTile {
            format: 0,
            size: 2,
            line: 1,
            mask_s: 2,
            ..RdpTile::default()
        };

        let words = [0x1111u16, 0x2220, 0x3331, 0x4441];
        for (index, word) in words.iter().copied().enumerate() {
            board.tmem[index * 2..index * 2 + 2].copy_from_slice(&word.to_be_bytes());
        }

        board.rdp_fill_rectangle(28 << 12, 0);
        for x in 0..8usize {
            let address = 0x6000 + x * 2;
            assert_eq!(
                u16::from_be_bytes([board.rdram[address], board.rdram[address + 1]]),
                words[x & 3]
            );
        }

        board.rdram[0x6000..0x6010].fill(0xaa);
        board.other_modes |= 1;
        board.rdp_fill_rectangle(28 << 12, 0);
        for x in 0..8usize {
            let address = 0x6000 + x * 2;
            let expected = if x & 3 == 1 { 0xaaaa } else { words[x & 3] };
            assert_eq!(
                u16::from_be_bytes([board.rdram[address], board.rdram[address + 1]]),
                expected
            );
        }
    }

    #[test]
    fn rdp_fill_rectangle_copy_cycle_supports_tlut_and_target_size_rules() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 4;
        board.color_size = 2;
        board.other_modes = (2u64 << 52) | (1u64 << 47);
        board.tiles[0] = RdpTile {
            format: 2,
            size: 1,
            line: 1,
            mask_s: 2,
            ..RdpTile::default()
        };
        board.tmem[0..4].copy_from_slice(&[1, 2, 3, 4]);
        board.tlut[1] = 0x1001;
        board.tlut[2] = 0x2001;
        board.tlut[3] = 0x3001;
        board.tlut[4] = 0x4001;

        board.rdp_fill_rectangle(12 << 12, 0);
        for (x, expected) in [0x1001u16, 0x2001, 0x3001, 0x4001].into_iter().enumerate() {
            let address = 0x6000 + x * 2;
            assert_eq!(
                u16::from_be_bytes([board.rdram[address], board.rdram[address + 1]]),
                expected
            );
        }

        board.color_image = 0x6100;
        board.color_width = 1;
        board.color_size = 0;
        board.rdram[0x6100] = 0xa5;
        board.rdp_pipeline_crashed = false;
        board.rdp_fill_rectangle(0, 0);
        assert_eq!(board.rdram[0x6100], 0);
        assert!(!board.rdp_pipeline_crashed);

        board.color_image = 0x6200;
        board.color_size = 3;
        board.rdp_pipeline_crashed = false;
        board.rdp_fill_rectangle(0, 0);
        assert!(board.rdp_pipeline_crashed);
    }

    #[test]
    fn rdp_fill_rectangle_normal_cycle_uses_subpixel_coverage() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_width = 4;
        board.scissor = [0, 0, 4, 2];
        board.scissor_fixed = [0, 0, 16, 8];
        board.other_modes = 1u64 << 3;

        assert_eq!(board.rdp_rectangle_coverage(1, 0, 8, 4, 0, 0), Some(6));
        assert_eq!(board.rdp_rectangle_coverage(1, 0, 8, 4, 1, 0), Some(8));
        assert_eq!(board.rdp_rectangle_coverage(1, 0, 8, 4, 2, 0), None);

        board.other_modes = 0;
        assert_eq!(board.rdp_rectangle_coverage(1, 0, 8, 4, 0, 0), None);
        assert_eq!(board.rdp_rectangle_coverage(0, 0, 8, 4, 0, 0), Some(8));
    }

    #[test]
    fn rdp_fill_rectangle_normal_cycles_use_combiner_not_fill_color() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 4;
        board.color_size = 2;
        board.color_format = 0;
        board.scissor = [0, 0, 4, 1];
        board.scissor_fixed = [0, 0, 16, 4];
        board.fill_color = 0x07c1_07c1;
        board.primitive_color = 0xf800_00ff;
        board.combine_mode = combine_one_cycle([15, 15, 31, 3], [7, 7, 7, 3]);
        board.other_modes = 3u64 << 38;

        board.rdp_fill_rectangle((16 << 12) | 4, 0);

        for x in 0..4usize {
            let offset = 0x6000 + x * 2;
            assert_eq!(
                u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]]),
                0xf801
            );
        }

        board.color_image = 0x6100;
        board.color_width = 1;
        board.color_size = 0;
        board.scissor = [0, 0, 1, 1];
        board.scissor_fixed = [0, 0, 4, 4];
        board.rdram[0x6100] = 0xa5;
        board.rdp_fill_rectangle((4 << 12) | 4, 0);
        assert_eq!(board.rdram[0x6100], 0);
        assert!(!board.rdp_pipeline_crashed);
    }

    #[test]
    fn rdp_fill_mode_crash_order_matches_angrylion() {
        let render = |modes: u64| {
            let mut board = N64Board::new(&rom_with_loop()).unwrap();
            board.color_image = 0x6000;
            board.color_width = 1;
            board.color_size = 2;
            board.fill_color = 0xf801_f801;
            board.scissor = [0, 0, 1, 1];
            board.scissor_fixed = [0, 0, 4, 4];
            board.other_modes = (3u64 << 52) | modes;
            board.rdp_fill_rectangle(0, 0);
            (
                board.rdp_pipeline_crashed,
                u16::from_be_bytes([board.rdram[0x6000], board.rdram[0x6001]]),
            )
        };

        assert_eq!(render(0x0040), (true, 0));
        assert_eq!(render(0x0010), (true, 0));
        assert_eq!(render(0x0020), (true, 0xf801));
        assert_eq!(render(0x0024), (false, 0xf801));
    }

    #[test]
    fn rdp_pipeline_crash_discards_later_command_lists_until_reset() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 4;
        board.color_size = 0;
        board.scissor = [0, 0, 4, 1];
        board.scissor_fixed = [0, 0, 16, 4];
        board.other_modes = 3u64 << 52;
        board.rdram[0x6000..0x6004].fill(0xa5);

        let base = 0x1000usize;
        let words = [
            0x37u32 << 24,
            0x1111_1111,
            (0x36u32 << 24) | (12 << 12),
            0,
            0x37u32 << 24,
            0x2222_2222,
        ];
        write_rdp_words(&mut board, base, &words);
        board.dp.current = base as u32;
        board.dp.end = (base + words.len() * 4) as u32;
        board.run_rdp();

        assert!(board.rdp_pipeline_crashed);
        assert_eq!(board.dp.current, board.dp.end);
        assert_eq!(board.fill_color, 0x1111_1111);
        assert_eq!(&board.rdram[0x6000..0x6004], &[0xa5; 4]);

        let next = base + 0x100;
        write_rdp_words(&mut board, next, &[0x37u32 << 24, 0x3333_3333]);
        board.dp.current = next as u32;
        board.dp.end = (next + 8) as u32;
        board.run_rdp();
        assert_eq!(board.fill_color, 0x1111_1111);
        assert_eq!(board.dp.current, board.dp.end);

        board.reset();
        assert!(!board.rdp_pipeline_crashed);
    }

    #[test]
    fn rdp_fill_writes_hardware_hidden_metadata() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 2;

        board.color_size = 2;
        board.fill_color = 0xf801_07c0;
        board.write_rdp_fill_pixel(0, 0);
        board.write_rdp_fill_pixel(1, 0);
        assert_eq!(board.rdp_hidden_read(0x6000), 3);
        assert_eq!(board.rdp_hidden_read(0x6002), 0);

        board.color_size = 3;
        board.fill_color = 0x1123_4454;
        board.write_rdp_fill_pixel(0, 0);
        assert_eq!(board.rdp_hidden_read(0x6000), 3);
        assert_eq!(board.rdp_hidden_read(0x6002), 0);
    }

    #[test]
    fn rdp_blender_uses_fixed_weights_and_chains_two_cycles() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 1;
        board.color_size = 3;
        board.rdram[0x6000..0x6004].copy_from_slice(&[0, 0, 255, 255]);

        let input = [255, 0, 0, 128];
        let memory = [0, 0, 255, 255];
        assert_eq!(
            board.rdp_blend_cycle([0, 0, 1, 0], input, [255; 4], memory, true, [0; 2]),
            [127, 0, 127]
        );

        board.other_modes = (1u64 << 52) | u64::from((1u32 << 22) | (1u32 << 20) | (1u32 << 14));
        assert_eq!(
            board.rdp_blend(
                0,
                0,
                input,
                [255; 4],
                RdpPixelState {
                    depth: RdpDepthResult {
                        blend_en: true,
                        ..RdpDepthResult::default()
                    },
                    memory_coverage: 7,
                },
            ),
            [63, 0, 191, 128]
        );
    }

    #[test]
    fn rdp_force_blend_mixes_pipeline_and_memory_color() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 1;
        board.color_size = 3;
        board.rdram[0x6000..0x6004].copy_from_slice(&[0, 0, 255, 255]);
        board.other_modes = u64::from(0x0040_4000u32);

        let rgba = board
            .rdp_process_pixel(0, 0, [[0; 4]; 2], [255; 4], [255, 0, 0, 128], 0)
            .unwrap();
        assert!((127..=129).contains(&rgba[0]));
        assert_eq!(rgba[1], 0);
        assert!((126..=128).contains(&rgba[2]));
        assert_eq!(rgba[3], 128);
    }

    #[test]
    fn rom_byte_orders_normalize_to_big_endian() {
        let z64 = rom_with_loop();
        let mut v64 = z64.clone();
        for pair in v64.as_chunks_mut::<2>().0 {
            pair.swap(0, 1);
        }
        let mut n64 = z64.clone();
        for word in n64.as_chunks_mut::<4>().0 {
            word.reverse();
        }
        assert_eq!(normalize_rom(&v64).unwrap(), z64);
        assert_eq!(normalize_rom(&n64).unwrap(), z64);
    }

    #[test]
    fn cic_crc_profiles_select_expected_seeds() {
        assert_eq!(cic_seed_for_crc(0x6170_a4a1), 0x3f);
        assert_eq!(cic_seed_for_crc(0x90bb_6cb5), 0x3f);
        assert_eq!(cic_seed_for_crc(0x0b05_0ee0), 0x78);
        assert_eq!(cic_seed_for_crc(0x98bc_2c86), 0x91);
        assert_eq!(cic_seed_for_crc(0xacc8_580a), 0x85);
        assert_eq!(cic_seed_for_crc(0x1234_5678), 0x3f);
    }

    #[test]
    fn pif_hle_boot_sets_tv_type_from_country_code() {
        for (region, tv_type) in [(b'E', 1), (b'J', 1), (b'P', 0), (b'U', 0), (b'B', 2)] {
            let machine = Nintendo64Machine::from_rom(&rom_with_region(region)).unwrap();
            assert_eq!(machine.cpu.regs[20], tv_type);
        }
    }

    #[test]
    fn cic_6105_profile_populates_seed_and_sp_imem_helper() {
        let rom = rom_with_loop();
        let mut machine = Nintendo64Machine::from_rom(&rom).unwrap();
        machine.boot_cpu_with_profile(0x91, 1);

        assert_eq!(machine.cpu.regs[22], 0x91);
        let expected = [
            0x3c0d_bfc0u32,
            0x8da8_07fc,
            0x25ad_07c0,
            0x3108_0080,
            0x5500_fffc,
            0x3c0d_bfc0,
            0x8da8_0024,
            0x3c0b_b000,
        ];
        for (index, word) in expected.into_iter().enumerate() {
            let offset = 0x1000 + index * 4;
            assert_eq!(
                u32::from_be_bytes(machine.board.sp_mem[offset..offset + 4].try_into().unwrap()),
                word
            );
        }
    }

    #[test]
    fn pif_hle_boot_copies_ipl3_and_initializes_cpu() {
        let rom = rom_with_loop();
        let mut machine = Nintendo64Machine::from_rom(&rom).unwrap();
        assert_eq!(machine.cpu.pc, 0xffff_ffff_a400_0040);
        assert_eq!(machine.cpu.regs[2], 0xffff_ffff_d173_1be9);
        assert_eq!(machine.cpu.regs[5], 0xffff_ffff_f452_31e5);
        assert_eq!(machine.cpu.regs[6], 0xffff_ffff_a400_1f0c);
        assert_eq!(machine.cpu.regs[11], 0xffff_ffff_a400_0040);
        assert_eq!(machine.cpu.regs[14], 0x0000_0000_2561_3a26);
        assert_eq!(machine.cpu.regs[20], 1);
        assert_eq!(machine.cpu.regs[22], 0x3f);
        assert_eq!(machine.cpu.regs[23], 6);
        assert_eq!(machine.cpu.regs[25], 0xffff_ffff_d73f_2993);
        assert_eq!(machine.cpu.regs[29], 0xffff_ffff_a400_1ff0);
        assert_eq!(machine.cpu.regs[31], 0xffff_ffff_a400_1554);
        assert_eq!(&machine.board.sp_mem[..0x1000], &rom[..0x1000]);
        let mut bus = CpuBus {
            board: &mut machine.board,
        };
        machine.cpu.step(&mut bus);
        machine.cpu.step(&mut bus);
        assert_eq!(machine.cpu.pc, 0xffff_ffff_a400_0040);
    }
    #[test]
    fn sp_memory_window_mirrors_until_register_block() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.write32(0x0400_0000, 0x0123_4567);
        board.write32(0x0400_1000, 0x89ab_cdef);
        board.write32(0x0403_e000, 0x7654_3210);

        assert_eq!(board.read32(0x0400_0000), 0x7654_3210);
        assert_eq!(board.read32(0x0400_1000), 0x89ab_cdef);
        assert_eq!(board.read32(0x0403_e000), 0x7654_3210);
    }

    #[test]
    fn rcp_internal_narrow_and_doubleword_stores_are_size_blind() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let mut bus = CpuBus { board: &mut board };

        for offset in [0u32, 5, 10, 15] {
            bus.write_sized(0x0400_0000 + offset, 1, 0x1234_5678);
        }
        assert_eq!(bus.read32(0x0400_0000), 0x7800_0000);
        assert_eq!(bus.read32(0x0400_0004), 0x5678_0000);
        assert_eq!(bus.read32(0x0400_0008), 0x3456_7800);
        assert_eq!(bus.read32(0x0400_000c), 0x1234_5678);

        bus.write32(0x0400_0000, 0xdead_beef);
        bus.write32(0x0400_0004, 0xbadd_ecaf);
        bus.write_sized(0x0400_0000, 2, 0x1234_5678);
        bus.write_sized(0x0400_0006, 2, 0x1234_5678);
        assert_eq!(bus.read32(0x0400_0000), 0x5678_0000);
        assert_eq!(bus.read32(0x0400_0004), 0x1234_5678);

        bus.write32(0x0400_0000, 0xdead_beef);
        bus.write32(0x0400_0004, 0xbadd_ecaf);
        bus.write_sized(0x0400_0000, 8, 0xabcd_ef98_7654_3210);
        assert_eq!(bus.read32(0x0400_0000), 0xabcd_ef98);
        assert_eq!(bus.read32(0x0400_0004), 0xbadd_ecaf);
    }

    #[test]
    fn sp_status_set_clear_pairs_leave_state_unchanged() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.sp.status = 1 | (1 << 5) | (1 << 6) | (1 << 7);
        board.raise(MI_SP);
        let original_status = board.sp.status;
        let paired = (1 << 0)
            | (1 << 1)
            | (1 << 3)
            | (1 << 4)
            | (1 << 5)
            | (1 << 6)
            | (1 << 7)
            | (1 << 8)
            | (1 << 9)
            | (1 << 10);

        board.write_sp_status(paired);

        assert_eq!(board.sp.status, original_status);
        assert_ne!(board.mi_intr & MI_SP, 0);
    }

    #[test]
    fn sp_dma_busy_full_and_pending_descriptor_follow_two_slot_queue() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.rdram[0x400..0x408].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        board.rdram[0x408..0x410].copy_from_slice(&[9, 10, 11, 12, 13, 14, 15, 16]);
        let first_before = board.sp_mem[..8].to_vec();
        let second_before = board.sp_mem[8..16].to_vec();

        board.write32(0x0404_0000, 0);
        board.write32(0x0404_0004, 0x400);
        board.write32(0x0404_0008, 7);
        assert!(board.sp.dma_busy);
        assert!(!board.sp.dma_full);
        assert_eq!(board.read32(0x0404_0018), 1);
        assert_eq!(board.sp_status() & 0x0c, 1 << 2);
        assert_eq!(board.sp_mem_addr_read(), 0);
        assert_eq!(board.sp_dram_addr_read(), 0x400);
        assert_eq!(&board.sp_mem[..8], first_before.as_slice());

        board.write32(0x0404_0000, 8);
        board.write32(0x0404_0004, 0x408);
        board.write32(0x0404_0008, 7);
        assert!(board.sp.dma_full);
        assert_eq!(board.read32(0x0404_0014), 1);
        assert_eq!(board.sp_status() & 0x0c, (1 << 2) | (1 << 3));
        assert_eq!(board.sp_mem_addr_read(), 0);
        assert_eq!(board.sp_dram_addr_read(), 0x400);

        board.tick_sp_dma(5);
        assert_eq!(&board.sp_mem[..8], &board.rdram[0x400..0x408]);
        assert_eq!(&board.sp_mem[8..16], second_before.as_slice());
        assert!(board.sp.dma_busy);
        assert!(!board.sp.dma_full);
        assert_eq!(board.sp_mem_addr_read(), 8);
        assert_eq!(board.sp_dram_addr_read(), 0x408);

        board.tick_sp_dma(5);
        assert_eq!(&board.sp_mem[8..16], &board.rdram[0x408..0x410]);
        assert!(!board.sp.dma_busy);
        assert_eq!(board.read32(0x0404_0018), 0);
        assert_eq!(board.sp.mem_addr, 16);
        assert_eq!(board.sp.dram_addr, 0x410);
        assert_eq!(board.sp_dma_len_read(), 0x0ff8);
    }

    #[test]
    fn sp_dma_wraps_inside_selected_dmem_or_imem_bank() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        for index in 0..16 {
            board.rdram[0x500 + index] = 0x80 + index as u8;
        }
        board.sp_mem[0x1000..0x1008].fill(0x55);
        board.sp.mem_addr = 0x0ff8;
        board.sp.dram_addr = 0x500;

        board.sp_dma(15, true);
        assert!(board.sp.dma_busy);
        assert_eq!(board.read32(0x0404_0018), 1);
        assert_eq!(&board.sp_mem[0x0ff8..0x1000], &[0; 8]);
        board.tick_sp_dma(16);

        assert_eq!(&board.sp_mem[0x0ff8..0x1000], &board.rdram[0x500..0x508]);
        assert_eq!(&board.sp_mem[..8], &board.rdram[0x508..0x510]);
        assert_eq!(&board.sp_mem[0x1000..0x1008], &[0x55; 8]);
        assert_eq!(board.sp.mem_addr, 0x0008);
    }

    #[test]
    fn sp_dma_finishes_with_terminal_length_and_advanced_addresses() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        for index in 0..8 {
            board.rdram[0x300 + index] = 0x10 + index as u8;
            board.rdram[0x310 + index] = 0x20 + index as u8;
        }
        board.sp.mem_addr = 0;
        board.sp.dram_addr = 0x300;

        board.sp_dma(7 | (1 << 12) | (8 << 20), true);
        assert!(board.sp.dma_busy);
        board.tick_sp_dma(16);

        assert_eq!(&board.sp_mem[..8], &board.rdram[0x300..0x308]);
        assert_eq!(&board.sp_mem[8..16], &board.rdram[0x310..0x318]);
        assert_eq!(board.sp.mem_addr, 16);
        assert_eq!(board.sp.dram_addr, 0x318);
        assert_eq!(board.sp_dma_len_read(), 0x0ff8);
        assert_eq!(board.sp_status() & (1 << 2), 0);
    }

    #[test]
    fn rsp_resumes_after_cpu_clears_halt_and_broke() {
        let mut machine = Nintendo64Machine::from_rom(&rom_with_loop()).unwrap();
        let break_instruction = 0x0000_000du32;
        let ori = (0x0du32 << 26) | (1 << 16) | 0x4321;
        machine.board.sp_mem[0x1000..0x1004].copy_from_slice(&break_instruction.to_be_bytes());
        machine.board.sp_mem[0x1004..0x1008].copy_from_slice(&ori.to_be_bytes());
        machine.board.sp_mem[0x1008..0x100c].copy_from_slice(&break_instruction.to_be_bytes());
        machine.rsp.reset();
        machine.board.sp.pc = 0;
        machine.board.sp.status = 0;
        machine.rsp_phase = CPU_HZ;

        machine.run_rsp_credit();

        assert!(machine.rsp.broke);
        assert_eq!(machine.board.sp.status & 3, 3);
        assert_eq!(machine.board.sp.pc, 4);

        machine.board.write_sp_status((1 << 0) | (1 << 2));
        machine.rsp_phase = CPU_HZ * 2;
        machine.run_rsp_credit();

        assert_eq!(machine.rsp.regs[1], 0x4321);
        assert!(machine.rsp.broke);
        assert_eq!(machine.board.sp.pc, 0x0c);
        assert_eq!(machine.board.sp.status & 3, 3);
    }

    #[test]
    fn sp_dma_and_rsp_break_share_board_state() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let ori = (0x0du32 << 26) | (1 << 16) | 0x1234;
        board.rdram[0x100..0x104].copy_from_slice(&ori.to_be_bytes());
        board.rdram[0x104..0x108].copy_from_slice(&0x0000_000du32.to_be_bytes());
        board.sp.mem_addr = 0x1000;
        board.sp.dram_addr = 0x100;
        board.sp_dma(7, true);
        assert!(board.sp.dma_busy);
        board.tick_sp_dma(8);
        assert_eq!(&board.sp_mem[0x1000..0x1008], &board.rdram[0x100..0x108]);
        let mut rsp = Rsp::new();
        rsp.running = true;
        {
            let mut bus = SignalBus { board: &mut board };
            rsp.step(&mut bus);
            rsp.step(&mut bus);
        }
        assert_eq!(rsp.regs[1], 0x1234);
        assert!(rsp.broke && !rsp.running);
    }

    #[test]
    fn pif_controller_state_and_controller_pak_round_trip() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.controllers[0] = FACE_SOUTH | START | UP | L1;
        board.controller_axes[0] = [16384, -16384];
        board.pif_ram[0..3].copy_from_slice(&[1, 4, 1]);
        board.pif_ram[7] = 0xfe;
        board.process_pif();
        assert_eq!(
            u16::from_be_bytes([board.pif_ram[3], board.pif_ram[4]]) & 0x9820,
            0x9820
        );
        assert!(board.pif_ram[5] > 0 && board.pif_ram[6] > 0);
    }
    #[test]
    fn pif_eeprom_4k_status_write_busy_and_read_round_trip() {
        let mut board = N64Board::new(&rom_with_id(b"NKT")).unwrap();
        assert_eq!(board.eeprom_size, 512);

        board.pif_ram.fill(0);
        board.pif_ram[4..7].copy_from_slice(&[1, 3, 0x00]);
        board.pif_ram[10] = 0xfe;
        board.process_pif();
        assert_eq!(&board.pif_ram[7..10], &[0x00, 0x80, 0x00]);

        board.pif_ram.fill(0);
        board.pif_ram[4] = 10;
        board.pif_ram[5] = 1;
        board.pif_ram[6] = 0x05;
        board.pif_ram[7] = 0x12;
        board.pif_ram[8..16].copy_from_slice(&[1, 2, 3, 4, 5, 6, 7, 8]);
        board.pif_ram[17] = 0xfe;
        board.process_pif();
        assert_eq!(board.pif_ram[16], 0);
        assert_eq!(&board.save[0x90..0x98], &[1, 2, 3, 4, 5, 6, 7, 8]);
        assert_eq!(board.eeprom_busy_cycles, EEPROM_WRITE_CYCLES);

        board.pif_ram.fill(0);
        board.pif_ram[4..7].copy_from_slice(&[1, 3, 0x00]);
        board.pif_ram[10] = 0xfe;
        board.process_pif();
        assert_eq!(board.pif_ram[9], 0x80);

        board.tick_cartridge(EEPROM_WRITE_CYCLES);
        board.pif_ram.fill(0);
        board.pif_ram[4] = 2;
        board.pif_ram[5] = 8;
        board.pif_ram[6] = 0x04;
        board.pif_ram[7] = 0x12;
        board.pif_ram[16] = 0xfe;
        board.process_pif();
        assert_eq!(&board.pif_ram[8..16], &[1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn pif_eeprom_16k_identifies_and_addresses_high_blocks() {
        let mut board = N64Board::new(&rom_with_id(b"NPD")).unwrap();
        assert_eq!(board.eeprom_size, 2 * 1024);

        board.pif_ram.fill(0);
        board.pif_ram[4..7].copy_from_slice(&[1, 3, 0x00]);
        board.pif_ram[10] = 0xfe;
        board.process_pif();
        assert_eq!(&board.pif_ram[7..10], &[0x00, 0xc0, 0x00]);

        board.pif_ram.fill(0);
        board.pif_ram[4] = 10;
        board.pif_ram[5] = 1;
        board.pif_ram[6] = 0x05;
        board.pif_ram[7] = 0xe0;
        board.pif_ram[8..16].copy_from_slice(&[8, 7, 6, 5, 4, 3, 2, 1]);
        board.pif_ram[17] = 0xfe;
        board.process_pif();
        assert_eq!(&board.save[0x700..0x708], &[8, 7, 6, 5, 4, 3, 2, 1]);
    }

    #[test]
    fn pif_eeprom_absent_sets_no_response_flag() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        assert_eq!(board.eeprom_size, 0);

        board.pif_ram.fill(0);
        board.pif_ram[4..7].copy_from_slice(&[1, 3, 0x00]);
        board.pif_ram[10] = 0xfe;
        board.process_pif();

        assert_ne!(board.pif_ram[5] & 0x80, 0);
    }

    #[test]
    fn pif_rtc_status_control_lock_and_calendar_tick_follow_cartridge_protocol() {
        let mut board = N64Board::new(&rom_with_id(b"NAF")).unwrap();
        assert!(board.flash_present);
        assert!(board.rtc_present);
        assert_eq!(board.eeprom_size, 0);

        board.pif_ram.fill(0);
        board.pif_ram[4..7].copy_from_slice(&[1, 3, 0x06]);
        board.pif_ram[10] = 0xfe;
        board.process_pif();
        assert_eq!(&board.pif_ram[7..10], &[0x00, 0x10, 0x00]);

        board.pif_ram.fill(0);
        board.pif_ram[4..8].copy_from_slice(&[2, 9, 0x07, 0x02]);
        board.pif_ram[17] = 0xfe;
        board.process_pif();
        assert_eq!(
            &board.pif_ram[8..17],
            &[0x00, 0x00, 0x80, 0x01, 0x06, 0x01, 0x00, 0x20, 0x00]
        );

        let end_of_year = [0x59, 0x59, 0xa3, 0x31, 0x00, 0x12, 0x00, 0x20];
        board.pif_ram.fill(0);
        board.pif_ram[4..8].copy_from_slice(&[10, 1, 0x08, 0x02]);
        board.pif_ram[8..16].copy_from_slice(&end_of_year);
        board.pif_ram[17] = 0xfe;
        board.process_pif();
        assert_eq!(board.pif_ram[16], 0x00);

        board.tick_cartridge(CPU_HZ as u32);
        assert_eq!(
            &board.rtc_ram[16..24],
            &[0x00, 0x00, 0x80, 0x01, 0x01, 0x01, 0x01, 0x20]
        );

        board.pif_ram.fill(0);
        board.pif_ram[4..8].copy_from_slice(&[10, 1, 0x08, 0x00]);
        board.pif_ram[8..16].copy_from_slice(&[0x04, 0x02, 0, 0, 0, 0, 0, 0]);
        board.pif_ram[17] = 0xfe;
        board.process_pif();
        assert_eq!(board.rtc_status & 0x80, 0x80);
        assert_eq!(board.rtc_write_lock & 0x04, 0x04);

        let locked_time = board.rtc_ram[16..24].to_vec();
        board.pif_ram.fill(0);
        board.pif_ram[4..8].copy_from_slice(&[10, 1, 0x08, 0x02]);
        board.pif_ram[8..16].fill(0x11);
        board.pif_ram[17] = 0xfe;
        board.process_pif();
        assert_eq!(&board.rtc_ram[16..24], locked_time.as_slice());

        board.tick_cartridge(CPU_HZ as u32);
        assert_eq!(&board.rtc_ram[16..24], locked_time.as_slice());

        board.pif_ram.fill(0);
        board.pif_ram[4..8].copy_from_slice(&[10, 1, 0x08, 0x00]);
        board.pif_ram[8..16].fill(0);
        board.pif_ram[17] = 0xfe;
        board.process_pif();
        assert_eq!(board.rtc_status & 0x80, 0);
        assert_eq!(board.rtc_write_lock & 0x06, 0);
    }

    #[test]
    fn rtc_state_and_persistent_slot_round_trip() {
        let mut machine = Nintendo64Machine::from_rom(&rom_with_id(b"NAF")).unwrap();
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 0), SAVE_SIZE);
        assert_eq!(machine.persistent_len(ResourceKind::Storage, 1), RTC_SIZE);

        let mut persisted = default_rtc_ram();
        persisted[16..24].copy_from_slice(&[0x58, 0x59, 0xa3, 0x28, 0x04, 0x02, 0x24, 0x20]);
        machine
            .write_persistent(ResourceKind::Storage, 1, &persisted)
            .unwrap();
        machine.board.rtc_cycle_phase = CPU_HZ - 7;
        machine.board.rtc_write_lock = 0x02;
        let state = machine.save_state().unwrap();

        machine.board.rtc_ram.fill(0);
        machine.board.rtc_status = 0x80;
        machine.board.rtc_write_lock = 0;
        machine.board.rtc_cycle_phase = 0;
        machine.load_state(&state).unwrap();
        assert_eq!(machine.board.rtc_ram, persisted);
        assert_eq!(machine.board.rtc_status, 0);
        assert_eq!(machine.board.rtc_write_lock, 0x02);
        assert_eq!(machine.board.rtc_cycle_phase, CPU_HZ - 7);

        let mut restored = [0u8; RTC_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 1, &mut restored)
            .unwrap();
        assert_eq!(restored, persisted);

        let without_rtc = Nintendo64Machine::from_rom(&rom_with_loop()).unwrap();
        assert_eq!(without_rtc.persistent_len(ResourceKind::Storage, 1), 0);
    }

    #[test]
    fn flashram_page_program_status_id_and_array_read_round_trip() {
        let mut board = N64Board::new(&rom_with_id(b"NPF")).unwrap();
        assert!(board.flash_present);
        assert_eq!(board.eeprom_size, 0);

        for index in 0..128 {
            board.rdram[0x100 + index] = index as u8 ^ 0x5a;
        }
        board.write32(0x0801_0000, 0xb400_0000);
        board.pi.dram_addr = 0x100;
        board.pi.cart_addr = 0x0800_0000;
        board.pi_dma(127, false);
        assert_eq!(&board.flash_page_buffer[..], &board.rdram[0x100..0x180]);
        board.tick_pi(board.pi.dma_cycles);
        board.write_pi_status(2);

        board.write32(0x0801_0000, 0xa500_0002);
        assert_eq!(board.flash_mode, FLASH_STATUS);
        assert_ne!(board.read32(0x0800_0000) & 0x04, 0);
        assert_eq!(&board.save[0x100..0x180], &board.rdram[0x100..0x180]);

        board.write32(0x0801_0000, 0xf000_0000);
        board.rdram[0x200..0x280].fill(0);
        board.pi.dram_addr = 0x200;
        board.pi.cart_addr = 0x0800_0080;
        board.pi_dma(127, true);
        assert_eq!(&board.rdram[0x200..0x280], &board.save[0x100..0x180]);
        board.tick_pi(board.pi.dma_cycles);
        board.write_pi_status(2);

        board.write32(0x0801_0000, 0xe100_0000);
        board.rdram[0x300..0x308].fill(0);
        board.pi.dram_addr = 0x300;
        board.pi.cart_addr = 0x0800_0000;
        board.pi_dma(7, true);
        assert_eq!(
            u32::from_be_bytes(board.rdram[0x300..0x304].try_into().unwrap()),
            FLASH_TYPE_ID
        );
        assert_eq!(
            u32::from_be_bytes(board.rdram[0x304..0x308].try_into().unwrap()),
            FLASH_DEVICE_ID
        );
    }

    #[test]
    fn flashram_sector_and_chip_erase_follow_command_sequence() {
        let mut board = N64Board::new(&rom_with_id(b"NPF")).unwrap();
        board.save[0x4000..0x8000].fill(0);
        board.save[0x8000..0xc000].fill(0x11);

        board.write32(0x0801_0000, 0x4b00_0080);
        board.write32(0x0801_0000, 0x7800_0000);

        assert!(board.save[0x4000..0x8000]
            .iter()
            .all(|&value| value == 0xff));
        assert!(board.save[0x8000..0xc000]
            .iter()
            .all(|&value| value == 0x11));
        assert_eq!(board.flash_mode, FLASH_STATUS);
        assert_ne!(board.flash_status & 0x08, 0);

        board.save[0x1234] = 0;
        board.write32(0x0801_0000, 0x3c00_0000);
        board.write32(0x0801_0000, 0x7800_0000);
        assert!(board.save.iter().all(|&value| value == 0xff));
    }

    #[test]
    fn flashram_profile_does_not_replace_sram_behavior() {
        let mut sram = N64Board::new(&rom_with_loop()).unwrap();
        assert!(!sram.flash_present);
        sram.write32(0x0800_0000, 0x1234_5678);
        assert_eq!(&sram.save[..4], &[0x12, 0x34, 0x56, 0x78]);
        assert_eq!(sram.read32(0x0800_0000), 0x1234_5678);

        let mut flash = N64Board::new(&rom_with_id(b"NPF")).unwrap();
        flash.write32(0x0800_0000, 0x1234_5678);
        assert_eq!(&flash.save[..4], &[0xff; 4]);
    }

    #[test]
    fn flashram_transient_state_round_trips() {
        let rom = rom_with_id(b"NPF");
        let mut machine = Nintendo64Machine::from_rom(&rom).unwrap();
        machine.board.write32(0x0801_0000, 0xb400_0000);
        machine.board.flash_page_buffer[17] = 0x42;
        machine.board.flash_status = 0x0d;
        machine.board.flash_erase_page = 0x0180;
        let state = machine.save_state().unwrap();

        machine.board.flash_mode = FLASH_READ_ARRAY;
        machine.board.flash_page_buffer.fill(0);
        machine.board.flash_status = 0;
        machine.board.flash_erase_page = 0;
        machine.load_state(&state).unwrap();

        assert_eq!(machine.board.flash_mode, FLASH_PAGE_PROGRAM);
        assert_eq!(machine.board.flash_page_buffer[17], 0x42);
        assert_eq!(machine.board.flash_status, 0x0d);
        assert_eq!(machine.board.flash_erase_page, 0x0180);
    }

    #[test]
    fn dpc_status_commands_use_pairs_and_clear_clock_counter() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.dp.status = DP_STATUS_XBUS | DP_STATUS_FREEZE | DP_STATUS_FLUSH;
        board.dp.clock = 123;

        board.write_dp_status(
            (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5) | (1 << 9),
        );

        assert_eq!(
            board.dp.status & (DP_STATUS_XBUS | DP_STATUS_FREEZE | DP_STATUS_FLUSH),
            DP_STATUS_XBUS | DP_STATUS_FREEZE | DP_STATUS_FLUSH
        );
        assert_eq!(board.dp.clock, 0);

        board.write_dp_status((1 << 0) | (1 << 2) | (1 << 4));
        assert_eq!(
            board.dp.status & (DP_STATUS_XBUS | DP_STATUS_FREEZE | DP_STATUS_FLUSH),
            0
        );

        board.write_dp_status((1 << 1) | (1 << 3) | (1 << 5));
        assert_eq!(
            board.dp.status & (DP_STATUS_XBUS | DP_STATUS_FREEZE | DP_STATUS_FLUSH),
            DP_STATUS_XBUS | DP_STATUS_FREEZE | DP_STATUS_FLUSH
        );
    }

    #[test]
    fn dpc_start_end_latch_and_incremental_transfer_preserve_current() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.dp.status = DP_STATUS_FREEZE;

        board.write_dp_start(0x0103);
        assert_eq!(board.dp.start, 0x0100);
        assert_ne!(board.dp.status & DP_STATUS_START_VALID, 0);
        board.write_dp_start(0x0200);
        assert_eq!(board.dp.start, 0x0100);

        board.write_dp_end(0x0108);
        assert_eq!(board.dp.current, 0x0100);
        assert_eq!(board.dp.status & DP_STATUS_START_VALID, 0);

        board.write_dp_end(0x0110);
        assert_eq!(board.dp.current, 0x0100);
        assert_eq!(board.dp.end, 0x0110);

        board.write_dp_status(1 << 2);
        assert_eq!(board.dp.current, 0x0110);
        assert_eq!(board.dp.clock, 2);
    }

    #[test]
    fn dpc_xbus_fetches_command_stream_from_rsp_dmem() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let dmem_command = [0x3700_0000u32, 0x1122_3344];
        let rdram_command = [0x3700_0000u32, 0xaabb_ccdd];

        for (index, word) in dmem_command.into_iter().enumerate() {
            let offset = index * 4;
            board.sp_mem[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
        }
        for (index, word) in rdram_command.into_iter().enumerate() {
            let offset = index * 4;
            board.rdram[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
        }

        board.write_dp_status(1 << 1);
        board.write_dp_start(0);
        board.write_dp_end(8);

        assert_eq!(board.fill_color, 0x1122_3344);
        assert_eq!(board.dp.current, 8);
    }

    #[test]
    fn rdp_key_and_convert_commands_program_signed_pipeline_state() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let raw = |value: i16| u32::from(value as u16) & 0x01ff;
        let coefficients = [175i16, -43, -89, 222, -3, 5];
        let encoded: [u32; 6] = coefficients.map(raw);
        let convert_high =
            (0x2cu32 << 24) | (encoded[0] << 13) | (encoded[1] << 4) | (encoded[2] >> 5);
        let convert_low =
            ((encoded[2] & 0x1f) << 27) | (encoded[3] << 18) | (encoded[4] << 9) | encoded[5];
        let base = 0x1800usize;
        write_rdp_words(
            &mut board,
            base,
            &[
                (0x2au32 << 24) | (0x123 << 12) | 0x456,
                (0x78 << 24) | (0x9a << 16) | (0xbc << 8) | 0xde,
                0x2bu32 << 24,
                (0x321 << 16) | (0x65 << 8) | 0x43,
                convert_high,
                convert_low,
            ],
        );
        board.dp.current = base as u32;
        board.dp.end = base as u32 + 24;

        board.run_rdp();

        assert_eq!(board.key_width, [0x321, 0x123, 0x456]);
        assert_eq!(board.key_center, [0x65, 0x78, 0xbc]);
        assert_eq!(board.key_scale, [0x43, 0x9a, 0xde]);
        assert_eq!(board.convert_k[..4], coefficients[..4]);
        assert_eq!(board.convert_k[4], encoded[4] as i16);
        assert_eq!(board.convert_k[5], encoded[5] as i16);
        let sources = [[0; 4]; 4];
        assert_eq!(board.rdp_rgb_mux(7, 1, 0, &sources, 0, 0), 0x1fd);
        assert_eq!(board.rdp_rgb_mux(15, 2, 0, &sources, 0, 0), 5);
        assert_eq!(board.dp.current, base as u32 + 24);
    }

    #[test]
    fn rdp_load_tlut_quadruples_entries_in_tmem_and_latches_size() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.texture_image = 0x4000;
        board.texture_width = 2;
        board.texture_size = 2;
        board.tiles[0] = RdpTile {
            tmem: 0x100,
            ..RdpTile::default()
        };
        board.rdram[0x4000..0x4004].copy_from_slice(&[0xf8, 0x01, 0x07, 0xc1]);

        board.load_rdp_tlut(0, 4 << 12);

        assert_eq!(
            &board.tmem[0x800..0x810],
            &[
                0xf8, 0x01, 0xf8, 0x01, 0xf8, 0x01, 0xf8, 0x01, 0x07, 0xc1, 0x07, 0xc1, 0x07, 0xc1,
                0x07, 0xc1,
            ]
        );
        assert_eq!(board.tlut[0], 0xf801);
        assert_eq!(board.tlut[1], 0x07c1);
        assert_eq!((board.tiles[0].sl, board.tiles[0].sh), (0, 4));
    }

    #[test]
    fn rdp_load_tile_uses_line_stride_and_odd_row_word_swap() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.texture_image = 0x4000;
        board.texture_width = 2;
        board.texture_format = 0;
        board.texture_size = 2;
        board.tiles[0] = RdpTile {
            format: 0,
            size: 2,
            line: 2,
            tmem: 0,
            ..RdpTile::default()
        };
        for (index, color) in [0x1123u16, 0x4567, 0x89ab, 0xcdef].into_iter().enumerate() {
            let offset = 0x4000 + index * 2;
            board.rdram[offset..offset + 2].copy_from_slice(&color.to_be_bytes());
        }

        board.load_rdp_tile(0, (4 << 12) | 4, false);

        assert_eq!(&board.tmem[0..4], &[0x11, 0x23, 0x45, 0x67]);
        assert_eq!(&board.tmem[16..20], &[0; 4]);
        assert_eq!(&board.tmem[20..24], &[0x89, 0xab, 0xcd, 0xef]);
        assert_eq!(
            board.sample_rdp_texel(0, 0, 0),
            Some(N64Board::rgba5551(0x1123))
        );
        assert_eq!(
            board.sample_rdp_texel(0, 1, 0),
            Some(N64Board::rgba5551(0x4567))
        );
        assert_eq!(
            board.sample_rdp_texel(0, 0, 1),
            Some(N64Board::rgba5551(0x89ab))
        );
        assert_eq!(
            board.sample_rdp_texel(0, 1, 1),
            Some(N64Board::rgba5551(0xcdef))
        );
        assert_eq!((board.tiles[0].sh, board.tiles[0].th), (4, 4));
    }

    #[test]
    fn rdp_rgba32_tmem_uses_split_low_and_high_banks() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.texture_image = 0x4000;
        board.texture_width = 2;
        board.texture_format = 0;
        board.texture_size = 3;
        board.tiles[0] = RdpTile {
            format: 0,
            size: 3,
            line: 1,
            tmem: 0,
            ..RdpTile::default()
        };
        board.rdram[0x4000..0x4008]
            .copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0xaa, 0xbb, 0xcc, 0xdd]);

        board.load_rdp_tile(0, 4 << 12, false);

        assert_eq!(&board.tmem[0..4], &[0x11, 0x22, 0xaa, 0xbb]);
        assert_eq!(&board.tmem[0x800..0x804], &[0x33, 0x44, 0xcc, 0xdd]);
        assert_eq!(
            board.sample_rdp_texel(0, 0, 0),
            Some([0x11, 0x22, 0x33, 0x44])
        );
        assert_eq!(
            board.sample_rdp_texel(0, 1, 0),
            Some([0xaa, 0xbb, 0xcc, 0xdd])
        );
    }

    #[test]
    fn rdp_load_block_dxt_swaps_odd_64bit_words() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.texture_image = 0x4000;
        board.texture_width = 8;
        board.texture_format = 0;
        board.texture_size = 2;
        board.tiles[0] = RdpTile {
            format: 0,
            size: 2,
            tmem: 0,
            ..RdpTile::default()
        };
        for index in 0..8usize {
            let value = (0x1000u16).wrapping_add(index as u16);
            board.rdram[0x4000 + index * 2..0x4002 + index * 2]
                .copy_from_slice(&value.to_be_bytes());
        }

        board.load_rdp_tile(0, (7 << 12) | 0x0800, true);

        let word = |offset: usize| u16::from_be_bytes([board.tmem[offset], board.tmem[offset + 1]]);
        assert_eq!(
            [word(0), word(2), word(4), word(6)],
            [0x1000, 0x1001, 0x1002, 0x1003]
        );
        assert_eq!(
            [word(8), word(10), word(12), word(14)],
            [0x1006, 0x1007, 0x1004, 0x1005]
        );
        assert_eq!(
            (
                board.tiles[0].sl,
                board.tiles[0].tl,
                board.tiles[0].sh,
                board.tiles[0].th,
            ),
            (0, 0, 7, 0x0800)
        );
    }

    #[test]
    fn rdp_load_block_enforces_2048_texel_limit() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.texture_image = 0x4000;
        board.texture_width = 4096;
        board.texture_size = 1;
        board.tiles[0] = RdpTile {
            format: 4,
            size: 1,
            tmem: 0,
            ..RdpTile::default()
        };
        board.tmem[..32].fill(0xa5);
        board.rdram[0x4000..0x5000].fill(0x3c);

        board.load_rdp_tile(0, 2048 << 12, true);
        assert_eq!(&board.tmem[..32], &[0xa5; 32]);
        assert_eq!((board.tiles[0].sl, board.tiles[0].sh), (0, 2048));

        board.tmem.fill(0);
        board.load_rdp_tile(0, 2047 << 12, true);
        assert_eq!(board.tmem[0], 0x3c);
        assert_eq!(board.tmem[2047], 0x3c);
        assert_eq!((board.tiles[0].sl, board.tiles[0].sh), (0, 2047));
    }

    #[test]
    fn rdp_yuv16_load_splits_tmem_and_default_convert_preserves_neutral_luma() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.texture_image = 0x200;
        board.texture_format = 1;
        board.texture_size = 2;
        board.texture_width = 2;
        board.rdram[0x200..0x204].copy_from_slice(&[0x80, 0x40, 0x80, 0xc0]);
        board.tiles[0] = RdpTile {
            format: 1,
            size: 2,
            tmem: 0,
            clamp_s: true,
            clamp_t: true,
            sh: 4,
            ..RdpTile::default()
        };

        board.load_rdp_tile(0, 1 << 12, true);
        assert_eq!((board.tiles[0].sl, board.tiles[0].sh), (0, 1));
        board.set_rdp_tile_size(0, 4 << 12);

        assert_eq!(&board.tmem[..2], &[0x80, 0x80]);
        assert_eq!(&board.tmem[0x800..0x802], &[0x40, 0xc0]);
        assert_eq!(board.sample_rdp_texture(0, 0, 0), Some([64, 64, 64, 64]));
        assert_eq!(
            board.sample_rdp_texture(0, 1, 0),
            Some([192, 192, 192, 192])
        );
    }

    #[test]
    fn rdp_chroma_key_uses_center_scale_width_and_bypasses_combined_rgb() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.other_modes = (1u64 << 40) | (3u64 << 38);
        board.key_center = [100, 120, 140];
        board.key_scale = [255; 3];
        board.key_width = [16; 3];
        board.combine_mode = combine_one_cycle([1, 6, 6, 7], [1, 7, 4, 7]);

        assert_eq!(board.rdp_key_alpha([100, 120, 140, 255]), 255);
        assert_eq!(board.rdp_key_alpha([101, 120, 140, 255]), 1);
        assert_eq!(board.rdp_key_alpha([102, 120, 140, 255]), 0);

        let rgba = board
            .rdp_process_pixel(0, 0, [[101, 120, 140, 255]; 2], [9, 8, 7, 6], [0; 4], 0)
            .unwrap();
        assert_eq!(rgba, [101, 120, 140, 1]);
    }

    #[test]
    fn rdp_variable_length_packets_preserve_stream_alignment_and_scissor_fill() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let base = 0x1000usize;
        board.rdram[base..base + 4].copy_from_slice(&(0x08u32 << 24).to_be_bytes());
        board.rdram[base + 4..base + 32].fill(0);

        let commands: [(u32, u32); 6] = [
            ((0x3fu32 << 24) | (2 << 19) | 3, 0x0000_3000),
            ((0x2fu32 << 24) | (3 << 20), 0),
            (0x37u32 << 24, 0xf801_f801),
            ((0x2du32 << 24) | (4 << 12) | 4, (12 << 12) | 12),
            ((0x36u32 << 24) | (12 << 12) | 12, 0),
            (0x29u32 << 24, 0),
        ];
        for (index, (high, low)) in commands.into_iter().enumerate() {
            let offset = base + 32 + index * 8;
            board.rdram[offset..offset + 4].copy_from_slice(&high.to_be_bytes());
            board.rdram[offset + 4..offset + 8].copy_from_slice(&low.to_be_bytes());
        }

        board.dp.start = base as u32;
        board.dp.current = base as u32;
        board.dp.end = (base + 32 + commands.len() * 8) as u32;
        board.run_rdp();

        assert_eq!(board.dp.current, board.dp.end);
        assert_ne!(board.mi_intr & MI_DP, 0);
        let pixel = |x: usize, y: usize| {
            u16::from_be_bytes([
                board.rdram[0x3000 + (y * 4 + x) * 2],
                board.rdram[0x3000 + (y * 4 + x) * 2 + 1],
            ])
        };
        assert_eq!(pixel(0, 0), 0);
        assert_eq!(pixel(1, 1), 0xf801);
        assert_eq!(pixel(2, 2), 0xf801);
        assert_eq!(pixel(3, 3), 0);
    }

    #[test]
    fn rdp_state_commands_decode_and_round_trip() {
        let mut machine = Nintendo64Machine::from_rom(&rom_with_loop()).unwrap();
        let base = 0x1800usize;
        let commands: [(u32, u32); 8] = [
            ((0x2fu32 << 24) | (3 << 20), 0x0000_4000),
            (0x38u32 << 24, 0x1122_3344),
            (0x39u32 << 24, 0x5566_7788),
            ((0x3au32 << 24) | (0x13 << 8) | 0x5a, 0x99aa_bbcc),
            (0x3bu32 << 24, 0xddee_ff11),
            ((0x3du32 << 24) | (2 << 21) | (2 << 19) | 31, 0x0012_3400),
            (0x3eu32 << 24, 0x0023_4500),
            (0x3cu32 << 24, 0xa5a5_5a5a),
        ];
        for (index, (high, low)) in commands.into_iter().enumerate() {
            let offset = base + index * 8;
            machine.board.rdram[offset..offset + 4].copy_from_slice(&high.to_be_bytes());
            machine.board.rdram[offset + 4..offset + 8].copy_from_slice(&low.to_be_bytes());
        }
        machine.board.dp.current = base as u32;
        machine.board.dp.end = (base + commands.len() * 8) as u32;
        machine.board.run_rdp();

        assert_eq!(machine.board.rdp_cycle_type(), 3);
        assert_eq!(machine.board.fog_color, 0x1122_3344);
        assert_eq!(machine.board.blend_color, 0x5566_7788);
        assert_eq!(machine.board.primitive_color, 0x99aa_bbcc);
        assert_eq!(machine.board.primitive_lod_frac, 0x5a);
        assert_eq!(machine.board.primitive_min_level, 0x13);
        assert_eq!(machine.board.environment_color, 0xddee_ff11);
        assert_eq!(machine.board.texture_format, 2);
        assert_eq!(machine.board.texture_size, 2);
        assert_eq!(machine.board.texture_width, 32);
        assert_eq!(machine.board.texture_image, 0x0012_3400);
        assert_eq!(machine.board.depth_image, 0x0023_4500);
        assert_eq!(
            machine.board.combine_mode,
            (u64::from(0x3c00_0000u32) << 32) | 0xa5a5_5a5a
        );

        machine.board.scissor = [3, 4, 100, 120];
        machine.board.scissor_fixed = [13, 18, 403, 482];
        machine.board.scissor_field = true;
        machine.board.scissor_keep_odd = true;
        machine.board.color_format = 4;
        machine.board.tmem[17] = 0xa5;
        machine.board.tlut[33] = 0xf801;
        machine.board.rdp_hidden_write(0x7000, 3);
        machine.board.rdp_noise_seed = 0x1234_5678;
        machine.board.rdp_pipeline_crashed = true;
        machine.board.tiles[2] = RdpTile {
            format: 2,
            size: 0,
            line: 4,
            tmem: 32,
            palette: 2,
            clamp_t: true,
            mirror_t: false,
            mask_t: 5,
            shift_t: 1,
            clamp_s: false,
            mirror_s: true,
            mask_s: 4,
            shift_s: 2,
            sl: 4,
            tl: 8,
            sh: 60,
            th: 64,
        };
        let saved = machine.save_state().unwrap();
        machine.board.scissor = [0; 4];
        machine.board.scissor_fixed = [0; 4];
        machine.board.scissor_field = false;
        machine.board.scissor_keep_odd = false;
        machine.board.color_format = 0;
        machine.board.primitive_color = 0;
        machine.board.primitive_lod_frac = 0;
        machine.board.primitive_min_level = 0;
        machine.board.tmem[17] = 0;
        machine.board.tlut[33] = 0;
        machine.board.rdp_hidden_write(0x7000, 0);
        machine.board.rdp_noise_seed = 1;
        machine.board.rdp_pipeline_crashed = false;
        machine.board.tiles[2] = RdpTile::default();
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.board.scissor, [3, 4, 100, 120]);
        assert_eq!(machine.board.scissor_fixed, [13, 18, 403, 482]);
        assert!(machine.board.scissor_field);
        assert!(machine.board.scissor_keep_odd);
        assert_eq!(machine.board.color_format, 4);
        assert_eq!(machine.board.primitive_color, 0x99aa_bbcc);
        assert_eq!(machine.board.primitive_lod_frac, 0x5a);
        assert_eq!(machine.board.primitive_min_level, 0x13);
        assert_eq!(machine.board.tmem[17], 0xa5);
        assert_eq!(machine.board.tlut[33], 0xf801);
        assert_eq!(machine.board.rdp_hidden_read(0x7000), 3);
        assert!(machine.board.rdp_pipeline_crashed);
        assert_eq!(machine.board.tiles[2].format, 2);
        assert_eq!(machine.board.tiles[2].tmem, 32);
        assert!(machine.board.tiles[2].clamp_t);
        assert!(machine.board.tiles[2].mirror_s);
        assert_eq!(machine.board.tiles[2].th, 64);
        assert_eq!(machine.save_state().unwrap(), saved);
    }

    #[test]
    fn rdp_copy_cycle_preserves_raw_ia16_and_tlut_words() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 1;
        board.color_size = 2;
        board.color_format = 0;
        board.other_modes = 2u64 << 52;
        board.tiles[0] = RdpTile {
            format: 3,
            size: 2,
            line: 1,
            clamp_s: true,
            clamp_t: true,
            ..RdpTile::default()
        };
        board.tmem[0..2].copy_from_slice(&0x80a5u16.to_be_bytes());

        board.rdp_texture_rectangle(0, 0, 0, 0, false);
        assert_eq!(
            u16::from_be_bytes([board.rdram[0x6000], board.rdram[0x6001]]),
            0x80a5
        );
        assert_eq!(board.rdp_hidden_read(0x6000), 3);

        board.rdram[0x6000..0x6002].copy_from_slice(&0x1234u16.to_be_bytes());
        board.tmem[0..2].copy_from_slice(&0x80a4u16.to_be_bytes());
        board.other_modes |= 1;
        board.rdp_texture_rectangle(0, 0, 0, 0, false);
        assert_eq!(
            u16::from_be_bytes([board.rdram[0x6000], board.rdram[0x6001]]),
            0x1234
        );

        board.other_modes = (2u64 << 52) | (1u64 << 47) | (1u64 << 46);
        board.tiles[0] = RdpTile {
            format: 2,
            size: 1,
            line: 1,
            clamp_s: true,
            clamp_t: true,
            ..RdpTile::default()
        };
        board.tmem[0] = 0x12;
        board.tlut[0x12] = 0x7f23;
        board.rdp_texture_rectangle(0, 0, 0, 0, false);
        assert_eq!(
            u16::from_be_bytes([board.rdram[0x6000], board.rdram[0x6001]]),
            0x7f23
        );
    }

    #[test]
    fn rdp_copy_cycle_uses_nearest_alpha_bits_and_rejects_32bit_target() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 4;
        board.color_size = 2;
        board.color_format = 0;
        board.blend_color = 0;
        board.other_modes = (2u64 << 52) | (1u64 << 45) | 1;
        board.tiles[0] = RdpTile {
            format: 0,
            size: 2,
            line: 1,
            clamp_s: true,
            clamp_t: true,
            sh: 12,
            ..RdpTile::default()
        };
        for (index, color) in [0xf800u16, 0x07c1, 0x003f, 0xffff].into_iter().enumerate() {
            let offset = index * 2;
            board.tmem[offset..offset + 2].copy_from_slice(&color.to_be_bytes());
        }
        board.rdram[0x6000..0x6002].copy_from_slice(&0x1234u16.to_be_bytes());

        let high = 12u32 << 12;
        let low = 0u32;
        let high2 = 0u32;
        let low2 = 4096u32 << 16;
        board.rdp_texture_rectangle(high, low, high2, low2, false);

        let pixel = |x: usize| {
            let offset = 0x6000 + x * 2;
            u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]])
        };
        assert_eq!(pixel(0), 0x1234);
        assert_eq!(pixel(1), 0x07c1);
        assert_eq!(pixel(2), 0x003f);
        assert_eq!(pixel(3), 0xffff);

        board.color_image = 0x7000;
        board.color_size = 3;
        board.rdram[0x7000..0x7010].fill(0xa5);
        board.rdp_texture_rectangle(high, low, high2, low2, false);
        assert_eq!(&board.rdram[0x7000..0x7010], &[0xa5; 16]);
        assert!(board.rdp_pipeline_crashed);
    }

    #[test]
    fn rdp_texture_rectangle_fill_cycle_uses_fill_pipeline() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 1;
        board.color_size = 2;
        board.color_format = 0;
        board.fill_color = 0xf801_07c1;
        board.other_modes = 3u64 << 52;
        board.tmem[0..2].copy_from_slice(&0x07c1u16.to_be_bytes());

        board.rdp_texture_rectangle(0, 0, 0, 0, false);

        assert_eq!(
            u16::from_be_bytes([board.rdram[0x6000], board.rdram[0x6001]]),
            0xf801
        );
    }

    #[test]
    fn rdp_load_block_and_texture_rectangle_render_rgba16_texels() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let source = 0x4000usize;
        for (index, color) in [0xf801u16, 0x07c1, 0x003f, 0xffff].into_iter().enumerate() {
            let offset = source + index * 2;
            board.rdram[offset..offset + 2].copy_from_slice(&color.to_be_bytes());
        }

        let base = 0x1000usize;
        let commands: [(u32, u32); 6] = [
            ((0x3du32 << 24) | (2 << 19), source as u32),
            ((0x35u32 << 24) | (2 << 19), 7 << 24),
            (0x33u32 << 24, (7 << 24) | (3 << 12)),
            ((0x35u32 << 24) | (2 << 19) | (1 << 9), 0),
            (0x32u32 << 24, 12 << 12),
            ((0x3fu32 << 24) | (2 << 19) | 3, 0x0000_6000),
        ];
        for (index, (high, low)) in commands.into_iter().enumerate() {
            let offset = base + index * 8;
            board.rdram[offset..offset + 4].copy_from_slice(&high.to_be_bytes());
            board.rdram[offset + 4..offset + 8].copy_from_slice(&low.to_be_bytes());
        }
        let rect = base + commands.len() * 8;
        let high = (0x24u32 << 24) | (16 << 12) | 4;
        let low = 0u32;
        let high2 = 0u32;
        let low2 = 1024u32 << 16;
        board.rdram[rect..rect + 4].copy_from_slice(&high.to_be_bytes());
        board.rdram[rect + 4..rect + 8].copy_from_slice(&low.to_be_bytes());
        board.rdram[rect + 8..rect + 12].copy_from_slice(&high2.to_be_bytes());
        board.rdram[rect + 12..rect + 16].copy_from_slice(&low2.to_be_bytes());

        board.dp.current = base as u32;
        board.dp.end = (rect + 16) as u32;
        board.run_rdp();

        let pixel = |x: usize| {
            let offset = 0x6000 + x * 2;
            u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]])
        };
        assert_eq!(pixel(0), 0xf801);
        assert_eq!(pixel(1), 0x07c1);
        assert_eq!(pixel(2), 0x003f);
        assert_eq!(pixel(3), 0xffff);
        assert_eq!(board.dp.current, board.dp.end);
    }

    #[test]
    fn rdp_alpha_compare_rejection_does_not_write_depth() {
        let render = |alpha: u16| {
            let mut board = N64Board::new(&rom_with_loop()).unwrap();
            board.color_image = 0x6000;
            board.depth_image = 0x7000;
            board.color_width = 8;
            board.color_size = 2;
            board.blend_color = 0x0000_0080;
            board.other_modes = (3u64 << 38) | 0x0021;

            let base = 0x1000usize;
            let edges = simple_triangle_edges(0x0d);
            write_rdp_words(&mut board, base, &edges);
            let mut shade = constant_red_shade();
            shade[1] = (shade[1] & 0xffff_0000) | u32::from(alpha);
            write_rdp_words(&mut board, base + 32, &shade);
            write_rdp_words(&mut board, base + 96, &[0x1000_0000, 0, 0, 0]);

            board.rdp_shade_triangle(base as u32, edges[0], edges[1], Some(96));
            let address = 0x7000 + 8 * 2;
            u16::from_be_bytes([board.rdram[address], board.rdram[address + 1]])
        };

        assert_eq!(render(0x40), 0, "alpha-rejected pixel must not write Z");
        assert_ne!(render(0xc0), 0, "alpha-passing pixel writes Z");
    }

    #[test]
    fn rdp_scissor_field_selects_requested_scanline_parity() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 4;
        board.color_size = 2;
        board.fill_color = 0xf801_f801;
        board.other_modes = 3u64 << 52;

        board.set_rdp_scissor(0, (1 << 25) | (1 << 24) | (16 << 12) | 16);
        assert_eq!(board.scissor, [0, 0, 4, 4]);
        assert!(board.scissor_field);
        assert!(board.scissor_keep_odd);
        assert!(!board.rdp_scissor_accepts_line(0));
        assert!(board.rdp_scissor_accepts_line(1));

        board.rdp_fill_rectangle((12 << 12) | 12, 0);
        for y in 0..4usize {
            for x in 0..4usize {
                let offset = 0x6000 + (y * 4 + x) * 2;
                let pixel = u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]]);
                assert_eq!(pixel, if y & 1 != 0 { 0xf801 } else { 0 });
            }
        }

        let base = 0x1000usize;
        let edges = simple_triangle_edges(0x0c);
        write_rdp_words(&mut board, base, &edges);
        let spans = board.rdp_triangle_spans(base as u32, edges[0], edges[1]);
        assert!(!spans.is_empty());
        assert!(spans.iter().all(|span| span.y & 1 != 0));

        board.set_rdp_scissor(0, (1 << 25) | (16 << 12) | 16);
        assert!(board.rdp_scissor_accepts_line(0));
        assert!(!board.rdp_scissor_accepts_line(1));
    }

    #[test]
    fn rdp_fractional_scissor_clips_subpixel_coverage_without_truncation() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_width = 8;
        board.set_rdp_scissor(1 << 12, (32 << 12) | 16);
        assert_eq!(board.scissor_fixed, [1, 0, 32, 16]);
        assert_eq!(board.scissor, [0, 0, 8, 4]);

        let base = 0x1000usize;
        let edges = simple_triangle_edges(0x0c);
        write_rdp_words(&mut board, base, &edges);
        let spans = board.rdp_triangle_spans(base as u32, edges[0], edges[1]);
        let row = spans.iter().find(|span| span.y == 1).copied().unwrap();
        assert_eq!(row.coverage_left, [2; 4]);
        assert_eq!(
            N64Board::rdp_coverage_mask(row.coverage_left, row.coverage_right, 0) & 1,
            0
        );
    }

    #[test]
    fn rdp_triangle_depth_packet_uses_coverage_centroid_and_xy_delta_sum() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let base = 0x1000usize;
        write_rdp_words(
            &mut board,
            base,
            &[1000u32 << 13, 2u32 << 16, 0, 1u32 << 16],
        );

        let full = board
            .rdp_triangle_depth(
                base as u32,
                0,
                RdpTrianglePixel {
                    coverage_mask: 0xff,
                    ..RdpTrianglePixel::default()
                },
            )
            .unwrap();
        assert_eq!(full.depth, 1000);
        assert_eq!(full.delta_z, 4);
        assert_eq!(full.compressed_delta_z, 2);

        let edge = board
            .rdp_triangle_depth(
                base as u32,
                0,
                RdpTrianglePixel {
                    coverage_mask: 0xee,
                    ..RdpTrianglePixel::default()
                },
            )
            .unwrap();
        assert_eq!(edge.depth, 1008);
        assert_eq!(edge.delta_z, 4);
        assert_eq!(edge.compressed_delta_z, 2);
    }

    #[test]
    fn rdp_coverage_centroid_corrects_shade_depth_and_delta_z() {
        assert_eq!(N64Board::rdp_coverage_offsets(0xff), [0, 0]);
        assert_eq!(N64Board::rdp_coverage_offsets(0xee), [2, 0]);
        assert_eq!(N64Board::rdp_coverage_offsets(0x50), [0, 2]);

        assert_eq!(
            N64Board::rdp_correct_shade_component(100i64 << 16, 4i64 << 16, 0, 0xff),
            100
        );
        assert_eq!(
            N64Board::rdp_correct_shade_component(100i64 << 16, 4i64 << 16, 0, 0xee),
            102
        );
        assert_eq!(
            N64Board::rdp_correct_shade_component(100i64 << 16, 0, 8i64 << 16, 0x50),
            104
        );

        assert_eq!(
            N64Board::rdp_correct_depth(1000 << 13, 32 << 10, 0, 0xff),
            1000
        );
        assert_eq!(
            N64Board::rdp_correct_depth(1000 << 13, 32 << 10, 0, 0xee),
            1002
        );

        assert_eq!(N64Board::rdp_normalize_delta_z(0), 1);
        assert_eq!(N64Board::rdp_normalize_delta_z(1), 3);
        assert_eq!(N64Board::rdp_normalize_delta_z(2), 4);
        assert_eq!(N64Board::rdp_normalize_delta_z(0x2001), 0x4000);
        assert_eq!(N64Board::rdp_normalize_delta_z(0x4000), 0x8000);
    }

    #[test]
    fn rdp_subpixel_coverage_matches_hardware_sample_pattern() {
        assert_eq!(N64Board::rdp_quantize_coverage_x(5i64 << 16), 40);
        assert_eq!(N64Board::rdp_quantize_coverage_x((5i64 << 16) | 0x1000), 41);
        assert_eq!(
            N64Board::rdp_quantize_coverage_x((5i64 << 16) | (1 << 15)),
            44
        );
        assert_eq!(N64Board::rdp_quantize_coverage_x(-(3i64 << 16)), -24);

        assert_eq!(N64Board::rdp_coverage_mask([0; 4], [800; 4], 5), 0xff);
        assert_eq!(N64Board::rdp_coverage_mask([800; 4], [0; 4], 5), 0x00);
        assert_eq!(N64Board::rdp_coverage_mask([43; 4], [800; 4], 5), 0xaa);
        assert_eq!(
            N64Board::rdp_coverage_mask([0, 800, 800, 800], [800, 0, 0, 0], 5),
            0x03
        );
        assert_eq!(N64Board::rdp_coverage_mask([41; 4], [800; 4], 5), 0xee);

        let span = RdpTriangleSpan {
            y: 0,
            x0: 0,
            x1: 0,
            major_x: 0,
            coverage_left: [0; 4],
            coverage_right: [0, 2, 4, 6],
            subpixel: true,
        };
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        assert_eq!(
            N64Board::rdp_coverage_mask(span.coverage_left, span.coverage_right, 0),
            0x50
        );
        assert_eq!(board.rdp_pixel_coverage(span, 0), None);
        board.other_modes = 1u64 << 3;
        assert_eq!(board.rdp_pixel_coverage(span, 0), Some(2));
    }

    #[test]
    fn rdp_eight_bit_color_image_matches_byte_lane_semantics() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 4;
        board.color_size = 1;
        board.other_modes = 1u64 << 6;

        board.rdram[0x6000] = 0x5a;
        assert_eq!(board.read_rdp_rgba(0, 0), [0x5a, 0x5a, 0x5a, 0xe0]);
        assert_eq!(board.rdp_framebuffer_coverage_read(0, 0), 7);

        board.write_rdp_coverage_pixel(0, 0, [0x11, 0x22, 0x33, 0xff], 2);
        board.write_rdp_coverage_pixel(1, 0, [0x44, 0x55, 0x66, 0xff], 3);
        assert_eq!(board.rdram[0x6000], 0x11);
        assert_eq!(board.rdram[0x6001], 0x55);

        board.write_rdp_rgba(2, 0, [0x77, 0x88, 0x99, 0xaa]);
        board.write_rdp_rgba(3, 0, [0xbb, 0xcc, 0xdd, 0xee]);
        assert_eq!(board.rdram[0x6002], 0x77);
        assert_eq!(board.rdram[0x6003], 0xcc);

        board.fill_color = 0x1234_5678;
        for x in 0..4 {
            board.write_rdp_fill_pixel(x, 0);
        }
        assert_eq!(&board.rdram[0x6000..0x6004], &[0x12, 0x34, 0x56, 0x78]);
    }

    #[test]
    fn rdp_color_image_format_controls_non_rgba16_memory_layout() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.set_rdp_color_image((4u32 << 21) | (2u32 << 19) | 7, 0x6000);
        assert_eq!(board.color_format, 4);
        assert_eq!(board.color_size, 2);
        assert_eq!(board.color_width, 8);
        assert_eq!(board.color_image, 0x6000);

        board.other_modes = 1u64 << 6;
        board.write_rdp_coverage_pixel(0, 0, [0x66, 0x11, 0x22, 0xff], 5);
        assert_eq!(
            u16::from_be_bytes([board.rdram[0x6000], board.rdram[0x6001]]),
            0x66a0
        );
        assert_eq!(board.rdp_hidden_read(0x6000), 0);
        assert_eq!(board.rdp_framebuffer_coverage_read(0, 0), 5);
        assert_eq!(board.read_rdp_rgba(0, 0), [0x66, 0x66, 0x66, 0xa0]);

        board.write_rdp_rgba(1, 0, [0x44, 1, 2, 3]);
        assert_eq!(
            u16::from_be_bytes([board.rdram[0x6002], board.rdram[0x6003]]),
            0x44e0
        );
        assert_eq!(board.rdp_hidden_read(0x6002), 0);
    }

    #[test]
    fn rdp_framebuffer_coverage_round_trips_and_destinations_match_angrylion() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 1;
        board.other_modes = 1u64 << 6;

        board.color_size = 2;
        board.write_rdp_coverage_pixel(0, 0, [0x11, 0x22, 0x33, 0xff], 5);
        assert_eq!(board.rdp_framebuffer_coverage_read(0, 0), 5);
        assert_eq!(board.rdp_hidden_read(0x6000), 1);
        assert_eq!(
            u16::from_be_bytes([board.rdram[0x6000], board.rdram[0x6001]]) & 1,
            1
        );

        board.color_size = 3;
        board.write_rdp_coverage_pixel(0, 0, [0x11, 0x23, 0x33, 0xff], 6);
        assert_eq!(board.rdp_framebuffer_coverage_read(0, 0), 6);
        assert_eq!(board.rdram[0x6003], 0xc0);
        assert_eq!(board.rdp_hidden_read(0x6000), 3);
        assert_eq!(board.rdp_hidden_read(0x6002), 0);

        board.other_modes = 0;
        assert_eq!(board.rdp_finalize_coverage(false, 2, 3), 1);
        board.other_modes = 1u64 << 8;
        assert_eq!(board.rdp_finalize_coverage(false, 2, 3), 5);
        board.other_modes = 2u64 << 8;
        assert_eq!(board.rdp_finalize_coverage(false, 2, 3), 7);
        board.other_modes = 3u64 << 8;
        assert_eq!(board.rdp_finalize_coverage(false, 2, 3), 3);
        board.other_modes = 0;
        assert_eq!(board.rdp_finalize_coverage(true, 6, 3), 7);
    }

    #[test]
    fn rdp_depth_codec_hidden_delta_and_z_modes_match_reference() {
        assert_eq!(N64Board::rdp_z_decompress(0), 0);
        assert_eq!(N64Board::rdp_z_decompress(0x3fff), 0x3_ffff);
        assert_eq!(N64Board::rdp_z_decompress(0x3000), 0x3_f000);
        for stored in [0u16, 0x2000, 0x3fff] {
            assert_eq!(
                N64Board::rdp_z_compress(N64Board::rdp_z_decompress(stored)),
                stored
            );
        }
        assert_eq!(N64Board::rdp_dz_decompress(15), 0x8000);
        assert_eq!(N64Board::rdp_dz_compress(0x8000), 15);
        assert_eq!(N64Board::rdp_combine_dz(0x180), 0x100);

        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_width = 4;
        board.depth_image = 0x2000;
        board.rdp_depth_write(2, 1, 0x3_f000, 0x0d);
        assert_eq!(board.rdp_depth_read(2, 1), Some((0x3000, 0x0d)));
        let address = 0x2000 + 6 * 2;
        assert_eq!(board.rdp_hidden_read(address), 1);

        let base = RdpDepthInputs {
            current_depth: 0x3000,
            current_dz: 0,
            current_coverage: 0,
            z_compare: true,
            z_mode: 0,
            force_blend: false,
            aa_enable: false,
        };
        assert!(N64Board::rdp_depth_decision(0x30000, 0, 0, 1, base).depth_pass);
        assert!(!N64Board::rdp_depth_decision(0x3_ff00, 0, 0, 1, base).depth_pass);

        let transparent = RdpDepthInputs { z_mode: 2, ..base };
        assert!(N64Board::rdp_depth_decision(0x30000, 0, 0, 1, transparent).depth_pass);
        assert!(!N64Board::rdp_depth_decision(0x3_ff00, 0, 0, 1, transparent).depth_pass);

        let decal = RdpDepthInputs { z_mode: 3, ..base };
        assert!(N64Board::rdp_depth_decision(0x3_f000, 0, 0, 1, decal).depth_pass);
        assert!(!N64Board::rdp_depth_decision(0x30000, 0, 0, 1, decal).depth_pass);

        let interpenetrating = RdpDepthInputs {
            current_coverage: 4,
            z_mode: 1,
            ..base
        };
        let result = N64Board::rdp_depth_decision(0x3_effc, 0, 0, 4, interpenetrating);
        assert!(result.depth_pass);
        assert_eq!(result.coverage_count, 2);

        let coplanar = RdpDepthInputs {
            current_depth: 0x1000,
            current_dz: 15,
            ..base
        };
        assert!(N64Board::rdp_depth_decision(0x3_ff00, 0, 0, 1, coplanar).depth_pass);
    }

    #[test]
    fn rdp_image_commands_mask_dram_addresses_to_24_bits() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let base = 0x1000usize;
        let words = [
            (0x3fu32 << 24) | (3 << 19) | 7,
            0xab65_4321,
            (0x3du32 << 24) | (2 << 19) | 15,
            0xcd12_3456,
            0x3eu32 << 24,
            0xefab_cdef,
        ];
        write_rdp_words(&mut board, base, &words);

        board.dp.current = base as u32;
        board.dp.end = (base + words.len() * 4) as u32;
        board.run_rdp();

        assert_eq!(board.color_image, 0x0065_4321);
        assert_eq!(board.texture_image, 0x0012_3456);
        assert_eq!(board.depth_image, 0x00ab_cdef);
    }

    #[test]
    fn rdp_set_prim_depth_decodes_z_and_delta_from_low_word() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let base = 0x1000usize;
        let high = (0x2eu32 << 24) | 0xbeef;
        let low = 0x4567_00a5u32;
        write_rdp_words(&mut board, base, &[high, low]);

        board.dp.current = base as u32;
        board.dp.end = (base + 8) as u32;
        board.run_rdp();

        assert_eq!(board.primitive_depth, 0x4567);
        assert_eq!(board.primitive_delta_z, 0x00a5);
    }

    #[test]
    fn rdp_primitive_depth_is_promoted_to_eighteen_bit_domain() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.other_modes = 1u64 << 2;
        board.primitive_depth = 0x1234;
        board.primitive_delta_z = 0x0080;
        assert_eq!(
            board.rdp_triangle_depth(
                0,
                0,
                RdpTrianglePixel {
                    coverage_mask: 0xff,
                    ..RdpTrianglePixel::default()
                },
            ),
            Some(RdpDepthSample {
                depth: 0x1234 << 3,
                delta_z: 0x80,
                compressed_delta_z: 7,
            })
        );
    }

    #[test]
    fn rdp_perspective_divide_uses_hardware_reciprocal_table() {
        assert_eq!(
            N64Board::rdp_perspective_divide(0x10, 0x20, 0x4000),
            [0x20, 0x40]
        );
        assert_eq!(
            N64Board::rdp_perspective_divide(0x100, 0, 0x2000),
            [0x400, 0]
        );
        assert_eq!(
            N64Board::rdp_perspective_divide(0x10, 0x20, -1),
            [0x7fff, 0x7fff]
        );
        assert_eq!(RDP_PERSPECTIVE_TABLE[0], (0x4000, -1008));
        assert_eq!(RDP_PERSPECTIVE_TABLE[63], (0x2041, -260));

        assert_eq!(
            N64Board::rdp_texture_coordinates(
                [0x10i64 << 16, 0x20i64 << 16, 0x4000i64 << 16, 0],
                true,
            ),
            [0x20, 0x40]
        );
        assert_eq!(
            N64Board::rdp_texture_coordinates([8i64 << 11, 16i64 << 11, 1, 0], false),
            [8, 16]
        );
    }

    #[test]
    fn rdp_lod_signals_and_mip_tiles_follow_two_cycle_hardware_rules() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();

        assert_eq!(N64Board::rdp_lod_delta(0, 48, 0, 16, 0), 48);
        assert_eq!(N64Board::rdp_lod_delta(48, 0, 0, 0, 0), 47);

        let signals = board.rdp_lod_signals(false, 112, 2);
        assert_eq!(
            signals,
            RdpLodSignals {
                frac: 0xc0,
                level: 1,
                magnify: false,
                distant: false,
            }
        );
        assert_eq!(board.rdp_lod_tiles(0, signals, 2), (1, 2));

        let distant = board.rdp_lod_signals(false, 112, 0);
        assert!(distant.distant);
        assert_eq!(distant.frac, 0xff);
        assert_eq!(board.rdp_lod_tiles(3, distant, 0), (3, 3));

        board.other_modes = 1u64 << 49;
        let sharpened = board.rdp_lod_signals(false, 16, 2);
        assert!(sharpened.magnify);
        assert_eq!(sharpened.frac, 0x180);
        assert_eq!(board.rdp_lod_tiles(0, sharpened, 2), (0, 1));

        board.other_modes = 1u64 << 50;
        let detailed = board.rdp_lod_signals(false, 112, 3);
        assert_eq!(board.rdp_lod_tiles(0, detailed, 3), (2, 3));
    }

    #[test]
    fn rdp_combiner_routes_derivative_and_primitive_lod_inputs() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.primitive_lod_frac = 0x5a;
        let sources = [[0; 4]; 4];

        assert_eq!(board.rdp_rgb_mux(13, 2, 0, &sources, 0, 0x7f), 0x7f);
        assert_eq!(board.rdp_rgb_mux(14, 2, 0, &sources, 0, 0), 0x5a);
        assert_eq!(
            board.rdp_alpha_mux(0, true, [0; 4], [[0; 4]; 2], [0; 4], 0x7f),
            0x7f
        );
        assert_eq!(
            board.rdp_alpha_mux(6, true, [0; 4], [[0; 4]; 2], [0; 4], 0),
            0x5a
        );
    }

    #[test]
    fn rdp_texture_filter_uses_subtexel_fraction_and_mid_texel_mode() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.tiles[0] = RdpTile {
            format: 0,
            size: 2,
            sh: 4,
            th: 4,
            ..RdpTile::default()
        };
        for (offset, color) in [0xf801u16, 0x07c1, 0x003f, 0xffff].into_iter().enumerate() {
            let address = offset * 2;
            board.tmem[address..address + 2].copy_from_slice(&color.to_be_bytes());
        }

        assert_eq!(
            board.sample_rdp_texture_fixed(0, 8, 8),
            Some([255, 0, 0, 255])
        );

        board.other_modes = 1u64 << 45;
        assert_eq!(
            board.sample_rdp_texture_fixed(0, 8, 8),
            Some([128, 64, 64, 255])
        );

        board.other_modes = (1u64 << 45) | (1u64 << 44);
        assert_eq!(
            board.sample_rdp_texture_fixed(0, 16, 16),
            Some([128, 128, 128, 255])
        );
    }

    #[test]
    fn rdp_ci_tlut_control_bits_gate_and_select_palette_decode() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.tiles[0] = RdpTile {
            format: 2,
            size: 0,
            palette: 1,
            ..RdpTile::default()
        };
        board.tmem[0] = 0x20;
        board.tlut[18] = 0xf801;

        assert_eq!(board.sample_rdp_texture(0, 0, 0), Some([0; 4]));

        board.other_modes = 1u64 << 47;
        assert_eq!(board.sample_rdp_texture(0, 0, 0), Some([255, 0, 0, 255]));

        board.tlut[18] = 0x8040;
        board.other_modes = (1u64 << 47) | (1u64 << 46);
        assert_eq!(board.sample_rdp_texture(0, 0, 0), Some([128, 128, 128, 64]));
    }

    #[test]
    fn rdp_ci4_load_block_and_tlut_render_palette_texels() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let palette = 0x5000usize;
        for index in 0..16usize {
            let color = match index {
                0 => 0x0001u16,
                1 => 0xf801,
                2 => 0x07c1,
                3 => 0x003f,
                _ => 0xffff,
            };
            board.rdram[palette + index * 2..palette + index * 2 + 2]
                .copy_from_slice(&color.to_be_bytes());
        }
        board.rdram[0x4000] = 0x01;
        board.rdram[0x4001] = 0x23;

        let base = 0x1000usize;
        let commands: [(u32, u32); 12] = [
            ((0x2fu32 << 24) | (1 << 15), 0),
            ((0x3du32 << 24) | (2 << 19), palette as u32),
            ((0x35u32 << 24) | (288), 7 << 24),
            (0x30u32 << 24, (7 << 24) | (15 << 14)),
            ((0x3du32 << 24) | (2 << 21) | (2 << 19), 0x0000_4000),
            ((0x35u32 << 24) | (2 << 21) | (2 << 19), 7 << 24),
            (0x33u32 << 24, 7 << 24),
            ((0x35u32 << 24) | (2 << 21) | (1 << 9), 2 << 20),
            (0x32u32 << 24, 12 << 12),
            ((0x3fu32 << 24) | (2 << 19) | 3, 0x0000_6000),
            (0x26u32 << 24, 0),
            (0x27u32 << 24, 0),
        ];
        for (index, (high, low)) in commands.into_iter().enumerate() {
            let offset = base + index * 8;
            board.rdram[offset..offset + 4].copy_from_slice(&high.to_be_bytes());
            board.rdram[offset + 4..offset + 8].copy_from_slice(&low.to_be_bytes());
        }
        let rect = base + commands.len() * 8;
        let high = (0x24u32 << 24) | (16 << 12) | 4;
        let low = 0u32;
        board.rdram[rect..rect + 4].copy_from_slice(&high.to_be_bytes());
        board.rdram[rect + 4..rect + 8].copy_from_slice(&low.to_be_bytes());
        board.rdram[rect + 8..rect + 12].copy_from_slice(&0u32.to_be_bytes());
        board.rdram[rect + 12..rect + 16].copy_from_slice(&((1024u32 << 16) | 1024).to_be_bytes());

        board.dp.current = base as u32;
        board.dp.end = (rect + 16) as u32;
        board.run_rdp();

        let pixel = |x: usize| {
            let offset = 0x6000 + x * 2;
            u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]])
        };
        assert_eq!(pixel(0), 0x0001);
        assert_eq!(pixel(1), 0xf801);
        assert_eq!(pixel(2), 0x07c1);
        assert_eq!(pixel(3), 0x003f);
    }

    #[test]
    fn rdp_no_shade_triangle_copy_cycle_preserves_reverse_byte_stream() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 8;
        board.color_size = 2;
        board.color_format = 0;
        board.fill_color = 0xffff_ffff;
        board.other_modes = 2u64 << 52;
        board.tiles[0] = RdpTile {
            format: 0,
            size: 2,
            line: 1,
            mask_s: 2,
            ..RdpTile::default()
        };
        for (index, word) in [0x1111u16, 0x2221, 0x3331, 0x4441].into_iter().enumerate() {
            board.tmem[index * 2..index * 2 + 2].copy_from_slice(&word.to_be_bytes());
        }

        let base = 0x1000usize;
        let edges = simple_triangle_edges(0x08);
        write_rdp_words(&mut board, base, &edges);
        board.rdp_fill_triangle(base as u32, edges[0], edges[1]);

        let row = 0x6000 + 3 * 8 * 2;
        assert_eq!(board.rdram[row - 1], 0x41);
        assert_eq!(
            &board.rdram[row..row + 8],
            &[0x44, 0x31, 0x33, 0x21, 0x22, 0x11, 0x11, 0x00]
        );
        assert_ne!(
            &board.rdram[row..row + 8],
            &0xffff_ffff_ffff_ffffu64.to_be_bytes()
        );
    }

    #[test]
    fn rdp_no_shade_triangle_normal_cycle_uses_combiner_and_coverage() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 8;
        board.color_size = 2;
        board.color_format = 0;
        board.fill_color = 0x07c1_07c1;
        board.primitive_color = 0xf800_00ff;
        board.combine_mode = combine_one_cycle([15, 15, 31, 3], [7, 7, 7, 3]);
        board.other_modes = 3u64 << 38;

        let base = 0x1000usize;
        let edges = simple_triangle_edges(0x08);
        write_rdp_words(&mut board, base, &edges);
        board.rdp_fill_triangle(base as u32, edges[0], edges[1]);

        let pixel = |x: usize, y: usize| {
            let offset = 0x6000 + (y * 8 + x) * 2;
            u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]])
        };
        assert_eq!(pixel(0, 1), 0xf801);
        assert_eq!(pixel(1, 1), 0);
        assert_eq!(pixel(0, 2), 0xf801);
        assert_ne!(pixel(0, 2), 0x07c1);
    }

    #[test]
    fn rdp_no_shade_triangle_fill_cycle_matches_crash_order() {
        let render = |modes: u64| {
            let mut board = N64Board::new(&rom_with_loop()).unwrap();
            board.color_image = 0x6000;
            board.color_width = 8;
            board.color_size = 2;
            board.fill_color = 0xf801_f801;
            board.other_modes = (3u64 << 52) | modes;

            let base = 0x1000usize;
            let edges = simple_triangle_edges(0x08);
            write_rdp_words(&mut board, base, &edges);
            board.rdp_fill_triangle(base as u32, edges[0], edges[1]);

            let pixel = |x: usize, y: usize| {
                let offset = 0x6000 + (y * 8 + x) * 2;
                u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]])
            };
            (board.rdp_pipeline_crashed, pixel(0, 0), pixel(0, 1))
        };

        assert_eq!(render(0x0040), (true, 0, 0));
        assert_eq!(render(0x0010), (true, 0, 0));
        assert_eq!(render(0x0020), (true, 0xf801, 0));
        assert_eq!(render(0x0024), (false, 0xf801, 0xf801));
    }

    #[test]
    fn rdp_fill_triangle_walks_fixed_point_edges_at_quarter_scanlines() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let base = 0x1000usize;
        let commands: [(u32, u32); 3] = [
            ((0x3fu32 << 24) | (2 << 19) | 7, 0x0000_6000),
            (0x37u32 << 24, 0xf801_f801),
            ((0x2fu32 << 24) | (3 << 20), 0),
        ];
        for (index, (high, low)) in commands.into_iter().enumerate() {
            let offset = base + index * 8;
            board.rdram[offset..offset + 4].copy_from_slice(&high.to_be_bytes());
            board.rdram[offset + 4..offset + 8].copy_from_slice(&low.to_be_bytes());
        }

        let triangle = base + commands.len() * 8;
        let words = [(0x08u32 << 24) | 16, 16u32 << 16, 0, 0, 0, 1u32 << 16, 0, 0];
        for (index, word) in words.into_iter().enumerate() {
            let offset = triangle + index * 4;
            board.rdram[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
        }

        board.dp.current = base as u32;
        board.dp.end = (triangle + 32) as u32;
        board.run_rdp();

        let pixel = |x: usize, y: usize| {
            let offset = 0x6000 + (y * 8 + x) * 2;
            u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]])
        };
        for y in 0..4usize {
            for x in 0..8usize {
                assert_eq!(pixel(x, y), if x <= y { 0xf801 } else { 0 });
            }
        }
        assert_eq!(pixel(0, 4), 0);
        assert_eq!(board.dp.current, board.dp.end);
    }

    #[test]
    fn rdp_texture_triangle_samples_tmem_from_texture_coefficients() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.tiles[0] = RdpTile {
            format: 0,
            size: 2,
            line: 1,
            tmem: 0,
            palette: 0,
            clamp_t: true,
            mirror_t: false,
            mask_t: 0,
            shift_t: 0,
            clamp_s: true,
            mirror_s: false,
            mask_s: 0,
            shift_s: 0,
            sl: 0,
            tl: 0,
            sh: 4,
            th: 0,
        };
        board.tmem[0..2].copy_from_slice(&0xf801u16.to_be_bytes());
        board.tmem[2..4].copy_from_slice(&0x07c1u16.to_be_bytes());

        let base = 0x1000usize;
        write_rdp_words(
            &mut board,
            base,
            &[((0x3fu32 << 24) | (2 << 19) | 7), 0x0000_6000],
        );
        let triangle = base + 8;
        write_rdp_words(&mut board, triangle, &simple_triangle_edges(0x0a));
        write_rdp_words(&mut board, triangle + 32, &constant_texture(1, 0, 1));

        board.dp.current = base as u32;
        board.dp.end = (triangle + 96) as u32;
        board.run_rdp();

        let offset = 0x6000 + (3 * 8 + 1) * 2;
        assert_eq!(
            u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]]),
            0x07c1
        );
    }

    #[test]
    fn rdp_shade_texture_z_triangle_combines_packets_and_depth() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.tiles[0] = RdpTile {
            format: 0,
            size: 2,
            line: 1,
            tmem: 0,
            palette: 0,
            clamp_t: true,
            mirror_t: false,
            mask_t: 0,
            shift_t: 0,
            clamp_s: true,
            mirror_s: false,
            mask_s: 0,
            shift_s: 0,
            sl: 0,
            tl: 0,
            sh: 0,
            th: 0,
        };
        board.tmem[0..2].copy_from_slice(&0xffffu16.to_be_bytes());

        let base = 0x1000usize;
        write_rdp_words(
            &mut board,
            base,
            &[
                ((0x3fu32 << 24) | (2 << 19) | 7),
                0x0000_6000,
                0x3eu32 << 24,
                0x0000_7000,
                ((0x2fu32 << 24) | (1 << 20)),
                0x0000_0030,
            ],
        );
        board.rdram[0x7000..0x7080].fill(0xff);

        let triangle = base + 24;
        write_rdp_words(&mut board, triangle, &simple_triangle_edges(0x0f));
        write_rdp_words(&mut board, triangle + 32, &constant_red_shade());
        write_rdp_words(&mut board, triangle + 96, &constant_texture(0, 0, 1));
        write_rdp_words(&mut board, triangle + 160, &[0x1000_0000, 0, 0, 0]);

        board.dp.current = base as u32;
        board.dp.end = (triangle + 176) as u32;
        board.run_rdp();

        let color_offset = 0x6000 + (3 * 8 + 1) * 2;
        assert_eq!(
            u16::from_be_bytes([board.rdram[color_offset], board.rdram[color_offset + 1]]),
            0xf801
        );
        let (stored_depth, stored_delta_z) = board.rdp_depth_read(1, 3).unwrap();
        assert_eq!(N64Board::rdp_z_decompress(stored_depth), 0x8000);
        assert_eq!(stored_delta_z, 0);
    }

    #[test]
    fn rdp_texture_z_triangle_samples_texture_and_updates_depth() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.tiles[0] = RdpTile {
            format: 0,
            size: 2,
            line: 1,
            tmem: 0,
            palette: 0,
            clamp_t: true,
            mirror_t: false,
            mask_t: 0,
            shift_t: 0,
            clamp_s: true,
            mirror_s: false,
            mask_s: 0,
            shift_s: 0,
            sl: 0,
            tl: 0,
            sh: 0,
            th: 0,
        };
        board.tmem[0..2].copy_from_slice(&0xffffu16.to_be_bytes());

        let base = 0x1000usize;
        write_rdp_words(
            &mut board,
            base,
            &[
                ((0x3fu32 << 24) | (2 << 19) | 7),
                0x0000_6000,
                0x3eu32 << 24,
                0x0000_7000,
                ((0x2fu32 << 24) | (1 << 20)),
                0x0000_0030,
            ],
        );
        board.rdram[0x7000..0x7080].fill(0xff);
        let triangle = base + 24;
        write_rdp_words(&mut board, triangle, &simple_triangle_edges(0x0b));
        write_rdp_words(&mut board, triangle + 32, &constant_texture(0, 0, 1));
        write_rdp_words(&mut board, triangle + 96, &[0x1800_0000, 0, 0, 0]);

        board.dp.current = base as u32;
        board.dp.end = (triangle + 112) as u32;
        board.run_rdp();

        let color_offset = 0x6000 + (3 * 8 + 1) * 2;
        assert_eq!(
            u16::from_be_bytes([board.rdram[color_offset], board.rdram[color_offset + 1]]),
            0xffff
        );
        let (stored_depth, stored_delta_z) = board.rdp_depth_read(1, 3).unwrap();
        assert_eq!(N64Board::rdp_z_decompress(stored_depth), 0xc000);
        assert_eq!(stored_delta_z, 0);
    }

    #[test]
    fn rdp_shade_texture_triangle_combines_attributes_without_z_packet() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.tiles[0] = RdpTile {
            format: 0,
            size: 2,
            line: 1,
            tmem: 0,
            palette: 0,
            clamp_t: true,
            mirror_t: false,
            mask_t: 0,
            shift_t: 0,
            clamp_s: true,
            mirror_s: false,
            mask_s: 0,
            shift_s: 0,
            sl: 0,
            tl: 0,
            sh: 0,
            th: 0,
        };
        board.tmem[0..2].copy_from_slice(&0xffffu16.to_be_bytes());

        let base = 0x1000usize;
        write_rdp_words(
            &mut board,
            base,
            &[((0x3fu32 << 24) | (2 << 19) | 7), 0x0000_6000],
        );
        let triangle = base + 8;
        write_rdp_words(&mut board, triangle, &simple_triangle_edges(0x0e));
        write_rdp_words(&mut board, triangle + 32, &constant_red_shade());
        write_rdp_words(&mut board, triangle + 96, &constant_texture(0, 0, 1));

        board.dp.current = base as u32;
        board.dp.end = (triangle + 160) as u32;
        board.run_rdp();

        let offset = 0x6000 + (3 * 8 + 1) * 2;
        assert_eq!(
            u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]]),
            0xf801
        );
    }

    #[test]
    fn rdp_shade_triangle_reconstructs_rgba_coefficients() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let base = 0x1000usize;
        let setup: [(u32, u32); 1] = [((0x3fu32 << 24) | (2 << 19) | 7, 0x0000_6000)];
        for (index, (high, low)) in setup.into_iter().enumerate() {
            let offset = base + index * 8;
            board.rdram[offset..offset + 4].copy_from_slice(&high.to_be_bytes());
            board.rdram[offset + 4..offset + 8].copy_from_slice(&low.to_be_bytes());
        }

        let triangle = base + setup.len() * 8;
        let edges = simple_triangle_edges(0x0c);
        for (index, word) in edges.into_iter().enumerate() {
            let offset = triangle + index * 4;
            board.rdram[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
        }
        let shade = constant_red_shade();
        for (index, word) in shade.into_iter().enumerate() {
            let offset = triangle + 32 + index * 4;
            board.rdram[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
        }

        board.dp.current = base as u32;
        board.dp.end = (triangle + 96) as u32;
        board.run_rdp();

        let pixel = |x: usize, y: usize| {
            let offset = 0x6000 + (y * 8 + x) * 2;
            u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]])
        };
        for y in 0..4usize {
            for x in 0..y {
                assert_eq!(pixel(x, y), 0xf801);
            }
            assert_eq!(pixel(y, y), 0);
        }
    }

    #[test]
    fn rdp_shade_z_triangle_keeps_attribute_and_depth_packets_aligned() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let base = 0x1000usize;
        let setup: [(u32, u32); 3] = [
            ((0x3fu32 << 24) | (2 << 19) | 7, 0x0000_6000),
            (0x3eu32 << 24, 0x0000_7000),
            ((0x2fu32 << 24) | (1 << 20), 0x0000_0030),
        ];
        for (index, (high, low)) in setup.into_iter().enumerate() {
            let offset = base + index * 8;
            board.rdram[offset..offset + 4].copy_from_slice(&high.to_be_bytes());
            board.rdram[offset + 4..offset + 8].copy_from_slice(&low.to_be_bytes());
        }
        board.rdram[0x7000..0x7080].fill(0xff);

        let triangle = base + setup.len() * 8;
        let edges = simple_triangle_edges(0x0d);
        for (index, word) in edges.into_iter().enumerate() {
            let offset = triangle + index * 4;
            board.rdram[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
        }
        let shade = constant_red_shade();
        for (index, word) in shade.into_iter().enumerate() {
            let offset = triangle + 32 + index * 4;
            board.rdram[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
        }
        for (index, word) in [0x1000_0000u32, 0, 0, 0].into_iter().enumerate() {
            let offset = triangle + 96 + index * 4;
            board.rdram[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
        }

        board.dp.current = base as u32;
        board.dp.end = (triangle + 112) as u32;
        board.run_rdp();

        let color_offset = 0x6000 + (3 * 8 + 2) * 2;
        assert_eq!(
            u16::from_be_bytes([board.rdram[color_offset], board.rdram[color_offset + 1]]),
            0xf801
        );
        let (stored_depth, stored_delta_z) = board.rdp_depth_read(2, 3).unwrap();
        assert_eq!(N64Board::rdp_z_decompress(stored_depth), 0x8000);
        assert_eq!(stored_delta_z, 0);
    }

    #[test]
    fn rdp_fill_z_triangle_updates_and_compares_depth() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let base = 0x1000usize;
        let setup: [(u32, u32); 4] = [
            ((0x3fu32 << 24) | (2 << 19) | 7, 0x0000_6000),
            (0x3eu32 << 24, 0x0000_7000),
            (0x2fu32 << 24, 0x0000_0030),
            (0x3au32 << 24, 0xf800_00ff),
        ];
        for (index, (high, low)) in setup.into_iter().enumerate() {
            let offset = base + index * 8;
            board.rdram[offset..offset + 4].copy_from_slice(&high.to_be_bytes());
            board.rdram[offset + 4..offset + 8].copy_from_slice(&low.to_be_bytes());
        }
        board.rdram[0x7000..0x7080].fill(0xff);

        let first = base + setup.len() * 8;
        let near = [
            (0x09u32 << 24) | 16,
            16u32 << 16,
            0,
            0,
            0,
            1u32 << 16,
            0,
            0,
            0x1000_0000,
            0,
            0,
            0,
        ];
        for (index, word) in near.into_iter().enumerate() {
            let offset = first + index * 4;
            board.rdram[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
        }

        let recolor = first + 48;
        board.rdram[recolor..recolor + 4].copy_from_slice(&(0x3au32 << 24).to_be_bytes());
        board.rdram[recolor + 4..recolor + 8].copy_from_slice(&0x00f8_00ffu32.to_be_bytes());

        let second = recolor + 8;
        let far = [
            (0x09u32 << 24) | 16,
            16u32 << 16,
            0,
            0,
            0,
            1u32 << 16,
            0,
            0,
            0x2000_0000,
            0,
            0,
            0,
        ];
        for (index, word) in far.into_iter().enumerate() {
            let offset = second + index * 4;
            board.rdram[offset..offset + 4].copy_from_slice(&word.to_be_bytes());
        }

        board.combine_mode = combine_one_cycle([15, 15, 31, 3], [7, 7, 7, 3]);
        board.dp.current = base as u32;
        board.dp.end = (second + 48) as u32;
        board.run_rdp();

        let color = |x: usize, y: usize| {
            let offset = 0x6000 + (y * 8 + x) * 2;
            u16::from_be_bytes([board.rdram[offset], board.rdram[offset + 1]])
        };
        let depth = |x: usize, y: usize| {
            let (stored_depth, stored_delta_z) = board.rdp_depth_read(x, y).unwrap();
            assert_eq!(stored_delta_z, 0);
            N64Board::rdp_z_decompress(stored_depth)
        };
        for y in 0..4usize {
            for x in 0..y {
                assert_eq!(color(x, y), 0xf801);
                assert_eq!(depth(x, y), 0x8000);
            }
            assert_eq!(color(y, y), 0);
        }
    }

    #[test]
    fn rdp_fill_reaches_vi_surface_and_raises_dp_interrupt() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let commands: [(u32, u32); 5] = [
            ((0x3fu32 << 24) | (2 << 19) | 3, 0x0000_2000),
            ((0x2fu32 << 24) | (3 << 20), 0),
            (0x37u32 << 24, 0xf801_f801),
            ((0x36u32 << 24) | (3 << 14) | (3 << 2), 0),
            (0x29u32 << 24, 0),
        ];
        for (index, (high, low)) in commands.into_iter().enumerate() {
            let offset = 0x1000 + index * 8;
            board.rdram[offset..offset + 4].copy_from_slice(&high.to_be_bytes());
            board.rdram[offset + 4..offset + 8].copy_from_slice(&low.to_be_bytes());
        }
        board.dp.start = 0x1000;
        board.dp.current = 0x1000;
        board.dp.end = 0x1028;
        board.run_rdp();
        assert_ne!(board.mi_intr & MI_DP, 0);
        board.vi.regs[0] = 2;
        board.vi.regs[1] = 0x2000;
        board.vi.regs[2] = 4;
        board.vi.regs[10] = 8;
        let mut video = VideoBuffer::new(WIDTH, HEIGHT);
        board.render_vi(&mut video);
        assert!(video.pixels()[0] > 200 && video.pixels()[1] < 16);
    }
    #[test]
    fn vi_divot_filter_uses_coverage_gated_channel_medians() {
        assert_eq!(
            N64Board::vi_divot_color(
                [250, 50, 100, 255],
                [10, 100, 200, 255],
                [20, 200, 50, 255],
                [3, 7, 7],
            ),
            [20, 100, 100, 255]
        );
        assert_eq!(
            N64Board::vi_divot_color(
                [250, 50, 100, 255],
                [10, 100, 200, 255],
                [20, 200, 50, 255],
                [7, 7, 7],
            ),
            [250, 50, 100, 255]
        );

        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let origin = 0x6000usize;
        let width = 5usize;
        let write = |board: &mut N64Board, x: usize, color: [u8; 3], coverage: u8| {
            let address = origin + x * 4;
            board.rdram[address..address + 4].copy_from_slice(&[
                color[0],
                color[1],
                color[2],
                coverage << 5,
            ]);
        };
        write(&mut board, 1, [10, 100, 200], 7);
        write(&mut board, 2, [250, 50, 100], 3);
        write(&mut board, 3, [20, 200, 50], 7);
        board.vi.regs[0] = 3 | (1 << 4) | (2 << 8);

        let center = board.vi_filter_rgba32(origin, width, 2, 0).unwrap();
        assert_eq!(
            board.vi_apply_divot32(origin, width, 2, 0, center),
            [20, 100, 100, 255]
        );
    }

    #[test]
    fn vi_rgba32_antialias_and_restore_filters_follow_coverage_controls() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        let origin = 0x6000usize;
        let width = 8usize;
        let write = |board: &mut N64Board, x: usize, y: usize, color: [u8; 3], coverage: u8| {
            let address = origin + (y * width + x) * 4;
            board.rdram[address..address + 4].copy_from_slice(&[
                color[0],
                color[1],
                color[2],
                coverage << 5,
            ]);
        };

        write(&mut board, 3, 1, [128, 128, 128], 3);
        for (x, y) in [(2, 0), (4, 0), (1, 1), (5, 1), (2, 2), (4, 2)] {
            write(&mut board, x, y, [255, 255, 255], 7);
        }
        board.vi.regs[0] = 3;
        assert_eq!(
            board.vi_filter_rgba32(origin, width, 3, 1),
            Some([192, 192, 192, 255])
        );

        write(&mut board, 3, 1, [128, 128, 128], 7);
        for (x, y) in [
            (2, 0),
            (3, 0),
            (4, 0),
            (2, 1),
            (4, 1),
            (2, 2),
            (3, 2),
            (4, 2),
        ] {
            write(&mut board, x, y, [255, 255, 255], 7);
        }
        board.vi.regs[0] = 3 | (1 << 16);
        assert_eq!(
            board.vi_filter_rgba32(origin, width, 3, 1),
            Some([136, 136, 136, 255])
        );
    }

    #[test]
    fn vi_rgba16_restore_filter_uses_full_neighbor_window() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 8;
        board.color_size = 2;
        board.color_format = 0;
        board.write_rdp_coverage_pixel(3, 1, [128, 128, 128, 255], 7);
        for (x, y) in [
            (2, 0),
            (3, 0),
            (4, 0),
            (2, 1),
            (4, 1),
            (2, 2),
            (3, 2),
            (4, 2),
        ] {
            board.write_rdp_coverage_pixel(x, y, [255, 255, 255, 255], 7);
        }

        board.vi.regs[0] = 2 | (1 << 16);
        assert_eq!(
            board.vi_filter_rgba16(0x6000, 8, 3, 1),
            Some([139, 139, 139, 255])
        );
    }

    #[test]
    fn vi_rgba16_antialias_filter_uses_hidden_coverage_neighbors() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.color_image = 0x6000;
        board.color_width = 8;
        board.color_size = 2;
        board.color_format = 0;
        board.vi.regs[0] = 2;

        board.write_rdp_coverage_pixel(3, 1, [128, 128, 128, 255], 3);
        for (x, y) in [(2, 0), (4, 0), (1, 1), (5, 1), (2, 2), (4, 2)] {
            board.write_rdp_coverage_pixel(x, y, [255, 255, 255, 255], 7);
        }

        assert_eq!(
            board.vi_filter_rgba16(0x6000, 8, 3, 1),
            Some([193, 193, 193, 255])
        );

        board.vi.regs[0] = 2 | (2 << 8);
        assert_eq!(
            board.vi_filter_rgba16(0x6000, 8, 3, 1),
            Some([131, 131, 131, 255])
        );
    }

    #[test]
    fn vi_gamma_and_gamma_dither_match_angrylion_tables() {
        assert_eq!(N64Board::vi_integer_sqrt(0), 0);
        assert_eq!(N64Board::vi_integer_sqrt(4_096), 64);
        assert_eq!(N64Board::vi_integer_sqrt(12_800), 113);

        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.vi.regs[0] = 2 | (1 << 3);
        assert_eq!(
            board.vi_apply_gamma([64, 100, 200, 0]),
            [128, 160, 226, 255]
        );

        board.rdp_noise_seed = 1;
        let random = board.rdp_next_random();
        board.rdp_noise_seed = 1;
        board.vi.regs[0] = 2 | (1 << 2);
        let dithered = board.vi_apply_gamma([100, 100, 100, 0]);
        assert_eq!(
            dithered,
            [
                100 + (random & 1) as u8,
                100 + ((random >> 1) & 1) as u8,
                100 + ((random >> 2) & 1) as u8,
                255,
            ]
        );

        board.rdp_noise_seed = 1;
        let random = board.rdp_next_random();
        board.rdp_noise_seed = 1;
        board.vi.regs[0] = 2 | (3 << 2);
        let combined = board.vi_apply_gamma([64, 100, 200, 0]);
        let dithers = [
            random & 0x3f,
            (random >> 6) & 0x3f,
            ((random >> 9) & 0x38) | (random & 7),
        ];
        assert_eq!(
            combined,
            [
                (N64Board::vi_integer_sqrt((64 << 6) | dithers[0]) << 1) as u8,
                (N64Board::vi_integer_sqrt((100 << 6) | dithers[1]) << 1) as u8,
                (N64Board::vi_integer_sqrt((200 << 6) | dithers[2]) << 1) as u8,
                255,
            ]
        );

        board.vi.regs[0] = 3 | (1 << 3);
        board.vi.regs[1] = 0x2000;
        board.vi.regs[2] = 1;
        board.vi.regs[10] = 2;
        board.rdram[0x2000..0x2004].copy_from_slice(&[64, 100, 200, 0]);
        let mut video = VideoBuffer::new(WIDTH, HEIGHT);
        board.render_vi(&mut video);
        assert_eq!(&video.pixels()[..4], &[128, 160, 226, 255]);
    }

    #[test]
    fn vi_serrate_alternates_and_preserves_interlaced_fields() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.vi.regs[0] = 3 | (1 << 6) | (3 << 8);
        board.vi.regs[1] = 0x2000;
        board.vi.regs[2] = 16;
        board.vi.regs[9] = (108 << 16) | 130;
        board.vi.regs[10] = (34 << 16) | 38;
        board.vi.regs[12] = 0x400;
        board.vi.regs[13] = 0x800;

        let write = |board: &mut N64Board, y: usize, color: [u8; 3]| {
            let address = 0x2000 + (y * 16 + 8) * 4;
            board.rdram[address..address + 4].copy_from_slice(&[
                color[0],
                color[1],
                color[2],
                7 << 5,
            ]);
        };
        write(&mut board, 0, [255, 0, 0]);
        write(&mut board, 1, [0, 255, 0]);
        write(&mut board, 2, [0, 0, 255]);
        write(&mut board, 3, [255, 255, 255]);

        let pixel = |video: &VideoBuffer, x: usize, y: usize| {
            let offset = (y * WIDTH as usize + x) * 4;
            [
                video.pixels()[offset],
                video.pixels()[offset + 1],
                video.pixels()[offset + 2],
                video.pixels()[offset + 3],
            ]
        };

        let mut video = VideoBuffer::new(WIDTH, HEIGHT);
        video.clear([0, 0, 0, 255]);

        board.end_frame(&mut video);
        assert_eq!(board.vi.current & 1, 1);
        assert_eq!(pixel(&video, 8, 0), [0, 0, 0, 255]);
        assert_eq!(pixel(&video, 8, 1), [0, 255, 0, 255]);

        board.end_frame(&mut video);
        assert_eq!(board.vi.current & 1, 0);
        assert_eq!(pixel(&video, 8, 0), [255, 0, 0, 255]);
        assert_eq!(pixel(&video, 8, 1), [0, 255, 0, 255]);
        assert_eq!(pixel(&video, 8, 2), [0, 0, 255, 255]);
    }

    #[test]
    fn vi_programmed_geometry_lerps_filtered_source_pixels() {
        assert_eq!(
            N64Board::vi_lerp([0, 0, 0, 255], [128, 128, 128, 255], 16),
            [64, 64, 64, 255]
        );

        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.vi.regs[0] = 3 | (2 << 8);
        board.vi.regs[1] = 0x2000;
        board.vi.regs[2] = 16;
        board.vi.regs[9] = (108 << 16) | 130;
        board.vi.regs[10] = (34 << 16) | 36;
        board.vi.regs[12] = (0x200 << 16) | 0x400;
        board.vi.regs[13] = (0x400 << 16) | 0x800;

        let write = |board: &mut N64Board, x: usize, y: usize, value: u8| {
            let address = 0x2000 + (y * 16 + x) * 4;
            board.rdram[address..address + 4].copy_from_slice(&[value, value, value, 7 << 5]);
        };
        write(&mut board, 8, 0, 0);
        write(&mut board, 9, 0, 64);
        write(&mut board, 8, 1, 128);
        write(&mut board, 9, 1, 255);

        let pixel = |video: &VideoBuffer, x: usize, y: usize| {
            let offset = (y * WIDTH as usize + x) * 4;
            [
                video.pixels()[offset],
                video.pixels()[offset + 1],
                video.pixels()[offset + 2],
                video.pixels()[offset + 3],
            ]
        };

        let mut video = VideoBuffer::new(WIDTH, HEIGHT);
        board.render_vi(&mut video);
        assert_eq!(pixel(&video, 8, 0), [112, 112, 112, 255]);

        board.vi.regs[0] = 3 | (3 << 8);
        board.render_vi(&mut video);
        assert_eq!(pixel(&video, 8, 0), [0, 0, 0, 255]);
    }

    #[test]
    fn vi_programmed_geometry_uses_start_scale_guard_band_and_subpixels() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.vi.regs[0] = 2;
        board.vi.regs[1] = 0x2000;
        board.vi.regs[2] = 16;
        board.vi.regs[9] = (108 << 16) | 130;
        board.vi.regs[10] = (34 << 16) | 36;
        board.vi.regs[12] = 0x400;
        board.vi.regs[13] = 0x800;

        let write_pixel = |board: &mut N64Board, x: usize, y: usize, value: u16| {
            let address = 0x2000 + (y * 16 + x) * 2;
            board.rdram[address..address + 2].copy_from_slice(&value.to_be_bytes());
        };
        write_pixel(&mut board, 8, 0, 0xf801);
        write_pixel(&mut board, 9, 0, 0x07c1);
        write_pixel(&mut board, 8, 1, 0x003f);
        write_pixel(&mut board, 9, 1, 0xffff);

        let pixel = |video: &VideoBuffer, x: usize, y: usize| {
            let offset = (y * WIDTH as usize + x) * 4;
            [
                video.pixels()[offset],
                video.pixels()[offset + 1],
                video.pixels()[offset + 2],
                video.pixels()[offset + 3],
            ]
        };

        let mut video = VideoBuffer::new(WIDTH, HEIGHT);
        board.render_vi(&mut video);
        assert_eq!(pixel(&video, 7, 0), [0, 0, 0, 255]);
        assert_eq!(pixel(&video, 8, 0), [255, 0, 0, 255]);
        assert_eq!(pixel(&video, 9, 0), [0, 255, 0, 255]);
        assert_eq!(pixel(&video, 8, 1), [0, 0, 255, 255]);

        board.vi.regs[12] = (0x400 << 16) | 0x400;
        board.vi.regs[13] = (0x800 << 16) | 0x800;
        board.render_vi(&mut video);
        assert_eq!(pixel(&video, 8, 0), [255, 255, 255, 255]);
    }

    #[test]
    fn vi_resamples_source_surface_across_every_output_pixel() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.vi.regs[0] = 2;
        board.vi.regs[1] = 0x2000;
        board.vi.regs[2] = 2;
        board.vi.regs[10] = 4;
        board.rdram[0x2000..0x2002].copy_from_slice(&0xf801u16.to_be_bytes());
        board.rdram[0x2002..0x2004].copy_from_slice(&0x07c1u16.to_be_bytes());

        let mut video = VideoBuffer::new(WIDTH, HEIGHT);
        board.render_vi(&mut video);

        let left = &video.pixels()[0..4];
        let middle = &video.pixels()[(WIDTH as usize / 2) * 4..(WIDTH as usize / 2) * 4 + 4];
        let right = &video.pixels()[(WIDTH as usize - 1) * 4..WIDTH as usize * 4];
        assert!(left[0] > 200 && left[1] < 16);
        assert!(middle[1] > 200 && middle[0] < 16);
        assert!(right[1] > 200 && right[0] < 16);
        assert!(video.pixels()[4] > 200);
    }

    #[test]
    fn si_dma_defers_transfer_and_interrupt_until_hardware_latency() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.si.dram_addr = 0x1200;
        board.rdram[0x1200] = 0xfe;

        board.si_dma(true);
        assert_eq!(board.si.dma_cycles, 4_065 * 3);
        assert_eq!(board.si.dma_direction, 2);
        assert_ne!(board.si.status & 1, 0);
        assert_eq!((board.si.status >> 4) & 0x0f, 1);
        assert_eq!((board.si.status >> 8) & 0x0f, 4);
        assert_eq!(board.pif_ram[0], 0);
        assert_eq!(board.mi_intr & MI_SI, 0);

        board.tick_si(4_065 * 3 - 1);
        assert_eq!(board.pif_ram[0], 0);
        assert_ne!(board.si.status & 1, 0);
        board.tick_si(1);
        assert_eq!(board.pif_ram[0], 0xfe);
        assert_eq!(board.si.status & 1, 0);
        assert_ne!(board.si.status & 0x1000, 0);
        assert_ne!(board.mi_intr & MI_SI, 0);

        board.write32(0x0480_0018, 0);
        assert_eq!(board.si.status & 0x1000, 0);
        assert_eq!(board.mi_intr & MI_SI, 0);

        board.pif_ram.fill(0);
        board.pif_ram[0] = 0xfe;
        board.si.dram_addr = 0x1300;
        board.rdram[0x1300..0x1340].fill(0);
        let expected = (13_600 + 1_420) * 3;
        assert_eq!(board.estimate_si_read_cycles(), expected);
        board.si_dma(false);
        assert_eq!(board.si.dma_cycles, expected);
        assert_eq!(board.si.dma_direction, 1);
        assert_eq!((board.si.status >> 4) & 0x0f, 4);
        assert_eq!((board.si.status >> 8) & 0x0f, 1);
        assert_eq!(board.rdram[0x1300], 0);
        board.tick_si(expected);
        assert_eq!(board.rdram[0x1300], 0xfe);
        assert_ne!(board.mi_intr & MI_SI, 0);
    }

    #[test]
    fn pi_dma_busy_error_completion_and_interrupt_follow_bsd_timing() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.pi.dram_addr = 0x400;
        board.pi.cart_addr = 0x1000_0000;
        board.pi.timing[..4].copy_from_slice(&[5, 4, 2, 1]);
        let expected_cycles = board.pi_dma_duration(15);

        board.pi_dma(15, true);
        assert_eq!(&board.rdram[0x400..0x410], &board.rom[..16]);
        assert_eq!(board.pi.dma_cycles, expected_cycles);
        assert_ne!(board.pi.status & 1, 0);
        assert_eq!(board.pi.status & (1 << 3), 0);
        assert_eq!(board.mi_intr & MI_PI, 0);

        let dram_after_dma = board.pi.dram_addr;
        board.write32(0x0460_0000, 0x1234);
        assert_eq!(board.pi.dram_addr, dram_after_dma);
        assert_ne!(board.pi.status & (1 << 2), 0);

        board.tick_pi(expected_cycles.saturating_sub(1));
        assert_ne!(board.pi.status & 1, 0);
        assert_eq!(board.mi_intr & MI_PI, 0);
        board.tick_pi(1);
        assert_eq!(board.pi.status & 1, 0);
        assert_ne!(board.pi.status & (1 << 3), 0);
        assert_ne!(board.mi_intr & MI_PI, 0);

        board.write_mi_mask(1 << 9);
        assert!(board.cpu_irq());
        board.write_pi_status(2);
        assert_eq!(board.pi.status & (1 << 3), 0);
        assert_eq!(board.mi_intr & MI_PI, 0);

        board.pi.dram_addr = 0x500;
        board.pi.cart_addr = 0x1000_0000;
        board.pi_dma(31, true);
        assert_ne!(board.pi.status & 1, 0);
        board.write_pi_status(1);
        assert_eq!(board.pi.status & ((1 << 2) | 1), 0);
        assert_eq!(board.pi.dma_cycles, 0);
    }

    #[test]
    fn ai_fifo_status_pause_and_descriptor_promotion_match_hardware() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.rdram[0x800..0x808]
            .copy_from_slice(&[0x40, 0x00, 0xc0, 0x00, 0x20, 0x00, 0xe0, 0x00]);
        board.rdram[0x900..0x908]
            .copy_from_slice(&[0x10, 0x00, 0xf0, 0x00, 0x08, 0x00, 0xf8, 0x00]);

        board.write_ai_dram_addr(0x800);
        board.queue_ai_dma(8);
        assert_eq!(board.ai.dma_count, 1);
        assert_eq!(
            board.ai.status & ((1 << 20) | (1 << 24) | (1 << 30)),
            (1 << 20) | (1 << 24) | (1 << 30)
        );
        assert_eq!(board.ai.status & ((1 << 31) | 1), 0);
        assert_ne!(board.mi_intr & MI_AI, 0);
        board.clear(MI_AI);

        board.write_ai_dram_addr(0x900);
        board.queue_ai_dma(8);
        assert_eq!(board.ai.dma_count, 2);
        assert_eq!(board.ai.next_dram_addr, 0x900);
        assert_eq!(board.ai.status & ((1 << 31) | 1), (1 << 31) | 1);

        board.write_ai_dram_addr(0xa00);
        board.queue_ai_dma(0x20);
        assert_eq!(board.ai.next_dram_addr, 0x900);
        assert_eq!(board.ai.next_len, 8);

        board.ai_dac_sample();
        assert_eq!(board.ai.dram_addr, 0x800);
        assert_eq!(board.ai.len, 8);

        board.ai.control = 1;
        board.update_ai_status();
        assert_ne!(board.ai.status & (1 << 25), 0);
        board.ai_dac_sample();
        assert_eq!(
            (board.ai.sample_left, board.ai.sample_right),
            (0x4000, -0x4000)
        );
        assert_eq!(board.ai.len, 4);

        board.ai_dac_sample();
        assert_eq!(
            (board.ai.sample_left, board.ai.sample_right),
            (0x2000, -0x2000)
        );
        assert_eq!(board.ai.dma_count, 1);
        assert_eq!(board.ai.dram_addr, 0x900);
        assert_eq!(board.ai.len, 8);
        assert_ne!(board.mi_intr & MI_AI, 0);
        board.clear(MI_AI);

        board.ai_dac_sample();
        board.ai_dac_sample();
        assert_eq!(board.ai.dma_count, 0);
        assert_eq!(board.ai.status & ((1 << 31) | (1 << 30) | 1), 0);
        assert_eq!(board.mi_intr & MI_AI, 0);
    }

    #[test]
    fn ai_clock_consumes_dac_samples_and_emits_fixed_host_rate() {
        let mut board = N64Board::new(&rom_with_loop()).unwrap();
        board.ai.dac_rate = 1_103;
        board.ai.control = 1;
        board.rdram[0x1000..0x5000].fill(0x20);
        board.write_ai_dram_addr(0x1000);
        board.queue_ai_dma(0x3ff8);
        board.audio_samples.clear();
        board.clear(MI_AI);

        let cycles = (CPU_HZ / FRAME_HZ) as u32;
        let initial_length = board.ai.len;
        board.tick_ai(cycles);

        let expected_dac_samples =
            (u64::from(cycles) * NTSC_VIDEO_CLOCK_HZ / (CPU_HZ * 1_104)) as u32;
        assert_eq!(
            initial_length - board.ai.len,
            expected_dac_samples.saturating_mul(4)
        );
        assert_eq!(
            board.audio_samples.len(),
            (AUDIO_RATE / FRAME_HZ as u32) as usize
        );
        assert_ne!(board.ai.status & (1 << 30), 0);
    }

    #[test]
    fn machine_state_and_persistent_memories_round_trip() {
        let mut machine = Nintendo64Machine::from_rom(&rom_with_loop()).unwrap();
        let save = vec![0x5a; SAVE_SIZE];
        let pak = vec![0xa5; CONTROLLER_PAK_SIZE];
        machine
            .write_persistent(ResourceKind::Storage, 0, &save)
            .unwrap();
        machine
            .write_persistent(ResourceKind::MemoryCard, 0, &pak)
            .unwrap();
        machine.board.rdram[0x1234] = 0x77;
        machine.board.ai.dram_addr = 0x1800;
        machine.board.ai.len = 0x80;
        machine.board.ai.next_dram_addr = 0x1c00;
        machine.board.ai.next_len = 0x100;
        machine.board.ai.dma_count = 2;
        machine.board.ai.control = 1;
        machine.board.ai.dac_rate = 1_103;
        machine.board.ai.bit_rate = 15;
        machine.board.ai.sample_phase = 0x1234_5678;
        machine.board.ai.output_phase = 0x2345_6789;
        machine.board.ai.sample_left = 0x2345;
        machine.board.ai.sample_right = -0x1234;
        machine.board.update_ai_status();
        machine.board.pi.status = 1;
        machine.board.pi.dma_cycles = 0x4567;
        machine.board.si.status = 1 | (4 << 4) | (1 << 8);
        machine.board.si.dma_cycles = 0x5678;
        machine.board.si.dma_direction = 1;
        let state = machine.save_state().unwrap();
        machine.board.rdram[0x1234] = 0;
        machine.board.save.fill(0);
        machine.board.controller_pak.fill(0);
        machine.load_state(&state).unwrap();
        assert_eq!(machine.board.rdram[0x1234], 0x77);
        assert_eq!(machine.board.ai.dram_addr, 0x1800);
        assert_eq!(machine.board.ai.len, 0x80);
        assert_eq!(machine.board.ai.next_dram_addr, 0x1c00);
        assert_eq!(machine.board.ai.next_len, 0x100);
        assert_eq!(machine.board.ai.dma_count, 2);
        assert_eq!(machine.board.ai.dac_rate, 1_103);
        assert_eq!(machine.board.ai.sample_phase, 0x1234_5678);
        assert_eq!(machine.board.ai.output_phase, 0x2345_6789);
        assert_eq!(machine.board.ai.sample_left, 0x2345);
        assert_eq!(machine.board.ai.sample_right, -0x1234);
        assert_eq!(machine.board.pi.status & 1, 1);
        assert_eq!(machine.board.pi.dma_cycles, 0x4567);
        assert_eq!(machine.board.si.status, 1 | (4 << 4) | (1 << 8));
        assert_eq!(machine.board.si.dma_cycles, 0x5678);
        assert_eq!(machine.board.si.dma_direction, 1);
        assert_eq!(machine.board.save[0], 0x5a);
        assert_eq!(machine.board.controller_pak[0], 0xa5);
        assert_eq!(machine.save_state().unwrap(), state);
        let mut restored_save = vec![0; SAVE_SIZE];
        let mut restored_pak = vec![0; CONTROLLER_PAK_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut restored_save)
            .unwrap();
        machine
            .read_persistent(ResourceKind::MemoryCard, 0, &mut restored_pak)
            .unwrap();
        assert_eq!(restored_save, save);
        assert_eq!(restored_pak, pak);
    }
}
