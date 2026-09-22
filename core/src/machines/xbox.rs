use crate::cpu_x86::{X86Bus, X86Cpu, EAX, ECX, EDX, ESP};
use std::collections::{BTreeMap, BTreeSet};

use crate::input::{
    AXIS_LEFT_TRIGGER, AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_TRIGGER, AXIS_RIGHT_X, AXIS_RIGHT_Y,
    DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, L1, L3, LEFT, R1, R3, RIGHT, SELECT, START,
    UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::{ResourceBlob, ResourceKind};
use crate::state::{StateReader, StateWriter};

const STATE_VERSION: u32 = 15;
const RAM_SIZE: usize = 64 * 1024 * 1024;
const NV2A_SIZE: usize = 16 * 1024 * 1024;
const APU_SIZE: usize = 512 * 1024;
const AC97_SIZE: usize = 4 * 1024;
const FRAME_RATE: f64 = 60.0;
const AUDIO_RATE: u32 = 48_000;
const CPU_STEPS_PER_FRAME: u64 = 300_000;
const VIDEO_WIDTH: u32 = 640;
const VIDEO_HEIGHT: u32 = 480;

const NV2A_BASE: u32 = 0xfd00_0000;
const APU_BASE: u32 = 0xfe80_0000;
const AC97_BASE: u32 = 0xfec0_0000;
const USB0_BASE: u32 = 0xfed0_0000;
const USB1_BASE: u32 = 0xfed0_8000;
const USB_MMIO_SIZE: u32 = 0x1000;
const PCRTC_START: usize = 0x600800;
const PCRTC_INTR: usize = 0x600100;
const PCRTC_INTR_EN: usize = 0x600140;
const XBE_MAGIC: u32 = 0x4845_4258;
const XBE_ENTRY_RETAIL_XOR: u32 = 0xa8fc_57ab;
const XBE_ENTRY_DEBUG_XOR: u32 = 0x9485_9d4b;
const XBE_THUNK_RETAIL_XOR: u32 = 0x5b6d_40b6;
const XBE_THUNK_DEBUG_XOR: u32 = 0xefb1_f152;
const XBOX_KERNEL_EXPORT_COUNT: u32 = 379;
const XBOX_KERNEL_HLE_BASE: u32 = 0xfff0_0000;
const XBOX_KERNEL_HLE_STRIDE: u32 = 16;
const XBOX_KERNEL_DATA_BASE: u32 = 0xffe0_0000;
const XBOX_KERNEL_DATA_STRIDE: usize = 16;
const XBOX_KERNEL_DATA_SIZE: usize = XBOX_KERNEL_EXPORT_COUNT as usize * XBOX_KERNEL_DATA_STRIDE;
const XBOX_KERNEL_HEAP_ALIAS: u32 = 0x8000_0000;
const XBOX_KERNEL_THREAD_RESERVE: u32 = 0x1000;
const XBOX_KERNEL_HEAP_LIMIT: u32 = 0x83c0_0000;
const XBOX_PAGE_READWRITE: u32 = 0x04;
const XBOX_GPU_INSTANCE_BASE: u32 = 0x03fe_0000;
const XBOX_GPU_INSTANCE_BYTES: u32 = 0x0001_0000;
const XBOX_GPU_INSTANCE_RETURN: u32 =
    XBOX_KERNEL_HEAP_ALIAS + XBOX_GPU_INSTANCE_BASE + XBOX_GPU_INSTANCE_BYTES;
const GENERIC_READ: u32 = 0x8000_0000;
const GENERIC_WRITE: u32 = 0x4000_0000;
const GENERIC_EXECUTE: u32 = 0x2000_0000;
const GENERIC_ALL: u32 = 0x1000_0000;
const STATUS_SUCCESS: u32 = 0x0000_0000;
const STATUS_BUFFER_OVERFLOW: u32 = 0x8000_0005;
const STATUS_ACCESS_VIOLATION: u32 = 0xc000_0005;
const STATUS_INVALID_PARAMETER: u32 = 0xc000_000d;
const STATUS_NO_MEMORY: u32 = 0xc000_0017;
const STATUS_BUFFER_TOO_SMALL: u32 = 0xc000_0023;
const STATUS_OBJECT_NAME_NOT_FOUND: u32 = 0xc000_0034;
const STATUS_SHARING_VIOLATION: u32 = 0xc000_0043;
const STATUS_SUSPEND_COUNT_EXCEEDED: u32 = 0xc000_004a;
const STATUS_INVALID_PARAMETER_2: u32 = 0xc000_00f0;
const FILE_READ_DATA: u32 = 0x0000_0001;
const FILE_WRITE_DATA: u32 = 0x0000_0002;
const FILE_APPEND_DATA: u32 = 0x0000_0004;
const FILE_EXECUTE: u32 = 0x0000_0020;
const DELETE_ACCESS: u32 = 0x0001_0000;
const FILE_SHARE_READ: u32 = 0x0000_0001;
const FILE_SHARE_WRITE: u32 = 0x0000_0002;
const FILE_SHARE_DELETE: u32 = 0x0000_0004;
const STATUS_INVALID_DEVICE_REQUEST: u32 = 0xc000_0010;
const XBOX_IRP_SIZE: u32 = 0x64;
const XBOX_IO_STACK_LOCATION_SIZE: u32 = 0x18;
const XBOX_IO_TYPE_IRP: u16 = 6;
const REG_BINARY: u32 = 3;
const REG_DWORD: u32 = 4;

const SMBUS_BASE: u16 = 0xc000;
const SMBUS_STATUS_PROTOCOL_ERROR: u8 = 1 << 2;
const SMBUS_STATUS_HOST_BUSY: u8 = 1 << 3;
const SMBUS_STATUS_CYCLE_COMPLETE: u8 = 1 << 4;
const SMBUS_CONTROL_START: u8 = 1 << 3;
const SMBUS_CONTROL_ABORT: u8 = 1 << 5;
const SMBUS_SMC_ADDRESS: u8 = 0x10;
const SMBUS_EEPROM_ADDRESS: u8 = 0x54;
const EEPROM_SIZE: usize = 256;

const IDE_PRIMARY_BASE: u16 = 0x01f0;
const IDE_PRIMARY_CONTROL: u16 = 0x03f6;
const ATA_STATUS_BSY: u8 = 0x80;
const ATA_STATUS_DRDY: u8 = 0x40;
const ATA_STATUS_DRQ: u8 = 0x08;
const ATA_STATUS_ERR: u8 = 0x01;
const ATA_ERROR_ABORT: u8 = 0x04;
const ATAPI_SECTOR_SIZE: usize = 2048;

const PCI_CONFIG_ADDRESS: u16 = 0x0cf8;
const PCI_CONFIG_DATA: u16 = 0x0cfc;
const PCI_VENDOR_NVIDIA: u16 = 0x10de;
const PCI_FUNCTION_COUNT: usize = 11;
const PCI_FUNCTIONS: [(u8, u8, u8); PCI_FUNCTION_COUNT] = [
    (0, 0, 0),
    (0, 1, 0),
    (0, 1, 1),
    (0, 2, 0),
    (0, 3, 0),
    (0, 4, 0),
    (0, 5, 0),
    (0, 6, 0),
    (0, 9, 0),
    (0, 30, 0),
    (1, 0, 0),
];

const OHCI_CONTROL_PLE: u32 = 1 << 2;
const OHCI_CONTROL_CLE: u32 = 1 << 4;
const OHCI_CONTROL_BLE: u32 = 1 << 5;
const OHCI_CONTROL_OPERATIONAL: u32 = 2 << 6;
const OHCI_COMMAND_HCR: u32 = 1;
const OHCI_INTR_WD: u32 = 1 << 1;
const OHCI_INTR_SF: u32 = 1 << 2;
const OHCI_INTR_RHSC: u32 = 1 << 6;
const OHCI_INTR_MIE: u32 = 1 << 31;
const OHCI_PORT_CCS: u32 = 1 << 0;
const OHCI_PORT_PES: u32 = 1 << 1;
const OHCI_PORT_PSS: u32 = 1 << 2;
const OHCI_PORT_PRS: u32 = 1 << 4;
const OHCI_PORT_PPS: u32 = 1 << 8;
const OHCI_PORT_CSC: u32 = 1 << 16;
const OHCI_PORT_PESC: u32 = 1 << 17;
const OHCI_PORT_PSSC: u32 = 1 << 18;
const OHCI_PORT_PRSC: u32 = 1 << 20;
const OHCI_PORT_WTC: u32 = OHCI_PORT_CSC | OHCI_PORT_PESC | OHCI_PORT_PSSC | OHCI_PORT_PRSC;
fn le32(bytes: &[u8], offset: usize) -> Result<u32, String> {
    let end = offset
        .checked_add(4)
        .ok_or_else(|| "XBE offset overflow".to_string())?;
    let value = bytes
        .get(offset..end)
        .ok_or_else(|| "XBE header is truncated".to_string())?;
    Ok(u32::from_le_bytes(value.try_into().unwrap()))
}

fn ram_index(address: u32) -> Option<usize> {
    match address {
        0x0000_0000..=0x03ff_ffff => Some(address as usize),
        0xf000_0000..=0xf3ff_ffff => Some((address - 0xf000_0000) as usize),
        0x8000_0000..=0x83ff_ffff => Some((address - 0x8000_0000) as usize),
        0xa000_0000..=0xa3ff_ffff => Some((address - 0xa000_0000) as usize),
        _ => None,
    }
}

fn ram_range(address: u32, length: u32) -> Option<std::ops::Range<usize>> {
    let start = ram_index(address)?;
    if length == 0 {
        return Some(start..start);
    }
    let last_address = address.checked_add(length - 1)?;
    let end = ram_index(last_address)?.checked_add(1)?;
    (end >= start && end - start == length as usize).then_some(start..end)
}

fn rtl_lower_char(character: u8) -> u8 {
    if character.is_ascii_uppercase() || ((0xc0..=0xde).contains(&character) && character != 0xd7) {
        character ^ 0x20
    } else {
        character
    }
}

fn rtl_upper_char(character: u8) -> u8 {
    if character.is_ascii_lowercase() || ((0xe0..=0xfe).contains(&character) && character != 0xf7) {
        character ^ 0x20
    } else if character == 0xff {
        b'?'
    } else {
        character
    }
}

fn ansi_byte_to_unicode(byte: u8) -> u16 {
    i16::from(byte as i8) as u16
}

fn unicode_to_ansi(unit: u16) -> u8 {
    if unit < 0x00ff {
        unit as u8
    } else {
        b'?'
    }
}

fn rtl_unicode_lower(unit: u16) -> u16 {
    let Some(character) = char::from_u32(u32::from(unit)) else {
        return unit;
    };
    let mut mapped = character.to_lowercase();
    let Some(first) = mapped.next() else {
        return unit;
    };
    if mapped.next().is_none() && u32::from(first) <= u32::from(u16::MAX) {
        first as u16
    } else {
        unit
    }
}

fn rtl_unicode_upper(unit: u16) -> u16 {
    let Some(character) = char::from_u32(u32::from(unit)) else {
        return unit;
    };
    let mut mapped = character.to_uppercase();
    let Some(first) = mapped.next() else {
        return unit;
    };
    if mapped.next().is_none() && u32::from(first) <= u32::from(u16::MAX) {
        first as u16
    } else {
        unit
    }
}

fn rtl_format_unsigned(mut value: u32, mut base: u32) -> Option<Vec<u8>> {
    if base == 0 {
        base = 10;
    } else if !matches!(base, 2 | 8 | 10 | 16) {
        return None;
    }

    let mut buffer = [0u8; 32];
    let mut position = buffer.len();
    loop {
        position -= 1;
        let digit = (value % base) as u8;
        value /= base;
        buffer[position] = if digit < 10 {
            b'0' + digit
        } else {
            b'A' + digit - 10
        };
        if value == 0 {
            break;
        }
    }
    Some(buffer[position..].to_vec())
}

fn rtl_is_leap_year(year: u32) -> bool {
    year.is_multiple_of(4) && (!year.is_multiple_of(100) || year.is_multiple_of(400))
}

fn rtl_month_length(year: u32, month: u32) -> Option<u32> {
    let leap = rtl_is_leap_year(year);
    Some(match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if leap => 29,
        2 => 28,
        _ => return None,
    })
}

fn rtl_time_fields_to_ticks(fields: [u16; 8]) -> Option<u64> {
    let year = u32::from(fields[0]);
    let month = u32::from(fields[1]);
    let day = u32::from(fields[2]);
    let hour = u32::from(fields[3]);
    let minute = u32::from(fields[4]);
    let second = u32::from(fields[5]);
    let millisecond = u32::from(fields[6]);

    if year < 1601
        || day == 0
        || day > rtl_month_length(year, month)?
        || hour > 23
        || minute > 59
        || second > 59
        || millisecond > 999
    {
        return None;
    }

    let (march_month, march_year) = if month < 3 {
        (month + 13, year - 1)
    } else {
        (month + 1, year)
    };
    let century_leaps = (3 * (march_year / 100)).div_ceil(4);
    let days = i64::from((36525 * march_year) / 100) - i64::from(century_leaps)
        + i64::from((1959 * march_month) / 64)
        + i64::from(day)
        - 584_817;
    if days < 0 {
        return None;
    }

    let ticks = i128::from(days)
        .checked_mul(24)?
        .checked_add(i128::from(hour))?
        .checked_mul(60)?
        .checked_add(i128::from(minute))?
        .checked_mul(60)?
        .checked_add(i128::from(second))?
        .checked_mul(1000)?
        .checked_add(i128::from(millisecond))?
        .checked_mul(10_000)?;
    (ticks <= i128::from(i64::MAX)).then_some(ticks as u64)
}

fn rtl_ticks_to_time_fields(ticks: u64) -> Option<[u16; 8]> {
    if ticks > i64::MAX as u64 {
        return None;
    }
    let milliseconds_total = ticks / 10_000;
    let days = milliseconds_total / 86_400_000;
    let milliseconds_in_day = milliseconds_total % 86_400_000;

    let millisecond = (milliseconds_in_day % 1000) as u16;
    let seconds_total = milliseconds_in_day / 1000;
    let second = (seconds_total % 60) as u16;
    let minutes_total = seconds_total / 60;
    let minute = (minutes_total % 60) as u16;
    let hour = (minutes_total / 60) as u16;
    let weekday = ((1 + days) % 7) as u16;

    const DAYS_PER_QUADRICENTENNIUM: u64 = 365 * 400 + 97;
    const DAYS_PER_NORMAL_QUADRENNIUM: u64 = 365 * 4 + 1;
    let century_leaps = (3 * ((4 * days + 1227) / DAYS_PER_QUADRICENTENNIUM)).div_ceil(4);
    let adjusted_days = days + 28_188 + century_leaps;
    let years = (20 * adjusted_days - 2442) / (5 * DAYS_PER_NORMAL_QUADRENNIUM);
    let year_day = adjusted_days - (years * DAYS_PER_NORMAL_QUADRENNIUM) / 4;
    let months = (64 * year_day) / 1959;
    let (month, year) = if months < 14 {
        (months - 1, years + 1524)
    } else {
        (months - 13, years + 1525)
    };
    let day = year_day - (1959 * months) / 64;

    Some([
        u16::try_from(year).ok()?,
        u16::try_from(month).ok()?,
        u16::try_from(day).ok()?,
        hour,
        minute,
        second,
        millisecond,
        weekday,
    ])
}

fn guest_string_descriptor(ram: &[u8], address: u32) -> Option<(u16, u16, u32)> {
    let range = ram_range(address, 8)?;
    let bytes = &ram[range];
    Some((
        u16::from_le_bytes(bytes[0..2].try_into().unwrap()),
        u16::from_le_bytes(bytes[2..4].try_into().unwrap()),
        u32::from_le_bytes(bytes[4..8].try_into().unwrap()),
    ))
}

fn guest_ansi_string_length(ram: &[u8], address: u32) -> Option<u16> {
    for length in 0..u16::MAX {
        let index = ram_index(address.checked_add(u32::from(length))?)?;
        if ram[index] == 0 {
            return Some(length);
        }
    }
    None
}

fn guest_unicode_string_length(ram: &[u8], address: u32) -> Option<u16> {
    let mut length = 0u32;
    while length <= u32::from(u16::MAX - 2) {
        let unit_address = address.checked_add(length)?;
        let range = ram_range(unit_address, 2)?;
        if u16::from_le_bytes(ram[range].try_into().unwrap()) == 0 {
            return u16::try_from(length).ok();
        }
        length += 2;
    }
    None
}

fn guest_address_valid(address: u32) -> bool {
    ram_index(address).is_some()
        || kernel_data_index(address).is_some()
        || address
            .checked_sub(NV2A_BASE)
            .is_some_and(|offset| offset < NV2A_SIZE as u32)
        || address
            .checked_sub(APU_BASE)
            .is_some_and(|offset| offset < APU_SIZE as u32)
        || address
            .checked_sub(AC97_BASE)
            .is_some_and(|offset| offset < AC97_SIZE as u32)
        || [USB0_BASE, USB1_BASE].into_iter().any(|base| {
            address
                .checked_sub(base)
                .is_some_and(|offset| offset < USB_MMIO_SIZE)
        })
}

fn guest_span_valid(address: u32, length: u32) -> bool {
    if length == 0 {
        return false;
    }
    if ram_range(address, length).is_some() {
        return true;
    }
    let Some(last) = address.checked_add(length - 1) else {
        return false;
    };
    for (base, size) in [
        (NV2A_BASE, NV2A_SIZE as u32),
        (APU_BASE, APU_SIZE as u32),
        (AC97_BASE, AC97_SIZE as u32),
        (USB0_BASE, USB_MMIO_SIZE),
        (USB1_BASE, USB_MMIO_SIZE),
    ] {
        let Some(end) = base.checked_add(size) else {
            continue;
        };
        if address >= base && last < end {
            return true;
        }
    }
    false
}

fn kernel_export_is_data(ordinal: u32) -> bool {
    matches!(
        ordinal,
        16 | 22
            | 30
            | 31
            | 40
            | 41
            | 42
            | 64
            | 70
            | 71
            | 88
            | 89
            | 102
            | 120
            | 154
            | 156
            | 157
            | 162
            | 164
            | 240
            | 245
            | 249
            | 259
            | 321
            | 322
            | 323
            | 324
            | 325
            | 326
            | 353
            | 354
            | 355
            | 356
            | 357
    )
}

fn kernel_hle_address(ordinal: u32) -> u32 {
    XBOX_KERNEL_HLE_BASE + ordinal * XBOX_KERNEL_HLE_STRIDE
}

fn kernel_data_address(ordinal: u32) -> u32 {
    XBOX_KERNEL_DATA_BASE + ordinal * XBOX_KERNEL_DATA_STRIDE as u32
}

fn kernel_data_offset(ordinal: u32) -> usize {
    ordinal as usize * XBOX_KERNEL_DATA_STRIDE
}

fn kernel_data_index(address: u32) -> Option<usize> {
    let offset = address.checked_sub(XBOX_KERNEL_DATA_BASE)? as usize;
    (offset < XBOX_KERNEL_DATA_SIZE).then_some(offset)
}

fn new_kernel_data() -> Box<[u8]> {
    let mut data = vec![0; XBOX_KERNEL_DATA_SIZE].into_boxed_slice();
    data[kernel_data_offset(89)] = 1;
    data[kernel_data_offset(157)..kernel_data_offset(157) + 4]
        .copy_from_slice(&0x2710u32.to_le_bytes());
    let version = kernel_data_offset(324);
    for (index, value) in [1u16, 0, 5838, 1].into_iter().enumerate() {
        let offset = version + index * 2;
        data[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
    }
    data
}

fn kernel_hle_ordinal(address: u32) -> Option<u32> {
    let offset = address.checked_sub(XBOX_KERNEL_HLE_BASE)?;
    if offset % XBOX_KERNEL_HLE_STRIDE != 0 {
        return None;
    }
    let ordinal = offset / XBOX_KERNEL_HLE_STRIDE;
    (ordinal < XBOX_KERNEL_EXPORT_COUNT).then_some(ordinal)
}

fn patch_xbe_kernel_thunks(header: &[u8], ram: &mut [u8], thunk_xor: u32) -> Result<(), String> {
    let encoded = le32(header, 0x158)?;
    if encoded == 0 {
        return Ok(());
    }
    let address = encoded ^ thunk_xor;
    let start = ram_index(address)
        .ok_or_else(|| format!("XBE kernel thunk table 0x{address:08x} is outside Xbox RAM"))?;
    for index in 0..=XBOX_KERNEL_EXPORT_COUNT as usize {
        let offset = start
            .checked_add(index * 4)
            .filter(|offset| offset + 4 <= ram.len())
            .ok_or_else(|| "XBE kernel thunk table exceeds Xbox RAM".to_string())?;
        let raw = u32::from_le_bytes(ram[offset..offset + 4].try_into().unwrap());
        if raw == 0 {
            return Ok(());
        }
        let ordinal = raw & 0x7fff_ffff;
        if ordinal >= XBOX_KERNEL_EXPORT_COUNT {
            return Err(format!("XBE imports unknown xboxkrnl ordinal {ordinal}"));
        }
        let target = if kernel_export_is_data(ordinal) {
            kernel_data_address(ordinal)
        } else {
            kernel_hle_address(ordinal)
        };
        ram[offset..offset + 4].copy_from_slice(&target.to_le_bytes());
    }
    Err("XBE kernel thunk table is missing its terminator".into())
}

struct LoadedXbe {
    entry: u32,
    heap_base: u32,
}

fn load_xbe(image: &ResourceBlob, ram: &mut [u8]) -> Result<LoadedXbe, String> {
    let mut header = vec![0; 0x178];
    image.read(0, &mut header)?;
    if le32(&header, 0)? != XBE_MAGIC {
        return Err("Xbox development loader requires an XBE image".into());
    }
    let base = le32(&header, 0x104)?;
    let header_size = le32(&header, 0x108)? as usize;
    let section_count = le32(&header, 0x11c)? as usize;
    let section_headers = le32(&header, 0x120)?;
    if section_count > 256 || section_headers < base {
        return Err("XBE section table is invalid".into());
    }
    let headers_to_copy = header_size.min(RAM_SIZE);
    let mut headers = vec![0; headers_to_copy];
    image.read(0, &mut headers)?;
    let base_index =
        ram_index(base).ok_or_else(|| "XBE base address is outside Xbox RAM".to_string())?;
    let header_end = base_index
        .checked_add(headers.len())
        .filter(|end| *end <= ram.len())
        .ok_or_else(|| "XBE headers exceed Xbox RAM".to_string())?;
    ram[base_index..header_end].copy_from_slice(&headers);
    let mut image_end = header_end;

    let table_offset = u64::from(section_headers - base);
    let table_len = section_count
        .checked_mul(56)
        .ok_or_else(|| "XBE section count overflow".to_string())?;
    let mut table = vec![0; table_len];
    image.read(table_offset, &mut table)?;
    let mut executable_ranges = Vec::new();
    for section in 0..section_count {
        let offset = section * 56;
        let flags = le32(&table, offset)?;
        let virtual_address = le32(&table, offset + 4)?;
        let virtual_size = le32(&table, offset + 8)? as usize;
        let raw_offset = le32(&table, offset + 12)? as u64;
        let raw_size = le32(&table, offset + 16)? as usize;
        if virtual_size == 0 {
            continue;
        }
        let start = ram_index(virtual_address)
            .ok_or_else(|| format!("XBE section {section} is outside Xbox RAM"))?;
        let end = start
            .checked_add(virtual_size)
            .filter(|end| *end <= ram.len())
            .ok_or_else(|| format!("XBE section {section} exceeds Xbox RAM"))?;
        image_end = image_end.max(end);
        ram[start..end].fill(0);
        let copy_len = raw_size.min(virtual_size);
        if copy_len != 0 {
            let raw_end = raw_offset
                .checked_add(copy_len as u64)
                .filter(|end| *end <= image.len())
                .ok_or_else(|| format!("XBE section {section} exceeds the image"))?;
            let mut bytes = vec![0; copy_len];
            image.read(raw_offset, &mut bytes)?;
            debug_assert_eq!(raw_end - raw_offset, copy_len as u64);
            ram[start..start + copy_len].copy_from_slice(&bytes);
        }
        if flags & (1 << 2) != 0 {
            executable_ranges.push((
                virtual_address,
                virtual_address.wrapping_add(virtual_size as u32),
            ));
        }
    }
    let heap_physical = u32::try_from(image_end)
        .map_err(|_| "XBE image end exceeds the Xbox address space".to_string())?
        .checked_add(0x0fff)
        .map(|value| value & !0x0fff)
        .ok_or_else(|| "XBE heap base overflow".to_string())?;
    let current_thread = XBOX_KERNEL_HEAP_ALIAS
        .checked_add(heap_physical)
        .ok_or_else(|| "XBE current-thread address overflow".to_string())?;
    let heap_base = current_thread
        .checked_add(XBOX_KERNEL_THREAD_RESERVE)
        .filter(|base| *base < XBOX_KERNEL_HEAP_LIMIT)
        .ok_or_else(|| "XBE image leaves no room for the kernel heap".to_string())?;

    let encoded_entry = le32(&header, 0x128)?;
    for (entry_key, thunk_key) in [
        (XBE_ENTRY_RETAIL_XOR, XBE_THUNK_RETAIL_XOR),
        (XBE_ENTRY_DEBUG_XOR, XBE_THUNK_DEBUG_XOR),
    ] {
        let entry = encoded_entry ^ entry_key;
        if executable_ranges
            .iter()
            .any(|&(start, end)| entry >= start && entry < end)
        {
            patch_xbe_kernel_thunks(&header, ram, thunk_key)?;
            return Ok(LoadedXbe { entry, heap_base });
        }
    }
    Err("XBE entry point does not fall inside an executable section".into())
}

fn eeprom_setting(value_index: u32) -> Option<(usize, usize, u32)> {
    match value_index {
        0x0000 => Some((0x64, 4, REG_DWORD)),
        0x0001 => Some((0x68, 4, REG_BINARY)),
        0x0002 => Some((0x78, 4, REG_DWORD)),
        0x0003 => Some((0x88, 4, REG_DWORD)),
        0x0004 => Some((0x6c, 4, REG_BINARY)),
        0x0005 => Some((0x7c, 4, REG_DWORD)),
        0x0006 => Some((0x8c, 4, REG_DWORD)),
        0x0007 => Some((0x90, 4, REG_DWORD)),
        0x0008 => Some((0x94, 4, REG_DWORD)),
        0x0009 => Some((0x98, 4, REG_DWORD)),
        0x000a => Some((0x9c, 4, REG_DWORD)),
        0x000b => Some((0xa0, 4, REG_DWORD)),
        0x000c => Some((0xa4, 4, REG_DWORD)),
        0x000d => Some((0xa8, 4, REG_DWORD)),
        0x000e => Some((0xac, 4, REG_DWORD)),
        0x000f => Some((0xb0, 4, REG_DWORD)),
        0x0010 => Some((0xb4, 4, REG_DWORD)),
        0x0011 => Some((0xb8, 4, REG_DWORD)),
        0x0012 => Some((0xbc, 4, REG_DWORD)),
        0x00ff => Some((0x60, 0x60, REG_BINARY)),
        0x0100 => Some((0x34, 12, REG_BINARY)),
        0x0101 => Some((0x40, 6, REG_BINARY)),
        0x0102 => Some((0x48, 16, REG_BINARY)),
        0x0103 => Some((0x58, 4, REG_DWORD)),
        0x0104 => Some((0x2c, 4, REG_DWORD)),
        0xfffe => Some((0x00, 0x30, REG_BINARY)),
        0xffff => Some((0x00, EEPROM_SIZE, REG_BINARY)),
        _ => None,
    }
}

fn eeprom_section_crc(data: &[u8]) -> [u8; 4] {
    let mut rotated = vec![0u8; data.len() + 4];
    if let Some((&last, prefix)) = data.split_last() {
        rotated[0] = last;
        rotated[1..=prefix.len()].copy_from_slice(prefix);
    }

    let mut crc = [0u8; 4];
    for (position, output) in crc.iter_mut().enumerate() {
        let mut accumulator = 0xffffu16;
        let mut offset = position;
        while offset < data.len() {
            let word = u16::from_le_bytes([rotated[offset], rotated[offset + 1]]);
            accumulator = accumulator.wrapping_sub(word);
            offset += 4;
        }
        *output = (accumulator >> 8) as u8;
    }
    crc
}

fn refresh_eeprom_checksums(eeprom: &mut [u8; EEPROM_SIZE]) {
    let factory = eeprom_section_crc(&eeprom[0x34..0x60]);
    eeprom[0x30..0x34].copy_from_slice(&factory);
    let user = eeprom_section_crc(&eeprom[0x64..0xc0]);
    eeprom[0x60..0x64].copy_from_slice(&user);
}

fn default_eeprom() -> [u8; EEPROM_SIZE] {
    let mut eeprom = [0u8; EEPROM_SIZE];
    eeprom[0x2c..0x30].copy_from_slice(&1u32.to_le_bytes());
    eeprom[0x34..0x40].copy_from_slice(b"000000000001");
    eeprom[0x40..0x46].copy_from_slice(&[0x00, 0x50, 0xf2, 0x00, 0x00, 0x01]);
    eeprom[0x58..0x5c].copy_from_slice(&0x0040_0100u32.to_le_bytes());
    eeprom[0x90..0x94].copy_from_slice(&1u32.to_le_bytes());
    refresh_eeprom_checksums(&mut eeprom);
    eeprom
}

#[derive(Clone)]
struct XboxSmbus {
    status: u8,
    control: u8,
    address: u8,
    data: [u8; 2],
    command: u8,
    eeprom: [u8; EEPROM_SIZE],
    eeprom_offset: u8,
    smc_command: u8,
    smc_version_index: u8,
    smc_scratch: u8,
    smc_error: u8,
    smc_interrupt_status: u8,
    power_action: u8,
    media_present: bool,
}

impl Default for XboxSmbus {
    fn default() -> Self {
        Self {
            status: 0,
            control: 0,
            address: 0,
            data: [0; 2],
            command: 0,
            eeprom: default_eeprom(),
            eeprom_offset: 0,
            smc_command: 0,
            smc_version_index: 0,
            smc_scratch: 0,
            smc_error: 0,
            smc_interrupt_status: 0,
            power_action: 0,
            media_present: false,
        }
    }
}

impl XboxSmbus {
    fn reset_runtime(&mut self) {
        self.status = 0;
        self.control = 0;
        self.address = 0;
        self.data = [0; 2];
        self.command = 0;
        self.eeprom_offset = 0;
        self.smc_command = 0;
        self.smc_version_index = 0;
        self.smc_interrupt_status = 0;
        self.power_action = 0;
    }

    fn device_present(address: u8) -> bool {
        matches!(address, SMBUS_SMC_ADDRESS | SMBUS_EEPROM_ADDRESS)
    }

    fn smc_read(&mut self, command: u8) -> u8 {
        match command {
            0x01 => {
                let version = *b"P11";
                let value = version[usize::from(self.smc_version_index % 3)];
                self.smc_version_index = self.smc_version_index.wrapping_add(1);
                value
            }
            0x03 => {
                if self.media_present {
                    0x60
                } else {
                    0x40
                }
            }
            0x04 => 0x01,
            0x09 => 45,
            0x0a => 40,
            0x0f => self.smc_error,
            0x10 => 10,
            0x11 => {
                let value = self.smc_interrupt_status;
                self.smc_interrupt_status = 0;
                value
            }
            0x1b => self.smc_scratch,
            0x1c => 0x52,
            0x1d => 0x72,
            0x1e => 0xea,
            0x1f => 0x46,
            _ => 0,
        }
    }

    fn smc_write(&mut self, command: u8, value: u8) {
        match command {
            0x01 => self.smc_version_index = value,
            0x02 => self.power_action = value & 0xc1,
            0x0d => self.smc_interrupt_status = 0,
            0x0e => self.smc_error = value,
            0x1b => self.smc_scratch = value,
            _ => {}
        }
    }

    fn receive_byte(&mut self, address: u8) -> Option<u8> {
        match address {
            SMBUS_SMC_ADDRESS => {
                let command = self.smc_command;
                self.smc_command = self.smc_command.wrapping_add(1);
                Some(self.smc_read(command))
            }
            SMBUS_EEPROM_ADDRESS => {
                let offset = self.eeprom_offset;
                self.eeprom_offset = self.eeprom_offset.wrapping_add(1);
                Some(self.eeprom[usize::from(offset)])
            }
            _ => None,
        }
    }

    fn send_byte(&mut self, address: u8, value: u8) -> bool {
        match address {
            SMBUS_SMC_ADDRESS => self.smc_command = value,
            SMBUS_EEPROM_ADDRESS => self.eeprom_offset = value,
            _ => return false,
        }
        true
    }

    fn read_byte_data(&mut self, address: u8, command: u8) -> Option<u8> {
        match address {
            SMBUS_SMC_ADDRESS => {
                self.smc_command = command.wrapping_add(1);
                Some(self.smc_read(command))
            }
            SMBUS_EEPROM_ADDRESS => {
                self.eeprom_offset = command.wrapping_add(1);
                Some(self.eeprom[usize::from(command)])
            }
            _ => None,
        }
    }

    fn write_byte_data(&mut self, address: u8, command: u8, value: u8) -> bool {
        match address {
            SMBUS_SMC_ADDRESS => self.smc_write(command, value),
            SMBUS_EEPROM_ADDRESS => {
                self.eeprom[usize::from(command)] = value;
                self.eeprom_offset = command.wrapping_add(1);
            }
            _ => return false,
        }
        true
    }

    fn execute_transaction(&mut self) {
        let protocol = self.control & 7;
        let read = self.address & 1 != 0;
        let address = (self.address >> 1) & 0x7f;
        self.status = SMBUS_STATUS_HOST_BUSY;
        let success = match protocol {
            0 => Self::device_present(address),
            1 if read => self
                .receive_byte(address)
                .map(|value| self.data[0] = value)
                .is_some(),
            1 => self.send_byte(address, self.command),
            2 if read => self
                .read_byte_data(address, self.command)
                .map(|value| self.data[0] = value)
                .is_some(),
            2 => self.write_byte_data(address, self.command, self.data[0]),
            3 if read => {
                if let Some(low) = self.read_byte_data(address, self.command) {
                    if let Some(high) = self.read_byte_data(address, self.command.wrapping_add(1)) {
                        self.data = [low, high];
                        true
                    } else {
                        false
                    }
                } else {
                    false
                }
            }
            3 => {
                self.write_byte_data(address, self.command, self.data[0])
                    && self.write_byte_data(address, self.command.wrapping_add(1), self.data[1])
            }
            _ => false,
        };
        self.status = if success {
            SMBUS_STATUS_CYCLE_COMPLETE
        } else {
            SMBUS_STATUS_PROTOCOL_ERROR
        };
    }

    fn read_port(&mut self, port: u16) -> u8 {
        match port.wrapping_sub(SMBUS_BASE) {
            0x00 => self.status,
            0x02 => self.control & 0x1f,
            0x04 => self.address,
            0x06 => self.data[0],
            0x07 => self.data[1],
            0x08 => self.command,
            _ => 0,
        }
    }

    fn write_port(&mut self, port: u16, value: u8) {
        match port.wrapping_sub(SMBUS_BASE) {
            0x00 => self.status &= !value,
            0x02 => {
                self.control = value;
                if value & SMBUS_CONTROL_ABORT != 0 {
                    self.status = 1;
                }
                if value & SMBUS_CONTROL_START != 0 {
                    self.execute_transaction();
                }
            }
            0x04 => self.address = value,
            0x06 => self.data[0] = value,
            0x07 => self.data[1] = value,
            0x08 => self.command = value,
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.status);
        out.u8(self.control);
        out.u8(self.address);
        out.u8(self.data[0]);
        out.u8(self.data[1]);
        out.u8(self.command);
        out.blob(&self.eeprom);
        out.u8(self.eeprom_offset);
        out.u8(self.smc_command);
        out.u8(self.smc_version_index);
        out.u8(self.smc_scratch);
        out.u8(self.smc_error);
        out.u8(self.smc_interrupt_status);
        out.u8(self.power_action);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.status = input.u8()?;
        self.control = input.u8()?;
        self.address = input.u8()?;
        self.data = [input.u8()?, input.u8()?];
        self.command = input.u8()?;
        let eeprom = input.blob()?;
        if eeprom.len() != EEPROM_SIZE {
            return Err(format!(
                "Xbox EEPROM state has {} bytes; expected {EEPROM_SIZE}",
                eeprom.len()
            ));
        }
        self.eeprom.copy_from_slice(eeprom);
        self.eeprom_offset = input.u8()?;
        self.smc_command = input.u8()?;
        self.smc_version_index = input.u8()?;
        self.smc_scratch = input.u8()?;
        self.smc_error = input.u8()?;
        self.smc_interrupt_status = input.u8()?;
        self.power_action = input.u8()?;
        Ok(())
    }
}

#[derive(Clone)]
struct XboxPci {
    config_address: u32,
    config: [[u8; 256]; PCI_FUNCTION_COUNT],
}

impl XboxPci {
    fn new() -> Self {
        let mut pci = Self {
            config_address: 0,
            config: [[0; 256]; PCI_FUNCTION_COUNT],
        };
        pci.init_function(0, 0x02a5, 0xa1, [0x06, 0x00, 0x00], false);
        pci.init_function(1, 0x01b2, 0xb2, [0x06, 0x01, 0x00], true);
        pci.init_function(2, 0x01b4, 0xb1, [0x0c, 0x05, 0x00], false);
        pci.init_function(3, 0x01c2, 0xb1, [0x0c, 0x03, 0x10], false);
        pci.init_function(4, 0x01c2, 0xb1, [0x0c, 0x03, 0x10], false);
        pci.init_function(5, 0x01c3, 0xb1, [0x02, 0x00, 0x00], false);
        pci.init_function(6, 0x01b0, 0xb1, [0x04, 0x01, 0x00], false);
        pci.init_function(7, 0x01b1, 0xb1, [0x04, 0x01, 0x00], false);
        pci.init_function(8, 0x01bc, 0xb1, [0x01, 0x01, 0x80], false);
        pci.init_function(9, 0x01b7, 0xa1, [0x06, 0x04, 0x00], false);
        pci.init_function(10, 0x02a0, 0xa1, [0x03, 0x00, 0x00], false);

        pci.set_bar(2, 1, u32::from(SMBUS_BASE) | 1);
        pci.set_bar(3, 0, USB0_BASE);
        pci.set_bar(4, 0, USB1_BASE);
        pci.set_bar(6, 0, APU_BASE);
        pci.set_bar(7, 0, AC97_BASE);
        pci.set_bar(8, 4, 1);
        pci.set_bar(10, 0, NV2A_BASE);
        pci.config[9][0x18] = 0;
        pci.config[9][0x19] = 1;
        pci.config[9][0x1a] = 1;
        pci
    }

    fn init_function(
        &mut self,
        index: usize,
        device: u16,
        revision: u8,
        class_code: [u8; 3],
        multifunction: bool,
    ) {
        self.config[index][0..2].copy_from_slice(&PCI_VENDOR_NVIDIA.to_le_bytes());
        self.config[index][2..4].copy_from_slice(&device.to_le_bytes());
        self.config[index][4..6].copy_from_slice(&0x0007u16.to_le_bytes());
        self.config[index][8] = revision;
        self.config[index][9] = class_code[2];
        self.config[index][10] = class_code[1];
        self.config[index][11] = class_code[0];
        self.config[index][0x0e] = if multifunction { 0x80 } else { 0 };
    }

    fn set_bar(&mut self, index: usize, bar: usize, value: u32) {
        let offset = 0x10 + bar * 4;
        self.config[index][offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn function_index(bus: u8, device: u8, function: u8) -> Option<usize> {
        PCI_FUNCTIONS
            .iter()
            .position(|&(b, d, f)| (bus, device, function) == (b, d, f))
    }

    fn selected(&self) -> Option<(usize, usize)> {
        if self.config_address >> 31 == 0 {
            return None;
        }
        let bus = (self.config_address >> 16) as u8;
        let device = ((self.config_address >> 11) & 0x1f) as u8;
        let function = ((self.config_address >> 8) & 7) as u8;
        let register = (self.config_address & 0xfc) as usize;
        Self::function_index(bus, device, function).map(|index| (index, register))
    }

    fn config_byte_writable(index: usize, offset: usize) -> bool {
        matches!(offset, 0x04..=0x07)
            || index == 2 && matches!(offset, 0x14..=0x17)
            || index == 9 && matches!(offset, 0x18..=0x1a)
    }

    fn read_config_byte(&self, bus: u8, device: u8, function: u8, offset: usize) -> u8 {
        if offset >= 256 {
            return 0xff;
        }
        let Some(index) = Self::function_index(bus, device, function) else {
            return 0xff;
        };
        self.config[index][offset]
    }

    fn write_config_byte(&mut self, bus: u8, device: u8, function: u8, offset: usize, value: u8) {
        if offset >= 256 {
            return;
        }
        let Some(index) = Self::function_index(bus, device, function) else {
            return;
        };
        if Self::config_byte_writable(index, offset) {
            self.config[index][offset] = value;
        }
    }

    fn read_port(&self, port: u16) -> u8 {
        if (PCI_CONFIG_ADDRESS..PCI_CONFIG_ADDRESS + 4).contains(&port) {
            return self.config_address.to_le_bytes()[usize::from(port - PCI_CONFIG_ADDRESS)];
        }
        if (PCI_CONFIG_DATA..PCI_CONFIG_DATA + 4).contains(&port) {
            let byte = usize::from(port - PCI_CONFIG_DATA);
            let Some((index, register)) = self.selected() else {
                return 0xff;
            };
            return self.config[index][register + byte];
        }
        0xff
    }

    fn write_port(&mut self, port: u16, value: u8) {
        if (PCI_CONFIG_ADDRESS..PCI_CONFIG_ADDRESS + 4).contains(&port) {
            let mut bytes = self.config_address.to_le_bytes();
            bytes[usize::from(port - PCI_CONFIG_ADDRESS)] = value;
            self.config_address = u32::from_le_bytes(bytes);
            return;
        }
        if !(PCI_CONFIG_DATA..PCI_CONFIG_DATA + 4).contains(&port) {
            return;
        }
        let byte = usize::from(port - PCI_CONFIG_DATA);
        let Some((index, register)) = self.selected() else {
            return;
        };
        let offset = register + byte;
        if Self::config_byte_writable(index, offset) {
            self.config[index][offset] = value;
        }
    }

    fn smbus_base(&self) -> u16 {
        let value = u32::from_le_bytes(self.config[2][0x14..0x18].try_into().unwrap());
        (value & 0xfffcu32) as u16
    }

    fn save(&self, out: &mut StateWriter) {
        out.u32(self.config_address);
        for config in &self.config {
            out.blob(config);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.config_address = input.u32()?;
        for config in &mut self.config {
            let bytes = input.blob()?;
            if bytes.len() != config.len() {
                return Err(format!(
                    "Xbox PCI config state has {} bytes; expected {}",
                    bytes.len(),
                    config.len()
                ));
            }
            config.copy_from_slice(bytes);
        }
        Ok(())
    }
}

#[derive(Clone)]
struct XboxXid {
    address: u8,
    configuration: u8,
    output_report: [u8; 6],
}

impl Default for XboxXid {
    fn default() -> Self {
        Self {
            address: 0,
            configuration: 0,
            output_report: [0, 6, 0, 0, 0, 0],
        }
    }
}

impl XboxXid {
    fn trigger(value: i16) -> u8 {
        (i32::from(value).clamp(0, 32767) * 255 / 32767) as u8
    }

    fn input_report(input: &InputState) -> [u8; 20] {
        let buttons = input.buttons[0];
        let mut report = [0u8; 20];
        report[1] = 20;
        let mut digital = 0u16;
        for (mask, bit) in [
            (UP, 0),
            (DOWN, 1),
            (LEFT, 2),
            (RIGHT, 3),
            (START, 4),
            (SELECT, 5),
            (L3, 6),
            (R3, 7),
        ] {
            if buttons & mask != 0 {
                digital |= 1 << bit;
            }
        }
        report[2..4].copy_from_slice(&digital.to_le_bytes());
        for (offset, mask) in [
            (4, FACE_SOUTH),
            (5, FACE_EAST),
            (6, FACE_WEST),
            (7, FACE_NORTH),
            (8, R1),
            (9, L1),
        ] {
            report[offset] = if buttons & mask != 0 { 0xff } else { 0 };
        }
        report[10] = Self::trigger(input.axes[0][AXIS_LEFT_TRIGGER]);
        report[11] = Self::trigger(input.axes[0][AXIS_RIGHT_TRIGGER]);
        for (offset, value) in [
            (12, input.axes[0][AXIS_LEFT_X]),
            (14, input.axes[0][AXIS_LEFT_Y]),
            (16, input.axes[0][AXIS_RIGHT_X]),
            (18, input.axes[0][AXIS_RIGHT_Y]),
        ] {
            report[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
        }
        report
    }

    fn string_descriptor(value: &str) -> Vec<u8> {
        let utf16: Vec<u16> = value.encode_utf16().collect();
        let mut descriptor = Vec::with_capacity(2 + utf16.len() * 2);
        descriptor.push((2 + utf16.len() * 2) as u8);
        descriptor.push(3);
        for unit in utf16 {
            descriptor.extend_from_slice(&unit.to_le_bytes());
        }
        descriptor
    }

    fn setup_response(&mut self, setup: [u8; 8], input: &InputState) -> Vec<u8> {
        let request_type = setup[0];
        let request = setup[1];
        let value = u16::from_le_bytes([setup[2], setup[3]]);
        match (request_type, request) {
            (0x80, 0x00) => vec![0, 0],
            (0x80, 0x06) => match (value >> 8) as u8 {
                1 => vec![
                    18, 1, 0x10, 0x01, 0, 0, 0, 0x40, 0x5e, 0x04, 0x02, 0x02, 0x00, 0x01, 1, 2, 3,
                    1,
                ],
                2 => vec![
                    9, 2, 32, 0, 1, 1, 0, 0x80, 50, 9, 4, 0, 0, 2, 0x58, 0x42, 0, 0, 7, 5, 0x82, 3,
                    0x20, 0, 4, 7, 5, 0x02, 3, 0x20, 0, 4,
                ],
                3 => match value as u8 {
                    0 => vec![4, 3, 0x09, 0x04],
                    1 => Self::string_descriptor("Microsoft"),
                    2 => Self::string_descriptor("Microsoft Xbox Controller"),
                    3 => Self::string_descriptor("1"),
                    _ => Vec::new(),
                },
                _ => Vec::new(),
            },
            (0x80, 0x08) => vec![self.configuration],
            (0x00, 0x05) => {
                self.address = value as u8 & 0x7f;
                Vec::new()
            }
            (0x00, 0x09) => {
                self.configuration = value as u8;
                Vec::new()
            }
            (0xa1, 0x01) if value == 0x0100 => Self::input_report(input).to_vec(),
            (0xc1, 0x06) if value == 0x4200 => vec![
                0x10, 0x42, 0x00, 0x01, 0x01, 0x01, 20, 6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                0xff, 0xff,
            ],
            (0xc1, 0x01) if value == 0x0100 => {
                let mut capabilities = vec![0xff; 20];
                capabilities[0] = 0;
                capabilities[1] = 20;
                capabilities
            }
            (0xc1, 0x01) if value == 0x0200 => vec![0, 6, 0xff, 0xff, 0xff, 0xff],
            _ => Vec::new(),
        }
    }

    fn control_out(&mut self, setup: [u8; 8], data: &[u8]) {
        let value = u16::from_le_bytes([setup[2], setup[3]]);
        if setup[0] == 0x21 && setup[1] == 0x09 && value == 0x0200 {
            let count = data.len().min(self.output_report.len());
            self.output_report[..count].copy_from_slice(&data[..count]);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(self.address);
        out.u8(self.configuration);
        out.blob(&self.output_report);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.address = input.u8()?;
        self.configuration = input.u8()?;
        let report = input.blob()?;
        if report.len() != self.output_report.len() {
            return Err("Xbox XID output report state has the wrong size".into());
        }
        self.output_report.copy_from_slice(report);
        Ok(())
    }
}

#[derive(Clone)]
struct XboxOhci {
    connected: bool,
    control: u32,
    command_status: u32,
    interrupt_status: u32,
    interrupt_enable: u32,
    hcca: u32,
    control_head: u32,
    bulk_head: u32,
    done_head: u32,
    fm_interval: u32,
    fm_number: u16,
    periodic_start: u32,
    root_status: u32,
    ports: [u32; 4],
    xid: XboxXid,
    setup: [u8; 8],
    setup_valid: bool,
    control_data: [u8; 256],
    control_len: u16,
    control_pos: u16,
}

impl XboxOhci {
    fn new(connected: bool) -> Self {
        let mut ohci = Self {
            connected,
            control: 0,
            command_status: 0,
            interrupt_status: 0,
            interrupt_enable: OHCI_INTR_MIE,
            hcca: 0,
            control_head: 0,
            bulk_head: 0,
            done_head: 0,
            fm_interval: 0x2edf,
            fm_number: 0,
            periodic_start: 0,
            root_status: 0,
            ports: [OHCI_PORT_PPS; 4],
            xid: XboxXid::default(),
            setup: [0; 8],
            setup_valid: false,
            control_data: [0; 256],
            control_len: 0,
            control_pos: 0,
        };
        ohci.reset_runtime();
        ohci
    }

    fn reset_runtime(&mut self) {
        self.control = 0xc0;
        self.command_status = 0;
        self.interrupt_status = 0;
        self.interrupt_enable = OHCI_INTR_MIE;
        self.hcca = 0;
        self.control_head = 0;
        self.bulk_head = 0;
        self.done_head = 0;
        self.fm_interval = 0x2edf;
        self.fm_number = 0;
        self.periodic_start = 0;
        self.root_status = 0;
        self.ports = [OHCI_PORT_PPS; 4];
        if self.connected {
            self.ports[2] |= OHCI_PORT_CCS | OHCI_PORT_CSC;
            self.interrupt_status |= OHCI_INTR_RHSC;
        }
        self.xid.address = 0;
        self.xid.configuration = 0;
        self.setup_valid = false;
        self.control_len = 0;
        self.control_pos = 0;
    }

    fn read32(&self, offset: u32) -> u32 {
        match offset {
            0x00 => 0x10,
            0x04 => self.control,
            0x08 => self.command_status,
            0x0c => self.interrupt_status,
            0x10 => self.interrupt_enable,
            0x14 => 0,
            0x18 => self.hcca,
            0x20 => self.control_head,
            0x24 => 0,
            0x28 => self.bulk_head,
            0x2c => 0,
            0x30 => self.done_head,
            0x34 => self.fm_interval,
            0x38 => 0,
            0x3c => u32::from(self.fm_number),
            0x40 => self.periodic_start,
            0x44 => 0x628,
            0x48 => (1 << 9) | 4,
            0x4c => 0,
            0x50 => self.root_status,
            0x54..=0x60 if offset & 3 == 0 => self.ports[((offset - 0x54) / 4) as usize],
            _ => 0,
        }
    }

    fn write_port_status(&mut self, index: usize, value: u32) {
        let connected = self.ports[index] & OHCI_PORT_CCS != 0;
        self.ports[index] &= !(value & OHCI_PORT_WTC);
        if value & OHCI_PORT_CCS != 0 {
            self.ports[index] &= !OHCI_PORT_PES;
        }
        if value & OHCI_PORT_PES != 0 && connected {
            self.ports[index] |= OHCI_PORT_PES | OHCI_PORT_PESC;
        }
        if value & OHCI_PORT_PSS != 0 && connected {
            self.ports[index] |= OHCI_PORT_PSS | OHCI_PORT_PSSC;
        }
        if value & OHCI_PORT_PRS != 0 && connected {
            self.ports[index] &= !OHCI_PORT_PSS;
            self.ports[index] |= OHCI_PORT_PES | OHCI_PORT_PRSC;
            self.xid.address = 0;
            self.xid.configuration = 0;
        }
        if self.ports[index] & OHCI_PORT_WTC != 0 {
            self.interrupt_status |= OHCI_INTR_RHSC;
        }
    }

    fn write8(&mut self, offset: u32, value: u8) {
        let aligned = offset & !3;
        let shift = (offset & 3) * 8;
        let byte = u32::from(value) << shift;
        match aligned {
            0x08 | 0x0c | 0x10 | 0x14 | 0x50 | 0x54..=0x60 => self.write32(aligned, byte),
            _ => {
                let mask = !(0xffu32 << shift);
                self.write32(aligned, (self.read32(aligned) & mask) | byte);
            }
        }
    }

    fn write32(&mut self, offset: u32, value: u32) {
        match offset {
            0x04 => self.control = value,
            0x08 => {
                self.command_status = value & !OHCI_COMMAND_HCR;
                if value & OHCI_COMMAND_HCR != 0 {
                    self.reset_runtime();
                }
            }
            0x0c => self.interrupt_status &= !value,
            0x10 => self.interrupt_enable |= value,
            0x14 => self.interrupt_enable &= !value,
            0x18 => self.hcca = value & 0xffff_ff00,
            0x20 => self.control_head = value & 0xffff_fff0,
            0x28 => self.bulk_head = value & 0xffff_fff0,
            0x34 => self.fm_interval = value,
            0x40 => self.periodic_start = value,
            0x50 => self.root_status = value,
            0x54..=0x60 if offset & 3 == 0 => {
                self.write_port_status(((offset - 0x54) / 4) as usize, value)
            }
            _ => {}
        }
    }

    fn guest_u32(ram: &[u8], address: u32) -> Option<u32> {
        let start = address as usize;
        let bytes = ram.get(start..start + 4)?;
        Some(u32::from_le_bytes(bytes.try_into().ok()?))
    }

    fn write_guest_u32(ram: &mut [u8], address: u32, value: u32) -> bool {
        let start = address as usize;
        let Some(bytes) = ram.get_mut(start..start + 4) else {
            return false;
        };
        bytes.copy_from_slice(&value.to_le_bytes());
        true
    }

    fn td_len(cbp: u32, be: u32) -> usize {
        if cbp == 0 {
            0
        } else if cbp & 0xffff_f000 == be & 0xffff_f000 {
            be.wrapping_sub(cbp).wrapping_add(1) as usize
        } else {
            (0x1000 - (cbp & 0xfff) + (be & 0xfff) + 1) as usize
        }
    }

    fn read_td_buffer(ram: &[u8], cbp: u32, be: u32) -> Vec<u8> {
        let length = Self::td_len(cbp, be);
        if length == 0 {
            return Vec::new();
        }
        let first = (0x1000 - (cbp & 0xfff)) as usize;
        if length <= first {
            let start = cbp as usize;
            return ram.get(start..start + length).unwrap_or(&[]).to_vec();
        }
        let mut data = Vec::with_capacity(length);
        let start = cbp as usize;
        if let Some(bytes) = ram.get(start..start + first) {
            data.extend_from_slice(bytes);
        }
        let second = length.saturating_sub(data.len());
        let second_start = (be & 0xffff_f000) as usize;
        if let Some(bytes) = ram.get(second_start..second_start + second) {
            data.extend_from_slice(bytes);
        }
        data
    }

    fn write_td_buffer(ram: &mut [u8], cbp: u32, be: u32, data: &[u8]) -> usize {
        let length = Self::td_len(cbp, be).min(data.len());
        if length == 0 {
            return 0;
        }
        let first = length.min((0x1000 - (cbp & 0xfff)) as usize);
        let start = cbp as usize;
        let Some(first_dst) = ram.get_mut(start..start + first) else {
            return 0;
        };
        first_dst.copy_from_slice(&data[..first]);
        if first < length {
            let second_start = (be & 0xffff_f000) as usize;
            if let Some(second_dst) = ram.get_mut(second_start..second_start + length - first) {
                second_dst.copy_from_slice(&data[first..length]);
            } else {
                return first;
            }
        }
        length
    }

    fn process_td(
        &mut self,
        ram: &mut [u8],
        input: &InputState,
        endpoint: u8,
        address: u8,
        ed_direction: u8,
        td_address: u32,
    ) -> Option<u32> {
        let mut flags = Self::guest_u32(ram, td_address)?;
        let cbp = Self::guest_u32(ram, td_address + 4)?;
        let next = Self::guest_u32(ram, td_address + 8)? & 0xffff_fff0;
        let be = Self::guest_u32(ram, td_address + 12)?;
        let direction = if ed_direction == 0 {
            ((flags >> 19) & 3) as u8
        } else {
            ed_direction
        };
        let active_address = self.xid.address;
        let port_enabled = self.ports[2] & OHCI_PORT_PES != 0;
        let mut condition = 0u32;
        if !port_enabled || address != active_address {
            condition = 5;
        } else {
            match (endpoint, direction) {
                (0, 0) => {
                    let setup = Self::read_td_buffer(ram, cbp, be);
                    if setup.len() >= 8 {
                        self.setup.copy_from_slice(&setup[..8]);
                        self.setup_valid = true;
                        let response = self.xid.setup_response(self.setup, input);
                        let count = response.len().min(self.control_data.len());
                        self.control_data[..count].copy_from_slice(&response[..count]);
                        self.control_len = count as u16;
                        self.control_pos = 0;
                    }
                }
                (0, 2) => {
                    let start = usize::from(self.control_pos);
                    let end = usize::from(self.control_len);
                    let written = if start < end {
                        Self::write_td_buffer(ram, cbp, be, &self.control_data[start..end])
                    } else {
                        0
                    };
                    self.control_pos = self.control_pos.saturating_add(written as u16);
                }
                (0, 1) if self.setup_valid => {
                    let data = Self::read_td_buffer(ram, cbp, be);
                    self.xid.control_out(self.setup, &data);
                }
                (2, 2) => {
                    let report = XboxXid::input_report(input);
                    Self::write_td_buffer(ram, cbp, be, &report);
                }
                (2, 1) => {
                    let data = Self::read_td_buffer(ram, cbp, be);
                    let count = data.len().min(self.xid.output_report.len());
                    self.xid.output_report[..count].copy_from_slice(&data[..count]);
                }
                _ => {}
            }
        }
        flags = (flags & 0x0fff_ffff) | (condition << 28);
        Self::write_guest_u32(ram, td_address, flags);
        Self::write_guest_u32(ram, td_address + 4, 0);
        Self::write_guest_u32(ram, td_address + 8, self.done_head);
        self.done_head = td_address;
        Some(next)
    }

    fn process_ed_list(&mut self, ram: &mut [u8], input: &InputState, mut ed: u32) {
        for _ in 0..32 {
            ed &= 0xffff_fff0;
            if ed == 0 {
                break;
            }
            let Some(flags) = Self::guest_u32(ram, ed) else {
                break;
            };
            let tail = Self::guest_u32(ram, ed + 4).unwrap_or(0) & 0xffff_fff0;
            let head_raw = Self::guest_u32(ram, ed + 8).unwrap_or(0);
            let next_ed = Self::guest_u32(ram, ed + 12).unwrap_or(0) & 0xffff_fff0;
            if flags & (1 << 14) == 0 && flags & (1 << 15) == 0 {
                let endpoint = ((flags >> 7) & 0xf) as u8;
                let address = (flags & 0x7f) as u8;
                let direction = ((flags >> 11) & 3) as u8;
                let mut head = head_raw & 0xffff_fff0;
                for _ in 0..32 {
                    if head == 0 || head == tail {
                        break;
                    }
                    let Some(next) =
                        self.process_td(ram, input, endpoint, address, direction, head)
                    else {
                        break;
                    };
                    let carry = head_raw & 2;
                    Self::write_guest_u32(ram, ed + 8, next | carry);
                    head = next;
                }
            }
            ed = next_ed;
        }
    }

    fn service(&mut self, ram: &mut [u8], input: &InputState) {
        if self.control & (3 << 6) != OHCI_CONTROL_OPERATIONAL {
            return;
        }
        self.fm_number = self.fm_number.wrapping_add(1);
        self.interrupt_status |= OHCI_INTR_SF;
        if self.hcca != 0 {
            let hcca = self.hcca as usize;
            if let Some(frame) = ram.get_mut(hcca + 0x80..hcca + 0x82) {
                frame.copy_from_slice(&self.fm_number.to_le_bytes());
            }
        }
        if self.control & OHCI_CONTROL_CLE != 0 && self.control_head != 0 {
            self.process_ed_list(ram, input, self.control_head);
        }
        if self.control & OHCI_CONTROL_BLE != 0 && self.bulk_head != 0 {
            self.process_ed_list(ram, input, self.bulk_head);
        }
        if self.control & OHCI_CONTROL_PLE != 0 && self.hcca != 0 {
            let slot = u32::from(self.fm_number & 31) * 4;
            if let Some(ed) = Self::guest_u32(ram, self.hcca + slot) {
                self.process_ed_list(ram, input, ed);
            }
        }
        if self.done_head != 0 {
            if self.hcca != 0 {
                Self::write_guest_u32(ram, self.hcca + 0x84, self.done_head);
            }
            self.interrupt_status |= OHCI_INTR_WD;
            self.done_head = 0;
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(u8::from(self.connected));
        for value in [
            self.control,
            self.command_status,
            self.interrupt_status,
            self.interrupt_enable,
            self.hcca,
            self.control_head,
            self.bulk_head,
            self.done_head,
            self.fm_interval,
            self.periodic_start,
            self.root_status,
        ] {
            out.u32(value);
        }
        out.u16(self.fm_number);
        for port in self.ports {
            out.u32(port);
        }
        self.xid.save(out);
        out.blob(&self.setup);
        out.u8(u8::from(self.setup_valid));
        out.blob(&self.control_data);
        out.u16(self.control_len);
        out.u16(self.control_pos);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.connected = input.u8()? != 0;
        self.control = input.u32()?;
        self.command_status = input.u32()?;
        self.interrupt_status = input.u32()?;
        self.interrupt_enable = input.u32()?;
        self.hcca = input.u32()?;
        self.control_head = input.u32()?;
        self.bulk_head = input.u32()?;
        self.done_head = input.u32()?;
        self.fm_interval = input.u32()?;
        self.periodic_start = input.u32()?;
        self.root_status = input.u32()?;
        self.fm_number = input.u16()?;
        for port in &mut self.ports {
            *port = input.u32()?;
        }
        self.xid.load(input)?;
        let setup = input.blob()?;
        if setup.len() != self.setup.len() {
            return Err("Xbox OHCI setup state has the wrong size".into());
        }
        self.setup.copy_from_slice(setup);
        self.setup_valid = input.u8()? != 0;
        let data = input.blob()?;
        if data.len() != self.control_data.len() {
            return Err("Xbox OHCI control-data state has the wrong size".into());
        }
        self.control_data.copy_from_slice(data);
        self.control_len = input.u16()?;
        self.control_pos = input.u16()?;
        Ok(())
    }
}

struct XboxIde {
    disc: Option<ResourceBlob>,
    selected_slave: bool,
    error: u8,
    feature: u8,
    sector_count: u8,
    lba_low: u8,
    byte_count_low: u8,
    byte_count_high: u8,
    drive_head: u8,
    status: u8,
    control: u8,
    packet: [u8; 12],
    packet_pos: u8,
    packet_expected: bool,
    data: Vec<u8>,
    data_pos: usize,
    sense_key: u8,
    sense_asc: u8,
    hdd_locked: bool,
    hdd_unlock_pending: bool,
    hdd_data_out_remaining: usize,
}

impl XboxIde {
    fn new(disc: Option<ResourceBlob>) -> Self {
        Self {
            disc,
            selected_slave: false,
            error: 0,
            feature: 0,
            sector_count: 0,
            lba_low: 0,
            byte_count_low: 0,
            byte_count_high: 0,
            drive_head: 0xa0,
            status: ATA_STATUS_DRDY,
            control: 0,
            packet: [0; 12],
            packet_pos: 0,
            packet_expected: false,
            data: Vec::new(),
            data_pos: 0,
            sense_key: 0,
            sense_asc: 0,
            hdd_locked: true,
            hdd_unlock_pending: false,
            hdd_data_out_remaining: 0,
        }
    }

    fn reset_runtime(&mut self) {
        self.selected_slave = false;
        self.error = 0;
        self.feature = 0;
        self.sector_count = 1;
        self.lba_low = 1;
        self.byte_count_low = 0;
        self.byte_count_high = 0;
        self.drive_head = 0xa0;
        self.status = ATA_STATUS_DRDY;
        self.control = 0;
        self.packet = [0; 12];
        self.packet_pos = 0;
        self.packet_expected = false;
        self.data.clear();
        self.data_pos = 0;
        self.sense_key = 0;
        self.sense_asc = 0;
        self.hdd_locked = true;
        self.hdd_unlock_pending = false;
        self.hdd_data_out_remaining = 0;
    }

    fn word_set(buffer: &mut [u8; 512], word: usize, value: u16) {
        buffer[word * 2..word * 2 + 2].copy_from_slice(&value.to_le_bytes());
    }

    fn ata_string(buffer: &mut [u8; 512], first_word: usize, words: usize, value: &str) {
        let mut padded = vec![b' '; words * 2];
        let bytes = value.as_bytes();
        let count = bytes.len().min(padded.len());
        padded[..count].copy_from_slice(&bytes[..count]);
        for (index, pair) in padded.as_chunks::<2>().0.iter().enumerate() {
            buffer[(first_word + index) * 2] = pair[1];
            buffer[(first_word + index) * 2 + 1] = pair[0];
        }
    }

    fn hdd_identify(&self) -> Vec<u8> {
        let mut identify = [0u8; 512];
        Self::word_set(&mut identify, 0, 0x0040);
        Self::word_set(&mut identify, 1, 16_383);
        Self::word_set(&mut identify, 3, 16);
        Self::word_set(&mut identify, 6, 63);
        Self::ata_string(&mut identify, 10, 10, "OMNIEMU000000000001");
        Self::ata_string(&mut identify, 23, 4, "1.00");
        Self::ata_string(&mut identify, 27, 20, "ST310014ACE");
        Self::word_set(&mut identify, 47, 0x8001);
        Self::word_set(&mut identify, 49, 0x0b00);
        Self::word_set(&mut identify, 53, 0x0007);
        Self::word_set(&mut identify, 60, 0x0000);
        Self::word_set(&mut identify, 61, 0x0100);
        Self::word_set(&mut identify, 63, 0x0407);
        Self::word_set(&mut identify, 80, 0x007e);
        Self::word_set(&mut identify, 83, 1 << 10);
        Self::word_set(&mut identify, 88, 0x203f);
        let security = 0x0003 | if self.hdd_locked { 0x0004 } else { 0 };
        Self::word_set(&mut identify, 128, security);
        identify.to_vec()
    }

    fn dvd_identify(&self) -> Vec<u8> {
        let mut identify = [0u8; 512];
        Self::word_set(&mut identify, 0, 0x8580);
        Self::ata_string(&mut identify, 10, 10, "OMNIEMU-DVD-0000001");
        Self::ata_string(&mut identify, 23, 4, "1.00");
        Self::ata_string(&mut identify, 27, 20, "SAMSUNG DVD-ROM SDG-605B");
        Self::word_set(&mut identify, 49, 0x0b00);
        Self::word_set(&mut identify, 53, 0x0007);
        Self::word_set(&mut identify, 63, 0x0407);
        Self::word_set(&mut identify, 80, 0x007e);
        Self::word_set(&mut identify, 88, 0x203f);
        identify.to_vec()
    }

    fn host_byte_limit(&self) -> usize {
        let limit = usize::from(u16::from_le_bytes([
            self.byte_count_low,
            self.byte_count_high,
        ]));
        if limit == 0 {
            65_536
        } else {
            limit
        }
    }

    fn set_data(&mut self, mut data: Vec<u8>) {
        data.truncate(self.host_byte_limit());
        self.data = data;
        self.data_pos = 0;
        self.error = 0;
        if self.data.is_empty() {
            self.complete();
        } else {
            let count = self.data.len().min(0xffff) as u16;
            [self.byte_count_low, self.byte_count_high] = count.to_le_bytes();
            self.sector_count = 0x02;
            self.status = ATA_STATUS_DRDY | ATA_STATUS_DRQ;
        }
    }

    fn complete(&mut self) {
        self.status = ATA_STATUS_DRDY;
        self.sector_count = 0x03;
        self.byte_count_low = 0;
        self.byte_count_high = 0;
    }

    fn fail(&mut self, key: u8, asc: u8) {
        self.sense_key = key;
        self.sense_asc = asc;
        self.error = key << 4;
        self.status = ATA_STATUS_DRDY | ATA_STATUS_ERR;
        self.sector_count = 0x03;
        self.data.clear();
        self.data_pos = 0;
    }

    fn read_disc_blocks(&mut self, lba: u32, blocks: u32) {
        let Some(disc) = self.disc.as_ref() else {
            self.fail(0x02, 0x3a);
            return;
        };
        let Some(length) = usize::try_from(blocks)
            .ok()
            .and_then(|count| count.checked_mul(ATAPI_SECTOR_SIZE))
        else {
            self.fail(0x05, 0x21);
            return;
        };
        let offset = u64::from(lba) * ATAPI_SECTOR_SIZE as u64;
        if offset.saturating_add(length as u64) > disc.len() {
            self.fail(0x05, 0x21);
            return;
        }
        let mut data = vec![0; length];
        if disc.read(offset, &mut data).is_err() {
            self.fail(0x03, 0x11);
            return;
        }
        self.set_data(data);
    }

    fn execute_packet(&mut self) {
        let packet = self.packet;
        self.packet_pos = 0;
        match packet[0] {
            0x00 => {
                if self.disc.is_some() {
                    self.complete();
                } else {
                    self.fail(0x02, 0x3a);
                }
            }
            0x03 => {
                let mut sense = vec![0; 18];
                sense[0] = 0x70;
                sense[2] = self.sense_key;
                sense[7] = 10;
                sense[12] = self.sense_asc;
                self.sense_key = 0;
                self.sense_asc = 0;
                sense.truncate(usize::from(packet[4]).min(sense.len()));
                self.set_data(sense);
            }
            0x12 => {
                let mut inquiry = vec![0; 36];
                inquiry[0] = 0x05;
                inquiry[1] = 0x80;
                inquiry[2] = 0x00;
                inquiry[3] = 0x21;
                inquiry[4] = 31;
                inquiry[8..16].copy_from_slice(b"SAMSUNG ");
                inquiry[16..32].copy_from_slice(b"DVD-ROM SDG-605B");
                inquiry[32..36].copy_from_slice(b"1.00");
                inquiry.truncate(usize::from(packet[4]).min(inquiry.len()));
                self.set_data(inquiry);
            }
            0x1b | 0x1e | 0x2b => self.complete(),
            0x25 => {
                let sectors = self
                    .disc
                    .as_ref()
                    .map(|disc| disc.len() / ATAPI_SECTOR_SIZE as u64)
                    .unwrap_or(0);
                if sectors == 0 {
                    self.fail(0x02, 0x3a);
                } else {
                    let last_lba = sectors.saturating_sub(1).min(u64::from(u32::MAX)) as u32;
                    let mut response = Vec::with_capacity(8);
                    response.extend_from_slice(&last_lba.to_be_bytes());
                    response.extend_from_slice(&(ATAPI_SECTOR_SIZE as u32).to_be_bytes());
                    self.set_data(response);
                }
            }
            0x28 => {
                let lba = u32::from_be_bytes([packet[2], packet[3], packet[4], packet[5]]);
                let blocks = u32::from(u16::from_be_bytes([packet[7], packet[8]]));
                self.read_disc_blocks(lba, blocks);
            }
            0x43 => {
                let allocation = usize::from(u16::from_be_bytes([packet[7], packet[8]]));
                let sectors = self
                    .disc
                    .as_ref()
                    .map(|disc| disc.len() / ATAPI_SECTOR_SIZE as u64)
                    .unwrap_or(0)
                    .min(u64::from(u32::MAX)) as u32;
                let mut toc = vec![0; 20];
                toc[0..2].copy_from_slice(&18u16.to_be_bytes());
                toc[2] = 1;
                toc[3] = 1;
                toc[5] = 0x14;
                toc[6] = 1;
                toc[13] = 0x16;
                toc[14] = 0xaa;
                toc[16..20].copy_from_slice(&sectors.to_be_bytes());
                toc.truncate(allocation.min(toc.len()));
                self.set_data(toc);
            }
            0x46 => {
                let allocation = usize::from(u16::from_be_bytes([packet[7], packet[8]]));
                let mut response = vec![0; 8];
                response[3] = 4;
                response[6..8].copy_from_slice(&0x0010u16.to_be_bytes());
                response.truncate(allocation.min(response.len()));
                self.set_data(response);
            }
            0x4a => {
                let allocation = usize::from(u16::from_be_bytes([packet[7], packet[8]]));
                let mut response = vec![0, 6, 0x04, 0x10, 0x02, 0, 0, 0];
                response.truncate(allocation.min(response.len()));
                self.set_data(response);
            }
            0x51 => {
                let allocation = usize::from(u16::from_be_bytes([packet[7], packet[8]]));
                let mut info = vec![0; 34];
                info[0..2].copy_from_slice(&32u16.to_be_bytes());
                info[2] = 0x0e;
                info[3] = 1;
                info[4] = 1;
                info[6] = 1;
                info.truncate(allocation.min(info.len()));
                self.set_data(info);
            }
            0x5a => {
                let allocation = usize::from(u16::from_be_bytes([packet[7], packet[8]]));
                let page_code = packet[2] & 0x3f;
                let mut mode = vec![0; 28];
                mode[0..2].copy_from_slice(&26u16.to_be_bytes());
                if page_code == 0x3e || page_code == 0x3f {
                    mode[8] = 0x3e;
                    mode[9] = 18;
                    mode[10] = 1;
                    mode[11] = 1;
                    mode[12] = 1;
                    mode[13] = 0xd0;
                }
                mode.truncate(allocation.min(mode.len()));
                self.set_data(mode);
            }
            0xa8 => {
                let lba = u32::from_be_bytes([packet[2], packet[3], packet[4], packet[5]]);
                let blocks = u32::from_be_bytes([packet[6], packet[7], packet[8], packet[9]]);
                self.read_disc_blocks(lba, blocks);
            }
            0xad => {
                let allocation = usize::from(u16::from_be_bytes([packet[8], packet[9]]));
                let mut structure = vec![0; 20];
                structure[0..2].copy_from_slice(&18u16.to_be_bytes());
                structure[4] = 0xd0;
                structure[5] = 0x0f;
                structure.truncate(allocation.min(structure.len()));
                self.set_data(structure);
            }
            _ => self.fail(0x05, 0x20),
        }
    }

    fn issue_command(&mut self, command: u8) {
        self.error = 0;
        self.data.clear();
        self.data_pos = 0;
        self.packet_pos = 0;
        self.packet_expected = false;
        self.hdd_unlock_pending = false;
        if self.selected_slave {
            match command {
                0xa0 => {
                    self.packet_expected = true;
                    self.sector_count = 0x01;
                    self.status = ATA_STATUS_DRDY | ATA_STATUS_DRQ;
                }
                0xa1 => self.set_data(self.dvd_identify()),
                0xef | 0x08 => self.complete(),
                _ => {
                    self.error = ATA_ERROR_ABORT;
                    self.status = ATA_STATUS_DRDY | ATA_STATUS_ERR;
                }
            }
        } else {
            match command {
                0xec => self.set_data(self.hdd_identify()),
                0xef | 0xc6 | 0x40 | 0x91 => self.complete(),
                0xf2 => {
                    self.hdd_unlock_pending = true;
                    self.hdd_data_out_remaining = 512;
                    self.status = ATA_STATUS_DRDY | ATA_STATUS_DRQ;
                }
                0x20 => {
                    let sectors = if self.sector_count == 0 {
                        256
                    } else {
                        usize::from(self.sector_count)
                    };
                    self.set_data(vec![0; sectors * 512]);
                }
                0x30 => {
                    let sectors = if self.sector_count == 0 {
                        256
                    } else {
                        usize::from(self.sector_count)
                    };
                    self.hdd_data_out_remaining = sectors * 512;
                    self.status = ATA_STATUS_DRDY | ATA_STATUS_DRQ;
                }
                _ => {
                    self.error = ATA_ERROR_ABORT;
                    self.status = ATA_STATUS_DRDY | ATA_STATUS_ERR;
                }
            }
        }
    }

    fn read_data8(&mut self) -> u8 {
        if self.data_pos >= self.data.len() {
            return 0;
        }
        let value = self.data[self.data_pos];
        self.data_pos += 1;
        if self.data_pos >= self.data.len() {
            self.data.clear();
            self.data_pos = 0;
            self.complete();
        }
        value
    }

    fn write_data8(&mut self, value: u8) {
        if self.selected_slave && self.packet_expected && self.status & ATA_STATUS_DRQ != 0 {
            if usize::from(self.packet_pos) < self.packet.len() {
                self.packet[usize::from(self.packet_pos)] = value;
                self.packet_pos += 1;
                if usize::from(self.packet_pos) == self.packet.len() {
                    self.packet_expected = false;
                    self.execute_packet();
                }
            }
            return;
        }
        if self.hdd_data_out_remaining != 0 {
            self.hdd_data_out_remaining -= 1;
            if self.hdd_data_out_remaining == 0 {
                if self.hdd_unlock_pending {
                    self.hdd_locked = false;
                    self.hdd_unlock_pending = false;
                }
                self.complete();
            }
        }
    }

    fn read_port(&mut self, port: u16) -> u8 {
        match port {
            IDE_PRIMARY_BASE => self.read_data8(),
            0x01f1 => self.error,
            0x01f2 => self.sector_count,
            0x01f3 => self.lba_low,
            0x01f4 => self.byte_count_low,
            0x01f5 => self.byte_count_high,
            0x01f6 => self.drive_head,
            0x01f7 | IDE_PRIMARY_CONTROL => self.status,
            _ => 0xff,
        }
    }

    fn write_port(&mut self, port: u16, value: u8) {
        match port {
            IDE_PRIMARY_BASE => self.write_data8(value),
            0x01f1 => self.feature = value,
            0x01f2 => self.sector_count = value,
            0x01f3 => self.lba_low = value,
            0x01f4 => self.byte_count_low = value,
            0x01f5 => self.byte_count_high = value,
            0x01f6 => {
                self.drive_head = value;
                self.selected_slave = value & 0x10 != 0;
                self.status = ATA_STATUS_DRDY;
                self.error = 0;
            }
            0x01f7 => self.issue_command(value),
            IDE_PRIMARY_CONTROL => {
                self.control = value;
                if value & 0x04 != 0 {
                    self.status = ATA_STATUS_BSY;
                } else if self.status & ATA_STATUS_BSY != 0 {
                    self.reset_runtime();
                }
            }
            _ => {}
        }
    }

    fn read_data16(&mut self) -> u16 {
        u16::from_le_bytes([self.read_data8(), self.read_data8()])
    }

    fn write_data16(&mut self, value: u16) {
        for byte in value.to_le_bytes() {
            self.write_data8(byte);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u8(u8::from(self.selected_slave));
        for value in [
            self.error,
            self.feature,
            self.sector_count,
            self.lba_low,
            self.byte_count_low,
            self.byte_count_high,
            self.drive_head,
            self.status,
            self.control,
        ] {
            out.u8(value);
        }
        out.blob(&self.packet);
        out.u8(self.packet_pos);
        out.u8(u8::from(self.packet_expected));
        out.blob(&self.data);
        out.u64(self.data_pos as u64);
        out.u8(self.sense_key);
        out.u8(self.sense_asc);
        out.u8(u8::from(self.hdd_locked));
        out.u8(u8::from(self.hdd_unlock_pending));
        out.u64(self.hdd_data_out_remaining as u64);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.selected_slave = input.u8()? != 0;
        self.error = input.u8()?;
        self.feature = input.u8()?;
        self.sector_count = input.u8()?;
        self.lba_low = input.u8()?;
        self.byte_count_low = input.u8()?;
        self.byte_count_high = input.u8()?;
        self.drive_head = input.u8()?;
        self.status = input.u8()?;
        self.control = input.u8()?;
        let packet = input.blob()?;
        if packet.len() != self.packet.len() {
            return Err("Xbox IDE packet state has the wrong size".into());
        }
        self.packet.copy_from_slice(packet);
        self.packet_pos = input.u8()?;
        self.packet_expected = input.u8()? != 0;
        self.data = input.blob()?.to_vec();
        self.data_pos = usize::try_from(input.u64()?)
            .map_err(|_| "Xbox IDE data position exceeds host size".to_string())?;
        self.sense_key = input.u8()?;
        self.sense_asc = input.u8()?;
        self.hdd_locked = input.u8()? != 0;
        self.hdd_unlock_pending = input.u8()? != 0;
        self.hdd_data_out_remaining = usize::try_from(input.u64()?)
            .map_err(|_| "Xbox IDE output position exceeds host size".to_string())?;
        Ok(())
    }
}

#[derive(Clone)]
struct XboxKernelHeap {
    start: u32,
    next: u32,
    allocations: Vec<(u32, u32)>,
    free: Vec<(u32, u32)>,
}

impl XboxKernelHeap {
    fn new(start: u32) -> Self {
        Self {
            start,
            next: start,
            allocations: Vec::new(),
            free: Vec::new(),
        }
    }

    fn reset(&mut self, start: u32) {
        *self = Self::new(start);
    }

    fn align_up(value: u32, alignment: u32) -> Option<u32> {
        let alignment = alignment.max(1);
        if !alignment.is_power_of_two() {
            return None;
        }
        value
            .checked_add(alignment - 1)
            .map(|value| value & !(alignment - 1))
    }

    fn allocate(&mut self, size: u32, alignment: u32) -> Option<u32> {
        let size = size.max(1);
        let alignment = alignment.max(1);
        if !alignment.is_power_of_two() {
            return None;
        }

        for index in 0..self.free.len() {
            let (base, length) = self.free[index];
            let aligned = Self::align_up(base, alignment)?;
            let end = aligned.checked_add(size)?;
            let block_end = base.checked_add(length)?;
            if end > block_end {
                continue;
            }
            self.free.swap_remove(index);
            if aligned > base {
                self.free.push((base, aligned - base));
            }
            if end < block_end {
                self.free.push((end, block_end - end));
            }
            self.coalesce_free();
            self.allocations.push((aligned, size));
            return Some(aligned);
        }

        let address = Self::align_up(self.next, alignment)?;
        let end = address.checked_add(size)?;
        if end > XBOX_KERNEL_HEAP_LIMIT {
            return None;
        }
        self.next = end;
        self.allocations.push((address, size));
        Some(address)
    }

    fn free(&mut self, address: u32) -> Option<u32> {
        let index = self
            .allocations
            .iter()
            .position(|&(base, _)| base == address)?;
        let (base, size) = self.allocations.swap_remove(index);
        self.free.push((base, size));
        self.coalesce_free();
        Some(size)
    }

    fn allocation_size(&self, address: u32) -> Option<u32> {
        self.allocations
            .iter()
            .find_map(|&(base, size)| (base == address).then_some(size))
    }

    fn coalesce_free(&mut self) {
        self.free.sort_unstable_by_key(|&(base, _)| base);
        let mut merged: Vec<(u32, u32)> = Vec::with_capacity(self.free.len());
        for (base, size) in self.free.drain(..) {
            if let Some(last) = merged.last_mut() {
                if last.0.checked_add(last.1) == Some(base) {
                    last.1 = last.1.saturating_add(size);
                    continue;
                }
            }
            merged.push((base, size));
        }
        self.free = merged;
    }

    fn save(&self, out: &mut StateWriter) {
        out.u32(self.start);
        out.u32(self.next);
        out.u32(self.allocations.len() as u32);
        for &(base, size) in &self.allocations {
            out.u32(base);
            out.u32(size);
        }
        out.u32(self.free.len() as u32);
        for &(base, size) in &self.free {
            out.u32(base);
            out.u32(size);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let start = input.u32()?;
        let next = input.u32()?;
        if !(XBOX_KERNEL_HEAP_ALIAS..XBOX_KERNEL_HEAP_LIMIT).contains(&start)
            || next < start
            || next > XBOX_KERNEL_HEAP_LIMIT
        {
            return Err("Xbox kernel heap state has invalid bounds".into());
        }
        let allocation_count = input.u32()? as usize;
        if allocation_count > 1_000_000 {
            return Err("Xbox kernel heap state has too many allocations".into());
        }
        let mut allocations = Vec::with_capacity(allocation_count);
        for _ in 0..allocation_count {
            let base = input.u32()?;
            let size = input.u32()?;
            if base < start
                || base
                    .checked_add(size)
                    .is_none_or(|end| end > XBOX_KERNEL_HEAP_LIMIT)
            {
                return Err("Xbox kernel heap allocation is out of range".into());
            }
            allocations.push((base, size));
        }
        let free_count = input.u32()? as usize;
        if free_count > 1_000_000 {
            return Err("Xbox kernel heap state has too many free blocks".into());
        }
        let mut free = Vec::with_capacity(free_count);
        for _ in 0..free_count {
            let base = input.u32()?;
            let size = input.u32()?;
            if base < start
                || base
                    .checked_add(size)
                    .is_none_or(|end| end > XBOX_KERNEL_HEAP_LIMIT)
            {
                return Err("Xbox kernel heap free block is out of range".into());
            }
            free.push((base, size));
        }
        self.start = start;
        self.next = next;
        self.allocations = allocations;
        self.free = free;
        self.coalesce_free();
        Ok(())
    }
}

struct XboxBoard {
    ram: Box<[u8]>,
    nv2a: Box<[u8]>,
    apu: Box<[u8]>,
    ac97: Box<[u8]>,
    kernel_data: Box<[u8]>,
    kernel_heap: XboxKernelHeap,
    smbus: XboxSmbus,
    pci: XboxPci,
    ohci: [XboxOhci; 2],
    ide: XboxIde,
    input: InputState,
    image: ResourceBlob,
    video: VideoBuffer,
    audio: AudioBuffer,
    frame_counter: u64,
    display_format: u32,
    display_pitch: u32,
}

impl XboxBoard {
    fn new(image: ResourceBlob, disc: Option<ResourceBlob>) -> Result<(Self, u32), String> {
        let mut ram = vec![0; RAM_SIZE].into_boxed_slice();
        let loaded = load_xbe(&image, &mut ram)?;
        Ok((
            Self {
                ram,
                nv2a: vec![0; NV2A_SIZE].into_boxed_slice(),
                apu: vec![0; APU_SIZE].into_boxed_slice(),
                ac97: vec![0; AC97_SIZE].into_boxed_slice(),
                kernel_data: new_kernel_data(),
                kernel_heap: XboxKernelHeap::new(loaded.heap_base),
                smbus: XboxSmbus {
                    media_present: disc.is_some(),
                    ..XboxSmbus::default()
                },
                pci: XboxPci::new(),
                ohci: [XboxOhci::new(true), XboxOhci::new(false)],
                ide: XboxIde::new(disc),
                input: InputState::default(),
                image,
                video: VideoBuffer::new(VIDEO_WIDTH, VIDEO_HEIGHT),
                audio: AudioBuffer::new(AUDIO_RATE, 2),
                frame_counter: 0,
                display_format: 0x12,
                display_pitch: VIDEO_WIDTH * 4,
            },
            loaded.entry,
        ))
    }

    fn nv2a_u32(&self, offset: usize) -> u32 {
        u32::from_le_bytes(self.nv2a[offset..offset + 4].try_into().unwrap())
    }
    fn present_video(&mut self) {
        let start = (self.nv2a_u32(PCRTC_START) as usize) & (RAM_SIZE - 1);
        let bytes_per_pixel = match self.display_format {
            0x10 | 0x11 | 0x1c => 2usize,
            _ => 4usize,
        };
        let minimum_pitch = VIDEO_WIDTH as usize * bytes_per_pixel;
        let pitch = if self.display_pitch == 0 {
            minimum_pitch
        } else {
            self.display_pitch as usize
        };
        let Some(last_row) = (VIDEO_HEIGHT as usize)
            .checked_sub(1)
            .and_then(|row| row.checked_mul(pitch))
        else {
            self.video.clear([0, 0, 0, 255]);
            return;
        };
        let Some(bytes) = last_row.checked_add(minimum_pitch) else {
            self.video.clear([0, 0, 0, 255]);
            return;
        };
        if start
            .checked_add(bytes)
            .is_none_or(|end| end > self.ram.len())
        {
            self.video.clear([0, 0, 0, 255]);
            return;
        }

        let pixels = self.video.pixels_mut().as_chunks_mut::<4>().0;
        for y in 0..VIDEO_HEIGHT as usize {
            let row = start + y * pitch;
            for x in 0..VIDEO_WIDTH as usize {
                let target = &mut pixels[y * VIDEO_WIDTH as usize + x];
                let source = row + x * bytes_per_pixel;
                match self.display_format {
                    0x10 | 0x1c => {
                        let value =
                            u16::from_le_bytes(self.ram[source..source + 2].try_into().unwrap());
                        let red = ((value >> 10) & 0x1f) as u32 * 255 / 31;
                        let green = ((value >> 5) & 0x1f) as u32 * 255 / 31;
                        let blue = (value & 0x1f) as u32 * 255 / 31;
                        *target = [red as u8, green as u8, blue as u8, 255];
                    }
                    0x11 => {
                        let value =
                            u16::from_le_bytes(self.ram[source..source + 2].try_into().unwrap());
                        let red = ((value >> 11) & 0x1f) as u32 * 255 / 31;
                        let green = ((value >> 5) & 0x3f) as u32 * 255 / 63;
                        let blue = (value & 0x1f) as u32 * 255 / 31;
                        *target = [red as u8, green as u8, blue as u8, 255];
                    }
                    _ => {
                        let source = &self.ram[source..source + 4];
                        *target = [source[2], source[1], source[0], 255];
                    }
                }
            }
        }
    }

    fn service_usb(&mut self) {
        let input = self.input.clone();
        for ohci in &mut self.ohci {
            ohci.service(&mut self.ram, &input);
        }
    }

    fn begin_frame(&mut self, input: &InputState) {
        self.input = input.clone();
        self.service_usb();
    }

    fn update_kernel_time(&mut self) {
        let milliseconds = self.frame_counter.saturating_mul(1000) / 60;
        let hundred_ns = milliseconds.saturating_mul(0x2710);
        let tick = kernel_data_offset(156);
        self.kernel_data[tick..tick + 4].copy_from_slice(&(milliseconds as u32).to_le_bytes());
        for ordinal in [120, 154] {
            let offset = kernel_data_offset(ordinal);
            let low = hundred_ns as u32;
            let high = (hundred_ns >> 32) as u32;
            self.kernel_data[offset..offset + 4].copy_from_slice(&low.to_le_bytes());
            self.kernel_data[offset + 4..offset + 8].copy_from_slice(&high.to_le_bytes());
            self.kernel_data[offset + 8..offset + 12].copy_from_slice(&high.to_le_bytes());
        }
    }

    fn end_frame(&mut self) {
        self.service_usb();
        self.present_video();
        self.audio.begin_frame();
        for _ in 0..(AUDIO_RATE / 60) {
            self.audio.push_stereo(0.0, 0.0);
        }
        let enabled = self.nv2a_u32(PCRTC_INTR_EN) & 1 != 0;
        if enabled {
            self.nv2a[PCRTC_INTR..PCRTC_INTR + 4].copy_from_slice(&1u32.to_le_bytes());
        }
        self.frame_counter = self.frame_counter.wrapping_add(1);
        self.update_kernel_time();
    }

    fn reset(&mut self) -> Result<u32, String> {
        self.ram.fill(0);
        self.nv2a.fill(0);
        self.apu.fill(0);
        self.ac97.fill(0);
        self.kernel_data = new_kernel_data();
        self.smbus.reset_runtime();
        self.pci = XboxPci::new();
        self.ohci = [XboxOhci::new(true), XboxOhci::new(false)];
        self.ide.reset_runtime();
        self.input = InputState::default();
        self.video.clear([0, 0, 0, 255]);
        self.audio.begin_frame();
        self.frame_counter = 0;
        self.display_format = 0x12;
        self.display_pitch = VIDEO_WIDTH * 4;
        let loaded = load_xbe(&self.image, &mut self.ram)?;
        self.kernel_heap.reset(loaded.heap_base);
        Ok(loaded.entry)
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.ram);
        out.blob(&self.nv2a);
        out.blob(&self.apu);
        out.blob(&self.ac97);
        out.blob(&self.kernel_data);
        self.kernel_heap.save(out);
        self.smbus.save(out);
        self.pci.save(out);
        for ohci in &self.ohci {
            ohci.save(out);
        }
        self.ide.save(out);
        out.u64(self.frame_counter);
        out.u32(self.display_format);
        out.u32(self.display_pitch);
    }

    fn load_blob(
        input: &mut StateReader<'_>,
        target: &mut [u8],
        label: &str,
    ) -> Result<(), String> {
        let bytes = input.blob()?;
        if bytes.len() != target.len() {
            return Err(format!(
                "Xbox {label} state has {} bytes; expected {}",
                bytes.len(),
                target.len()
            ));
        }
        target.copy_from_slice(bytes);
        Ok(())
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        Self::load_blob(input, &mut self.ram, "RAM")?;
        Self::load_blob(input, &mut self.nv2a, "NV2A")?;
        Self::load_blob(input, &mut self.apu, "APU")?;
        Self::load_blob(input, &mut self.ac97, "AC97")?;
        Self::load_blob(input, &mut self.kernel_data, "kernel data")?;
        self.kernel_heap.load(input)?;
        self.smbus.load(input)?;
        self.pci.load(input)?;
        for ohci in &mut self.ohci {
            ohci.load(input)?;
        }
        self.ide.load(input)?;
        self.input = InputState::default();
        self.frame_counter = input.u64()?;
        self.display_format = input.u32()?;
        self.display_pitch = input.u32()?;
        self.present_video();
        self.audio.begin_frame();
        Ok(())
    }
}

impl X86Bus for XboxBoard {
    fn read8(&mut self, address: u32) -> u8 {
        if let Some(index) = ram_index(address) {
            return self.ram[index];
        }
        if let Some(index) = kernel_data_index(address) {
            return self.kernel_data[index];
        }
        for (index, base) in [USB0_BASE, USB1_BASE].into_iter().enumerate() {
            if let Some(offset) = address
                .checked_sub(base)
                .filter(|offset| *offset < USB_MMIO_SIZE)
            {
                let value = self.ohci[index].read32(offset & !3);
                return (value >> ((offset & 3) * 8)) as u8;
            }
        }
        if let Some(offset) = address
            .checked_sub(NV2A_BASE)
            .filter(|offset| *offset < NV2A_SIZE as u32)
        {
            return self.nv2a[offset as usize];
        }
        if let Some(offset) = address
            .checked_sub(APU_BASE)
            .filter(|offset| *offset < APU_SIZE as u32)
        {
            return self.apu[offset as usize];
        }
        if let Some(offset) = address
            .checked_sub(AC97_BASE)
            .filter(|offset| *offset < AC97_SIZE as u32)
        {
            return self.ac97[offset as usize];
        }
        0xff
    }

    fn write8(&mut self, address: u32, value: u8) {
        if let Some(index) = ram_index(address) {
            self.ram[index] = value;
            return;
        }
        if let Some(index) = kernel_data_index(address) {
            self.kernel_data[index] = value;
            return;
        }
        for (index, base) in [USB0_BASE, USB1_BASE].into_iter().enumerate() {
            if let Some(offset) = address
                .checked_sub(base)
                .filter(|offset| *offset < USB_MMIO_SIZE)
            {
                self.ohci[index].write8(offset, value);
                return;
            }
        }
        if let Some(offset) = address
            .checked_sub(NV2A_BASE)
            .filter(|offset| *offset < NV2A_SIZE as u32)
        {
            self.nv2a[offset as usize] = value;
            return;
        }
        if let Some(offset) = address
            .checked_sub(APU_BASE)
            .filter(|offset| *offset < APU_SIZE as u32)
        {
            self.apu[offset as usize] = value;
            return;
        }
        if let Some(offset) = address
            .checked_sub(AC97_BASE)
            .filter(|offset| *offset < AC97_SIZE as u32)
        {
            self.ac97[offset as usize] = value;
        }
    }

    fn read32(&mut self, address: u32) -> u32 {
        for (index, base) in [USB0_BASE, USB1_BASE].into_iter().enumerate() {
            if let Some(offset) = address
                .checked_sub(base)
                .filter(|offset| *offset < USB_MMIO_SIZE && *offset & 3 == 0)
            {
                return self.ohci[index].read32(offset);
            }
        }
        u32::from_le_bytes([
            self.read8(address),
            self.read8(address.wrapping_add(1)),
            self.read8(address.wrapping_add(2)),
            self.read8(address.wrapping_add(3)),
        ])
    }

    fn write32(&mut self, address: u32, value: u32) {
        for (index, base) in [USB0_BASE, USB1_BASE].into_iter().enumerate() {
            if let Some(offset) = address
                .checked_sub(base)
                .filter(|offset| *offset < USB_MMIO_SIZE && *offset & 3 == 0)
            {
                self.ohci[index].write32(offset, value);
                return;
            }
        }
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u32), byte);
        }
    }

    fn io_read8(&mut self, port: u16) -> u8 {
        if (IDE_PRIMARY_BASE..=IDE_PRIMARY_BASE + 7).contains(&port) || port == IDE_PRIMARY_CONTROL
        {
            return self.ide.read_port(port);
        }
        if (PCI_CONFIG_ADDRESS..PCI_CONFIG_DATA + 4).contains(&port) {
            return self.pci.read_port(port);
        }
        let smbus_base = self.pci.smbus_base();
        if (smbus_base..smbus_base + 0x10).contains(&port) {
            self.smbus.read_port(SMBUS_BASE + (port - smbus_base))
        } else {
            0xff
        }
    }

    fn io_write8(&mut self, port: u16, value: u8) {
        if (IDE_PRIMARY_BASE..=IDE_PRIMARY_BASE + 7).contains(&port) || port == IDE_PRIMARY_CONTROL
        {
            self.ide.write_port(port, value);
            return;
        }
        if (PCI_CONFIG_ADDRESS..PCI_CONFIG_DATA + 4).contains(&port) {
            self.pci.write_port(port, value);
            return;
        }
        let smbus_base = self.pci.smbus_base();
        if (smbus_base..smbus_base + 0x10).contains(&port) {
            self.smbus
                .write_port(SMBUS_BASE + (port - smbus_base), value);
        }
    }

    fn io_read16(&mut self, port: u16) -> u16 {
        if port == IDE_PRIMARY_BASE {
            return self.ide.read_data16();
        }
        u16::from_le_bytes([self.io_read8(port), self.io_read8(port.wrapping_add(1))])
    }

    fn io_write16(&mut self, port: u16, value: u16) {
        if port == IDE_PRIMARY_BASE {
            self.ide.write_data16(value);
            return;
        }
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.io_write8(port.wrapping_add(offset as u16), byte);
        }
    }

    fn io_read32(&mut self, port: u16) -> u32 {
        if port == IDE_PRIMARY_BASE {
            return u32::from(self.ide.read_data16()) | (u32::from(self.ide.read_data16()) << 16);
        }
        let mut bytes = [0; 4];
        for (offset, byte) in bytes.iter_mut().enumerate() {
            *byte = self.io_read8(port.wrapping_add(offset as u16));
        }
        u32::from_le_bytes(bytes)
    }

    fn io_write32(&mut self, port: u16, value: u32) {
        if port == IDE_PRIMARY_BASE {
            let [low, high] = [value as u16, (value >> 16) as u16];
            self.ide.write_data16(low);
            self.ide.write_data16(high);
            return;
        }
        for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
            self.io_write8(port.wrapping_add(offset as u16), byte);
        }
    }
}

pub struct XboxMachine {
    cpu: X86Cpu,
    board: XboxBoard,
    powered: bool,
    current_irql: u8,
    av_saved_data_address: u32,
    fsc_cache_pages: u32,
    hal_software_interrupts: u32,
    hal_system_interrupts_enabled: u32,
    hal_latched_interrupts: u32,
    mm_gpu_instance_bytes: u32,
    mm_persistent_pages: BTreeSet<u32>,
    mm_page_protection: BTreeMap<u32, u32>,
    hal_interrupt_objects: [u32; 28],
    dpc_queue: Vec<u32>,
    timer_queue: Vec<u32>,
}

impl XboxMachine {
    pub fn from_xbe(image: ResourceBlob) -> Result<Self, String> {
        Self::from_images(image, None)
    }

    pub fn from_images(image: ResourceBlob, disc: Option<ResourceBlob>) -> Result<Self, String> {
        let (board, entry) = XboxBoard::new(image, disc)?;
        let mut cpu = X86Cpu::new();
        Self::boot_cpu(&mut cpu, entry);
        let mut machine = Self {
            cpu,
            board,
            powered: true,
            current_irql: 0,
            av_saved_data_address: 0,
            fsc_cache_pages: 16,
            hal_software_interrupts: 0,
            hal_system_interrupts_enabled: 0,
            hal_latched_interrupts: 0,
            mm_gpu_instance_bytes: XBOX_GPU_INSTANCE_BYTES,
            mm_persistent_pages: BTreeSet::new(),
            mm_page_protection: BTreeMap::new(),
            hal_interrupt_objects: [0; 28],
            dpc_queue: Vec::new(),
            timer_queue: Vec::new(),
        };
        machine.initialize_current_thread_state();
        Ok(machine)
    }

    fn boot_cpu(cpu: &mut X86Cpu, entry: u32) {
        cpu.reset_to(entry);
        cpu.regs[ESP] = 0x03ff_ffc0;
    }

    fn current_thread_address(&self) -> u32 {
        self.board.kernel_heap.start - XBOX_KERNEL_THREAD_RESERVE
    }

    fn initialize_current_thread_state(&mut self) {
        let thread = self.current_thread_address();
        let mutant_list = thread.wrapping_add(0x10);
        self.board.write32(mutant_list, mutant_list);
        self.board.write32(mutant_list.wrapping_add(4), mutant_list);
        self.board.write8(thread.wrapping_add(0x32), 8);
        self.board.write8(thread.wrapping_add(0x4b), 1);
        self.board.write32(thread.wrapping_add(0x68), 0);
        self.board.write8(thread.wrapping_add(0x70), 8);
        self.board.write8(thread.wrapping_add(0x75), 0);
    }

    fn memory_statistics(&self) -> [u32; 9] {
        let total_pages = (RAM_SIZE / 0x1000) as u32;
        let mut image_base_bytes = [0; 4];
        let image_base = if self.board.image.read(0x104, &mut image_base_bytes).is_ok() {
            u32::from_le_bytes(image_base_bytes)
        } else {
            0
        };
        let image_end = self
            .current_thread_address()
            .saturating_sub(XBOX_KERNEL_HEAP_ALIAS);
        let image_pages = image_end.saturating_sub(image_base).div_ceil(0x1000);
        let pool_pages = self
            .board
            .kernel_heap
            .allocations
            .iter()
            .map(|&(_, size)| size.div_ceil(0x1000))
            .fold(0u32, u32::saturating_add);
        let cache_pages = self.fsc_cache_pages.min(total_pages);
        let stack_pages = 1u32;
        let gpu_pages = self.mm_gpu_instance_bytes.div_ceil(0x1000);
        let used_pages = image_pages
            .saturating_add(pool_pages)
            .saturating_add(cache_pages)
            .saturating_add(stack_pages)
            .saturating_add(gpu_pages)
            .min(total_pages);
        let image_bytes = image_pages.saturating_mul(0x1000);

        [
            9 * 4,
            total_pages,
            total_pages - used_pages,
            image_bytes,
            image_bytes,
            cache_pages,
            pool_pages,
            stack_pages,
            image_pages,
        ]
    }

    fn initialize_event(&mut self, address: u32, event_type: u8, signal_state: u32) -> bool {
        let Some(range) = ram_range(address, 16) else {
            return false;
        };
        let start = range.start;
        self.board.ram[range].fill(0);
        self.board.ram[start] = event_type;
        self.board.ram[start + 2] = 4;
        self.board.ram[start + 4..start + 8].copy_from_slice(&signal_state.to_le_bytes());
        let wait_list = address.wrapping_add(8);
        self.board.ram[start + 8..start + 12].copy_from_slice(&wait_list.to_le_bytes());
        self.board.ram[start + 12..start + 16].copy_from_slice(&wait_list.to_le_bytes());
        true
    }

    fn initialize_semaphore(&mut self, address: u32, count: i32, limit: i32) -> bool {
        if count < 0 || limit <= 0 || count > limit {
            return false;
        }
        let Some(range) = ram_range(address, 20) else {
            return false;
        };
        let start = range.start;
        self.board.ram[range].fill(0);
        self.board.ram[start] = 5;
        self.board.ram[start + 2] = 5;
        self.board.ram[start + 4..start + 8].copy_from_slice(&(count as u32).to_le_bytes());
        let wait_list = address.wrapping_add(8);
        self.board.ram[start + 8..start + 12].copy_from_slice(&wait_list.to_le_bytes());
        self.board.ram[start + 12..start + 16].copy_from_slice(&wait_list.to_le_bytes());
        self.board.ram[start + 16..start + 20].copy_from_slice(&(limit as u32).to_le_bytes());
        true
    }

    fn dispatcher_wait_list_empty(&mut self, address: u32) -> bool {
        let wait_list = address.wrapping_add(8);
        self.board.read32(wait_list) == wait_list
            && self.board.read32(wait_list.wrapping_add(4)) == wait_list
    }

    fn unlink_list_entry(&mut self, entry: u32) -> bool {
        if ram_range(entry, 8).is_none() {
            return false;
        }
        let next = self.board.read32(entry);
        let previous = self.board.read32(entry.wrapping_add(4));
        if ram_range(next, 8).is_none() || ram_range(previous, 8).is_none() {
            return false;
        }
        self.board.write32(previous, next);
        self.board.write32(next.wrapping_add(4), previous);
        true
    }

    fn initialize_device_queue(&mut self, address: u32) -> bool {
        let Some(range) = ram_range(address, 0x10) else {
            return false;
        };
        let start = range.start;
        self.board.ram[range].fill(0);
        self.board.ram[start..start + 2].copy_from_slice(&0x14u16.to_le_bytes());
        self.board.ram[start + 2] = 0x10;
        let list = address.wrapping_add(8);
        self.board.write32(list, list);
        self.board.write32(list.wrapping_add(4), list);
        true
    }

    fn initialize_dpc(&mut self, address: u32, routine: u32, context: u32) -> bool {
        let Some(range) = ram_range(address, 0x1c) else {
            return false;
        };
        let start = range.start;
        self.board.ram[range].fill(0);
        self.board.ram[start..start + 2].copy_from_slice(&0x13u16.to_le_bytes());
        self.board.write32(address.wrapping_add(0x0c), routine);
        self.board.write32(address.wrapping_add(0x10), context);
        true
    }

    fn rebuild_dpc_links(&mut self) -> bool {
        if self.dpc_queue.is_empty() {
            return true;
        }
        if self
            .dpc_queue
            .iter()
            .any(|&dpc| ram_range(dpc, 0x1c).is_none())
        {
            return false;
        }
        let queue = self.dpc_queue.clone();
        for (index, &dpc) in queue.iter().enumerate() {
            let previous = queue[(index + queue.len() - 1) % queue.len()].wrapping_add(4);
            let next = queue[(index + 1) % queue.len()].wrapping_add(4);
            let entry = dpc.wrapping_add(4);
            self.board.write32(entry, next);
            self.board.write32(entry.wrapping_add(4), previous);
        }
        true
    }

    fn enqueue_dpc(
        &mut self,
        dpc: u32,
        system_argument1: u32,
        system_argument2: u32,
    ) -> Option<bool> {
        ram_range(dpc, 0x1c)?;
        if self.board.read8(dpc.wrapping_add(2)) != 0 {
            return Some(false);
        }
        self.board.write8(dpc.wrapping_add(2), 1);
        self.board.write32(dpc.wrapping_add(0x14), system_argument1);
        self.board.write32(dpc.wrapping_add(0x18), system_argument2);
        self.dpc_queue.push(dpc);
        if !self.rebuild_dpc_links() {
            return None;
        }
        self.hal_software_interrupts |= 1u32 << 2;
        Some(true)
    }

    fn current_time_100ns(&self) -> u64 {
        let milliseconds = self.board.frame_counter.saturating_mul(1000) / 60;
        milliseconds.saturating_mul(10_000)
    }

    fn timer_due_time(&self, timer: u32) -> u64 {
        let start = ram_index(timer.wrapping_add(0x10)).unwrap();
        u64::from_le_bytes(self.board.ram[start..start + 8].try_into().unwrap())
    }

    fn set_timer_due_time(&mut self, timer: u32, due_time: u64) {
        self.board
            .write32(timer.wrapping_add(0x10), due_time as u32);
        self.board
            .write32(timer.wrapping_add(0x14), (due_time >> 32) as u32);
    }

    fn rebuild_timer_links(&mut self) -> bool {
        if self.timer_queue.is_empty() {
            return true;
        }
        if self
            .timer_queue
            .iter()
            .any(|&timer| ram_range(timer, 0x28).is_none())
        {
            return false;
        }
        let ram = &self.board.ram;
        self.timer_queue.sort_by_key(|&timer| {
            let start = ram_index(timer.wrapping_add(0x10)).unwrap();
            u64::from_le_bytes(ram[start..start + 8].try_into().unwrap())
        });
        let queue = self.timer_queue.clone();
        for (index, &timer) in queue.iter().enumerate() {
            let previous = queue[(index + queue.len() - 1) % queue.len()].wrapping_add(0x18);
            let next = queue[(index + 1) % queue.len()].wrapping_add(0x18);
            let entry = timer.wrapping_add(0x18);
            self.board.write32(entry, next);
            self.board.write32(entry.wrapping_add(4), previous);
        }
        true
    }

    fn remove_timer(&mut self, timer: u32) -> bool {
        if let Some(index) = self.timer_queue.iter().position(|&queued| queued == timer) {
            self.timer_queue.remove(index);
            self.board.write8(timer.wrapping_add(3), 0);
            self.board.write32(timer.wrapping_add(0x18), 0);
            self.board.write32(timer.wrapping_add(0x1c), 0);
            return self.rebuild_timer_links();
        }

        let entry = timer.wrapping_add(0x18);
        let next = self.board.read32(entry);
        let previous = self.board.read32(entry.wrapping_add(4));
        if next != 0 && previous != 0 && !self.unlink_list_entry(entry) {
            return false;
        }
        self.board.write8(timer.wrapping_add(3), 0);
        self.board.write32(entry, 0);
        self.board.write32(entry.wrapping_add(4), 0);
        true
    }

    fn set_timer(&mut self, timer: u32, due_time: i64, period: i32, dpc: u32) -> Option<bool> {
        if period < 0 || ram_range(timer, 0x28).is_none() {
            return None;
        }
        let timer_type = self.board.read8(timer);
        if !matches!(timer_type, 8 | 9) {
            return None;
        }

        let was_inserted = self.board.read8(timer.wrapping_add(3)) != 0;
        if was_inserted && !self.remove_timer(timer) {
            return None;
        }

        self.board.write32(timer.wrapping_add(0x20), dpc);
        self.board.write32(timer.wrapping_add(0x24), period as u32);

        let now = self.current_time_100ns();
        let deadline = if due_time < 0 {
            now.saturating_add(due_time.unsigned_abs())
        } else {
            due_time as u64
        };
        self.set_timer_due_time(timer, deadline);

        if deadline <= now {
            self.board.write8(timer.wrapping_add(3), 0);
            self.board.write32(timer.wrapping_add(4), 1);
            if period > 0 {
                let next = now.saturating_add(period as u64 * 10_000);
                self.set_timer_due_time(timer, next);
                self.board.write8(timer.wrapping_add(3), 1);
                self.timer_queue.push(timer);
                if !self.rebuild_timer_links() {
                    return None;
                }
            }
            if dpc != 0 {
                let _ = self.enqueue_dpc(dpc, now as u32, (now >> 32) as u32)?;
            }
        } else {
            if period == 0 {
                self.board.write32(timer.wrapping_add(4), 0);
            }
            self.board.write8(timer.wrapping_add(3), 1);
            self.timer_queue.push(timer);
            if !self.rebuild_timer_links() {
                return None;
            }
        }

        Some(was_inserted)
    }

    fn service_timers(&mut self) {
        let now = self.current_time_100ns();
        let due: Vec<u32> = self
            .timer_queue
            .iter()
            .copied()
            .filter(|&timer| self.timer_due_time(timer) <= now)
            .collect();

        for timer in due {
            let Some(index) = self.timer_queue.iter().position(|&queued| queued == timer) else {
                continue;
            };
            self.timer_queue.remove(index);
            self.board.write8(timer.wrapping_add(3), 0);
            self.board.write32(timer.wrapping_add(0x18), 0);
            self.board.write32(timer.wrapping_add(0x1c), 0);
            self.board.write32(timer.wrapping_add(4), 1);

            let period = self.board.read32(timer.wrapping_add(0x24));
            if period != 0 {
                let next = now.saturating_add(u64::from(period) * 10_000);
                self.set_timer_due_time(timer, next);
                self.board.write8(timer.wrapping_add(3), 1);
                self.timer_queue.push(timer);
            }

            let dpc = self.board.read32(timer.wrapping_add(0x20));
            if dpc != 0 {
                let _ = self.enqueue_dpc(dpc, now as u32, (now >> 32) as u32);
            }
        }

        let _ = self.rebuild_timer_links();
    }

    fn initialize_apc(
        &mut self,
        address: u32,
        thread: u32,
        routines: [u32; 3],
        apc_mode: u8,
        normal_context: u32,
    ) -> bool {
        if apc_mode > 1 {
            return false;
        }
        let [kernel_routine, rundown_routine, normal_routine] = routines;
        let Some(range) = ram_range(address, 0x28) else {
            return false;
        };
        let start = range.start;
        self.board.ram[range].fill(0);
        self.board.ram[start..start + 2].copy_from_slice(&0x12u16.to_le_bytes());
        self.board.ram[start + 2] = if normal_routine == 0 { 0 } else { apc_mode };
        self.board.write32(address.wrapping_add(4), thread);
        self.board
            .write32(address.wrapping_add(0x10), kernel_routine);
        self.board
            .write32(address.wrapping_add(0x14), rundown_routine);
        self.board
            .write32(address.wrapping_add(0x18), normal_routine);
        self.board.write32(
            address.wrapping_add(0x1c),
            if normal_routine == 0 {
                0
            } else {
                normal_context
            },
        );
        true
    }

    fn initialize_interrupt(
        &mut self,
        address: u32,
        service_routine: u32,
        service_context: u32,
        vector: u32,
        irql: u32,
        mode: u32,
    ) -> bool {
        let Some(bus_interrupt_level) = vector.checked_sub(0x30) else {
            return false;
        };
        if bus_interrupt_level > 27 || irql > 31 || mode > 1 {
            return false;
        }
        let Some(range) = ram_range(address, 0x70) else {
            return false;
        };
        let start = range.start;
        self.board.ram[range].fill(0);
        self.board.write32(address, service_routine);
        self.board.write32(address.wrapping_add(4), service_context);
        self.board
            .write32(address.wrapping_add(8), bus_interrupt_level);
        self.board.write32(address.wrapping_add(0x0c), irql);
        self.board.ram[start + 0x12..start + 0x14].copy_from_slice(&(mode as u16).to_le_bytes());
        true
    }

    fn initialize_mutant(&mut self, address: u32, initial_owner: bool) -> bool {
        let Some(range) = ram_range(address, 0x20) else {
            return false;
        };
        let start = range.start;
        self.board.ram[range].fill(0);
        self.board.ram[start] = 2;
        self.board.ram[start + 2] = 8;
        let wait_list = address.wrapping_add(8);
        self.board.write32(wait_list, wait_list);
        self.board.write32(wait_list.wrapping_add(4), wait_list);
        if !initial_owner {
            self.board.write32(address.wrapping_add(4), 1);
            return true;
        }

        let thread = self.current_thread_address();
        let list_head = thread.wrapping_add(0x10);
        let entry = address.wrapping_add(0x10);
        let first = self.board.read32(list_head);
        if ram_range(first, 8).is_none() {
            return false;
        }
        self.board.write32(entry, first);
        self.board.write32(entry.wrapping_add(4), list_head);
        self.board.write32(first.wrapping_add(4), entry);
        self.board.write32(list_head, entry);
        self.board.write32(address.wrapping_add(0x18), thread);
        true
    }

    fn initialize_queue(&mut self, address: u32, count: u32) -> bool {
        let Some(range) = ram_range(address, 0x28) else {
            return false;
        };
        let start = range.start;
        self.board.ram[range].fill(0);
        self.board.ram[start] = 4;
        self.board.ram[start + 2] = 10;
        let wait_list = address.wrapping_add(8);
        self.board.write32(wait_list, wait_list);
        self.board.write32(wait_list.wrapping_add(4), wait_list);
        let entry_list = address.wrapping_add(0x10);
        self.board.write32(entry_list, entry_list);
        self.board.write32(entry_list.wrapping_add(4), entry_list);
        self.board.write32(address.wrapping_add(0x1c), count.max(1));
        let thread_list = address.wrapping_add(0x20);
        self.board.write32(thread_list, thread_list);
        self.board.write32(thread_list.wrapping_add(4), thread_list);
        true
    }

    fn initialize_timer(&mut self, address: u32, timer_type: u8) -> bool {
        if timer_type > 1 {
            return false;
        }
        let Some(range) = ram_range(address, 0x28) else {
            return false;
        };
        let start = range.start;
        self.board.ram[range].fill(0);
        self.board.ram[start] = 8 + timer_type;
        self.board.ram[start + 2] = 10;
        let wait_list = address.wrapping_add(8);
        self.board.write32(wait_list, wait_list);
        self.board.write32(wait_list.wrapping_add(4), wait_list);
        true
    }

    fn initialize_read_write_lock(&mut self, address: u32) -> bool {
        if ram_range(address, 0x34).is_none() {
            return false;
        }
        self.board.write32(address, u32::MAX);
        self.board.write32(address.wrapping_add(4), 0);
        self.board.write32(address.wrapping_add(8), 0);
        self.board.write32(address.wrapping_add(0x0c), 0);
        self.initialize_event(address.wrapping_add(0x10), 1, 0)
            && self.initialize_semaphore(address.wrapping_add(0x20), 0, i32::MAX)
    }

    fn acquire_read_write_lock_exclusive(&mut self, address: u32) -> bool {
        if ram_range(address, 0x34).is_none() {
            return false;
        }
        let lock_count = self.board.read32(address) as i32;
        let readers = self.board.read32(address.wrapping_add(0x0c));
        if lock_count != -1 || readers != 0 {
            return false;
        }
        self.board.write32(address, 0);
        true
    }

    fn acquire_read_write_lock_shared(&mut self, address: u32) -> bool {
        if ram_range(address, 0x34).is_none() {
            return false;
        }
        let lock_count = self.board.read32(address) as i32;
        let writers_waiting = self.board.read32(address.wrapping_add(4));
        let readers = self.board.read32(address.wrapping_add(0x0c));
        if lock_count == -1 {
            self.board.write32(address, 0);
            self.board.write32(address.wrapping_add(0x0c), 1);
            return true;
        }
        if lock_count < 0 || readers == 0 || writers_waiting != 0 {
            return false;
        }
        let Some(next_lock_count) = lock_count.checked_add(1) else {
            return false;
        };
        let Some(next_readers) = readers.checked_add(1) else {
            return false;
        };
        self.board.write32(address, next_lock_count as u32);
        self.board.write32(address.wrapping_add(0x0c), next_readers);
        true
    }

    fn release_read_write_lock(&mut self, address: u32) -> bool {
        if ram_range(address, 0x34).is_none() {
            return false;
        }
        let lock_count = self.board.read32(address) as i32;
        if lock_count < 0 {
            return false;
        }
        let next_lock_count = lock_count - 1;
        self.board.write32(address, next_lock_count as u32);
        if next_lock_count == -1 {
            self.board.write32(address.wrapping_add(0x0c), 0);
            return true;
        }

        let readers = self.board.read32(address.wrapping_add(0x0c));
        if readers <= 1 || self.board.read32(address.wrapping_add(4)) != 0 {
            return false;
        }
        self.board.write32(address.wrapping_add(0x0c), readers - 1);
        true
    }

    fn enter_critical_region(&mut self) -> bool {
        let address = self.current_thread_address().wrapping_add(0x68);
        let value = self.board.read32(address).wrapping_sub(1);
        self.board.write32(address, value);
        true
    }

    fn leave_critical_region(&mut self) -> bool {
        let address = self.current_thread_address().wrapping_add(0x68);
        let value = self.board.read32(address).wrapping_add(1);
        self.board.write32(address, value);
        true
    }

    fn initialize_critical_section(&mut self, address: u32) -> bool {
        if !self.initialize_event(address, 1, 0) {
            return false;
        }
        self.board.write32(address.wrapping_add(0x10), u32::MAX);
        self.board.write32(address.wrapping_add(0x14), 0);
        self.board.write32(address.wrapping_add(0x18), 0);
        true
    }

    fn enter_critical_section(&mut self, address: u32) -> bool {
        if ram_range(address, 0x1c).is_none() {
            return false;
        }
        let thread = self.current_thread_address();
        let lock_address = address.wrapping_add(0x10);
        let recursion_address = address.wrapping_add(0x14);
        let owner_address = address.wrapping_add(0x18);
        let lock_count = self.board.read32(lock_address).wrapping_add(1);
        self.board.write32(lock_address, lock_count);
        if lock_count == 0 {
            self.board.write32(owner_address, thread);
            self.board.write32(recursion_address, 1);
            return true;
        }
        let owner = self.board.read32(owner_address);
        if owner == thread {
            let recursion = self.board.read32(recursion_address).wrapping_add(1);
            self.board.write32(recursion_address, recursion);
            return true;
        }
        if owner == 0 {
            self.board.write32(owner_address, thread);
            self.board.write32(recursion_address, 1);
            return true;
        }
        false
    }

    fn leave_critical_section(&mut self, address: u32) -> bool {
        if ram_range(address, 0x1c).is_none() {
            return false;
        }
        let recursion_address = address.wrapping_add(0x14);
        let recursion = self.board.read32(recursion_address);
        if recursion == 0 {
            return false;
        }
        let next_recursion = recursion - 1;
        self.board.write32(recursion_address, next_recursion);
        let lock_address = address.wrapping_add(0x10);
        let lock_count = self.board.read32(lock_address).wrapping_sub(1);
        self.board.write32(lock_address, lock_count);
        if next_recursion == 0 {
            self.board.write32(address.wrapping_add(0x18), 0);
            if lock_count as i32 >= 0 {
                self.board.write32(address.wrapping_add(4), 1);
            }
        }
        true
    }

    fn try_enter_critical_section(&mut self, address: u32) -> Option<bool> {
        ram_range(address, 0x1c)?;
        let thread = self.current_thread_address();
        let lock_address = address.wrapping_add(0x10);
        let recursion_address = address.wrapping_add(0x14);
        let owner_address = address.wrapping_add(0x18);
        let lock_count = self.board.read32(lock_address);
        if lock_count == u32::MAX {
            self.board.write32(lock_address, 0);
            self.board.write32(owner_address, thread);
            self.board.write32(recursion_address, 1);
            return Some(true);
        }
        if self.board.read32(owner_address) == thread {
            self.board.write32(lock_address, lock_count.wrapping_add(1));
            let recursion = self.board.read32(recursion_address).wrapping_add(1);
            self.board.write32(recursion_address, recursion);
            return Some(true);
        }
        Some(false)
    }

    fn kernel_argument(&mut self, index: u32) -> u32 {
        self.board
            .read32(self.cpu.regs[ESP].wrapping_add(4 + index * 4))
    }

    fn return_from_kernel(&mut self, stack_bytes: u32) {
        let return_address = self.board.read32(self.cpu.regs[ESP]);
        self.cpu.regs[ESP] = self.cpu.regs[ESP].wrapping_add(4 + stack_bytes);
        self.cpu.eip = return_address;
    }

    fn raise_irql(&mut self, new_irql: u8) -> Option<u8> {
        if new_irql > 31 || new_irql < self.current_irql {
            return None;
        }
        let old_irql = self.current_irql;
        self.current_irql = new_irql;
        Some(old_irql)
    }

    fn event_signal_state(&mut self, address: u32) -> Option<u32> {
        ram_range(address, 16)?;
        Some(self.board.read32(address.wrapping_add(4)))
    }

    fn set_event_signal_state(&mut self, address: u32, state: u32) -> Option<u32> {
        let old = self.event_signal_state(address)?;
        self.board.write32(address.wrapping_add(4), state);
        Some(old)
    }

    fn allocate_kernel_memory(&mut self, size: u32, alignment: u32) -> u32 {
        let requested = size.max(1);
        let Some(address) = self.board.kernel_heap.allocate(requested, alignment) else {
            return 0;
        };
        let Some(range) = ram_range(address, requested) else {
            let _ = self.board.kernel_heap.free(address);
            return 0;
        };
        self.board.ram[range].fill(0);
        address
    }

    fn initialize_irp(&mut self, irp: u32, packet_size: u16, stack_size: u8) -> bool {
        let required =
            XBOX_IRP_SIZE.saturating_add(u32::from(stack_size) * XBOX_IO_STACK_LOCATION_SIZE);
        if u32::from(packet_size) < required {
            return false;
        }
        let Some(range) = ram_range(irp, u32::from(packet_size)) else {
            return false;
        };
        self.board.ram[range].fill(0);
        self.board.write16(irp, XBOX_IO_TYPE_IRP);
        self.board.write16(irp.wrapping_add(2), packet_size);
        self.board.write32(irp.wrapping_add(8), irp.wrapping_add(8));
        self.board
            .write32(irp.wrapping_add(12), irp.wrapping_add(8));
        self.board.write8(irp.wrapping_add(0x18), stack_size);
        self.board
            .write8(irp.wrapping_add(0x19), stack_size.wrapping_add(1));
        let current_stack = irp.wrapping_add(required);
        self.board.write32(irp.wrapping_add(0x58), current_stack);
        true
    }

    fn set_persistent_pages(&mut self, base: u32, length: u32, persist: bool) -> bool {
        if length == 0 {
            return true;
        }
        let Some(range) = ram_range(base, length) else {
            return false;
        };
        let first_page = (range.start / 0x1000) as u32;
        let last_page = ((range.end - 1) / 0x1000) as u32;
        for page in first_page..=last_page {
            if persist {
                self.mm_persistent_pages.insert(page);
            } else {
                self.mm_persistent_pages.remove(&page);
            }
        }
        true
    }

    fn set_page_protection(&mut self, base: u32, length: u32, protection: u32) -> bool {
        if length == 0 || !guest_span_valid(base, length) {
            return false;
        }
        let Some(last) = base.checked_add(length - 1) else {
            return false;
        };
        let mut page = base & !0xfff;
        let last_page = last & !0xfff;
        loop {
            self.mm_page_protection.insert(page, protection);
            if page == last_page {
                break;
            }
            let Some(next) = page.checked_add(0x1000) else {
                return false;
            };
            page = next;
        }
        true
    }

    fn page_protection(&self, address: u32) -> u32 {
        if !guest_address_valid(address) {
            return 0;
        }
        self.mm_page_protection
            .get(&(address & !0xfff))
            .copied()
            .unwrap_or(XBOX_PAGE_READWRITE)
    }

    fn quick_reboot(&mut self) {
        let pages = self.mm_persistent_pages.clone();
        let snapshots: Vec<(u32, Vec<u8>)> = pages
            .iter()
            .filter_map(|&page| {
                let start = usize::try_from(page).ok()?.checked_mul(0x1000)?;
                let end = start.checked_add(0x1000)?;
                (end <= self.board.ram.len()).then(|| (page, self.board.ram[start..end].to_vec()))
            })
            .collect();

        <Self as Machine>::reset(self);
        self.mm_persistent_pages = pages;
        for (page, bytes) in snapshots {
            let start = page as usize * 0x1000;
            self.board.ram[start..start + bytes.len()].copy_from_slice(&bytes);
        }
    }

    fn append_kernel_string(
        &mut self,
        destination: u32,
        source_buffer: u32,
        source_length: u16,
        unicode_terminate: bool,
    ) -> Option<u32> {
        let (destination_length, maximum_length, destination_buffer) =
            guest_string_descriptor(&self.board.ram, destination)?;
        if source_length == 0 {
            return Some(STATUS_SUCCESS);
        }
        let Some(new_length) = destination_length.checked_add(source_length) else {
            return Some(STATUS_BUFFER_TOO_SMALL);
        };
        if new_length > maximum_length {
            return Some(STATUS_BUFFER_TOO_SMALL);
        }
        let source_range = ram_range(source_buffer, u32::from(source_length))?;
        let write_address = destination_buffer.checked_add(u32::from(destination_length))?;
        let destination_range = ram_range(write_address, u32::from(source_length))?;
        let bytes = self.board.ram[source_range].to_vec();
        self.board.ram[destination_range].copy_from_slice(&bytes);
        let length_range = ram_range(destination, 2)?;
        self.board.ram[length_range].copy_from_slice(&new_length.to_le_bytes());

        if unicode_terminate && new_length < maximum_length {
            let terminator = destination_buffer.checked_add(u32::from(new_length))?;
            let terminator_range = ram_range(terminator, 2)?;
            self.board.ram[terminator_range].copy_from_slice(&0u16.to_le_bytes());
        }
        Some(STATUS_SUCCESS)
    }

    fn av_capabilities(&mut self) -> u32 {
        let av_pack = match self.board.smbus.smc_read(0x04) {
            0 => 0x0000_0003,
            1 => 0x0000_0004,
            2 => 0x0000_0005,
            3 => 0x0000_0002,
            4 => 0x0000_0006,
            6 => 0x0000_0001,
            _ => 0,
        };
        let av_region = u32::from_le_bytes(self.board.smbus.eeprom[0x58..0x5c].try_into().unwrap());
        let user_settings =
            u32::from_le_bytes(self.board.smbus.eeprom[0x94..0x98].try_into().unwrap());
        av_pack | (av_region & 0x00c0_0f00) | (user_settings & !(0x0000_0f00 | 0x0000_00ff))
    }

    fn dispatch_kernel_hle(&mut self, ordinal: u32) -> bool {
        match ordinal {
            1 => {
                self.cpu.regs[EAX] = self.av_saved_data_address;
                self.return_from_kernel(0);
            }
            2 => {
                let _register_base = self.kernel_argument(0);
                let option = self.kernel_argument(1);
                let _param = self.kernel_argument(2);
                let result = self.kernel_argument(3);
                if option == 6 {
                    if ram_range(result, 4).is_none() {
                        return false;
                    }
                    let capabilities = self.av_capabilities();
                    self.board.write32(result, capabilities);
                }
                self.return_from_kernel(16);
            }
            3 => {
                let _register_base = self.kernel_argument(0);
                let _step = self.kernel_argument(1);
                let _mode = self.kernel_argument(2);
                let format = self.kernel_argument(3);
                let pitch = self.kernel_argument(4);
                let framebuffer = self.kernel_argument(5);
                self.board.display_format = format;
                self.board.display_pitch = pitch;
                self.board.nv2a[PCRTC_START..PCRTC_START + 4]
                    .copy_from_slice(&framebuffer.to_le_bytes());
                self.board.present_video();
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(24);
            }
            4 => {
                self.av_saved_data_address = self.kernel_argument(0);
                self.return_from_kernel(4);
            }
            9 => {
                let state = self.kernel_argument(0);
                let count = self.kernel_argument(1);
                if ram_range(state, 4).is_none() || count != 0 && ram_range(count, 4).is_none() {
                    self.cpu.regs[EAX] = STATUS_ACCESS_VIOLATION;
                    self.return_from_kernel(8);
                    return true;
                }
                let tray_state = u32::from(self.board.smbus.smc_read(0x03));
                self.board.write32(state, tray_state);
                if count != 0 {
                    self.board.write32(count, 0);
                }
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(8);
            }
            12 => {
                let lock = self.kernel_argument(0);
                if !self.acquire_read_write_lock_exclusive(lock) {
                    return false;
                }
                self.return_from_kernel(4);
            }
            13 => {
                let lock = self.kernel_argument(0);
                if !self.acquire_read_write_lock_shared(lock) {
                    return false;
                }
                self.return_from_kernel(4);
            }
            14 => {
                let size = self.kernel_argument(0);
                self.cpu.regs[EAX] = self.allocate_kernel_memory(size, 16);
                self.return_from_kernel(4);
            }
            15 => {
                let size = self.kernel_argument(0);
                self.cpu.regs[EAX] = self.allocate_kernel_memory(size, 16);
                self.return_from_kernel(8);
            }
            17 => {
                let address = self.kernel_argument(0);
                let _ = self.board.kernel_heap.free(address);
                self.return_from_kernel(4);
            }
            18 => {
                let lock = self.kernel_argument(0);
                if !self.initialize_read_write_lock(lock) {
                    return false;
                }
                self.return_from_kernel(4);
            }
            19 => {
                let addend = self.kernel_argument(0);
                let increment =
                    u64::from(self.kernel_argument(1)) | (u64::from(self.kernel_argument(2)) << 32);
                let Some(range) = ram_range(addend, 8) else {
                    return false;
                };
                let old = u64::from_le_bytes(self.board.ram[range.clone()].try_into().unwrap());
                let result = old.wrapping_add(increment);
                self.board.ram[range].copy_from_slice(&result.to_le_bytes());
                self.cpu.regs[EAX] = old as u32;
                self.cpu.regs[EDX] = (old >> 32) as u32;
                self.return_from_kernel(16);
            }
            20 => {
                let addend = self.cpu.regs[ECX];
                let increment = u64::from(self.cpu.regs[EDX]);
                let Some(range) = ram_range(addend, 8) else {
                    return false;
                };
                let old = u64::from_le_bytes(self.board.ram[range.clone()].try_into().unwrap());
                self.board.ram[range].copy_from_slice(&old.wrapping_add(increment).to_le_bytes());
                self.return_from_kernel(0);
            }
            21 => {
                let destination = self.cpu.regs[ECX];
                let exchange = self.cpu.regs[EDX];
                let comparand = self.kernel_argument(0);
                let Some(destination_range) = ram_range(destination, 8) else {
                    return false;
                };
                let Some(exchange_range) = ram_range(exchange, 8) else {
                    return false;
                };
                let Some(comparand_range) = ram_range(comparand, 8) else {
                    return false;
                };
                let old = u64::from_le_bytes(
                    self.board.ram[destination_range.clone()]
                        .try_into()
                        .unwrap(),
                );
                let expected =
                    u64::from_le_bytes(self.board.ram[comparand_range].try_into().unwrap());
                if old == expected {
                    let replacement = self.board.ram[exchange_range].to_vec();
                    self.board.ram[destination_range].copy_from_slice(&replacement);
                }
                self.cpu.regs[EAX] = old as u32;
                self.cpu.regs[EDX] = (old >> 32) as u32;
                self.return_from_kernel(4);
            }
            23 => {
                let address = self.kernel_argument(0);
                self.cpu.regs[EAX] = self.board.kernel_heap.allocation_size(address).unwrap_or(0);
                self.return_from_kernel(4);
            }
            24 => {
                let value_index = self.kernel_argument(0);
                let value_type = self.kernel_argument(1);
                let value = self.kernel_argument(2);
                let value_length = self.kernel_argument(3);
                let result_length = self.kernel_argument(4);
                let Some((offset, length, setting_type)) = eeprom_setting(value_index) else {
                    self.cpu.regs[EAX] = STATUS_OBJECT_NAME_NOT_FOUND;
                    self.return_from_kernel(20);
                    return true;
                };
                if result_length != 0 {
                    if ram_range(result_length, 4).is_none() {
                        self.cpu.regs[EAX] = STATUS_ACCESS_VIOLATION;
                        self.return_from_kernel(20);
                        return true;
                    }
                    self.board.write32(result_length, length as u32);
                }
                if value_length < length as u32 {
                    self.cpu.regs[EAX] = STATUS_BUFFER_TOO_SMALL;
                    self.return_from_kernel(20);
                    return true;
                }
                if ram_range(value_type, 4).is_none() || ram_range(value, value_length).is_none() {
                    self.cpu.regs[EAX] = STATUS_ACCESS_VIOLATION;
                    self.return_from_kernel(20);
                    return true;
                }
                self.board.write32(value_type, setting_type);
                let value_range = ram_range(value, value_length).unwrap();
                self.board.ram[value_range.clone()].fill(0);
                self.board.ram[value_range.start..value_range.start + length]
                    .copy_from_slice(&self.board.smbus.eeprom[offset..offset + length]);
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(20);
            }
            28 => {
                let lock = self.kernel_argument(0);
                if !self.release_read_write_lock(lock) {
                    return false;
                }
                self.return_from_kernel(4);
            }
            29 => {
                let value_index = self.kernel_argument(0);
                let _value_type = self.kernel_argument(1);
                let value = self.kernel_argument(2);
                let value_length = self.kernel_argument(3);
                if value_index == 0xfffe || (0x0100..=0x01ff).contains(&value_index) {
                    self.cpu.regs[EAX] = STATUS_OBJECT_NAME_NOT_FOUND;
                    self.return_from_kernel(16);
                    return true;
                }
                let Some((offset, length, _)) = eeprom_setting(value_index) else {
                    self.cpu.regs[EAX] = STATUS_OBJECT_NAME_NOT_FOUND;
                    self.return_from_kernel(16);
                    return true;
                };
                if value_length > length as u32 {
                    self.cpu.regs[EAX] = STATUS_INVALID_PARAMETER;
                    self.return_from_kernel(16);
                    return true;
                }
                let source = if value_length == 0 {
                    Vec::new()
                } else {
                    let Some(range) = ram_range(value, value_length) else {
                        self.cpu.regs[EAX] = STATUS_ACCESS_VIOLATION;
                        self.return_from_kernel(16);
                        return true;
                    };
                    self.board.ram[range].to_vec()
                };
                self.board.smbus.eeprom[offset..offset + length].fill(0);
                self.board.smbus.eeprom[offset..offset + source.len()].copy_from_slice(&source);
                refresh_eeprom_checksums(&mut self.board.smbus.eeprom);
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(16);
            }
            32 => {
                let list_head = self.cpu.regs[ECX];
                let list_entry = self.cpu.regs[EDX];
                if ram_range(list_head, 8).is_none() || ram_range(list_entry, 8).is_none() {
                    return false;
                }
                let first = self.board.read32(list_head);
                self.board.write32(list_entry, first);
                self.board.write32(list_entry.wrapping_add(4), list_head);
                self.board.write32(first.wrapping_add(4), list_entry);
                self.board.write32(list_head, list_entry);
                self.cpu.regs[EAX] = if first == list_head { 0 } else { first };
                self.return_from_kernel(0);
            }
            33 => {
                let list_head = self.cpu.regs[ECX];
                let list_entry = self.cpu.regs[EDX];
                if ram_range(list_head, 8).is_none() || ram_range(list_entry, 8).is_none() {
                    return false;
                }
                let last = self.board.read32(list_head.wrapping_add(4));
                self.board.write32(list_entry, list_head);
                self.board.write32(list_entry.wrapping_add(4), last);
                self.board.write32(last, list_entry);
                self.board.write32(list_head.wrapping_add(4), list_entry);
                self.cpu.regs[EAX] = if last == list_head { 0 } else { last };
                self.return_from_kernel(0);
            }
            34 => {
                let list_head = self.cpu.regs[ECX];
                if ram_range(list_head, 8).is_none() {
                    return false;
                }
                let first = self.board.read32(list_head);
                if first == list_head {
                    self.cpu.regs[EAX] = 0;
                } else {
                    if ram_range(first, 8).is_none() {
                        return false;
                    }
                    let next = self.board.read32(first);
                    self.board.write32(list_head, next);
                    self.board.write32(next.wrapping_add(4), list_head);
                    self.cpu.regs[EAX] = first;
                }
                self.return_from_kernel(0);
            }
            35 => {
                self.cpu.regs[EAX] = self.fsc_cache_pages;
                self.return_from_kernel(0);
            }
            36 => self.return_from_kernel(0),
            37 => {
                let pages = self.kernel_argument(0);
                self.cpu.regs[EAX] = if pages > 2048 {
                    STATUS_INVALID_PARAMETER
                } else {
                    self.fsc_cache_pages = pages;
                    STATUS_SUCCESS
                };
                self.return_from_kernel(4);
            }
            38 => {
                let request = self.cpu.regs[ECX] as u8;
                if request < 32 {
                    self.hal_software_interrupts &= !(1u32 << request);
                }
                self.return_from_kernel(0);
            }
            39 => {
                let level = self.kernel_argument(0);
                if level <= 27 {
                    self.hal_system_interrupts_enabled &= !(1u32 << level);
                }
                self.return_from_kernel(4);
            }
            43 => {
                let level = self.kernel_argument(0);
                let mode = self.kernel_argument(1);
                if level <= 27 {
                    let mask = 1u32 << level;
                    self.hal_system_interrupts_enabled |= mask;
                    if mode != 0 {
                        self.hal_latched_interrupts |= mask;
                    } else {
                        self.hal_latched_interrupts &= !mask;
                    }
                }
                self.return_from_kernel(8);
            }
            44 => {
                let level = self.kernel_argument(0);
                let irql_output = self.kernel_argument(1);
                self.cpu.regs[EAX] = if level <= 27 {
                    if irql_output != 0 {
                        let Some(index) = ram_index(irql_output) else {
                            return false;
                        };
                        self.board.ram[index] = (27 - level) as u8;
                    }
                    level + 0x30
                } else {
                    0
                };
                self.return_from_kernel(8);
            }
            45 => {
                let address = (self.kernel_argument(0) as u8 >> 1) & 0x7f;
                let command = self.kernel_argument(1) as u8;
                let read_word = self.kernel_argument(2) != 0;
                let output = self.kernel_argument(3);
                let value = if read_word {
                    self.board
                        .smbus
                        .read_byte_data(address, command)
                        .zip(
                            self.board
                                .smbus
                                .read_byte_data(address, command.wrapping_add(1)),
                        )
                        .map(|(low, high)| u32::from(low) | (u32::from(high) << 8))
                } else {
                    self.board
                        .smbus
                        .read_byte_data(address, command)
                        .map(u32::from)
                };
                self.cpu.regs[EAX] = if let Some(value) = value {
                    self.board.write32(output, value);
                    0
                } else {
                    0xc000_0001
                };
                self.return_from_kernel(16);
            }
            46 => {
                let bus = self.kernel_argument(0) as u8;
                let slot = self.kernel_argument(1);
                let register = self.kernel_argument(2) as usize;
                let buffer = self.kernel_argument(3);
                let length = self.kernel_argument(4);
                let write = self.kernel_argument(5) != 0;
                let Some(range) = ram_range(buffer, length) else {
                    return false;
                };
                let device = (slot & 0x1f) as u8;
                let function = ((slot >> 5) & 7) as u8;
                if write {
                    let bytes = self.board.ram[range].to_vec();
                    for (offset, value) in bytes.into_iter().enumerate() {
                        self.board.pci.write_config_byte(
                            bus,
                            device,
                            function,
                            register + offset,
                            value,
                        );
                    }
                } else {
                    for offset in 0..range.len() {
                        let value = self.board.pci.read_config_byte(
                            bus,
                            device,
                            function,
                            register + offset,
                        );
                        self.board.ram[range.start + offset] = value;
                    }
                }
                self.return_from_kernel(24);
            }
            48 => {
                let request = self.cpu.regs[ECX] as u8;
                if request < 32 {
                    self.hal_software_interrupts |= 1u32 << request;
                }
                self.return_from_kernel(0);
            }
            49 => {
                let routine = self.kernel_argument(0);
                self.board.smbus.power_action = match routine {
                    0 => 0x80,
                    1 | 3 => 0x01,
                    2 => 0x02,
                    4 => {
                        self.board.smbus.smc_scratch |= 0x02;
                        0x01
                    }
                    _ => 0x80,
                };
                self.powered = false;
            }
            50 => {
                let address = (self.kernel_argument(0) as u8 >> 1) & 0x7f;
                let command = self.kernel_argument(1) as u8;
                let write_word = self.kernel_argument(2) != 0;
                let value = self.kernel_argument(3);
                let success = self
                    .board
                    .smbus
                    .write_byte_data(address, command, value as u8)
                    && (!write_word
                        || self.board.smbus.write_byte_data(
                            address,
                            command.wrapping_add(1),
                            (value >> 8) as u8,
                        ));
                self.cpu.regs[EAX] = if success { 0 } else { 0xc000_0001 };
                self.return_from_kernel(16);
            }
            51 => {
                let destination = self.cpu.regs[ECX];
                let exchange = self.cpu.regs[EDX];
                let comparand = self.kernel_argument(0);
                let Some(range) = ram_range(destination, 4) else {
                    return false;
                };
                let old = u32::from_le_bytes(self.board.ram[range.clone()].try_into().unwrap());
                if old == comparand {
                    self.board.ram[range].copy_from_slice(&exchange.to_le_bytes());
                }
                self.cpu.regs[EAX] = old;
                self.return_from_kernel(4);
            }
            52 => {
                let destination = self.cpu.regs[ECX];
                let Some(range) = ram_range(destination, 4) else {
                    return false;
                };
                let old = u32::from_le_bytes(self.board.ram[range.clone()].try_into().unwrap());
                let result = old.wrapping_sub(1);
                self.board.ram[range].copy_from_slice(&result.to_le_bytes());
                self.cpu.regs[EAX] = result;
                self.return_from_kernel(0);
            }
            53 => {
                let destination = self.cpu.regs[ECX];
                let Some(range) = ram_range(destination, 4) else {
                    return false;
                };
                let old = u32::from_le_bytes(self.board.ram[range.clone()].try_into().unwrap());
                let result = old.wrapping_add(1);
                self.board.ram[range].copy_from_slice(&result.to_le_bytes());
                self.cpu.regs[EAX] = result;
                self.return_from_kernel(0);
            }
            54 => {
                let destination = self.cpu.regs[ECX];
                let value = self.cpu.regs[EDX];
                let Some(range) = ram_range(destination, 4) else {
                    return false;
                };
                let old = u32::from_le_bytes(self.board.ram[range.clone()].try_into().unwrap());
                self.board.ram[range].copy_from_slice(&value.to_le_bytes());
                self.cpu.regs[EAX] = old;
                self.return_from_kernel(0);
            }
            55 => {
                let destination = self.cpu.regs[ECX];
                let value = self.cpu.regs[EDX];
                let Some(range) = ram_range(destination, 4) else {
                    return false;
                };
                let old = u32::from_le_bytes(self.board.ram[range.clone()].try_into().unwrap());
                let result = old.wrapping_add(value);
                self.board.ram[range].copy_from_slice(&result.to_le_bytes());
                self.cpu.regs[EAX] = old;
                self.return_from_kernel(0);
            }
            56 => {
                let list_head = self.cpu.regs[ECX];
                if ram_range(list_head, 8).is_none() {
                    return false;
                }
                let first = self.board.read32(list_head);
                if first != 0 {
                    let meta = self.board.read32(list_head.wrapping_add(4));
                    self.board.write32(list_head, 0);
                    self.board
                        .write32(list_head.wrapping_add(4), meta & 0xffff_0000);
                }
                self.cpu.regs[EAX] = first;
                self.return_from_kernel(0);
            }
            57 => {
                let list_head = self.cpu.regs[ECX];
                if ram_range(list_head, 8).is_none() {
                    return false;
                }
                let first = self.board.read32(list_head);
                if first != 0 {
                    if ram_range(first, 4).is_none() {
                        return false;
                    }
                    let next = self.board.read32(first);
                    let meta = self.board.read32(list_head.wrapping_add(4));
                    let depth = (meta as u16).wrapping_sub(1);
                    self.board.write32(list_head, next);
                    self.board.write32(
                        list_head.wrapping_add(4),
                        (meta & 0xffff_0000) | u32::from(depth),
                    );
                }
                self.cpu.regs[EAX] = first;
                self.return_from_kernel(0);
            }
            58 => {
                let list_head = self.cpu.regs[ECX];
                let list_entry = self.cpu.regs[EDX];
                if ram_range(list_head, 8).is_none() || ram_range(list_entry, 4).is_none() {
                    return false;
                }
                let first = self.board.read32(list_head);
                let meta = self.board.read32(list_head.wrapping_add(4));
                let depth = (meta as u16).wrapping_add(1);
                let sequence = ((meta >> 16) as u16).wrapping_add(1);
                self.board.write32(list_entry, first);
                self.board.write32(list_head, list_entry);
                self.board.write32(
                    list_head.wrapping_add(4),
                    u32::from(depth) | (u32::from(sequence) << 16),
                );
                self.cpu.regs[EAX] = first;
                self.return_from_kernel(0);
            }
            59 => {
                let raw_stack_size = self.kernel_argument(0) as u8;
                let stack_size = raw_stack_size as i8;
                if stack_size < 0 {
                    self.cpu.regs[EAX] = 0;
                    self.return_from_kernel(4);
                    return true;
                }
                let stack_size = stack_size as u8;
                let packet_size =
                    XBOX_IRP_SIZE.checked_add(u32::from(stack_size) * XBOX_IO_STACK_LOCATION_SIZE);
                let Some(packet_size) = packet_size.filter(|size| *size <= u32::from(u16::MAX))
                else {
                    self.cpu.regs[EAX] = 0;
                    self.return_from_kernel(4);
                    return true;
                };
                let irp = self.allocate_kernel_memory(packet_size, 16);
                if irp != 0 && !self.initialize_irp(irp, packet_size as u16, stack_size) {
                    let _ = self.board.kernel_heap.free(irp);
                    self.cpu.regs[EAX] = 0;
                } else {
                    self.cpu.regs[EAX] = irp;
                }
                self.return_from_kernel(4);
            }
            63 => {
                let desired_access = self.kernel_argument(0);
                let desired_share = self.kernel_argument(1);
                let file_object = self.kernel_argument(2);
                let share_access = self.kernel_argument(3);
                let update = self.kernel_argument(4) != 0;
                let Some(file_range) = ram_range(file_object, 3) else {
                    return false;
                };
                let Some(share_range) = ram_range(share_access, 7) else {
                    return false;
                };
                let file_flags_index = file_range.start + 2;
                let read_access = desired_access & (FILE_READ_DATA | FILE_EXECUTE) != 0;
                let write_access = desired_access & (FILE_WRITE_DATA | FILE_APPEND_DATA) != 0;
                let delete_access = desired_access & DELETE_ACCESS != 0;
                let mut file_flags = self.board.ram[file_flags_index] & !0x0e;
                file_flags |= u8::from(read_access) << 1;
                file_flags |= u8::from(write_access) << 2;
                file_flags |= u8::from(delete_access) << 3;
                self.board.ram[file_flags_index] = file_flags;

                if read_access || write_access || delete_access {
                    let shared_read = desired_share & FILE_SHARE_READ != 0;
                    let shared_write = desired_share & FILE_SHARE_WRITE != 0;
                    let shared_delete = desired_share & FILE_SHARE_DELETE != 0;
                    let share = &self.board.ram[share_range.clone()];
                    let violation = (read_access && share[4] < share[0])
                        || (write_access && share[5] < share[0])
                        || (delete_access && share[6] < share[0])
                        || (share[1] != 0 && !shared_read)
                        || (share[2] != 0 && !shared_write)
                        || (share[3] != 0 && !shared_delete);
                    if violation {
                        self.cpu.regs[EAX] = STATUS_SHARING_VIOLATION;
                        self.return_from_kernel(20);
                        return true;
                    }

                    let mut file_flags = self.board.ram[file_flags_index] & !0x70;
                    file_flags |= u8::from(shared_read) << 4;
                    file_flags |= u8::from(shared_write) << 5;
                    file_flags |= u8::from(shared_delete) << 6;
                    self.board.ram[file_flags_index] = file_flags;
                    if update {
                        let share = &mut self.board.ram[share_range];
                        share[0] = share[0].wrapping_add(1);
                        share[1] = share[1].wrapping_add(u8::from(read_access));
                        share[2] = share[2].wrapping_add(u8::from(write_access));
                        share[3] = share[3].wrapping_add(u8::from(delete_access));
                        share[4] = share[4].wrapping_add(u8::from(shared_read));
                        share[5] = share[5].wrapping_add(u8::from(shared_write));
                        share[6] = share[6].wrapping_add(u8::from(shared_delete));
                    }
                }
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(20);
            }
            72 => {
                let irp = self.kernel_argument(0);
                let _ = self.board.kernel_heap.free(irp);
                self.return_from_kernel(4);
            }
            73 => {
                let irp = self.kernel_argument(0);
                let packet_size = self.kernel_argument(1) as u16;
                let raw_stack_size = self.kernel_argument(2) as u8;
                let stack_size = raw_stack_size as i8;
                if stack_size < 0 || !self.initialize_irp(irp, packet_size, stack_size as u8) {
                    return false;
                }
                self.return_from_kernel(12);
            }
            74 => {
                let _device_object = self.kernel_argument(0);
                let irp = self.kernel_argument(1);
                if ram_range(irp.wrapping_add(0x10), 8).is_none() {
                    return false;
                }
                self.board
                    .write32(irp.wrapping_add(0x10), STATUS_INVALID_DEVICE_REQUEST);
                self.cpu.regs[EAX] = STATUS_INVALID_DEVICE_REQUEST;
                self.return_from_kernel(8);
            }
            78 => {
                let file_object = self.kernel_argument(0);
                let share_access = self.kernel_argument(1);
                let Some(file_range) = ram_range(file_object, 3) else {
                    return false;
                };
                let Some(share_range) = ram_range(share_access, 7) else {
                    return false;
                };
                let flags = self.board.ram[file_range.start + 2];
                let read_access = flags & (1 << 1) != 0;
                let write_access = flags & (1 << 2) != 0;
                let delete_access = flags & (1 << 3) != 0;
                if read_access || write_access || delete_access {
                    let shared_read = flags & (1 << 4) != 0;
                    let shared_write = flags & (1 << 5) != 0;
                    let shared_delete = flags & (1 << 6) != 0;
                    let share = &mut self.board.ram[share_range];
                    share[0] = share[0].wrapping_sub(1);
                    share[1] = share[1].wrapping_sub(u8::from(read_access));
                    share[2] = share[2].wrapping_sub(u8::from(write_access));
                    share[3] = share[3].wrapping_sub(u8::from(delete_access));
                    share[4] = share[4].wrapping_sub(u8::from(shared_read));
                    share[5] = share[5].wrapping_sub(u8::from(shared_write));
                    share[6] = share[6].wrapping_sub(u8::from(shared_delete));
                }
                self.return_from_kernel(8);
            }
            80 => {
                let desired_access = self.kernel_argument(0);
                let desired_share = self.kernel_argument(1);
                let file_object = self.kernel_argument(2);
                let share_access = self.kernel_argument(3);
                let Some(file_range) = ram_range(file_object, 3) else {
                    return false;
                };
                let Some(share_range) = ram_range(share_access, 7) else {
                    return false;
                };
                let file_flags_index = file_range.start + 2;
                let read_access = desired_access & (FILE_READ_DATA | FILE_EXECUTE) != 0;
                let write_access = desired_access & (FILE_WRITE_DATA | FILE_APPEND_DATA) != 0;
                let delete_access = desired_access & DELETE_ACCESS != 0;
                let mut flags = self.board.ram[file_flags_index] & !0x0e;
                flags |= u8::from(read_access) << 1;
                flags |= u8::from(write_access) << 2;
                flags |= u8::from(delete_access) << 3;

                let share = &mut self.board.ram[share_range];
                if !read_access && !write_access && !delete_access {
                    share.fill(0);
                } else {
                    let shared_read = desired_share & FILE_SHARE_READ != 0;
                    let shared_write = desired_share & FILE_SHARE_WRITE != 0;
                    let shared_delete = desired_share & FILE_SHARE_DELETE != 0;
                    flags &= !0x70;
                    flags |= u8::from(shared_read) << 4;
                    flags |= u8::from(shared_write) << 5;
                    flags |= u8::from(shared_delete) << 6;
                    share.copy_from_slice(&[
                        1,
                        u8::from(read_access),
                        u8::from(write_access),
                        u8::from(delete_access),
                        u8::from(shared_read),
                        u8::from(shared_write),
                        u8::from(shared_delete),
                    ]);
                }
                self.board.ram[file_flags_index] = flags;
                self.return_from_kernel(16);
            }
            95 | 96 => {
                self.board.smbus.smc_scratch |= 0x02;
                self.board.smbus.power_action = 0x01;
                self.powered = false;
            }
            97 => {
                let timer = self.kernel_argument(0);
                if ram_range(timer, 0x28).is_none() {
                    return false;
                }
                let inserted = self.board.read8(timer.wrapping_add(3)) != 0;
                if inserted && !self.remove_timer(timer) {
                    return false;
                }
                self.cpu.regs[EAX] = u32::from(inserted);
                self.return_from_kernel(4);
            }
            98 => {
                let interrupt = self.kernel_argument(0);
                if ram_range(interrupt, 0x70).is_none() {
                    return false;
                }
                let level = self.board.read32(interrupt.wrapping_add(8));
                let mode = self.board.read8(interrupt.wrapping_add(0x12));
                if level > 27 || mode > 1 {
                    return false;
                }
                let index = level as usize;
                let connected = self.board.read8(interrupt.wrapping_add(0x10)) != 0;
                if connected || self.hal_interrupt_objects[index] != 0 {
                    self.cpu.regs[EAX] = 0;
                    self.return_from_kernel(4);
                    return true;
                }
                self.board.write8(interrupt.wrapping_add(0x10), 1);
                self.hal_interrupt_objects[index] = interrupt;
                let mask = 1u32 << level;
                self.hal_system_interrupts_enabled |= mask;
                if mode != 0 {
                    self.hal_latched_interrupts |= mask;
                } else {
                    self.hal_latched_interrupts &= !mask;
                }
                self.cpu.regs[EAX] = 1;
                self.return_from_kernel(4);
            }
            100 => {
                let interrupt = self.kernel_argument(0);
                if ram_range(interrupt, 0x70).is_none() {
                    return false;
                }
                let level = self.board.read32(interrupt.wrapping_add(8));
                if level > 27 {
                    return false;
                }
                let index = level as usize;
                if self.board.read8(interrupt.wrapping_add(0x10)) != 0 {
                    self.board.write8(interrupt.wrapping_add(0x10), 0);
                    if self.hal_interrupt_objects[index] == interrupt {
                        self.hal_interrupt_objects[index] = 0;
                    }
                    let mask = 1u32 << level;
                    self.hal_system_interrupts_enabled &= !mask;
                    self.hal_latched_interrupts &= !mask;
                }
                self.return_from_kernel(4);
            }
            101 => {
                if !self.enter_critical_region() {
                    return false;
                }
                self.return_from_kernel(0);
            }
            103 => {
                self.cpu.regs[EAX] = u32::from(self.current_irql);
                self.return_from_kernel(0);
            }
            104 => {
                self.cpu.regs[EAX] = self.current_thread_address();
                self.return_from_kernel(0);
            }
            105 => {
                let apc = self.kernel_argument(0);
                let thread = self.kernel_argument(1);
                let kernel_routine = self.kernel_argument(2);
                let rundown_routine = self.kernel_argument(3);
                let normal_routine = self.kernel_argument(4);
                let apc_mode = self.kernel_argument(5) as u8;
                let normal_context = self.kernel_argument(6);
                if !self.initialize_apc(
                    apc,
                    thread,
                    [kernel_routine, rundown_routine, normal_routine],
                    apc_mode,
                    normal_context,
                ) {
                    return false;
                }
                self.return_from_kernel(28);
            }
            106 => {
                let queue = self.kernel_argument(0);
                if !self.initialize_device_queue(queue) {
                    return false;
                }
                self.return_from_kernel(4);
            }
            107 => {
                let dpc = self.kernel_argument(0);
                let routine = self.kernel_argument(1);
                let context = self.kernel_argument(2);
                if !self.initialize_dpc(dpc, routine, context) {
                    return false;
                }
                self.return_from_kernel(12);
            }
            108 => {
                let event = self.kernel_argument(0);
                let event_type = self.kernel_argument(1) as u8;
                let signal_state = u32::from(self.kernel_argument(2) != 0);
                if !self.initialize_event(event, event_type, signal_state) {
                    return false;
                }
                self.return_from_kernel(12);
            }
            109 => {
                let interrupt = self.kernel_argument(0);
                let service_routine = self.kernel_argument(1);
                let service_context = self.kernel_argument(2);
                let vector = self.kernel_argument(3);
                let irql = self.kernel_argument(4);
                let mode = self.kernel_argument(5);
                let _share_vector = self.kernel_argument(6);
                if !self.initialize_interrupt(
                    interrupt,
                    service_routine,
                    service_context,
                    vector,
                    irql,
                    mode,
                ) {
                    return false;
                }
                self.return_from_kernel(28);
            }
            110 => {
                let mutant = self.kernel_argument(0);
                let initial_owner = self.kernel_argument(1) != 0;
                if !self.initialize_mutant(mutant, initial_owner) {
                    return false;
                }
                self.return_from_kernel(8);
            }
            111 => {
                let queue = self.kernel_argument(0);
                let count = self.kernel_argument(1);
                if !self.initialize_queue(queue, count) {
                    return false;
                }
                self.return_from_kernel(8);
            }
            112 => {
                let semaphore = self.kernel_argument(0);
                let count = self.kernel_argument(1) as i32;
                let limit = self.kernel_argument(2) as i32;
                if !self.initialize_semaphore(semaphore, count, limit) {
                    return false;
                }
                self.return_from_kernel(12);
            }
            113 => {
                let timer = self.kernel_argument(0);
                let timer_type = self.kernel_argument(1) as u8;
                if !self.initialize_timer(timer, timer_type) {
                    return false;
                }
                self.return_from_kernel(8);
            }
            114 | 115 => {
                let queue = self.kernel_argument(0);
                let entry = self.kernel_argument(1);
                let sort_key = (ordinal == 114).then(|| self.kernel_argument(2));
                if ram_range(queue, 0x10).is_none() || ram_range(entry, 0x10).is_none() {
                    return false;
                }
                if let Some(sort_key) = sort_key {
                    self.board.write32(entry.wrapping_add(8), sort_key);
                }
                let busy = self.board.ram[ram_index(queue.wrapping_add(4)).unwrap()] != 0;
                if !busy {
                    self.board.ram[ram_index(queue.wrapping_add(4)).unwrap()] = 1;
                    self.board.ram[ram_index(entry.wrapping_add(0x0c)).unwrap()] = 0;
                    self.cpu.regs[EAX] = 0;
                    self.return_from_kernel(if ordinal == 114 { 12 } else { 8 });
                    return true;
                }

                let head = queue.wrapping_add(8);
                let mut insert_before = head;
                if let Some(sort_key) = sort_key {
                    let mut current = self.board.read32(head);
                    let mut traversed = 0usize;
                    while current != head {
                        if traversed >= 65_536 || ram_range(current, 0x10).is_none() {
                            return false;
                        }
                        if sort_key < self.board.read32(current.wrapping_add(8)) {
                            insert_before = current;
                            break;
                        }
                        current = self.board.read32(current);
                        traversed += 1;
                    }
                }
                if insert_before == head && sort_key.is_none() {
                    insert_before = head;
                }
                let previous = self.board.read32(insert_before.wrapping_add(4));
                if ram_range(previous, 8).is_none() {
                    return false;
                }
                self.board.write32(entry, insert_before);
                self.board.write32(entry.wrapping_add(4), previous);
                self.board.write32(previous, entry);
                self.board.write32(insert_before.wrapping_add(4), entry);
                self.board.ram[ram_index(entry.wrapping_add(0x0c)).unwrap()] = 1;
                self.cpu.regs[EAX] = 1;
                self.return_from_kernel(if ordinal == 114 { 12 } else { 8 });
            }
            116 | 117 => {
                let queue = self.kernel_argument(0);
                let entry = self.kernel_argument(1);
                if ram_range(queue, 0x28).is_none()
                    || ram_range(entry, 8).is_none()
                    || !self.dispatcher_wait_list_empty(queue)
                {
                    return false;
                }
                let head = queue.wrapping_add(0x10);
                let first = self.board.read32(head);
                let last = self.board.read32(head.wrapping_add(4));
                if ram_range(first, 8).is_none() || ram_range(last, 8).is_none() {
                    return false;
                }
                let previous_state = self.board.read32(queue.wrapping_add(4));
                self.board
                    .write32(queue.wrapping_add(4), previous_state.wrapping_add(1));
                if ordinal == 116 {
                    self.board.write32(entry, first);
                    self.board.write32(entry.wrapping_add(4), head);
                    self.board.write32(first.wrapping_add(4), entry);
                    self.board.write32(head, entry);
                } else {
                    self.board.write32(entry, head);
                    self.board.write32(entry.wrapping_add(4), last);
                    self.board.write32(last, entry);
                    self.board.write32(head.wrapping_add(4), entry);
                }
                self.cpu.regs[EAX] = previous_state;
                self.return_from_kernel(8);
            }
            119 => {
                let dpc = self.kernel_argument(0);
                let system_argument1 = self.kernel_argument(1);
                let system_argument2 = self.kernel_argument(2);
                let Some(inserted) = self.enqueue_dpc(dpc, system_argument1, system_argument2)
                else {
                    return false;
                };
                self.cpu.regs[EAX] = u32::from(inserted);
                self.return_from_kernel(12);
            }
            121 => {
                self.cpu.regs[EAX] = 0;
                self.return_from_kernel(0);
            }
            122 => {
                if !self.leave_critical_region() {
                    return false;
                }
                self.return_from_kernel(0);
            }
            123 => {
                let event = self.kernel_argument(0);
                let wait = self.kernel_argument(2) != 0;
                if wait {
                    return false;
                }
                let Some(old_state) = self.set_event_signal_state(event, 0) else {
                    return false;
                };
                self.cpu.regs[EAX] = old_state;
                self.return_from_kernel(12);
            }
            124 => {
                let thread = self.kernel_argument(0);
                if ram_range(thread, 0x33).is_none() {
                    return false;
                }
                self.cpu.regs[EAX] =
                    i32::from(self.board.read8(thread.wrapping_add(0x32)) as i8) as u32;
                self.return_from_kernel(4);
            }
            125 => {
                let value = self.board.frame_counter.saturating_mul(10_000_000) / 60;
                self.cpu.regs[EAX] = value as u32;
                self.cpu.regs[EDX] = (value >> 32) as u32;
                self.return_from_kernel(0);
            }
            126 => {
                let value = self.board.frame_counter.saturating_mul(3_375_000) / 60;
                self.cpu.regs[EAX] = value as u32;
                self.cpu.regs[EDX] = (value >> 32) as u32;
                self.return_from_kernel(0);
            }
            127 => {
                self.cpu.regs[EAX] = 3_375_000;
                self.cpu.regs[EDX] = 0;
                self.return_from_kernel(0);
            }
            128 => {
                let output = self.kernel_argument(0);
                let system_time = kernel_data_offset(154);
                let low = u32::from_le_bytes(
                    self.board.kernel_data[system_time..system_time + 4]
                        .try_into()
                        .unwrap(),
                );
                let high = u32::from_le_bytes(
                    self.board.kernel_data[system_time + 4..system_time + 8]
                        .try_into()
                        .unwrap(),
                );
                if ram_range(output, 8).is_none() {
                    return false;
                }
                self.board.write32(output, low);
                self.board.write32(output.wrapping_add(4), high);
                self.return_from_kernel(4);
            }
            129 => {
                let Some(old_irql) = self.raise_irql(2) else {
                    return false;
                };
                self.cpu.regs[EAX] = u32::from(old_irql);
                self.return_from_kernel(0);
            }
            130 => {
                let Some(old_irql) = self.raise_irql(28) else {
                    return false;
                };
                self.cpu.regs[EAX] = u32::from(old_irql);
                self.return_from_kernel(0);
            }
            131 => {
                let mutant = self.kernel_argument(0);
                let _increment = self.kernel_argument(1);
                let abandoned = self.kernel_argument(2) != 0;
                let wait = self.kernel_argument(3) != 0;
                if ram_range(mutant, 0x20).is_none() {
                    return false;
                }
                let thread = self.current_thread_address();
                if self.board.read32(mutant.wrapping_add(0x18)) != thread {
                    return false;
                }
                let old_state = self.board.read32(mutant.wrapping_add(4));
                let new_state = old_state.wrapping_add(1);
                self.board.write32(mutant.wrapping_add(4), new_state);
                if new_state == 1 {
                    if !self.unlink_list_entry(mutant.wrapping_add(0x10)) {
                        return false;
                    }
                    self.board.write32(mutant.wrapping_add(0x18), 0);
                    self.board
                        .write8(mutant.wrapping_add(0x1c), u8::from(abandoned));
                }
                if wait {
                    self.board
                        .write8(thread.wrapping_add(0x54), self.current_irql);
                    self.board.write8(thread.wrapping_add(0x56), 1);
                }
                self.cpu.regs[EAX] = old_state;
                self.return_from_kernel(16);
            }
            132 => {
                let semaphore = self.kernel_argument(0);
                let adjustment = self.kernel_argument(2) as i32;
                let wait = self.kernel_argument(3) != 0;
                if wait
                    || adjustment <= 0
                    || ram_range(semaphore, 20).is_none()
                    || !self.dispatcher_wait_list_empty(semaphore)
                {
                    return false;
                }
                let initial = self.board.read32(semaphore.wrapping_add(4)) as i32;
                let limit = self.board.read32(semaphore.wrapping_add(16)) as i32;
                let Some(adjusted) = initial.checked_add(adjustment) else {
                    return false;
                };
                if adjusted > limit {
                    return false;
                }
                self.board
                    .write32(semaphore.wrapping_add(4), adjusted as u32);
                self.cpu.regs[EAX] = initial as u32;
                self.return_from_kernel(16);
            }
            133 => {
                let queue = self.kernel_argument(0);
                let sort_key = self.kernel_argument(1);
                if ram_range(queue, 0x10).is_none() {
                    return false;
                }
                let head = queue.wrapping_add(8);
                let first = self.board.read32(head);
                if first == head {
                    self.board.ram[ram_index(queue.wrapping_add(4)).unwrap()] = 0;
                    self.cpu.regs[EAX] = 0;
                    self.return_from_kernel(8);
                    return true;
                }

                let mut current = first;
                let mut candidate = 0;
                let mut traversed = 0usize;
                while current != head {
                    if traversed >= 65_536 || ram_range(current, 0x10).is_none() {
                        return false;
                    }
                    if sort_key <= self.board.read32(current.wrapping_add(8)) {
                        candidate = current;
                        break;
                    }
                    current = self.board.read32(current);
                    traversed += 1;
                }
                if candidate == 0 {
                    candidate = first;
                }
                if !self.unlink_list_entry(candidate) {
                    return false;
                }
                self.board.ram[ram_index(candidate.wrapping_add(0x0c)).unwrap()] = 0;
                self.cpu.regs[EAX] = candidate;
                self.return_from_kernel(8);
            }
            134 => {
                let queue = self.kernel_argument(0);
                if ram_range(queue, 0x10).is_none() {
                    return false;
                }
                let head = queue.wrapping_add(8);
                let first = self.board.read32(head);
                if first == head {
                    self.board.ram[ram_index(queue.wrapping_add(4)).unwrap()] = 0;
                    self.cpu.regs[EAX] = 0;
                } else {
                    if ram_range(first, 0x10).is_none() || !self.unlink_list_entry(first) {
                        return false;
                    }
                    self.board.ram[ram_index(first.wrapping_add(0x0c)).unwrap()] = 0;
                    self.cpu.regs[EAX] = first;
                }
                self.return_from_kernel(4);
            }
            135 => {
                let queue = self.kernel_argument(0);
                let entry = self.kernel_argument(1);
                if ram_range(queue, 0x10).is_none() || ram_range(entry, 0x10).is_none() {
                    return false;
                }
                let inserted = self.board.ram[ram_index(entry.wrapping_add(0x0c)).unwrap()] != 0;
                if inserted {
                    if !self.unlink_list_entry(entry) {
                        return false;
                    }
                    self.board.ram[ram_index(entry.wrapping_add(0x0c)).unwrap()] = 0;
                }
                self.cpu.regs[EAX] = u32::from(inserted);
                self.return_from_kernel(8);
            }
            137 => {
                let dpc = self.kernel_argument(0);
                if ram_range(dpc, 0x1c).is_none() {
                    return false;
                }
                let inserted = self.board.read8(dpc.wrapping_add(2)) != 0;
                if inserted {
                    let Some(index) = self.dpc_queue.iter().position(|&queued| queued == dpc)
                    else {
                        return false;
                    };
                    self.dpc_queue.remove(index);
                    self.board.write8(dpc.wrapping_add(2), 0);
                    if !self.rebuild_dpc_links() {
                        return false;
                    }
                    if self.dpc_queue.is_empty() {
                        self.hal_software_interrupts &= !(1u32 << 2);
                    }
                }
                self.cpu.regs[EAX] = u32::from(inserted);
                self.return_from_kernel(4);
            }
            138 => {
                let event = self.kernel_argument(0);
                let Some(old_state) = self.set_event_signal_state(event, 0) else {
                    return false;
                };
                self.cpu.regs[EAX] = old_state;
                self.return_from_kernel(4);
            }
            140 => {
                let thread = self.kernel_argument(0);
                if ram_range(thread, 0x76).is_none() {
                    return false;
                }
                let address = thread.wrapping_add(0x75);
                let old_count = self.board.read8(address);
                if old_count != 0 {
                    self.board.write8(address, old_count - 1);
                }
                self.cpu.regs[EAX] = u32::from(old_count);
                self.return_from_kernel(4);
            }
            141 => {
                let queue = self.kernel_argument(0);
                if ram_range(queue, 0x28).is_none() {
                    return false;
                }
                let entry_head = queue.wrapping_add(0x10);
                let first = self.board.read32(entry_head);
                self.cpu.regs[EAX] = if first == entry_head { 0 } else { first };
                if first != entry_head {
                    let last = self.board.read32(entry_head.wrapping_add(4));
                    if ram_range(first, 8).is_none() || ram_range(last, 8).is_none() {
                        return false;
                    }
                    self.board.write32(first.wrapping_add(4), last);
                    self.board.write32(last, first);
                }

                let thread_head = queue.wrapping_add(0x20);
                loop {
                    let entry = self.board.read32(thread_head);
                    if entry == thread_head {
                        break;
                    }
                    let Some(thread) = entry.checked_sub(0x7c) else {
                        return false;
                    };
                    if ram_range(thread, 0x84).is_none() || !self.unlink_list_entry(entry) {
                        return false;
                    }
                    self.board.write32(thread.wrapping_add(0x78), 0);
                }
                self.return_from_kernel(4);
            }
            143 => {
                let thread = self.kernel_argument(0);
                let priority = self.kernel_argument(1) as i32;
                if ram_range(thread, 0x33).is_none() || !(-128..=127).contains(&priority) {
                    return false;
                }
                self.board
                    .write8(thread.wrapping_add(0x32), priority as i8 as u8);
                self.cpu.regs[EAX] =
                    i32::from(self.board.read8(thread.wrapping_add(0x32)) as i8) as u32;
                self.return_from_kernel(8);
            }
            144 => {
                let thread = self.kernel_argument(0);
                let disable = self.kernel_argument(1) != 0;
                if ram_range(thread, 0x74).is_none() {
                    return false;
                }
                let address = thread.wrapping_add(0x73);
                let previous = self.board.read8(address) != 0;
                self.board.write8(address, u8::from(disable));
                self.cpu.regs[EAX] = u32::from(previous);
                self.return_from_kernel(8);
            }
            145 => {
                let event = self.kernel_argument(0);
                let wait = self.kernel_argument(2) != 0;
                if wait {
                    return false;
                }
                let Some(old_state) = self.set_event_signal_state(event, 1) else {
                    return false;
                };
                self.cpu.regs[EAX] = old_state;
                self.return_from_kernel(12);
            }
            146 => {
                let event = self.kernel_argument(0);
                let thread_output = self.kernel_argument(1);
                if ram_range(event, 0x10).is_none()
                    || (thread_output != 0 && ram_range(thread_output, 4).is_none())
                {
                    return false;
                }
                let wait_head = event.wrapping_add(8);
                let first = self.board.read32(wait_head);
                if first == wait_head {
                    self.board.write32(event.wrapping_add(4), 1);
                } else {
                    if ram_range(first, 0x18).is_none() {
                        return false;
                    }
                    let thread = self.board.read32(first.wrapping_add(8));
                    if ram_range(thread, 0x5c).is_none() || !self.unlink_list_entry(first) {
                        return false;
                    }
                    if thread_output != 0 {
                        self.board.write32(thread_output, thread);
                    }
                    self.board
                        .write32(thread.wrapping_add(0x50), STATUS_SUCCESS);
                    self.board.write32(thread.wrapping_add(0x58), 0);
                }
                self.return_from_kernel(8);
            }
            147 => {
                let process = self.kernel_argument(0);
                let priority = self.kernel_argument(1) as i32;
                if ram_range(process, 0x1b).is_none() || !(-128..=127).contains(&priority) {
                    return false;
                }
                self.board
                    .write8(process.wrapping_add(0x18), priority as i8 as u8);
                self.cpu.regs[EAX] = priority as u32;
                self.return_from_kernel(8);
            }
            148 => {
                let thread = self.kernel_argument(0);
                let mut priority = self.kernel_argument(1);
                if priority > 31 || ram_range(thread, 0x73).is_none() {
                    return false;
                }
                let priority_address = thread.wrapping_add(0x32);
                let base_priority = self.board.read8(thread.wrapping_add(0x70));
                let old_priority = self.board.read8(priority_address);
                self.board.write8(thread.wrapping_add(0x72), 0);
                if base_priority != 0 && priority == 0 {
                    priority = 1;
                }
                self.board.write8(priority_address, priority as u8);
                self.cpu.regs[EAX] = u32::from(old_priority);
                self.return_from_kernel(8);
            }
            149 => {
                let timer = self.kernel_argument(0);
                let due_time =
                    u64::from(self.kernel_argument(1)) | (u64::from(self.kernel_argument(2)) << 32);
                let dpc = self.kernel_argument(3);
                let Some(previously_inserted) = self.set_timer(timer, due_time as i64, 0, dpc)
                else {
                    return false;
                };
                self.cpu.regs[EAX] = u32::from(previously_inserted);
                self.return_from_kernel(16);
            }
            150 => {
                let timer = self.kernel_argument(0);
                let due_time =
                    u64::from(self.kernel_argument(1)) | (u64::from(self.kernel_argument(2)) << 32);
                let period = self.kernel_argument(3) as i32;
                let dpc = self.kernel_argument(4);
                let Some(previously_inserted) = self.set_timer(timer, due_time as i64, period, dpc)
                else {
                    return false;
                };
                self.cpu.regs[EAX] = u32::from(previously_inserted);
                self.return_from_kernel(20);
            }
            151 => {
                let microseconds = u64::from(self.kernel_argument(0));
                let stall_cycles = microseconds
                    .saturating_mul(CPU_STEPS_PER_FRAME)
                    .saturating_mul(60)
                    / 1_000_000;
                self.cpu.cycles = self.cpu.cycles.saturating_add(stall_cycles);
                self.return_from_kernel(4);
            }
            152 => {
                let thread = self.kernel_argument(0);
                if ram_range(thread, 0x76).is_none() {
                    return false;
                }
                let address = thread.wrapping_add(0x75);
                let old_count = self.board.read8(address);
                if old_count == 127 {
                    self.cpu.regs[EAX] = STATUS_SUSPEND_COUNT_EXCEEDED;
                } else if self.board.read8(thread.wrapping_add(0x4b)) != 0 {
                    self.board.write8(address, old_count + 1);
                    self.cpu.regs[EAX] = u32::from(old_count);
                } else {
                    self.cpu.regs[EAX] = u32::from(old_count);
                }
                self.return_from_kernel(4);
            }
            155 => {
                let mode = self.kernel_argument(0);
                if mode > 1 {
                    return false;
                }
                let address = self.current_thread_address().wrapping_add(0x2d + mode);
                let old = self.board.read8(address) != 0;
                if old {
                    self.board.write8(address, 0);
                }
                self.cpu.regs[EAX] = u32::from(old);
                self.return_from_kernel(4);
            }
            160 => {
                let Some(old_irql) = self.raise_irql(self.cpu.regs[ECX] as u8) else {
                    return false;
                };
                self.cpu.regs[EAX] = u32::from(old_irql);
                self.return_from_kernel(0);
            }
            161 => {
                let new_irql = self.cpu.regs[ECX] as u8;
                if new_irql > 31 {
                    return false;
                }
                self.current_irql = new_irql;
                self.return_from_kernel(0);
            }
            165 => {
                let size = self.kernel_argument(0);
                self.cpu.regs[EAX] = self.allocate_kernel_memory(size, 0x1000);
                self.return_from_kernel(4);
            }
            166 => {
                let size = self.kernel_argument(0);
                let lowest = self.kernel_argument(1);
                let highest = self.kernel_argument(2);
                let alignment = self.kernel_argument(3).max(0x1000);
                let address = self.allocate_kernel_memory(size, alignment);
                let physical = ram_index(address).map(|index| index as u32);
                let physical_end =
                    physical.and_then(|start| start.checked_add(size.saturating_sub(1)));
                self.cpu.regs[EAX] = match (physical, physical_end) {
                    (Some(start), Some(end)) if start >= lowest && end <= highest => address,
                    _ => {
                        if address != 0 {
                            let _ = self.board.kernel_heap.free(address);
                        }
                        0
                    }
                };
                self.return_from_kernel(20);
            }
            167 => {
                let size = self.kernel_argument(0);
                self.cpu.regs[EAX] = self.allocate_kernel_memory(size, 0x1000);
                self.return_from_kernel(8);
            }
            168 => {
                let requested = self.kernel_argument(0);
                let padding = self.kernel_argument(1);
                if requested != u32::MAX {
                    self.mm_gpu_instance_bytes =
                        requested.saturating_add(0xfff).min(XBOX_GPU_INSTANCE_BYTES) & !0xfff;
                }
                if padding != 0 {
                    if ram_range(padding, 4).is_none() {
                        return false;
                    }
                    self.board.write32(padding, XBOX_GPU_INSTANCE_BYTES);
                }
                self.cpu.regs[EAX] = XBOX_GPU_INSTANCE_RETURN;
                self.return_from_kernel(8);
            }
            169 => {
                let size = self.kernel_argument(0);
                let _debugger_thread = self.kernel_argument(1) != 0;
                let Some(actual_size) = size.checked_add(0x1000) else {
                    return false;
                };
                let allocation = self.allocate_kernel_memory(actual_size, 0x1000);
                self.cpu.regs[EAX] = if allocation == 0 {
                    0
                } else {
                    allocation.wrapping_add(actual_size)
                };
                self.return_from_kernel(8);
            }
            170 => {
                let stack_base = self.kernel_argument(0);
                let stack_limit = self.kernel_argument(1);
                let Some(stack_size) = stack_base.checked_sub(stack_limit) else {
                    return false;
                };
                let Some(actual_size) = stack_size.checked_add(0x1000) else {
                    return false;
                };
                let Some(allocation) = stack_base.checked_sub(actual_size) else {
                    return false;
                };
                if self.board.kernel_heap.allocation_size(allocation) != Some(actual_size) {
                    return false;
                }
                let _ = self.board.kernel_heap.free(allocation);
                self.return_from_kernel(8);
            }
            171 => {
                let address = self.kernel_argument(0);
                let _ = self.board.kernel_heap.free(address);
                self.return_from_kernel(4);
            }
            172 => {
                let address = self.kernel_argument(0);
                let requested = self.kernel_argument(1);
                let freed = self.board.kernel_heap.free(address).unwrap_or(0);
                let counted = if requested == 0 {
                    freed
                } else {
                    freed.min(requested)
                };
                self.cpu.regs[EAX] = counted.div_ceil(0x1000);
                self.return_from_kernel(8);
            }
            173 => {
                let address = self.kernel_argument(0);
                self.cpu.regs[EAX] = ram_index(address)
                    .map(|index| index as u32)
                    .or_else(|| guest_address_valid(address).then_some(address))
                    .unwrap_or(0);
                self.cpu.regs[EDX] = 0;
                self.return_from_kernel(4);
            }
            174 => {
                self.cpu.regs[EAX] = u32::from(guest_address_valid(self.kernel_argument(0)));
                self.return_from_kernel(4);
            }
            175 => {
                let base = self.kernel_argument(0);
                let length = self.kernel_argument(1);
                let _unlock = self.kernel_argument(2) != 0;
                if ram_range(base, length).is_none() {
                    return false;
                }
                self.return_from_kernel(12);
            }
            176 => {
                let physical = self.kernel_argument(0);
                let _unlock = self.kernel_argument(1) != 0;
                if physical >= RAM_SIZE as u32 {
                    return false;
                }
                self.return_from_kernel(8);
            }
            177 => {
                let physical = self.kernel_argument(0);
                let length = self.kernel_argument(1);
                let _protection = self.kernel_argument(2);
                self.cpu.regs[EAX] = if physical < RAM_SIZE as u32
                    && physical
                        .checked_add(length)
                        .is_some_and(|end| end <= RAM_SIZE as u32)
                    && length != 0
                {
                    XBOX_KERNEL_HEAP_ALIAS + physical
                } else if guest_span_valid(physical, length) {
                    physical
                } else {
                    0
                };
                self.return_from_kernel(12);
            }
            178 => {
                let base = self.kernel_argument(0);
                let length = self.kernel_argument(1);
                let persist = self.kernel_argument(2) != 0;
                if !self.set_persistent_pages(base, length, persist) {
                    return false;
                }
                self.return_from_kernel(12);
            }
            179 => {
                let address = self.kernel_argument(0);
                self.cpu.regs[EAX] = self.page_protection(address);
                self.return_from_kernel(4);
            }
            180 => {
                let address = self.kernel_argument(0);
                self.cpu.regs[EAX] = self.board.kernel_heap.allocation_size(address).unwrap_or(0);
                self.return_from_kernel(4);
            }
            181 => {
                let statistics = self.kernel_argument(0);
                if ram_range(statistics, 9 * 4).is_none() {
                    return false;
                }
                if self.board.read32(statistics) != 9 * 4 {
                    self.cpu.regs[EAX] = STATUS_INVALID_PARAMETER;
                    self.return_from_kernel(4);
                    return true;
                }
                for (index, value) in self.memory_statistics().into_iter().enumerate() {
                    self.board
                        .write32(statistics.wrapping_add((index as u32) * 4), value);
                }
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(4);
            }
            182 => {
                let base = self.kernel_argument(0);
                let length = self.kernel_argument(1);
                let protection = self.kernel_argument(2);
                if !self.set_page_protection(base, length, protection) {
                    return false;
                }
                self.return_from_kernel(12);
            }
            183 => {
                let base = self.kernel_argument(0);
                let length = self.kernel_argument(1);
                if !guest_span_valid(base, length) {
                    return false;
                }
                self.return_from_kernel(8);
            }
            238 => self.return_from_kernel(0),
            260 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let allocate = self.kernel_argument(2) != 0;
                let Some((source_length, _, source_buffer)) =
                    guest_string_descriptor(&self.board.ram, source)
                else {
                    return false;
                };
                let total = u32::from(source_length).saturating_add(1).saturating_mul(2);
                if total > u32::from(u16::MAX) {
                    self.cpu.regs[EAX] = STATUS_INVALID_PARAMETER_2;
                    self.return_from_kernel(12);
                    return true;
                }
                let Some((_, destination_maximum, existing_buffer)) =
                    guest_string_descriptor(&self.board.ram, destination)
                else {
                    return false;
                };
                let unicode_length = u16::try_from(total - 2).unwrap();
                let buffer = if allocate {
                    let buffer = self.allocate_kernel_memory(total, 16);
                    if buffer == 0 {
                        self.cpu.regs[EAX] = STATUS_NO_MEMORY;
                        self.return_from_kernel(12);
                        return true;
                    }
                    let Some(maximum_range) = ram_range(destination + 2, 2) else {
                        let _ = self.board.kernel_heap.free(buffer);
                        return false;
                    };
                    self.board.ram[maximum_range].copy_from_slice(&(total as u16).to_le_bytes());
                    let Some(buffer_range) = ram_range(destination + 4, 4) else {
                        let _ = self.board.kernel_heap.free(buffer);
                        return false;
                    };
                    self.board.ram[buffer_range].copy_from_slice(&buffer.to_le_bytes());
                    buffer
                } else {
                    if total > u32::from(destination_maximum) {
                        let Some(length_range) = ram_range(destination, 2) else {
                            return false;
                        };
                        self.board.ram[length_range].copy_from_slice(&unicode_length.to_le_bytes());
                        self.cpu.regs[EAX] = STATUS_BUFFER_OVERFLOW;
                        self.return_from_kernel(12);
                        return true;
                    }
                    existing_buffer
                };
                let Some(length_range) = ram_range(destination, 2) else {
                    return false;
                };
                self.board.ram[length_range].copy_from_slice(&unicode_length.to_le_bytes());

                let Some(source_range) = ram_range(source_buffer, u32::from(source_length)) else {
                    return false;
                };
                let Some(output_range) = ram_range(buffer, total) else {
                    return false;
                };
                let source_bytes = self.board.ram[source_range].to_vec();
                let output_start = output_range.start;
                for (index, byte) in source_bytes.into_iter().enumerate() {
                    self.board.ram[output_start + index * 2..output_start + index * 2 + 2]
                        .copy_from_slice(&ansi_byte_to_unicode(byte).to_le_bytes());
                }
                self.board.ram
                    [output_start + usize::from(unicode_length)..output_start + total as usize]
                    .copy_from_slice(&0u16.to_le_bytes());
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(12);
            }
            261 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let Some((source_length, _, source_buffer)) =
                    guest_string_descriptor(&self.board.ram, source)
                else {
                    return false;
                };
                let Some(status) =
                    self.append_kernel_string(destination, source_buffer, source_length, false)
                else {
                    return false;
                };
                self.cpu.regs[EAX] = status;
                self.return_from_kernel(8);
            }
            262 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let Some((source_length, _, source_buffer)) =
                    guest_string_descriptor(&self.board.ram, source)
                else {
                    return false;
                };
                let Some(status) =
                    self.append_kernel_string(destination, source_buffer, source_length, true)
                else {
                    return false;
                };
                self.cpu.regs[EAX] = status;
                self.return_from_kernel(8);
            }
            263 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let status = if source == 0 {
                    STATUS_SUCCESS
                } else {
                    let Some(source_length) = guest_unicode_string_length(&self.board.ram, source)
                    else {
                        return false;
                    };
                    let Some(status) =
                        self.append_kernel_string(destination, source, source_length, true)
                    else {
                        return false;
                    };
                    status
                };
                self.cpu.regs[EAX] = status;
                self.return_from_kernel(8);
            }
            267 => {
                let string = self.kernel_argument(0);
                let mut base = self.kernel_argument(1);
                let value_pointer = self.kernel_argument(2);
                if string == 0 {
                    return false;
                }
                if base != 0 && !matches!(base, 2 | 8 | 10 | 16) {
                    self.cpu.regs[EAX] = STATUS_INVALID_PARAMETER;
                    self.return_from_kernel(12);
                    return true;
                }
                if value_pointer == 0 {
                    self.cpu.regs[EAX] = STATUS_ACCESS_VIOLATION;
                    self.return_from_kernel(12);
                    return true;
                }
                let Some(length) = guest_ansi_string_length(&self.board.ram, string) else {
                    return false;
                };
                let Some(range) = ram_range(string, u32::from(length)) else {
                    return false;
                };
                let bytes = &self.board.ram[range];
                let mut position = 0usize;
                while position < bytes.len() && bytes[position] <= b' ' {
                    position += 1;
                }
                let mut negative = false;
                if position < bytes.len() {
                    match bytes[position] {
                        b'+' => position += 1,
                        b'-' => {
                            negative = true;
                            position += 1;
                        }
                        _ => {}
                    }
                }
                if base == 0 {
                    base = 10;
                    if position + 1 < bytes.len() && bytes[position] == b'0' {
                        base = match bytes[position + 1] {
                            b'b' => 2,
                            b'o' => 8,
                            b'x' => 16,
                            _ => 10,
                        };
                        if base != 10 {
                            position += 2;
                        }
                    }
                }
                let mut total = 0u32;
                while position < bytes.len() {
                    let byte = bytes[position];
                    let digit = match byte {
                        b'0'..=b'9' => u32::from(byte - b'0'),
                        b'A'..=b'Z' => u32::from(byte - b'A') + 10,
                        b'a'..=b'z' => u32::from(byte - b'a') + 10,
                        _ => break,
                    };
                    if digit >= base {
                        break;
                    }
                    total = total.wrapping_mul(base).wrapping_add(digit);
                    position += 1;
                }
                if negative {
                    total = 0u32.wrapping_sub(total);
                }
                let Some(value_range) = ram_range(value_pointer, 4) else {
                    return false;
                };
                self.board.ram[value_range].copy_from_slice(&total.to_le_bytes());
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(12);
            }
            268 => {
                let source1 = self.kernel_argument(0);
                let source2 = self.kernel_argument(1);
                let length = self.kernel_argument(2);
                let Some(range1) = ram_range(source1, length) else {
                    return false;
                };
                let Some(range2) = ram_range(source2, length) else {
                    return false;
                };
                let matched = self.board.ram[range1]
                    .iter()
                    .zip(&self.board.ram[range2])
                    .take_while(|(left, right)| left == right)
                    .count();
                self.cpu.regs[EAX] = matched as u32;
                self.return_from_kernel(12);
            }
            269 => {
                let source = self.kernel_argument(0);
                let length = self.kernel_argument(1);
                let pattern = self.kernel_argument(2);
                let compared = length & !3;
                let Some(range) = ram_range(source, compared) else {
                    return false;
                };
                let matched = self.board.ram[range]
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .take_while(|word| u32::from_le_bytes(**word) == pattern)
                    .count()
                    * 4;
                self.cpu.regs[EAX] = matched as u32;
                self.return_from_kernel(12);
            }
            270 => {
                let string1 = self.kernel_argument(0);
                let string2 = self.kernel_argument(1);
                let case_insensitive = self.kernel_argument(2) != 0;
                let Some((length1, _, buffer1)) = guest_string_descriptor(&self.board.ram, string1)
                else {
                    return false;
                };
                let Some((length2, _, buffer2)) = guest_string_descriptor(&self.board.ram, string2)
                else {
                    return false;
                };
                let compare_length = length1.min(length2);
                let mut result = i32::from(length1) - i32::from(length2);
                if compare_length != 0 {
                    let Some(range1) = ram_range(buffer1, u32::from(compare_length)) else {
                        return false;
                    };
                    let Some(range2) = ram_range(buffer2, u32::from(compare_length)) else {
                        return false;
                    };
                    for (&left, &right) in
                        self.board.ram[range1].iter().zip(&self.board.ram[range2])
                    {
                        let difference = if case_insensitive {
                            i32::from(rtl_lower_char(left)) - i32::from(rtl_lower_char(right))
                        } else {
                            i32::from(left as i8) - i32::from(right as i8)
                        };
                        if difference != 0 {
                            result = difference;
                            break;
                        }
                    }
                }
                self.cpu.regs[EAX] = result as u32;
                self.return_from_kernel(12);
            }
            271 => {
                let string1 = self.kernel_argument(0);
                let string2 = self.kernel_argument(1);
                let case_insensitive = self.kernel_argument(2) != 0;
                let Some((length1, _, buffer1)) = guest_string_descriptor(&self.board.ram, string1)
                else {
                    return false;
                };
                let Some((length2, _, buffer2)) = guest_string_descriptor(&self.board.ram, string2)
                else {
                    return false;
                };
                let compare_bytes = u32::from(length1.min(length2) & !1);
                let mut result = i32::from(length1) - i32::from(length2);
                if compare_bytes != 0 {
                    let Some(range1) = ram_range(buffer1, compare_bytes) else {
                        return false;
                    };
                    let Some(range2) = ram_range(buffer2, compare_bytes) else {
                        return false;
                    };
                    for (left, right) in self.board.ram[range1]
                        .as_chunks::<2>()
                        .0
                        .iter()
                        .zip(self.board.ram[range2].as_chunks::<2>().0)
                    {
                        let mut left = u16::from_le_bytes(*left);
                        let mut right = u16::from_le_bytes(*right);
                        if case_insensitive {
                            left = rtl_unicode_lower(left);
                            right = rtl_unicode_lower(right);
                        }
                        if left != right {
                            result = i32::from(left) - i32::from(right);
                            break;
                        }
                    }
                }
                self.cpu.regs[EAX] = result as u32;
                self.return_from_kernel(12);
            }
            272 | 273 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let Some((_, maximum_length, destination_buffer)) =
                    guest_string_descriptor(&self.board.ram, destination)
                else {
                    return false;
                };
                if source == 0 {
                    let Some(destination_range) = ram_range(destination, 2) else {
                        return false;
                    };
                    self.board.ram[destination_range].copy_from_slice(&0u16.to_le_bytes());
                    self.return_from_kernel(8);
                    return true;
                }
                let Some((source_length, _, source_buffer)) =
                    guest_string_descriptor(&self.board.ram, source)
                else {
                    return false;
                };
                let copy_length = source_length.min(maximum_length);
                let Some(source_range) = ram_range(source_buffer, u32::from(copy_length)) else {
                    return false;
                };
                let Some(destination_range) = ram_range(destination_buffer, u32::from(copy_length))
                else {
                    return false;
                };
                let bytes = self.board.ram[source_range].to_vec();
                self.board.ram[destination_range].copy_from_slice(&bytes);
                let Some(length_range) = ram_range(destination, 2) else {
                    return false;
                };
                self.board.ram[length_range].copy_from_slice(&copy_length.to_le_bytes());
                self.return_from_kernel(8);
            }
            274 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let Some(length) = guest_unicode_string_length(&self.board.ram, source) else {
                    return false;
                };
                let Some(maximum_length) = length.checked_add(2) else {
                    return false;
                };
                let buffer = self.allocate_kernel_memory(u32::from(maximum_length), 16);
                if buffer == 0 {
                    self.cpu.regs[EAX] = 0;
                    self.return_from_kernel(8);
                    return true;
                }
                let Some(source_range) = ram_range(source, u32::from(maximum_length)) else {
                    let _ = self.board.kernel_heap.free(buffer);
                    return false;
                };
                let Some(destination_range) = ram_range(buffer, u32::from(maximum_length)) else {
                    let _ = self.board.kernel_heap.free(buffer);
                    return false;
                };
                let bytes = self.board.ram[source_range].to_vec();
                self.board.ram[destination_range].copy_from_slice(&bytes);
                let Some(descriptor_range) = ram_range(destination, 8) else {
                    let _ = self.board.kernel_heap.free(buffer);
                    return false;
                };
                let start = descriptor_range.start;
                self.board.ram[start..start + 2].copy_from_slice(&length.to_le_bytes());
                self.board.ram[start + 2..start + 4].copy_from_slice(&maximum_length.to_le_bytes());
                self.board.ram[start + 4..start + 8].copy_from_slice(&buffer.to_le_bytes());
                self.cpu.regs[EAX] = 1;
                self.return_from_kernel(8);
            }
            275 | 313 => {
                let source = self.kernel_argument(0) as u16;
                let result = if ordinal == 275 {
                    rtl_unicode_lower(source)
                } else {
                    rtl_unicode_upper(source)
                };
                self.cpu.regs[EAX] = u32::from(result);
                self.return_from_kernel(4);
            }
            276 | 314 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let allocate = self.kernel_argument(2) != 0;
                let Some(destination_range) = ram_range(destination, 8) else {
                    return false;
                };
                let Some((source_length, _, source_buffer)) =
                    guest_string_descriptor(&self.board.ram, source)
                else {
                    return false;
                };
                let (maximum_length, destination_buffer) = if allocate {
                    let buffer = self.allocate_kernel_memory(u32::from(source_length), 16);
                    if buffer == 0 {
                        self.cpu.regs[EAX] = STATUS_NO_MEMORY;
                        self.return_from_kernel(12);
                        return true;
                    }
                    (source_length, buffer)
                } else {
                    let Some((_, maximum_length, buffer)) =
                        guest_string_descriptor(&self.board.ram, destination)
                    else {
                        return false;
                    };
                    if source_length > maximum_length {
                        self.cpu.regs[EAX] = STATUS_BUFFER_OVERFLOW;
                        self.return_from_kernel(12);
                        return true;
                    }
                    (maximum_length, buffer)
                };

                let character_bytes = u32::from(source_length & !1);
                let Some(source_range) = ram_range(source_buffer, character_bytes) else {
                    if allocate {
                        let _ = self.board.kernel_heap.free(destination_buffer);
                    }
                    return false;
                };
                let Some(output_range) = ram_range(destination_buffer, character_bytes) else {
                    if allocate {
                        let _ = self.board.kernel_heap.free(destination_buffer);
                    }
                    return false;
                };
                let source_bytes = self.board.ram[source_range].to_vec();
                let start = output_range.start;
                for (index, unit) in source_bytes.as_chunks::<2>().0.iter().enumerate() {
                    let unit = u16::from_le_bytes(*unit);
                    let mapped = if ordinal == 276 {
                        rtl_unicode_lower(unit)
                    } else {
                        rtl_unicode_upper(unit)
                    };
                    let offset = start + index * 2;
                    self.board.ram[offset..offset + 2].copy_from_slice(&mapped.to_le_bytes());
                }

                let descriptor_start = destination_range.start;
                self.board.ram[descriptor_start..descriptor_start + 2]
                    .copy_from_slice(&source_length.to_le_bytes());
                if allocate {
                    self.board.ram[descriptor_start + 2..descriptor_start + 4]
                        .copy_from_slice(&maximum_length.to_le_bytes());
                    self.board.ram[descriptor_start + 4..descriptor_start + 8]
                        .copy_from_slice(&destination_buffer.to_le_bytes());
                }
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(12);
            }
            277 => {
                let critical_section = self.kernel_argument(0);
                if !self.enter_critical_section(critical_section) {
                    return false;
                }
                self.return_from_kernel(4);
            }
            278 => {
                let critical_section = self.kernel_argument(0);
                if !self.enter_critical_region() {
                    return false;
                }
                if !self.enter_critical_section(critical_section) {
                    let _ = self.leave_critical_region();
                    return false;
                }
                self.return_from_kernel(4);
            }
            279 => {
                let string1 = self.kernel_argument(0);
                let string2 = self.kernel_argument(1);
                let case_insensitive = self.kernel_argument(2) != 0;
                let Some((length1, _, buffer1)) = guest_string_descriptor(&self.board.ram, string1)
                else {
                    return false;
                };
                let Some((length2, _, buffer2)) = guest_string_descriptor(&self.board.ram, string2)
                else {
                    return false;
                };
                let equal = if length1 != length2 {
                    false
                } else if length1 == 0 {
                    true
                } else {
                    let Some(range1) = ram_range(buffer1, u32::from(length1)) else {
                        return false;
                    };
                    let Some(range2) = ram_range(buffer2, u32::from(length2)) else {
                        return false;
                    };
                    self.board.ram[range1]
                        .iter()
                        .zip(&self.board.ram[range2])
                        .all(|(&left, &right)| {
                            if case_insensitive {
                                rtl_upper_char(left) == rtl_upper_char(right)
                            } else {
                                left == right
                            }
                        })
                };
                self.cpu.regs[EAX] = u32::from(equal);
                self.return_from_kernel(12);
            }
            280 => {
                let string1 = self.kernel_argument(0);
                let string2 = self.kernel_argument(1);
                let case_insensitive = self.kernel_argument(2) != 0;
                let Some((length1, _, buffer1)) = guest_string_descriptor(&self.board.ram, string1)
                else {
                    return false;
                };
                let Some((length2, _, buffer2)) = guest_string_descriptor(&self.board.ram, string2)
                else {
                    return false;
                };
                let equal = if length1 != length2 {
                    false
                } else if length1 == 0 {
                    true
                } else {
                    let bytes = u32::from(length1 & !1);
                    let Some(range1) = ram_range(buffer1, bytes) else {
                        return false;
                    };
                    let Some(range2) = ram_range(buffer2, bytes) else {
                        return false;
                    };
                    self.board.ram[range1]
                        .as_chunks::<2>()
                        .0
                        .iter()
                        .zip(self.board.ram[range2].as_chunks::<2>().0)
                        .all(|(left, right)| {
                            let left = u16::from_le_bytes(*left);
                            let right = u16::from_le_bytes(*right);
                            if case_insensitive {
                                rtl_unicode_upper(left) == rtl_unicode_upper(right)
                            } else {
                                left == right
                            }
                        })
                };
                self.cpu.regs[EAX] = u32::from(equal);
                self.return_from_kernel(12);
            }
            281 => {
                let low = u64::from(self.kernel_argument(0));
                let high = u64::from(self.kernel_argument(1));
                let multiplicand = ((high << 32) | low) as i64;
                let multiplier = i64::from(self.kernel_argument(2) as i32);
                let result = multiplicand.wrapping_mul(multiplier) as u64;
                self.cpu.regs[EAX] = result as u32;
                self.cpu.regs[EDX] = (result >> 32) as u32;
                self.return_from_kernel(12);
            }
            282 => {
                let low = u64::from(self.kernel_argument(0));
                let high = u64::from(self.kernel_argument(1));
                let dividend = (high << 32) | low;
                let divisor = self.kernel_argument(2);
                let remainder = self.kernel_argument(3);
                if divisor == 0 {
                    return false;
                }
                let quotient = dividend / u64::from(divisor);
                let remainder_value = (dividend % u64::from(divisor)) as u32;
                if remainder != 0 {
                    let Some(range) = ram_range(remainder, 4) else {
                        return false;
                    };
                    self.board.ram[range].copy_from_slice(&remainder_value.to_le_bytes());
                }
                self.cpu.regs[EAX] = quotient as u32;
                self.cpu.regs[EDX] = (quotient >> 32) as u32;
                self.return_from_kernel(16);
            }
            283 => {
                let dividend_low = u64::from(self.kernel_argument(0));
                let dividend_high = u64::from(self.kernel_argument(1));
                let dividend = ((dividend_high << 32) | dividend_low) as i64;
                let magic_low = u64::from(self.kernel_argument(2));
                let magic_high = u64::from(self.kernel_argument(3));
                let magic = (magic_high << 32) | magic_low;
                let shift = self.kernel_argument(4) as u8;
                if shift > 63 {
                    return false;
                }
                let high_product =
                    ((u128::from(dividend.unsigned_abs()) * u128::from(magic)) >> 64) as u64;
                let magnitude = high_product >> shift;
                let result = if dividend < 0 {
                    0u64.wrapping_sub(magnitude)
                } else {
                    magnitude
                };
                self.cpu.regs[EAX] = result as u32;
                self.cpu.regs[EDX] = (result >> 32) as u32;
                self.return_from_kernel(20);
            }
            284 => {
                let destination = self.kernel_argument(0);
                let length = self.kernel_argument(1);
                let fill = self.kernel_argument(2) as u8;
                let Some(range) = ram_range(destination, length) else {
                    return false;
                };
                self.board.ram[range].fill(fill);
                self.return_from_kernel(12);
            }
            285 => {
                let destination = self.kernel_argument(0);
                let length = self.kernel_argument(1);
                let pattern = self.kernel_argument(2).to_le_bytes();
                let Some(range) = ram_range(destination, length) else {
                    return false;
                };
                for (index, byte) in self.board.ram[range].iter_mut().enumerate() {
                    *byte = pattern[index & 3];
                }
                self.return_from_kernel(12);
            }
            286 | 287 => {
                let string = self.kernel_argument(0);
                let Some((_, _, buffer)) = guest_string_descriptor(&self.board.ram, string) else {
                    return false;
                };
                if buffer != 0 {
                    let _ = self.board.kernel_heap.free(buffer);
                    let Some(range) = ram_range(string, 8) else {
                        return false;
                    };
                    self.board.ram[range].fill(0);
                }
                self.return_from_kernel(4);
            }
            289 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let Some(range) = ram_range(destination, 8) else {
                    return false;
                };
                let (length, maximum_length) = if source == 0 {
                    (0u16, 0u16)
                } else {
                    let Some(length) = guest_ansi_string_length(&self.board.ram, source) else {
                        return false;
                    };
                    let Some(maximum_length) = length.checked_add(1) else {
                        return false;
                    };
                    (length, maximum_length)
                };
                let start = range.start;
                self.board.ram[start..start + 2].copy_from_slice(&length.to_le_bytes());
                self.board.ram[start + 2..start + 4].copy_from_slice(&maximum_length.to_le_bytes());
                self.board.ram[start + 4..start + 8].copy_from_slice(&source.to_le_bytes());
                self.return_from_kernel(8);
            }
            290 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let Some(range) = ram_range(destination, 8) else {
                    return false;
                };
                let (length, maximum_length) = if source == 0 {
                    (0u16, 0u16)
                } else {
                    let Some(length) = guest_unicode_string_length(&self.board.ram, source) else {
                        return false;
                    };
                    let Some(maximum_length) = length.checked_add(2) else {
                        return false;
                    };
                    (length, maximum_length)
                };
                let start = range.start;
                self.board.ram[start..start + 2].copy_from_slice(&length.to_le_bytes());
                self.board.ram[start + 2..start + 4].copy_from_slice(&maximum_length.to_le_bytes());
                self.board.ram[start + 4..start + 8].copy_from_slice(&source.to_le_bytes());
                self.return_from_kernel(8);
            }
            291 => {
                let critical_section = self.kernel_argument(0);
                if !self.initialize_critical_section(critical_section) {
                    return false;
                }
                self.return_from_kernel(4);
            }
            292 => {
                let value = self.kernel_argument(0);
                let base = self.kernel_argument(1);
                let output_length = self.kernel_argument(2);
                let string = self.kernel_argument(3);
                let Some(digits) = rtl_format_unsigned(value, base) else {
                    self.cpu.regs[EAX] = STATUS_INVALID_PARAMETER;
                    self.return_from_kernel(16);
                    return true;
                };
                if digits.len() > output_length as usize {
                    self.cpu.regs[EAX] = STATUS_BUFFER_OVERFLOW;
                    self.return_from_kernel(16);
                    return true;
                }
                if string == 0 {
                    self.cpu.regs[EAX] = STATUS_ACCESS_VIOLATION;
                    self.return_from_kernel(16);
                    return true;
                }
                let copy_terminator = digits.len() != output_length as usize;
                let copy_length = digits.len() + usize::from(copy_terminator);
                let Some(range) = ram_range(string, copy_length as u32) else {
                    return false;
                };
                let start = range.start;
                self.board.ram[start..start + digits.len()].copy_from_slice(&digits);
                if copy_terminator {
                    self.board.ram[start + digits.len()] = 0;
                }
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(16);
            }
            293 => {
                let value = self.kernel_argument(0);
                let base = self.kernel_argument(1);
                let string = self.kernel_argument(2);
                let Some(digits) = rtl_format_unsigned(value, base) else {
                    self.cpu.regs[EAX] = STATUS_INVALID_PARAMETER;
                    self.return_from_kernel(12);
                    return true;
                };
                let Some((_, maximum_length, buffer)) =
                    guest_string_descriptor(&self.board.ram, string)
                else {
                    return false;
                };
                let unicode_length = digits.len().saturating_mul(2);
                let total = unicode_length.saturating_add(2);
                let Some(length_range) = ram_range(string, 2) else {
                    return false;
                };
                self.board.ram[length_range]
                    .copy_from_slice(&(unicode_length as u16).to_le_bytes());
                if total > usize::from(maximum_length) {
                    self.cpu.regs[EAX] = STATUS_BUFFER_OVERFLOW;
                    self.return_from_kernel(12);
                    return true;
                }
                let Some(output_range) = ram_range(buffer, total as u32) else {
                    return false;
                };
                let start = output_range.start;
                for (index, byte) in digits.iter().copied().enumerate() {
                    self.board.ram[start + index * 2..start + index * 2 + 2]
                        .copy_from_slice(&u16::from(byte).to_le_bytes());
                }
                self.board.ram[start + unicode_length..start + unicode_length + 2]
                    .copy_from_slice(&0u16.to_le_bytes());
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(12);
            }
            294 => {
                let critical_section = self.kernel_argument(0);
                if !self.leave_critical_section(critical_section) {
                    return false;
                }
                self.return_from_kernel(4);
            }
            295 => {
                let critical_section = self.kernel_argument(0);
                if !self.leave_critical_section(critical_section) {
                    return false;
                }
                if self.board.read32(critical_section.wrapping_add(0x14)) == 0
                    && !self.leave_critical_region()
                {
                    return false;
                }
                self.return_from_kernel(4);
            }
            296 => {
                let character = self.kernel_argument(0) as u8;
                self.cpu.regs[EAX] = i32::from(rtl_lower_char(character) as i8) as u32;
                self.return_from_kernel(4);
            }
            297 => {
                let access_mask = self.kernel_argument(0);
                let generic_mapping = self.kernel_argument(1);
                let Some(mask_range) = ram_range(access_mask, 4) else {
                    return false;
                };
                let Some(mapping_range) = ram_range(generic_mapping, 16) else {
                    return false;
                };
                let mapping = &self.board.ram[mapping_range];
                let generic_read = u32::from_le_bytes(mapping[0..4].try_into().unwrap());
                let generic_write = u32::from_le_bytes(mapping[4..8].try_into().unwrap());
                let generic_execute = u32::from_le_bytes(mapping[8..12].try_into().unwrap());
                let generic_all = u32::from_le_bytes(mapping[12..16].try_into().unwrap());
                let mut mask =
                    u32::from_le_bytes(self.board.ram[mask_range.clone()].try_into().unwrap());
                if mask & GENERIC_READ != 0 {
                    mask |= generic_read;
                }
                if mask & GENERIC_WRITE != 0 {
                    mask |= generic_write;
                }
                if mask & GENERIC_EXECUTE != 0 {
                    mask |= generic_execute;
                }
                if mask & GENERIC_ALL != 0 {
                    mask |= generic_all;
                }
                mask &= !(GENERIC_READ | GENERIC_WRITE | GENERIC_EXECUTE | GENERIC_ALL);
                self.board.ram[mask_range].copy_from_slice(&mask.to_le_bytes());
                self.return_from_kernel(8);
            }
            298 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let length = self.kernel_argument(2);
                let Some(source_range) = ram_range(source, length) else {
                    return false;
                };
                let Some(destination_range) = ram_range(destination, length) else {
                    return false;
                };
                let bytes = self.board.ram[source_range].to_vec();
                self.board.ram[destination_range].copy_from_slice(&bytes);
                self.return_from_kernel(12);
            }
            299 => {
                let unicode_string = self.kernel_argument(0);
                let max_unicode_bytes = self.kernel_argument(1);
                let bytes_in_unicode = self.kernel_argument(2);
                let multi_byte_string = self.kernel_argument(3);
                let bytes_in_multi_byte = self.kernel_argument(4);
                let character_count = (max_unicode_bytes / 2).min(bytes_in_multi_byte);
                if bytes_in_unicode != 0 {
                    let Some(result_range) = ram_range(bytes_in_unicode, 4) else {
                        return false;
                    };
                    self.board.ram[result_range]
                        .copy_from_slice(&character_count.wrapping_mul(2).to_le_bytes());
                }
                if character_count != 0 {
                    let Some(source_range) = ram_range(multi_byte_string, character_count) else {
                        return false;
                    };
                    let Some(output_range) =
                        ram_range(unicode_string, character_count.wrapping_mul(2))
                    else {
                        return false;
                    };
                    let source = self.board.ram[source_range].to_vec();
                    let start = output_range.start;
                    for (index, byte) in source.into_iter().enumerate() {
                        self.board.ram[start + index * 2..start + index * 2 + 2]
                            .copy_from_slice(&ansi_byte_to_unicode(byte).to_le_bytes());
                    }
                }
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(20);
            }
            300 => {
                let bytes_in_unicode = self.kernel_argument(0);
                let _multi_byte_string = self.kernel_argument(1);
                let bytes_in_multi_byte = self.kernel_argument(2);
                let Some(result_range) = ram_range(bytes_in_unicode, 4) else {
                    return false;
                };
                self.board.ram[result_range]
                    .copy_from_slice(&bytes_in_multi_byte.wrapping_mul(2).to_le_bytes());
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(12);
            }
            304 => {
                let time_fields = self.kernel_argument(0);
                let time = self.kernel_argument(1);
                let Some(fields_range) = ram_range(time_fields, 16) else {
                    return false;
                };
                let bytes = &self.board.ram[fields_range];
                let mut fields = [0u16; 8];
                for (index, field) in fields.iter_mut().enumerate() {
                    let offset = index * 2;
                    *field = u16::from_le_bytes(bytes[offset..offset + 2].try_into().unwrap());
                }
                let Some(ticks) = rtl_time_fields_to_ticks(fields) else {
                    self.cpu.regs[EAX] = 0;
                    self.return_from_kernel(8);
                    return true;
                };
                let Some(time_range) = ram_range(time, 8) else {
                    return false;
                };
                self.board.ram[time_range].copy_from_slice(&ticks.to_le_bytes());
                self.cpu.regs[EAX] = 1;
                self.return_from_kernel(8);
            }
            305 => {
                let time = self.kernel_argument(0);
                let time_fields = self.kernel_argument(1);
                let Some(time_range) = ram_range(time, 8) else {
                    return false;
                };
                let ticks = u64::from_le_bytes(self.board.ram[time_range].try_into().unwrap());
                let Some(fields) = rtl_ticks_to_time_fields(ticks) else {
                    return false;
                };
                let Some(fields_range) = ram_range(time_fields, 16) else {
                    return false;
                };
                let start = fields_range.start;
                for (index, field) in fields.into_iter().enumerate() {
                    let offset = start + index * 2;
                    self.board.ram[offset..offset + 2].copy_from_slice(&field.to_le_bytes());
                }
                self.return_from_kernel(8);
            }
            306 => {
                let critical_section = self.kernel_argument(0);
                let Some(entered) = self.try_enter_critical_section(critical_section) else {
                    return false;
                };
                self.cpu.regs[EAX] = u32::from(entered);
                self.return_from_kernel(4);
            }
            307 => {
                self.cpu.regs[EAX] = self.cpu.regs[ECX].swap_bytes();
                self.return_from_kernel(0);
            }
            308 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let allocate = self.kernel_argument(2) != 0;
                let Some((source_length, _, source_buffer)) =
                    guest_string_descriptor(&self.board.ram, source)
                else {
                    return false;
                };
                let source_characters = u32::from(source_length / 2);
                let ansi_maximum = source_characters.saturating_add(1);
                let Some((_, destination_maximum, existing_buffer)) =
                    guest_string_descriptor(&self.board.ram, destination)
                else {
                    return false;
                };
                let mut destination_length = source_characters as u16;
                let mut status = STATUS_SUCCESS;
                let buffer = if allocate {
                    let buffer = self.allocate_kernel_memory(ansi_maximum, 16);
                    if buffer == 0 {
                        self.cpu.regs[EAX] = STATUS_NO_MEMORY;
                        self.return_from_kernel(12);
                        return true;
                    }
                    let Some(maximum_address) = destination.checked_add(2) else {
                        let _ = self.board.kernel_heap.free(buffer);
                        return false;
                    };
                    let Some(maximum_range) = ram_range(maximum_address, 2) else {
                        let _ = self.board.kernel_heap.free(buffer);
                        return false;
                    };
                    self.board.ram[maximum_range]
                        .copy_from_slice(&(ansi_maximum as u16).to_le_bytes());
                    let Some(buffer_address) = destination.checked_add(4) else {
                        let _ = self.board.kernel_heap.free(buffer);
                        return false;
                    };
                    let Some(buffer_range) = ram_range(buffer_address, 4) else {
                        let _ = self.board.kernel_heap.free(buffer);
                        return false;
                    };
                    self.board.ram[buffer_range].copy_from_slice(&buffer.to_le_bytes());
                    buffer
                } else {
                    if u32::from(destination_maximum) < ansi_maximum {
                        status = STATUS_BUFFER_OVERFLOW;
                        if destination_maximum == 0 {
                            let Some(length_range) = ram_range(destination, 2) else {
                                return false;
                            };
                            self.board.ram[length_range]
                                .copy_from_slice(&destination_length.to_le_bytes());
                            self.cpu.regs[EAX] = status;
                            self.return_from_kernel(12);
                            return true;
                        }
                        destination_length = destination_maximum - 1;
                    }
                    existing_buffer
                };
                let Some(length_range) = ram_range(destination, 2) else {
                    return false;
                };
                self.board.ram[length_range].copy_from_slice(&destination_length.to_le_bytes());

                let convert_characters =
                    source_characters.min(u32::from(destination_length)) as usize;
                let Some(source_range) = ram_range(source_buffer, (convert_characters * 2) as u32)
                else {
                    return false;
                };
                let Some(output_range) = ram_range(buffer, convert_characters as u32 + 1) else {
                    return false;
                };
                let source_bytes = self.board.ram[source_range].to_vec();
                let output_start = output_range.start;
                for index in 0..convert_characters {
                    let unit =
                        u16::from_le_bytes([source_bytes[index * 2], source_bytes[index * 2 + 1]]);
                    self.board.ram[output_start + index] = unicode_to_ansi(unit);
                }
                self.board.ram[output_start + convert_characters] = 0;
                self.cpu.regs[EAX] = status;
                self.return_from_kernel(12);
            }
            309 => {
                let string = self.kernel_argument(0);
                let mut base = self.kernel_argument(1);
                let value_pointer = self.kernel_argument(2);
                if string == 0 {
                    return false;
                }
                let Some((length, _, buffer)) = guest_string_descriptor(&self.board.ram, string)
                else {
                    return false;
                };
                let character_count = usize::from(length / 2);
                let Some(range) = ram_range(buffer, (character_count * 2) as u32) else {
                    return false;
                };
                let bytes = &self.board.ram[range];
                let mut position = 0usize;
                let unit =
                    |index: usize| u16::from_le_bytes([bytes[index * 2], bytes[index * 2 + 1]]);
                while position < character_count && unit(position) <= u16::from(b' ') {
                    position += 1;
                }
                let mut negative = false;
                if position < character_count {
                    match unit(position) {
                        value if value == u16::from(b'+') => position += 1,
                        value if value == u16::from(b'-') => {
                            negative = true;
                            position += 1;
                        }
                        _ => {}
                    }
                }
                if base == 0 {
                    base = 10;
                    if position + 1 < character_count && unit(position) == u16::from(b'0') {
                        base = match unit(position + 1) {
                            value if value == u16::from(b'b') => 2,
                            value if value == u16::from(b'o') => 8,
                            value if value == u16::from(b'x') => 16,
                            _ => 10,
                        };
                        if base != 10 {
                            position += 2;
                        }
                    }
                } else if !matches!(base, 2 | 8 | 10 | 16) {
                    self.cpu.regs[EAX] = STATUS_INVALID_PARAMETER;
                    self.return_from_kernel(12);
                    return true;
                }
                if value_pointer == 0 {
                    self.cpu.regs[EAX] = STATUS_ACCESS_VIOLATION;
                    self.return_from_kernel(12);
                    return true;
                }
                let mut total = 0u32;
                while position < character_count {
                    let character = unit(position);
                    let digit = match character {
                        value if (u16::from(b'0')..=u16::from(b'9')).contains(&value) => {
                            u32::from(value - u16::from(b'0'))
                        }
                        value if (u16::from(b'A')..=u16::from(b'Z')).contains(&value) => {
                            u32::from(value - u16::from(b'A')) + 10
                        }
                        value if (u16::from(b'a')..=u16::from(b'z')).contains(&value) => {
                            u32::from(value - u16::from(b'a')) + 10
                        }
                        _ => break,
                    };
                    if digit >= base {
                        break;
                    }
                    total = total.wrapping_mul(base).wrapping_add(digit);
                    position += 1;
                }
                if negative {
                    total = 0u32.wrapping_sub(total);
                }
                let Some(value_range) = ram_range(value_pointer, 4) else {
                    return false;
                };
                self.board.ram[value_range].copy_from_slice(&total.to_le_bytes());
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(12);
            }
            310 => {
                let multi_byte_string = self.kernel_argument(0);
                let max_multi_byte_bytes = self.kernel_argument(1);
                let bytes_in_multi_byte = self.kernel_argument(2);
                let unicode_string = self.kernel_argument(3);
                let bytes_in_unicode = self.kernel_argument(4);
                let character_count = (bytes_in_unicode / 2).min(max_multi_byte_bytes);
                if bytes_in_multi_byte != 0 {
                    let Some(result_range) = ram_range(bytes_in_multi_byte, 4) else {
                        return false;
                    };
                    self.board.ram[result_range].copy_from_slice(&character_count.to_le_bytes());
                }
                if character_count != 0 {
                    let Some(source_range) =
                        ram_range(unicode_string, character_count.wrapping_mul(2))
                    else {
                        return false;
                    };
                    let Some(output_range) = ram_range(multi_byte_string, character_count) else {
                        return false;
                    };
                    let source = self.board.ram[source_range].to_vec();
                    let start = output_range.start;
                    for index in 0..character_count as usize {
                        let unit = u16::from_le_bytes([source[index * 2], source[index * 2 + 1]]);
                        self.board.ram[start + index] = unicode_to_ansi(unit);
                    }
                }
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(20);
            }
            311 => {
                let bytes_in_multi_byte = self.kernel_argument(0);
                let _unicode_string = self.kernel_argument(1);
                let bytes_in_unicode = self.kernel_argument(2);
                let Some(result_range) = ram_range(bytes_in_multi_byte, 4) else {
                    return false;
                };
                self.board.ram[result_range].copy_from_slice(&(bytes_in_unicode / 2).to_le_bytes());
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(12);
            }
            315 => {
                let multi_byte_string = self.kernel_argument(0);
                let max_bytes = self.kernel_argument(1);
                let bytes_written = self.kernel_argument(2);
                let unicode_string = self.kernel_argument(3);
                let bytes_in_unicode = self.kernel_argument(4);
                let characters = (bytes_in_unicode / 2).min(max_bytes);
                if bytes_written != 0 {
                    let Some(result_range) = ram_range(bytes_written, 4) else {
                        return false;
                    };
                    self.board.ram[result_range].copy_from_slice(&characters.to_le_bytes());
                }
                let Some(source_range) = ram_range(unicode_string, characters.saturating_mul(2))
                else {
                    return false;
                };
                let Some(output_range) = ram_range(multi_byte_string, characters) else {
                    return false;
                };
                let source = self.board.ram[source_range].to_vec();
                let start = output_range.start;
                for (index, unit) in source.as_chunks::<2>().0.iter().enumerate() {
                    let unit = u16::from_le_bytes(*unit);
                    let unit = if unit < 256 { unit } else { u16::from(b'?') };
                    let upper = rtl_unicode_upper(unit);
                    self.board.ram[start + index] = if upper < 256 { upper as u8 } else { b'?' };
                }
                self.cpu.regs[EAX] = STATUS_SUCCESS;
                self.return_from_kernel(20);
            }
            316 => {
                let character = self.kernel_argument(0) as u8;
                self.cpu.regs[EAX] = i32::from(rtl_upper_char(character) as i8) as u32;
                self.return_from_kernel(4);
            }
            317 => {
                let destination = self.kernel_argument(0);
                let source = self.kernel_argument(1);
                let Some((_, destination_maximum, destination_buffer)) =
                    guest_string_descriptor(&self.board.ram, destination)
                else {
                    return false;
                };
                let Some((source_length, _, source_buffer)) =
                    guest_string_descriptor(&self.board.ram, source)
                else {
                    return false;
                };
                let length = source_length.min(destination_maximum);
                let Some(source_range) = ram_range(source_buffer, u32::from(length)) else {
                    return false;
                };
                let Some(destination_range) = ram_range(destination_buffer, u32::from(length))
                else {
                    return false;
                };
                let bytes: Vec<u8> = self.board.ram[source_range]
                    .iter()
                    .copied()
                    .map(rtl_upper_char)
                    .collect();
                self.board.ram[destination_range].copy_from_slice(&bytes);
                let Some(length_range) = ram_range(destination, 2) else {
                    return false;
                };
                self.board.ram[length_range].copy_from_slice(&length.to_le_bytes());
                self.return_from_kernel(8);
            }
            318 => {
                self.cpu.regs[EAX] = u32::from((self.cpu.regs[ECX] as u16).swap_bytes());
                self.return_from_kernel(0);
            }
            320 => {
                let destination = self.kernel_argument(0);
                let length = self.kernel_argument(1);
                let Some(range) = ram_range(destination, length) else {
                    return false;
                };
                self.board.ram[range].fill(0);
                self.return_from_kernel(8);
            }
            _ => return false,
        }
        true
    }

    fn run_cpu_frame(&mut self) {
        let target = self.cpu.cycles.saturating_add(CPU_STEPS_PER_FRAME);
        let mut next_usb = self.cpu.cycles.saturating_add(1024);
        while self.powered && self.cpu.cycles < target {
            if let Some(ordinal) = kernel_hle_ordinal(self.cpu.eip) {
                if !self.dispatch_kernel_hle(ordinal) {
                    self.powered = false;
                    break;
                }
                continue;
            }
            self.cpu.step(&mut self.board);
            if self.cpu.cycles >= next_usb {
                self.board.service_usb();
                next_usb = self.cpu.cycles.saturating_add(1024);
            }
            if self.cpu.invalid_opcode() {
                self.powered = false;
                break;
            }
            if self.cpu.halted() || self.board.smbus.power_action != 0 {
                break;
            }
        }
    }
}

impl Machine for XboxMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Xbox
    }
    fn reset(&mut self) {
        match self.board.reset() {
            Ok(entry) => {
                Self::boot_cpu(&mut self.cpu, entry);
                self.powered = true;
                self.current_irql = 0;
                self.av_saved_data_address = 0;
                self.fsc_cache_pages = 16;
                self.hal_software_interrupts = 0;
                self.hal_system_interrupts_enabled = 0;
                self.hal_latched_interrupts = 0;
                self.mm_gpu_instance_bytes = XBOX_GPU_INSTANCE_BYTES;
                self.mm_persistent_pages.clear();
                self.mm_page_protection.clear();
                self.hal_interrupt_objects.fill(0);
                self.dpc_queue.clear();
                self.timer_queue.clear();
                self.initialize_current_thread_state();
            }
            Err(_) => self.powered = false,
        }
    }
    fn run_frame(&mut self, input: &InputState) {
        if !self.powered {
            return;
        }
        self.board.begin_frame(input);
        self.run_cpu_frame();
        let power_action = std::mem::take(&mut self.board.smbus.power_action);
        if power_action & 0x80 != 0 {
            self.powered = false;
            return;
        }
        if power_action & 0x02 != 0 {
            self.quick_reboot();
            return;
        }
        if power_action & 0x41 != 0 {
            self.reset();
            return;
        }
        self.board.end_frame();
        self.service_timers();
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
        let mut out = StateWriter::new(PlatformId::Xbox, STATE_VERSION);
        self.cpu.save(&mut out);
        self.board.save(&mut out);
        out.u8(u8::from(self.powered));
        out.u8(self.current_irql);
        out.u32(self.av_saved_data_address);
        out.u32(self.fsc_cache_pages);
        out.u32(self.hal_software_interrupts);
        out.u32(self.hal_system_interrupts_enabled);
        out.u32(self.hal_latched_interrupts);
        out.u32(self.mm_gpu_instance_bytes);
        out.u32(self.mm_persistent_pages.len() as u32);
        for &page in &self.mm_persistent_pages {
            out.u32(page);
        }
        out.u32(self.mm_page_protection.len() as u32);
        for (&page, &protection) in &self.mm_page_protection {
            out.u32(page);
            out.u32(protection);
        }
        for &interrupt in &self.hal_interrupt_objects {
            out.u32(interrupt);
        }
        out.u32(self.dpc_queue.len() as u32);
        for &dpc in &self.dpc_queue {
            out.u32(dpc);
        }
        out.u32(self.timer_queue.len() as u32);
        for &timer in &self.timer_queue {
            out.u32(timer);
        }
        Ok(out.finish())
    }
    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Xbox, STATE_VERSION)?;
        self.cpu.load(&mut input)?;
        self.board.load(&mut input)?;
        self.powered = input.u8()? != 0;
        self.current_irql = input.u8()?;
        self.av_saved_data_address = input.u32()?;
        self.fsc_cache_pages = input.u32()?;
        self.hal_software_interrupts = input.u32()?;
        self.hal_system_interrupts_enabled = input.u32()?;
        self.hal_latched_interrupts = input.u32()?;
        self.mm_gpu_instance_bytes = input.u32()?;
        let persistent_count = input.u32()? as usize;
        if persistent_count > RAM_SIZE / 0x1000 {
            return Err("Xbox state has too many persistent memory pages".into());
        }
        self.mm_persistent_pages.clear();
        for _ in 0..persistent_count {
            let page = input.u32()?;
            if page >= (RAM_SIZE / 0x1000) as u32 {
                return Err("Xbox state has an invalid persistent memory page".into());
            }
            self.mm_persistent_pages.insert(page);
        }
        let protection_count = input.u32()? as usize;
        if protection_count > 0x1_0000 {
            return Err("Xbox state has too many memory-protection entries".into());
        }
        self.mm_page_protection.clear();
        for _ in 0..protection_count {
            self.mm_page_protection.insert(input.u32()?, input.u32()?);
        }
        for interrupt in &mut self.hal_interrupt_objects {
            *interrupt = input.u32()?;
        }
        let dpc_count = input.u32()? as usize;
        if dpc_count > 0x1_0000 {
            return Err("Xbox state has too many queued DPCs".into());
        }
        self.dpc_queue.clear();
        for _ in 0..dpc_count {
            let dpc = input.u32()?;
            if ram_range(dpc, 0x1c).is_none() {
                return Err("Xbox state has an invalid queued DPC address".into());
            }
            self.dpc_queue.push(dpc);
        }
        let timer_count = input.u32()? as usize;
        if timer_count > 0x1_0000 {
            return Err("Xbox state has too many queued timers".into());
        }
        self.timer_queue.clear();
        for _ in 0..timer_count {
            let timer = input.u32()?;
            if ram_range(timer, 0x28).is_none() {
                return Err("Xbox state has an invalid queued timer address".into());
            }
            self.timer_queue.push(timer);
        }
        input.finish()?;
        Ok(())
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            EEPROM_SIZE
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("Xbox persistent resource is Storage slot 0".into());
        }
        if out.len() != EEPROM_SIZE {
            return Err("Xbox EEPROM buffer has the wrong size".into());
        }
        out.copy_from_slice(&self.board.smbus.eeprom);
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("Xbox persistent resource is Storage slot 0".into());
        }
        if data.len() != EEPROM_SIZE {
            return Err("Xbox EEPROM image has the wrong size".into());
        }
        self.board.smbus.eeprom.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn synthetic_xbe(program: &[u8]) -> ResourceBlob {
        let base = 0x0001_0000u32;
        let section_va = 0x0001_1000u32;
        let section_raw = 0x400u32;
        let section_table = 0x178u32;
        let mut bytes = vec![0; section_raw as usize + program.len()];
        bytes[0..4].copy_from_slice(&XBE_MAGIC.to_le_bytes());
        bytes[0x104..0x108].copy_from_slice(&base.to_le_bytes());
        bytes[0x108..0x10c].copy_from_slice(&0x400u32.to_le_bytes());
        bytes[0x11c..0x120].copy_from_slice(&1u32.to_le_bytes());
        bytes[0x120..0x124].copy_from_slice(&(base + section_table).to_le_bytes());
        bytes[0x128..0x12c].copy_from_slice(&(section_va ^ XBE_ENTRY_RETAIL_XOR).to_le_bytes());
        let table = section_table as usize;
        bytes[table..table + 4].copy_from_slice(&(1u32 << 2).to_le_bytes());
        bytes[table + 4..table + 8].copy_from_slice(&section_va.to_le_bytes());
        bytes[table + 8..table + 12].copy_from_slice(&0x1000u32.to_le_bytes());
        bytes[table + 12..table + 16].copy_from_slice(&section_raw.to_le_bytes());
        bytes[table + 16..table + 20].copy_from_slice(&(program.len() as u32).to_le_bytes());
        bytes[section_raw as usize..section_raw as usize + program.len()].copy_from_slice(program);
        ResourceBlob::from_bytes(&bytes)
    }

    fn synthetic_xbe_with_thunks(program: &[u8], ordinals: &[u32]) -> ResourceBlob {
        let base = 0x0001_0000u32;
        let section_va = 0x0001_1000u32;
        let section_raw = 0x400u32;
        let section_table = 0x178u32;
        let thunk_offset = 0x200usize;
        let raw_len = program.len().max(thunk_offset + (ordinals.len() + 1) * 4);
        let mut bytes = vec![0; section_raw as usize + raw_len];
        bytes[0..4].copy_from_slice(&XBE_MAGIC.to_le_bytes());
        bytes[0x104..0x108].copy_from_slice(&base.to_le_bytes());
        bytes[0x108..0x10c].copy_from_slice(&0x400u32.to_le_bytes());
        bytes[0x11c..0x120].copy_from_slice(&1u32.to_le_bytes());
        bytes[0x120..0x124].copy_from_slice(&(base + section_table).to_le_bytes());
        bytes[0x128..0x12c].copy_from_slice(&(section_va ^ XBE_ENTRY_RETAIL_XOR).to_le_bytes());
        let thunk_va = section_va + thunk_offset as u32;
        bytes[0x158..0x15c].copy_from_slice(&(thunk_va ^ XBE_THUNK_RETAIL_XOR).to_le_bytes());
        let table = section_table as usize;
        bytes[table..table + 4].copy_from_slice(&(1u32 << 2).to_le_bytes());
        bytes[table + 4..table + 8].copy_from_slice(&section_va.to_le_bytes());
        bytes[table + 8..table + 12].copy_from_slice(&0x1000u32.to_le_bytes());
        bytes[table + 12..table + 16].copy_from_slice(&section_raw.to_le_bytes());
        bytes[table + 16..table + 20].copy_from_slice(&(raw_len as u32).to_le_bytes());
        let raw = section_raw as usize;
        bytes[raw..raw + program.len()].copy_from_slice(program);
        for (index, ordinal) in ordinals.iter().copied().enumerate() {
            let offset = raw + thunk_offset + index * 4;
            bytes[offset..offset + 4].copy_from_slice(&(ordinal | 0x8000_0000).to_le_bytes());
        }
        ResourceBlob::from_bytes(&bytes)
    }

    #[test]
    fn xbe_loads_and_x86_executes_into_xbox_ram() {
        let program = [
            0xb8, 0x78, 0x56, 0x34, 0x12, 0xa3, 0x00, 0x20, 0x00, 0x00, 0xf4,
        ];
        let image = synthetic_xbe(&program);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(
            machine.board.ram[0x2000..0x2004],
            0x1234_5678u32.to_le_bytes()
        );
        assert_eq!(machine.cpu.eip, 0x0001_100b);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn xbe_kernel_thunks_patch_and_dispatch_supported_exports() {
        let program = [
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // call [0x11200] KeGetCurrentIrql
            0xa3, 0x00, 0x20, 0x00, 0x00, // mov [0x2000],eax
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // call [0x11204] KeQueryPerformanceFrequency
            0xa3, 0x04, 0x20, 0x00, 0x00, // mov [0x2004],eax
            0x89, 0x15, 0x08, 0x20, 0x00, 0x00, // mov [0x2008],edx
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // call [0x11208] NtYieldExecution
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[103, 127, 238, 324]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x11200..0x11204].try_into().unwrap()),
            kernel_hle_address(103)
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x1120c..0x11210].try_into().unwrap()),
            kernel_data_address(324)
        );
        assert_eq!(machine.board.read16(kernel_data_address(324)), 1);
        assert_eq!(machine.board.read16(kernel_data_address(324) + 4), 5838);
        machine.run_frame(&InputState::default());
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap()),
            0
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap()),
            3_375_000
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2008..0x200c].try_into().unwrap()),
            0
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn hal_interrupt_controls_track_pending_enabled_and_trigger_modes() {
        let program = [
            0xb9, 0x01, 0x00, 0x00, 0x00, // mov ecx,APC_LEVEL
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // HalRequestSoftwareInterrupt
            0xb9, 0x02, 0x00, 0x00, 0x00, // mov ecx,DPC_LEVEL
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // HalRequestSoftwareInterrupt
            0xb9, 0x02, 0x00, 0x00, 0x00, // mov ecx,DPC_LEVEL
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // HalClearSoftwareInterrupt
            0x6a, 0x01, // push Latched
            0x6a, 0x03, // push bus interrupt level
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // HalEnableSystemInterrupt
            0x6a, 0x00, // push LevelSensitive
            0x6a, 0x05, // push bus interrupt level
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // HalEnableSystemInterrupt
            0x6a, 0x03, // push bus interrupt level
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // HalDisableSystemInterrupt
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[48, 38, 43, 39]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        assert_eq!(machine.hal_software_interrupts, 1 << 1);
        assert_eq!(machine.hal_system_interrupts_enabled, 1 << 5);
        assert_eq!(machine.hal_latched_interrupts, 1 << 3);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());

        let state = machine.save_state().unwrap();
        let mut restored = XboxMachine::from_xbe(synthetic_xbe(&[0xf4])).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.hal_software_interrupts, 1 << 1);
        assert_eq!(restored.hal_system_interrupts_enabled, 1 << 5);
        assert_eq!(restored.hal_latched_interrupts, 1 << 3);
    }

    #[test]
    fn hal_return_to_firmware_halts_or_resets_without_returning_to_guest() {
        let halt_program = [
            0x6a, 0x00, // push HalHaltRoutine
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // HalReturnToFirmware
            0xc6, 0x05, 0x00, 0x20, 0x00, 0x00, 0x7f, // unreachable marker
            0xf4,
        ];
        let mut halted =
            XboxMachine::from_xbe(synthetic_xbe_with_thunks(&halt_program, &[49])).unwrap();
        halted.run_frame(&InputState::default());
        assert!(!halted.powered);
        assert_eq!(halted.board.ram[0x2000], 0);

        for routine in 1u8..=3 {
            let reboot_program = [
                0x6a, routine, 0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // HalReturnToFirmware
                0xc6, 0x05, 0x00, 0x20, 0x00, 0x00, 0x7f, // unreachable marker
                0xf4,
            ];
            let mut rebooted =
                XboxMachine::from_xbe(synthetic_xbe_with_thunks(&reboot_program, &[49])).unwrap();
            rebooted.hal_system_interrupts_enabled = 0xffff;
            rebooted.run_frame(&InputState::default());
            assert!(rebooted.powered);
            assert_eq!(rebooted.board.ram[0x2000], 0);
            assert_eq!(rebooted.hal_system_interrupts_enabled, 0);
            assert_eq!(rebooted.board.smbus.power_action, 0);
        }

        let fatal_program = [
            0x6a, 0x04, // push HalFatalErrorRebootRoutine
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // HalReturnToFirmware
            0xf4,
        ];
        let mut fatal =
            XboxMachine::from_xbe(synthetic_xbe_with_thunks(&fatal_program, &[49])).unwrap();
        fatal.run_frame(&InputState::default());
        assert!(fatal.powered);
        assert_eq!(fatal.board.smbus.smc_scratch & 0x02, 0x02);
        assert_eq!(fatal.board.smbus.power_action, 0);
    }

    #[test]
    fn hal_tray_state_matches_staged_atapi_media_and_raw_smc_state() {
        let program = [
            0x68, 0x04, 0x20, 0x00, 0x00, // push Count
            0x68, 0x00, 0x20, 0x00, 0x00, // push State
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // HalReadSMCTrayState
            0xa3, 0x08, 0x20, 0x00, 0x00, // store status
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[9]);

        let mut empty = XboxMachine::from_xbe(image.clone()).unwrap();
        empty.run_frame(&InputState::default());
        assert_eq!(
            u32::from_le_bytes(empty.board.ram[0x2000..0x2004].try_into().unwrap()),
            0x40
        );
        assert_eq!(
            u32::from_le_bytes(empty.board.ram[0x2004..0x2008].try_into().unwrap()),
            0
        );
        assert_eq!(
            u32::from_le_bytes(empty.board.ram[0x2008..0x200c].try_into().unwrap()),
            STATUS_SUCCESS
        );
        assert_eq!(
            empty.board.smbus.read_byte_data(SMBUS_SMC_ADDRESS, 0x03),
            Some(0x40)
        );

        let disc = ResourceBlob::from_bytes(&vec![0; ATAPI_SECTOR_SIZE]);
        let mut loaded = XboxMachine::from_images(image, Some(disc)).unwrap();
        loaded.run_frame(&InputState::default());
        assert_eq!(
            u32::from_le_bytes(loaded.board.ram[0x2000..0x2004].try_into().unwrap()),
            0x60
        );
        assert_eq!(
            u32::from_le_bytes(loaded.board.ram[0x2008..0x200c].try_into().unwrap()),
            STATUS_SUCCESS
        );
        assert_eq!(
            loaded.board.smbus.read_byte_data(SMBUS_SMC_ADDRESS, 0x03),
            Some(0x60)
        );
        assert_eq!(loaded.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(loaded.cpu.halted());
        assert!(loaded.powered);
    }

    #[test]
    fn kernel_smbus_thunks_use_stdcall_and_reach_smc() {
        let program = [
            0x6a, 0x5a, // push 0x5a data
            0x6a, 0x00, // push false
            0x6a, 0x1b, // push scratch command
            0x6a, 0x20, // push SMC address
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // call HalWriteSMBusValue
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store status
            0x68, 0x10, 0x20, 0x00, 0x00, // push output pointer
            0x6a, 0x00, // push false
            0x6a, 0x1b, // push scratch command
            0x6a, 0x20, // push SMC address
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // call HalReadSMBusValue
            0xa3, 0x14, 0x20, 0x00, 0x00, // store status
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[50, 45]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.board.smbus.smc_scratch, 0x5a);
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2010..0x2014].try_into().unwrap()),
            0x5a
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x200c..0x2010].try_into().unwrap()),
            0
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2014..0x2018].try_into().unwrap()),
            0
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_interrupt_connect_disconnect_enforces_irq_ownership_and_persists_state() {
        let program = [
            0x6a, 0x00, // push ShareVector=false
            0x6a, 0x01, // push Latched mode
            0x6a, 0x05, // push IRQL
            0x6a, 0x35, // push vector => IRQ 5
            0x68, 0x11, 0x11, 0x00, 0x00, // push service context
            0x68, 0x22, 0x22, 0x00, 0x00, // push service routine
            0x68, 0x00, 0x30, 0x00, 0x00, // push interrupt A
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeInitializeInterrupt
            0x6a, 0x00, // push ShareVector=false
            0x6a, 0x00, // push LevelSensitive mode
            0x6a, 0x05, // push IRQL
            0x6a, 0x35, // push vector => IRQ 5
            0x68, 0x33, 0x33, 0x00, 0x00, // push service context
            0x68, 0x44, 0x44, 0x00, 0x00, // push service routine
            0x68, 0x00, 0x31, 0x00, 0x00, // push interrupt B
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeInitializeInterrupt
            0x68, 0x00, 0x30, 0x00, 0x00, // push interrupt A
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeConnectInterrupt
            0xa3, 0x00, 0x20, 0x00, 0x00, // store first connect
            0x68, 0x00, 0x31, 0x00, 0x00, // push interrupt B
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeConnectInterrupt
            0xa3, 0x04, 0x20, 0x00, 0x00, // store collision
            0x68, 0x00, 0x30, 0x00, 0x00, // push interrupt A
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeDisconnectInterrupt
            0x68, 0x00, 0x31, 0x00, 0x00, // push interrupt B
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeConnectInterrupt
            0xa3, 0x08, 0x20, 0x00, 0x00, // store handoff result
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[109, 98, 100]);
        let mut machine = XboxMachine::from_xbe(image.clone()).unwrap();
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), 1);
        assert_eq!(machine.board.read32(0x2004), 0);
        assert_eq!(machine.board.read32(0x2008), 1);
        assert_eq!(machine.board.read8(0x3010), 0);
        assert_eq!(machine.board.read8(0x3110), 1);
        assert_eq!(machine.hal_interrupt_objects[5], 0x3100);
        assert_ne!(machine.hal_system_interrupts_enabled & (1 << 5), 0);
        assert_eq!(machine.hal_latched_interrupts & (1 << 5), 0);

        let state = machine.save_state().unwrap();
        let mut restored = XboxMachine::from_xbe(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.hal_interrupt_objects[5], 0x3100);
        assert_eq!(restored.board.read8(0x3110), 1);
        assert_ne!(restored.hal_system_interrupts_enabled & (1 << 5), 0);
        assert_eq!(restored.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(restored.cpu.halted());
    }

    #[test]
    fn kernel_apc_interrupt_and_vector_initializers_match_xbox_layouts() {
        let program = [
            0x68, 0x00, 0x20, 0x00, 0x00, // push IRQL output
            0x6a, 0x01, // push USB0 bus interrupt level
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // HalGetInterruptVector
            0xa3, 0x04, 0x20, 0x00, 0x00, // store vector
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeGetCurrentThread
            0xa3, 0x08, 0x20, 0x00, 0x00, // store thread
            0x68, 0x01, 0xef, 0xcd, 0xab, // push normal context
            0x6a, 0x01, // push UserMode
            0x68, 0x44, 0x44, 0x44, 0x44, // push normal routine
            0x68, 0x33, 0x33, 0x33, 0x33, // push rundown routine
            0x68, 0x22, 0x22, 0x22, 0x22, // push kernel routine
            0xa1, 0x08, 0x20, 0x00, 0x00, // mov eax,[thread]
            0x50, // push thread
            0x68, 0x00, 0x30, 0x00, 0x00, // push APC
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeInitializeApc
            0x68, 0x78, 0x56, 0x34, 0x12, // push ignored context
            0x6a, 0x01, // push UserMode
            0x6a, 0x00, // push NormalRoutine=NULL
            0x68, 0x33, 0x33, 0x33, 0x33, // push rundown routine
            0x68, 0x22, 0x22, 0x22, 0x22, // push kernel routine
            0xa1, 0x08, 0x20, 0x00, 0x00, // mov eax,[thread]
            0x50, // push thread
            0x68, 0x40, 0x30, 0x00, 0x00, // push APC
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeInitializeApc
            0x6a, 0x01, // push ShareVector=true
            0x6a, 0x01, // push Latched mode
            0x6a, 0x1a, // push IRQL=26
            0x6a, 0x31, // push vector
            0x68, 0x55, 0x55, 0x55, 0x55, // push service context
            0x68, 0x66, 0x66, 0x66, 0x66, // push service routine
            0x68, 0x80, 0x30, 0x00, 0x00, // push interrupt
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // KeInitializeInterrupt
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[44, 104, 105, 109]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.ram[0x2000], 26);
        assert_eq!(machine.board.read32(0x2004), 0x31);
        let thread = machine.board.read32(0x2008);
        assert_eq!(thread, machine.current_thread_address());

        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x3000..0x3002].try_into().unwrap()),
            0x12
        );
        assert_eq!(machine.board.ram[0x3002], 1);
        assert_eq!(machine.board.ram[0x3003], 0);
        assert_eq!(machine.board.read32(0x3004), thread);
        assert_eq!(machine.board.read32(0x3010), 0x2222_2222);
        assert_eq!(machine.board.read32(0x3014), 0x3333_3333);
        assert_eq!(machine.board.read32(0x3018), 0x4444_4444);
        assert_eq!(machine.board.read32(0x301c), 0xabcd_ef01);

        assert_eq!(machine.board.ram[0x3042], 0);
        assert_eq!(machine.board.read32(0x3058), 0);
        assert_eq!(machine.board.read32(0x305c), 0);

        assert_eq!(machine.board.read32(0x3080), 0x6666_6666);
        assert_eq!(machine.board.read32(0x3084), 0x5555_5555);
        assert_eq!(machine.board.read32(0x3088), 1);
        assert_eq!(machine.board.read32(0x308c), 26);
        assert_eq!(machine.board.ram[0x3090], 0);
        assert_eq!(machine.board.ram[0x3091], 0);
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x3092..0x3094].try_into().unwrap()),
            1
        );
        assert_eq!(machine.board.read32(0x3094), 0);
        assert_eq!(&machine.board.ram[0x3098..0x30f0], &[0; 0x58]);

        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_device_queue_thunks_preserve_busy_order_and_inserted_state() {
        let program = [
            0x68, 0x00, 0x30, 0x00, 0x00, // push device queue
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeInitializeDeviceQueue
            0x68, 0x20, 0x30, 0x00, 0x00, // push entry1
            0x68, 0x00, 0x30, 0x00, 0x00, // push queue
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeInsertDeviceQueue
            0xa3, 0x00, 0x20, 0x00, 0x00, 0x68, 0x40, 0x30, 0x00, 0x00, // push entry2
            0x68, 0x00, 0x30, 0x00, 0x00, // push queue
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeInsertDeviceQueue
            0xa3, 0x04, 0x20, 0x00, 0x00, 0x6a, 0x0a, // push SortKey=10
            0x68, 0x60, 0x30, 0x00, 0x00, // push entry3
            0x68, 0x00, 0x30, 0x00, 0x00, // push queue
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeInsertByKeyDeviceQueue
            0xa3, 0x08, 0x20, 0x00, 0x00, 0x6a, 0x05, // push SortKey=5
            0x68, 0x80, 0x30, 0x00, 0x00, // push entry4
            0x68, 0x00, 0x30, 0x00, 0x00, // push queue
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeInsertByKeyDeviceQueue
            0xa3, 0x0c, 0x20, 0x00, 0x00, 0x6a, 0x06, // push SortKey=6
            0x68, 0x00, 0x30, 0x00, 0x00, // push queue
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // KeRemoveByKeyDeviceQueue
            0xa3, 0x10, 0x20, 0x00, 0x00, 0x68, 0x80, 0x30, 0x00, 0x00, // push entry4
            0x68, 0x00, 0x30, 0x00, 0x00, // push queue
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // KeRemoveEntryDeviceQueue
            0xa3, 0x14, 0x20, 0x00, 0x00, 0x68, 0x00, 0x30, 0x00, 0x00, // push queue
            0xff, 0x15, 0x14, 0x12, 0x01, 0x00, // KeRemoveDeviceQueue
            0xa3, 0x18, 0x20, 0x00, 0x00, 0x68, 0x00, 0x30, 0x00, 0x00, // push queue
            0xff, 0x15, 0x14, 0x12, 0x01, 0x00, // KeRemoveDeviceQueue
            0xa3, 0x1c, 0x20, 0x00, 0x00, 0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[106, 115, 114, 133, 135, 134]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, 0),
            (0x2004, 1),
            (0x2008, 1),
            (0x200c, 1),
            (0x2010, 0x3060),
            (0x2014, 1),
            (0x2018, 0x3040),
            (0x201c, 0),
        ] {
            assert_eq!(machine.board.read32(offset), expected);
        }
        assert_eq!(machine.board.ram[0x3004], 0);
        assert_eq!(machine.board.read32(0x3008), 0x3008);
        assert_eq!(machine.board.read32(0x300c), 0x3008);
        assert_eq!(machine.board.ram[0x302c], 0);
        assert_eq!(machine.board.ram[0x304c], 0);
        assert_eq!(machine.board.ram[0x306c], 0);
        assert_eq!(machine.board.ram[0x308c], 0);
        assert_eq!(machine.board.read32(0x3068), 10);
        assert_eq!(machine.board.read32(0x3088), 5);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_dispatcher_initializers_match_xbox_object_layouts() {
        let program = [
            0x68, 0x00, 0x30, 0x00, 0x00, // push device queue
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeInitializeDeviceQueue
            0x68, 0xef, 0xbe, 0xad, 0xde, // push context
            0x68, 0x78, 0x56, 0x34, 0x12, // push routine
            0x68, 0x20, 0x30, 0x00, 0x00, // push DPC
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeInitializeDpc
            0x6a, 0x00, // push InitialOwner=false
            0x68, 0x40, 0x30, 0x00, 0x00, // push mutant
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeInitializeMutant
            0x6a, 0x01, // push InitialOwner=true
            0x68, 0x60, 0x30, 0x00, 0x00, // push mutant
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeInitializeMutant
            0x6a, 0x00, // push Count=0
            0x68, 0x80, 0x30, 0x00, 0x00, // push queue
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // KeInitializeQueue
            0x6a, 0x00, // push NotificationTimer
            0x68, 0xc0, 0x30, 0x00, 0x00, // push timer
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // KeInitializeTimerEx
            0x6a, 0x01, // push SynchronizationTimer
            0x68, 0xf0, 0x30, 0x00, 0x00, // push timer
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // KeInitializeTimerEx
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[106, 107, 110, 111, 113]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x3000..0x3002].try_into().unwrap()),
            0x14
        );
        assert_eq!(machine.board.ram[0x3002], 0x10);
        assert_eq!(machine.board.ram[0x3004], 0);
        assert_eq!(machine.board.read32(0x3008), 0x3008);
        assert_eq!(machine.board.read32(0x300c), 0x3008);

        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x3020..0x3022].try_into().unwrap()),
            0x13
        );
        assert_eq!(machine.board.ram[0x3022], 0);
        assert_eq!(machine.board.read32(0x302c), 0x1234_5678);
        assert_eq!(machine.board.read32(0x3030), 0xdead_beef);

        assert_eq!(machine.board.ram[0x3040], 2);
        assert_eq!(machine.board.ram[0x3042], 8);
        assert_eq!(machine.board.read32(0x3044), 1);
        assert_eq!(machine.board.read32(0x3048), 0x3048);
        assert_eq!(machine.board.read32(0x304c), 0x3048);
        assert_eq!(machine.board.read32(0x3058), 0);

        let thread = machine.current_thread_address();
        assert_eq!(machine.board.ram[0x3060], 2);
        assert_eq!(machine.board.ram[0x3062], 8);
        assert_eq!(machine.board.read32(0x3064), 0);
        assert_eq!(machine.board.read32(0x3078), thread);
        let mutant_list = thread.wrapping_add(0x10);
        assert_eq!(machine.board.read32(mutant_list), 0x3070);
        assert_eq!(machine.board.read32(mutant_list.wrapping_add(4)), 0x3070);
        assert_eq!(machine.board.read32(0x3070), mutant_list);
        assert_eq!(machine.board.read32(0x3074), mutant_list);

        assert_eq!(machine.board.ram[0x3080], 4);
        assert_eq!(machine.board.ram[0x3082], 10);
        assert_eq!(machine.board.read32(0x3088), 0x3088);
        assert_eq!(machine.board.read32(0x308c), 0x3088);
        assert_eq!(machine.board.read32(0x3090), 0x3090);
        assert_eq!(machine.board.read32(0x3094), 0x3090);
        assert_eq!(machine.board.read32(0x3098), 0);
        assert_eq!(machine.board.read32(0x309c), 1);
        assert_eq!(machine.board.read32(0x30a0), 0x30a0);
        assert_eq!(machine.board.read32(0x30a4), 0x30a0);

        for (timer, object_type) in [(0x30c0, 8u8), (0x30f0, 9u8)] {
            assert_eq!(machine.board.ram[timer], object_type);
            assert_eq!(machine.board.ram[timer + 2], 10);
            assert_eq!(machine.board.ram[timer + 3], 0);
            assert_eq!(machine.board.read32((timer + 8) as u32), (timer + 8) as u32);
            assert_eq!(
                machine.board.read32((timer + 12) as u32),
                (timer + 8) as u32
            );
            assert_eq!(&machine.board.ram[timer + 0x10..timer + 0x28], &[0; 0x18]);
        }

        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_read_write_lock_and_semaphore_thunks_follow_uncontended_state_rules() {
        let program = [
            0x68, 0x00, 0x30, 0x00, 0x00, // push lock
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // ExInitializeReadWriteLock
            0x68, 0x00, 0x30, 0x00, 0x00, // push lock
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // ExAcquireReadWriteLockShared
            0xa1, 0x00, 0x30, 0x00, 0x00, // mov eax,[LockCount]
            0xa3, 0x00, 0x20, 0x00, 0x00, 0xa1, 0x0c, 0x30, 0x00,
            0x00, // mov eax,[ReadersEntryCount]
            0xa3, 0x04, 0x20, 0x00, 0x00, 0x68, 0x00, 0x30, 0x00, 0x00, // push lock
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // ExAcquireReadWriteLockShared
            0xa1, 0x00, 0x30, 0x00, 0x00, 0xa3, 0x08, 0x20, 0x00, 0x00, 0xa1, 0x0c, 0x30, 0x00,
            0x00, 0xa3, 0x0c, 0x20, 0x00, 0x00, 0x68, 0x00, 0x30, 0x00, 0x00, // push lock
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // ExReleaseReadWriteLock
            0xa1, 0x00, 0x30, 0x00, 0x00, 0xa3, 0x10, 0x20, 0x00, 0x00, 0xa1, 0x0c, 0x30, 0x00,
            0x00, 0xa3, 0x14, 0x20, 0x00, 0x00, 0x68, 0x00, 0x30, 0x00, 0x00, // push lock
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // ExReleaseReadWriteLock
            0xa1, 0x00, 0x30, 0x00, 0x00, 0xa3, 0x18, 0x20, 0x00, 0x00, 0xa1, 0x0c, 0x30, 0x00,
            0x00, 0xa3, 0x1c, 0x20, 0x00, 0x00, 0x68, 0x00, 0x30, 0x00, 0x00, // push lock
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // ExAcquireReadWriteLockExclusive
            0xa1, 0x00, 0x30, 0x00, 0x00, 0xa3, 0x20, 0x20, 0x00, 0x00, 0x68, 0x00, 0x30, 0x00,
            0x00, // push lock
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // ExReleaseReadWriteLock
            0xa1, 0x00, 0x30, 0x00, 0x00, 0xa3, 0x24, 0x20, 0x00, 0x00, 0x6a,
            0x03, // push Limit
            0x6a, 0x01, // push Count
            0x68, 0x60, 0x30, 0x00, 0x00, // push semaphore
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // KeInitializeSemaphore
            0x6a, 0x00, // push Wait=false
            0x6a, 0x01, // push Adjustment
            0x6a, 0x01, // push Increment
            0x68, 0x60, 0x30, 0x00, 0x00, // push semaphore
            0xff, 0x15, 0x14, 0x12, 0x01, 0x00, // KeReleaseSemaphore
            0xa3, 0x28, 0x20, 0x00, 0x00, // store initial state
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[18, 13, 28, 12, 112, 132]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, 0),
            (0x2004, 1),
            (0x2008, 1),
            (0x200c, 2),
            (0x2010, 0),
            (0x2014, 1),
            (0x2018, u32::MAX),
            (0x201c, 0),
            (0x2020, 0),
            (0x2024, u32::MAX),
            (0x2028, 1),
        ] {
            assert_eq!(machine.board.read32(offset), expected);
        }
        assert_eq!(machine.board.ram[0x3060], 5);
        assert_eq!(machine.board.ram[0x3062], 5);
        assert_eq!(machine.board.read32(0x3064), 2);
        assert_eq!(machine.board.read32(0x3068), 0x3068);
        assert_eq!(machine.board.read32(0x306c), 0x3068);
        assert_eq!(machine.board.read32(0x3070), 3);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn io_irp_thunks_initialize_allocate_complete_free_and_reuse_packets() {
        let program = [
            0x6a, 0x03, // push StackSize
            0x68, 0xac, 0x00, 0x00, 0x00, // push PacketSize
            0x68, 0x00, 0x32, 0x00, 0x00, // push caller IRP
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // IoInitializeIrp
            0x68, 0x00, 0x32, 0x00, 0x00, // push IRP
            0x6a, 0x00, // push DeviceObject
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // IoInvalidDeviceRequest
            0xa3, 0x08, 0x20, 0x00, 0x00, // store returned status
            0x6a, 0x02, // push StackSize
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // IoAllocateIrp
            0xa3, 0x00, 0x20, 0x00, 0x00, // store first allocation
            0x50, // push eax / IRP
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // IoFreeIrp
            0x6a, 0x02, // push StackSize
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // IoAllocateIrp
            0xa3, 0x04, 0x20, 0x00, 0x00, // store reused allocation
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[73, 74, 59, 72]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read16(0x3200), XBOX_IO_TYPE_IRP);
        assert_eq!(machine.board.read16(0x3202), 0x00ac);
        assert_eq!(machine.board.read32(0x3208), 0x3208);
        assert_eq!(machine.board.read32(0x320c), 0x3208);
        assert_eq!(machine.board.read32(0x3210), STATUS_INVALID_DEVICE_REQUEST);
        assert_eq!(machine.board.ram[0x3218], 3);
        assert_eq!(machine.board.ram[0x3219], 4);
        assert_eq!(machine.board.read32(0x3258), 0x32ac);
        assert_eq!(machine.board.read32(0x2008), STATUS_INVALID_DEVICE_REQUEST);

        let first = machine.board.read32(0x2000);
        let second = machine.board.read32(0x2004);
        assert_ne!(first, 0);
        assert_eq!(second, first);
        assert_eq!(machine.board.read16(second), XBOX_IO_TYPE_IRP);
        assert_eq!(machine.board.read16(second.wrapping_add(2)), 0x0094);
        assert_eq!(machine.board.read8(second.wrapping_add(0x18)), 2);
        assert_eq!(machine.board.read8(second.wrapping_add(0x19)), 3);
        assert_eq!(
            machine.board.read32(second.wrapping_add(0x58)),
            second.wrapping_add(0x94)
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn io_share_access_thunks_enforce_open_compatibility_and_update_counts() {
        let program = [
            0x68, 0x00, 0x31, 0x00, 0x00, // push ShareAccess
            0x68, 0x00, 0x30, 0x00, 0x00, // push FileObject A
            0x6a, 0x01, // push FILE_SHARE_READ
            0x6a, 0x01, // push FILE_READ_DATA
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // IoSetShareAccess
            0x6a, 0x01, // push Update=true
            0x68, 0x00, 0x31, 0x00, 0x00, // push ShareAccess
            0x68, 0x40, 0x30, 0x00, 0x00, // push FileObject B
            0x6a, 0x03, // push FILE_SHARE_READ|FILE_SHARE_WRITE
            0x6a, 0x02, // push FILE_WRITE_DATA
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // IoCheckShareAccess
            0xa3, 0x00, 0x20, 0x00, 0x00, // store violation
            0x68, 0x00, 0x31, 0x00, 0x00, // push ShareAccess
            0x68, 0x00, 0x30, 0x00, 0x00, // push FileObject A
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // IoRemoveShareAccess
            0x6a, 0x01, // push Update=true
            0x68, 0x00, 0x31, 0x00, 0x00, // push ShareAccess
            0x68, 0x40, 0x30, 0x00, 0x00, // push FileObject B
            0x6a, 0x02, // push FILE_SHARE_WRITE
            0x6a, 0x02, // push FILE_WRITE_DATA
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // IoCheckShareAccess
            0xa3, 0x04, 0x20, 0x00, 0x00, // store success
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[80, 63, 78]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), STATUS_SHARING_VIOLATION);
        assert_eq!(machine.board.read32(0x2004), STATUS_SUCCESS);
        assert_eq!(machine.board.ram[0x3002], 0x12);
        assert_eq!(machine.board.ram[0x3042], 0x24);
        assert_eq!(&machine.board.ram[0x3100..0x3107], &[1, 0, 1, 0, 0, 1, 0]);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn filesystem_cache_thunks_validate_size_and_persist_kernel_state() {
        let program = [
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // FscGetCacheSize
            0xa3, 0x00, 0x20, 0x00, 0x00, // store default
            0x6a, 0x20, // push 32 pages
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // FscSetCacheSize
            0xa3, 0x04, 0x20, 0x00, 0x00, // store success
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // FscGetCacheSize
            0xa3, 0x08, 0x20, 0x00, 0x00, // store changed size
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // FscInvalidateIdleBlocks
            0x68, 0x01, 0x08, 0x00, 0x00, // push 2049 pages
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // FscSetCacheSize
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store invalid status
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // FscGetCacheSize
            0xa3, 0x10, 0x20, 0x00, 0x00, // store unchanged size
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[35, 37, 36]);
        let mut machine = XboxMachine::from_xbe(image.clone()).unwrap();
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), 16);
        assert_eq!(machine.board.read32(0x2004), STATUS_SUCCESS);
        assert_eq!(machine.board.read32(0x2008), 32);
        assert_eq!(machine.board.read32(0x200c), STATUS_INVALID_PARAMETER);
        assert_eq!(machine.board.read32(0x2010), 32);
        assert_eq!(machine.fsc_cache_pages, 32);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);

        let state = machine.save_state().unwrap();
        let mut restored = XboxMachine::from_xbe(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.fsc_cache_pages, 32);
        assert!(restored.powered);
    }

    #[test]
    fn kernel_interlocked_list_thunks_preserve_links_depth_and_sequence() {
        let program = [
            0xb9, 0x00, 0x30, 0x00, 0x00, // mov ecx,list head
            0xba, 0x10, 0x30, 0x00, 0x00, // mov edx,entry1
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // ExfInterlockedInsertHeadList
            0xa3, 0x00, 0x20, 0x00, 0x00, // store old first
            0xb9, 0x00, 0x30, 0x00, 0x00, // mov ecx,list head
            0xba, 0x20, 0x30, 0x00, 0x00, // mov edx,entry2
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // ExfInterlockedInsertTailList
            0xa3, 0x04, 0x20, 0x00, 0x00, // store old last
            0xb9, 0x00, 0x30, 0x00, 0x00, // mov ecx,list head
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // ExfInterlockedRemoveHeadList
            0xa3, 0x08, 0x20, 0x00, 0x00, // store removed entry
            0xb9, 0x40, 0x30, 0x00, 0x00, // mov ecx,slist head
            0xba, 0x50, 0x30, 0x00, 0x00, // mov edx,slist entry1
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // InterlockedPushEntrySList
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store old first
            0xb9, 0x40, 0x30, 0x00, 0x00, // mov ecx,slist head
            0xba, 0x60, 0x30, 0x00, 0x00, // mov edx,slist entry2
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // InterlockedPushEntrySList
            0xa3, 0x10, 0x20, 0x00, 0x00, // store old first
            0xb9, 0x40, 0x30, 0x00, 0x00, // mov ecx,slist head
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // InterlockedPopEntrySList
            0xa3, 0x14, 0x20, 0x00, 0x00, // store popped entry
            0xb9, 0x40, 0x30, 0x00, 0x00, // mov ecx,slist head
            0xff, 0x15, 0x14, 0x12, 0x01, 0x00, // InterlockedFlushSList
            0xa3, 0x18, 0x20, 0x00, 0x00, // store flushed first
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[32, 33, 34, 58, 57, 56]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.write32(0x3000, 0x3000);
        machine.board.write32(0x3004, 0x3000);
        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, 0),
            (0x2004, 0x3010),
            (0x2008, 0x3010),
            (0x200c, 0),
            (0x2010, 0x3050),
            (0x2014, 0x3060),
            (0x2018, 0x3050),
        ] {
            assert_eq!(machine.board.read32(offset), expected);
        }
        assert_eq!(machine.board.read32(0x3000), 0x3020);
        assert_eq!(machine.board.read32(0x3004), 0x3020);
        assert_eq!(machine.board.read32(0x3020), 0x3000);
        assert_eq!(machine.board.read32(0x3024), 0x3000);
        assert_eq!(machine.board.read32(0x3040), 0);
        assert_eq!(machine.board.read32(0x3044), 0x0002_0000);
        assert_eq!(machine.board.read32(0x3050), 0);
        assert_eq!(machine.board.read32(0x3060), 0x3050);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_nonvolatile_setting_thunks_share_persistent_eeprom_state() {
        let program = [
            0x68, 0x04, 0x20, 0x00, 0x00, // push ResultLength
            0x6a, 0x08, // push ValueLength
            0x68, 0x00, 0x30, 0x00, 0x00, // push Value
            0x68, 0x00, 0x20, 0x00, 0x00, // push Type
            0x6a, 0x07, // push XC_LANGUAGE
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // ExQueryNonVolatileSetting
            0xa3, 0x08, 0x20, 0x00, 0x00, // store status
            0x6a, 0x04, // push ValueLength
            0x68, 0x10, 0x30, 0x00, 0x00, // push Value
            0x6a, 0x04, // push Type
            0x6a, 0x07, // push XC_LANGUAGE
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // ExSaveNonVolatileSetting
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store status
            0x68, 0x14, 0x20, 0x00, 0x00, // push ResultLength
            0x6a, 0x04, // push ValueLength
            0x68, 0x20, 0x30, 0x00, 0x00, // push Value
            0x68, 0x10, 0x20, 0x00, 0x00, // push Type
            0x6a, 0x07, // push XC_LANGUAGE
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // ExQueryNonVolatileSetting
            0xa3, 0x18, 0x20, 0x00, 0x00, // store status
            0x68, 0x1c, 0x20, 0x00, 0x00, // push ResultLength
            0x6a, 0x04, // push too-small ValueLength
            0x68, 0x30, 0x30, 0x00, 0x00, // push Value
            0x68, 0x24, 0x20, 0x00, 0x00, // push Type
            0x68, 0xff, 0x00, 0x00, 0x00, // push XC_MAX_OS
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // ExQueryNonVolatileSetting
            0xa3, 0x20, 0x20, 0x00, 0x00, // store status
            0x6a, 0x04, // push ValueLength
            0x68, 0x40, 0x30, 0x00, 0x00, // push Value
            0x6a, 0x03, // push Type
            0x68, 0x00, 0x01, 0x00, 0x00, // push XC_FACTORY_SERIAL_NUMBER
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // ExSaveNonVolatileSetting
            0xa3, 0x28, 0x20, 0x00, 0x00, // store status
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[24, 29]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.smbus.eeprom[0x90..0x94].copy_from_slice(&1u32.to_le_bytes());
        machine.board.ram[0x3000..0x3008].fill(0xaa);
        machine.board.ram[0x3010..0x3014].copy_from_slice(&7u32.to_le_bytes());
        machine.board.ram[0x3040..0x3044].copy_from_slice(b"NOPE");
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), REG_DWORD);
        assert_eq!(machine.board.read32(0x2004), 4);
        assert_eq!(machine.board.read32(0x2008), STATUS_SUCCESS);
        assert_eq!(machine.board.read32(0x3000), 1);
        assert_eq!(&machine.board.ram[0x3004..0x3008], &[0; 4]);
        assert_eq!(machine.board.read32(0x200c), STATUS_SUCCESS);
        assert_eq!(machine.board.read32(0x2010), REG_DWORD);
        assert_eq!(machine.board.read32(0x2014), 4);
        assert_eq!(machine.board.read32(0x2018), STATUS_SUCCESS);
        assert_eq!(machine.board.read32(0x3020), 7);
        assert_eq!(machine.board.read32(0x201c), 0x60);
        assert_eq!(machine.board.read32(0x2020), STATUS_BUFFER_TOO_SMALL);
        assert_eq!(machine.board.read32(0x2028), STATUS_OBJECT_NAME_NOT_FOUND);
        assert_eq!(
            &machine.board.smbus.eeprom[0x30..0x34],
            &eeprom_section_crc(&machine.board.smbus.eeprom[0x34..0x60])
        );
        assert_eq!(
            &machine.board.smbus.eeprom[0x60..0x64],
            &eeprom_section_crc(&machine.board.smbus.eeprom[0x64..0xc0])
        );

        let mut persisted = [0; EEPROM_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut persisted)
            .unwrap();
        assert_eq!(
            u32::from_le_bytes(persisted[0x90..0x94].try_into().unwrap()),
            7
        );
        assert_eq!(&persisted[0x34..0x40], b"000000000001");
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_ex_interlocked_64bit_thunks_follow_xbox_abi_and_atomic_semantics() {
        let program = [
            0x68, 0x70, 0x30, 0x00, 0x00, // push lock
            0x68, 0x02, 0x00, 0x00, 0x00, // push increment high
            0x68, 0xfe, 0xff, 0xff, 0xff, // push increment low
            0x68, 0x00, 0x30, 0x00, 0x00, // push addend
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // ExInterlockedAddLargeInteger
            0xa3, 0x00, 0x20, 0x00, 0x00, // store old low
            0x89, 0x15, 0x04, 0x20, 0x00, 0x00, // store old high
            0xb9, 0x10, 0x30, 0x00, 0x00, // mov ecx,addend
            0xba, 0x07, 0x00, 0x00, 0x00, // mov edx,7
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // ExInterlockedAddLargeStatistic
            0xb9, 0x20, 0x30, 0x00, 0x00, // mov ecx,destination
            0xba, 0x30, 0x30, 0x00, 0x00, // mov edx,exchange
            0x68, 0x40, 0x30, 0x00, 0x00, // push comparand
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // ExInterlockedCompareExchange64
            0xa3, 0x08, 0x20, 0x00, 0x00, // store old low
            0x89, 0x15, 0x0c, 0x20, 0x00, 0x00, // store old high
            0xb9, 0x20, 0x30, 0x00, 0x00, // mov ecx,destination
            0xba, 0x50, 0x30, 0x00, 0x00, // mov edx,second exchange
            0x68, 0x60, 0x30, 0x00, 0x00, // push mismatching comparand
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // ExInterlockedCompareExchange64
            0xa3, 0x10, 0x20, 0x00, 0x00, // store old low
            0x89, 0x15, 0x14, 0x20, 0x00, 0x00, // store old high
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[19, 20, 21]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();

        machine.board.ram[0x3000..0x3008].copy_from_slice(&0x0000_0001_0000_0005u64.to_le_bytes());
        machine.board.ram[0x3010..0x3018].copy_from_slice(&0x0000_0000_ffff_fffcu64.to_le_bytes());
        machine.board.ram[0x3020..0x3028].copy_from_slice(&0x1111_2222_3333_4444u64.to_le_bytes());
        machine.board.ram[0x3030..0x3038].copy_from_slice(&0xaaaa_bbbb_cccc_ddddu64.to_le_bytes());
        machine.board.ram[0x3040..0x3048].copy_from_slice(&0x1111_2222_3333_4444u64.to_le_bytes());
        machine.board.ram[0x3050..0x3058].copy_from_slice(&0x5555_6666_7777_8888u64.to_le_bytes());
        machine.board.ram[0x3060..0x3068].copy_from_slice(&0x9999_aaaa_bbbb_ccccu64.to_le_bytes());

        machine.run_frame(&InputState::default());

        assert_eq!(
            u64::from_le_bytes(machine.board.ram[0x3000..0x3008].try_into().unwrap()),
            0x0000_0004_0000_0003
        );
        assert_eq!(
            u64::from_le_bytes(machine.board.ram[0x3010..0x3018].try_into().unwrap()),
            0x0000_0001_0000_0003
        );
        assert_eq!(
            u64::from_le_bytes(machine.board.ram[0x2000..0x2008].try_into().unwrap()),
            0x0000_0001_0000_0005
        );
        assert_eq!(
            u64::from_le_bytes(machine.board.ram[0x2008..0x2010].try_into().unwrap()),
            0x1111_2222_3333_4444
        );
        assert_eq!(
            u64::from_le_bytes(machine.board.ram[0x2010..0x2018].try_into().unwrap()),
            0xaaaa_bbbb_cccc_dddd
        );
        assert_eq!(
            u64::from_le_bytes(machine.board.ram[0x3020..0x3028].try_into().unwrap()),
            0xaaaa_bbbb_cccc_dddd
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_interlocked_fastcalls_follow_xbox_register_abi_and_return_values() {
        let program = [
            0xb9, 0x00, 0x30, 0x00, 0x00, // mov ecx,0x3000
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // InterlockedIncrement
            0xa3, 0x00, 0x20, 0x00, 0x00, // store result
            0xb9, 0x00, 0x30, 0x00, 0x00, // mov ecx,0x3000
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // InterlockedDecrement
            0xa3, 0x04, 0x20, 0x00, 0x00, // store result
            0xb9, 0x00, 0x30, 0x00, 0x00, // mov ecx,0x3000
            0xba, 0x0a, 0x00, 0x00, 0x00, // mov edx,10
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // InterlockedExchange
            0xa3, 0x08, 0x20, 0x00, 0x00, // store old
            0xb9, 0x00, 0x30, 0x00, 0x00, // mov ecx,0x3000
            0xba, 0x07, 0x00, 0x00, 0x00, // mov edx,7
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // InterlockedExchangeAdd
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store old
            0x6a, 0x11, // push comparand 17
            0xb9, 0x00, 0x30, 0x00, 0x00, // mov ecx,0x3000
            0xba, 0x63, 0x00, 0x00, 0x00, // mov edx,99
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // InterlockedCompareExchange
            0xa3, 0x10, 0x20, 0x00, 0x00, // store old
            0x6a, 0x01, // push mismatching comparand
            0xb9, 0x00, 0x30, 0x00, 0x00, // mov ecx,0x3000
            0xba, 0x37, 0x00, 0x00, 0x00, // mov edx,55
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // InterlockedCompareExchange
            0xa3, 0x14, 0x20, 0x00, 0x00, // store old
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[53, 52, 54, 55, 51]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.ram[0x3000..0x3004].copy_from_slice(&5u32.to_le_bytes());
        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, 6),
            (0x2004, 5),
            (0x2008, 5),
            (0x200c, 10),
            (0x2010, 17),
            (0x2014, 99),
        ] {
            assert_eq!(
                u32::from_le_bytes(machine.board.ram[offset..offset + 4].try_into().unwrap()),
                expected
            );
        }
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x3000..0x3004].try_into().unwrap()),
            99
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_irql_thunks_raise_query_lower_and_preserve_fastcall_stack() {
        let program = [
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeGetCurrentIrql
            0xa3, 0x00, 0x20, 0x00, 0x00, // store initial
            0xb9, 0x02, 0x00, 0x00, 0x00, // mov ecx,DISPATCH_LEVEL
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KfRaiseIrql
            0xa3, 0x04, 0x20, 0x00, 0x00, // store old IRQL
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeGetCurrentIrql
            0xa3, 0x08, 0x20, 0x00, 0x00, // store raised
            0xb9, 0x00, 0x00, 0x00, 0x00, // mov ecx,PASSIVE_LEVEL
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KfLowerIrql
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeGetCurrentIrql
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store lowered
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[103, 160, 161]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());
        for (offset, expected) in [(0x2000, 0), (0x2004, 0), (0x2008, 2), (0x200c, 0)] {
            assert_eq!(
                u32::from_le_bytes(machine.board.ram[offset..offset + 4].try_into().unwrap()),
                expected
            );
        }
        assert_eq!(machine.current_irql, 0);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_thread_control_mutant_event_and_queue_rundown_thunks_update_guest_state() {
        let program = [
            0x6a, 0x01, // push InitialOwner=true
            0x68, 0x00, 0x30, 0x00, 0x00, // push Mutant
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeInitializeMutant
            0x6a, 0x00, // push Wait=false
            0x6a, 0x01, // push Abandoned=true
            0x6a, 0x00, // push Increment
            0x68, 0x00, 0x30, 0x00, 0x00, // push Mutant
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeReleaseMutant
            0xa3, 0x00, 0x20, 0x00, 0x00, // store old state
            0x6a, 0x00, // push Count
            0x68, 0x40, 0x30, 0x00, 0x00, // push Queue
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeInitializeQueue
            0x68, 0x80, 0x30, 0x00, 0x00, // push Entry1
            0x68, 0x40, 0x30, 0x00, 0x00, // push Queue
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // KeInsertQueue
            0x68, 0x90, 0x30, 0x00, 0x00, // push Entry2
            0x68, 0x40, 0x30, 0x00, 0x00, // push Queue
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // KeInsertQueue
            0x68, 0x40, 0x30, 0x00, 0x00, // push Queue
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // KeRundownQueue
            0xa3, 0x04, 0x20, 0x00, 0x00, // store first entry
            0x6a, 0x00, // push Thread output=null
            0x68, 0xc0, 0x30, 0x00, 0x00, // push Event
            0xff, 0x15, 0x14, 0x12, 0x01, 0x00, // KeSetEventBoostPriority
            0x6a, 0x0c, // push BasePriority=12
            0x68, 0xe0, 0x30, 0x00, 0x00, // push Process
            0xff, 0x15, 0x18, 0x12, 0x01, 0x00, // KeSetPriorityProcess
            0xa3, 0x08, 0x20, 0x00, 0x00, // store priority
            0x68, 0x00, 0x31, 0x00, 0x00, // push Thread
            0xff, 0x15, 0x1c, 0x12, 0x01, 0x00, // KeSuspendThread
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store old count 0
            0x68, 0x00, 0x31, 0x00, 0x00, // push Thread
            0xff, 0x15, 0x1c, 0x12, 0x01, 0x00, // KeSuspendThread
            0xa3, 0x10, 0x20, 0x00, 0x00, // store old count 1
            0x68, 0x00, 0x31, 0x00, 0x00, // push Thread
            0xff, 0x15, 0x20, 0x12, 0x01, 0x00, // KeResumeThread
            0xa3, 0x14, 0x20, 0x00, 0x00, // store old count 2
            0x6a, 0x01, // push UserMode
            0xff, 0x15, 0x24, 0x12, 0x01, 0x00, // KeTestAlertThread
            0xa3, 0x18, 0x20, 0x00, 0x00, // store first alert result
            0x6a, 0x01, // push UserMode
            0xff, 0x15, 0x24, 0x12, 0x01, 0x00, // KeTestAlertThread
            0xa3, 0x1c, 0x20, 0x00, 0x00, // store second alert result
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(
            &program,
            &[110, 131, 111, 117, 141, 146, 147, 152, 140, 155],
        );
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.initialize_event(0x30c0, 0, 0);
        machine.board.write8(0x3100 + 0x4b, 1);
        machine
            .board
            .write8(machine.current_thread_address().wrapping_add(0x2e), 1);
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), 0);
        assert_eq!(machine.board.read32(0x3004), 1);
        assert_eq!(machine.board.read32(0x3018), 0);
        assert_eq!(machine.board.read8(0x301c), 1);

        assert_eq!(machine.board.read32(0x2004), 0x3080);
        assert_eq!(machine.board.read32(0x3080), 0x3090);
        assert_eq!(machine.board.read32(0x3084), 0x3090);
        assert_eq!(machine.board.read32(0x3090), 0x3080);
        assert_eq!(machine.board.read32(0x3094), 0x3080);

        assert_eq!(machine.board.read32(0x30c4), 1);
        assert_eq!(machine.board.read32(0x2008), 12);
        assert_eq!(machine.board.read8(0x30f8), 12);
        assert_eq!(machine.board.read32(0x200c), 0);
        assert_eq!(machine.board.read32(0x2010), 1);
        assert_eq!(machine.board.read32(0x2014), 2);
        assert_eq!(machine.board.read8(0x3100 + 0x75), 1);
        assert_eq!(machine.board.read32(0x2018), 1);
        assert_eq!(machine.board.read32(0x201c), 0);
        assert_eq!(
            machine
                .board
                .read8(machine.current_thread_address().wrapping_add(0x2e)),
            0
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_thread_priority_and_boost_controls_update_kthread_fields() {
        let program = [
            0x6a, 0x01, // push Disable=true
            0x68, 0x00, 0x30, 0x00, 0x00, // push thread
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeSetDisableBoostThread
            0xa3, 0x00, 0x20, 0x00, 0x00, // store previous DisableBoost
            0x6a, 0x00, // push requested priority 0
            0x68, 0x00, 0x30, 0x00, 0x00, // push thread
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeSetPriorityThread
            0xa3, 0x04, 0x20, 0x00, 0x00, // store old priority
            0x68, 0x00, 0x30, 0x00, 0x00, // push thread
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeQueryBasePriorityThread
            0xa3, 0x08, 0x20, 0x00, 0x00, // store current priority
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[144, 148, 124]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.write8(0x3000 + 0x32, 5);
        machine.board.write8(0x3000 + 0x70, 8);
        machine.board.write8(0x3000 + 0x72, 3);
        machine.board.write8(0x3000 + 0x73, 0);
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), 0);
        assert_eq!(machine.board.read32(0x2004), 5);
        assert_eq!(machine.board.read32(0x2008), 1);
        assert_eq!(machine.board.read8(0x3000 + 0x32), 1);
        assert_eq!(machine.board.read8(0x3000 + 0x70), 8);
        assert_eq!(machine.board.read8(0x3000 + 0x72), 0);
        assert_eq!(machine.board.read8(0x3000 + 0x73), 1);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_dpc_queue_and_base_priority_thunks_preserve_guest_visible_state() {
        let program = [
            0x68, 0x01, 0x00, 0xaa, 0xaa, // push DeferredContext
            0x68, 0x00, 0x10, 0x00, 0x00, // push DeferredRoutine
            0x68, 0x00, 0x30, 0x00, 0x00, // push DPC1
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeInitializeDpc
            0x68, 0x02, 0x00, 0xbb, 0xbb, // push DeferredContext
            0x68, 0x00, 0x11, 0x00, 0x00, // push DeferredRoutine
            0x68, 0x40, 0x30, 0x00, 0x00, // push DPC2
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeInitializeDpc
            0x68, 0x22, 0x22, 0x00, 0x00, // push SystemArgument2
            0x68, 0x11, 0x11, 0x00, 0x00, // push SystemArgument1
            0x68, 0x00, 0x30, 0x00, 0x00, // push DPC1
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeInsertQueueDpc
            0xa3, 0x00, 0x20, 0x00, 0x00, // store inserted
            0x68, 0x44, 0x44, 0x00, 0x00, // push replacement SystemArgument2
            0x68, 0x33, 0x33, 0x00, 0x00, // push replacement SystemArgument1
            0x68, 0x00, 0x30, 0x00, 0x00, // push DPC1 again
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeInsertQueueDpc
            0xa3, 0x04, 0x20, 0x00, 0x00, // store duplicate result
            0x68, 0x66, 0x66, 0x00, 0x00, // push SystemArgument2
            0x68, 0x55, 0x55, 0x00, 0x00, // push SystemArgument1
            0x68, 0x40, 0x30, 0x00, 0x00, // push DPC2
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeInsertQueueDpc
            0xa3, 0x08, 0x20, 0x00, 0x00, // store inserted
            0x68, 0x00, 0x30, 0x00, 0x00, // push DPC1
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeRemoveQueueDpc
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store removed
            0x68, 0x00, 0x30, 0x00, 0x00, // push DPC1 again
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeRemoveQueueDpc
            0xa3, 0x10, 0x20, 0x00, 0x00, // store second removal
            0x6a, 0xfd, // push Priority=-3
            0x68, 0x00, 0x32, 0x00, 0x00, // push Thread
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // KeSetBasePriorityThread
            0xa3, 0x14, 0x20, 0x00, 0x00, // store priority
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[107, 119, 137, 143]);
        let mut machine = XboxMachine::from_xbe(image.clone()).unwrap();
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), 1);
        assert_eq!(machine.board.read32(0x2004), 0);
        assert_eq!(machine.board.read32(0x2008), 1);
        assert_eq!(machine.board.read32(0x200c), 1);
        assert_eq!(machine.board.read32(0x2010), 0);
        assert_eq!(machine.board.read32(0x2014), 0xffff_fffd);
        assert_eq!(machine.board.read8(0x3002), 0);
        assert_eq!(machine.board.read8(0x3042), 1);
        assert_eq!(machine.board.read32(0x3014), 0x1111);
        assert_eq!(machine.board.read32(0x3018), 0x2222);
        assert_eq!(machine.board.read32(0x3054), 0x5555);
        assert_eq!(machine.board.read32(0x3058), 0x6666);
        assert_eq!(machine.board.read32(0x3044), 0x3044);
        assert_eq!(machine.board.read32(0x3048), 0x3044);
        assert_eq!(machine.board.read8(0x3200 + 0x32), 0xfd);
        assert_eq!(machine.dpc_queue, vec![0x3040]);
        assert_ne!(machine.hal_software_interrupts & (1 << 2), 0);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);

        let state = machine.save_state().unwrap();
        let mut restored = XboxMachine::from_xbe(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.dpc_queue, vec![0x3040]);
        assert_eq!(restored.board.read8(0x3042), 1);
        assert_ne!(restored.hal_software_interrupts & (1 << 2), 0);
    }

    #[test]
    fn kernel_bugcheck_exports_trigger_fatal_reboot_without_returning() {
        let cases: [(&[u8], u32); 2] = [
            (
                &[
                    0x68, 0xef, 0xbe, 0xad, 0xde, // push BugCheckMode
                    0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeBugCheck
                    0xf4,
                ],
                95,
            ),
            (
                &[
                    0x6a, 0x04, // parameter4
                    0x6a, 0x03, // parameter3
                    0x6a, 0x02, // parameter2
                    0x6a, 0x01, // parameter1
                    0x68, 0xef, 0xbe, 0xad, 0xde, // BugCheckCode
                    0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeBugCheckEx
                    0xf4,
                ],
                96,
            ),
        ];

        for (program, ordinal) in cases {
            let image = synthetic_xbe_with_thunks(program, &[ordinal]);
            let mut machine = XboxMachine::from_xbe(image).unwrap();
            machine.run_cpu_frame();
            assert!(!machine.powered);
            assert_eq!(machine.board.smbus.smc_scratch & 0x02, 0x02);
            assert_eq!(machine.board.smbus.power_action, 0x01);
            assert!(!machine.cpu.halted());
        }
    }

    #[test]
    fn kernel_timer_set_expire_periodic_cancel_and_state_round_trip() {
        let program = [
            0x68, 0x78, 0x56, 0x34, 0x12, // push DeferredContext
            0x68, 0x00, 0x10, 0x00, 0x00, // push DeferredRoutine
            0x68, 0x80, 0x30, 0x00, 0x00, // push DPC
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeInitializeDpc
            0x6a, 0x00, // push NotificationTimer
            0x68, 0x00, 0x30, 0x00, 0x00, // push timer
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeInitializeTimerEx
            0x68, 0x80, 0x30, 0x00, 0x00, // push DPC
            0x6a, 0x14, // push Period=20ms
            0x68, 0xff, 0xff, 0xff, 0xff, // push DueTime high
            0x68, 0x60, 0x79, 0xfe, 0xff, // push DueTime low (-100000)
            0x68, 0x00, 0x30, 0x00, 0x00, // push timer
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeSetTimerEx
            0xa3, 0x00, 0x20, 0x00, 0x00, // store previous inserted
            0x6a, 0x00, // push NotificationTimer
            0x68, 0x40, 0x30, 0x00, 0x00, // push second timer
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeInitializeTimerEx
            0x6a, 0x00, // push DPC=NULL
            0x68, 0xff, 0xff, 0xff, 0xff, // push DueTime high
            0x68, 0xc0, 0xbd, 0xf0, 0xff, // push DueTime low (-1000000)
            0x68, 0x40, 0x30, 0x00, 0x00, // push second timer
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // KeSetTimer
            0xa3, 0x04, 0x20, 0x00, 0x00, // store previous inserted
            0x68, 0x40, 0x30, 0x00, 0x00, // push second timer
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // KeCancelTimer
            0xa3, 0x08, 0x20, 0x00, 0x00, // store cancel result
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[107, 113, 150, 149, 97]);
        let mut machine = XboxMachine::from_xbe(image.clone()).unwrap();
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), 0);
        assert_eq!(machine.board.read32(0x2004), 0);
        assert_eq!(machine.board.read32(0x2008), 1);
        assert_eq!(machine.board.read32(0x3004), 1);
        assert_eq!(machine.board.read8(0x3003), 1);
        assert_eq!(machine.timer_due_time(0x3000), 360_000);
        assert_eq!(machine.timer_queue, vec![0x3000]);
        assert_eq!(machine.board.read8(0x3043), 0);
        assert_eq!(machine.dpc_queue, vec![0x3080]);
        assert_eq!(machine.board.read32(0x3094), 160_000);
        assert_eq!(machine.board.read32(0x3098), 0);
        assert_ne!(machine.hal_software_interrupts & (1 << 2), 0);

        let state = machine.save_state().unwrap();
        let mut restored = XboxMachine::from_xbe(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.timer_queue, vec![0x3000]);
        assert_eq!(restored.timer_due_time(0x3000), 360_000);

        restored.run_frame(&InputState::default());
        assert_eq!(restored.timer_due_time(0x3000), 360_000);
        restored.run_frame(&InputState::default());
        assert_eq!(restored.timer_due_time(0x3000), 700_000);
        assert_eq!(restored.timer_queue, vec![0x3000]);
        assert_eq!(restored.dpc_queue, vec![0x3080]);
        assert!(restored.powered);
    }

    #[test]
    fn kernel_timer_priority_dpc_query_and_stall_thunks_are_deterministic() {
        let program = [
            0x68, 0x00, 0x30, 0x00, 0x00, // push timer
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeCancelTimer
            0xa3, 0x00, 0x20, 0x00, 0x00, // store first cancel result
            0x68, 0x00, 0x30, 0x00, 0x00, // push timer
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeCancelTimer
            0xa3, 0x04, 0x20, 0x00, 0x00, // store second cancel result
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeIsExecutingDpc
            0xa3, 0x08, 0x20, 0x00, 0x00, // store DPC-active state
            0x68, 0x00, 0x32, 0x00, 0x00, // push thread
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeQueryBasePriorityThread
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store signed priority
            0x68, 0xe8, 0x03, 0x00, 0x00, // push 1000 microseconds
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // KeStallExecutionProcessor
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[97, 121, 124, 151]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();

        let timer = 0x3000;
        let head = 0x3100;
        let entry = timer + 0x18;
        machine.board.write8(timer + 3, 1);
        machine.board.write32(head, entry);
        machine.board.write32(head + 4, entry);
        machine.board.write32(entry, head);
        machine.board.write32(entry + 4, head);
        machine.board.write8(0x3200 + 0x32, 0xf9);

        let cycles_before = machine.cpu.cycles;
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), 1);
        assert_eq!(machine.board.read32(0x2004), 0);
        assert_eq!(machine.board.read32(0x2008), 0);
        assert_eq!(machine.board.read32(0x200c), 0xffff_fff9);
        assert_eq!(machine.board.read8(timer + 3), 0);
        assert_eq!(machine.board.read32(head), head);
        assert_eq!(machine.board.read32(head + 4), head);
        assert!(machine.cpu.cycles.saturating_sub(cycles_before) >= 18_000);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_event_time_and_irql_convenience_thunks_follow_xbox_state_semantics() {
        let program = [
            0x6a, 0x00, // push signal state
            0x6a, 0x00, // push NotificationEvent
            0x68, 0x00, 0x30, 0x00, 0x00, // push event
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeInitializeEvent
            0x6a, 0x00, // push Wait=false
            0x6a, 0x01, // push Increment
            0x68, 0x00, 0x30, 0x00, 0x00, // push event
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeSetEvent
            0xa3, 0x00, 0x20, 0x00, 0x00, // store old state
            0x68, 0x00, 0x30, 0x00, 0x00, // push event
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // KeResetEvent
            0xa3, 0x04, 0x20, 0x00, 0x00, // store old state
            0x6a, 0x00, // push Wait=false
            0x6a, 0x01, // push Increment
            0x68, 0x00, 0x30, 0x00, 0x00, // push event
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeSetEvent
            0x6a, 0x00, // push Wait=false
            0x6a, 0x01, // push Increment
            0x68, 0x00, 0x30, 0x00, 0x00, // push event
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // KePulseEvent
            0xa3, 0x08, 0x20, 0x00, 0x00, // store old state
            0x68, 0x20, 0x30, 0x00, 0x00, // push system-time output
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // KeQuerySystemTime
            0xff, 0x15, 0x14, 0x12, 0x01, 0x00, // KeRaiseIrqlToDpcLevel
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store old IRQL
            0xb9, 0x00, 0x00, 0x00, 0x00, // mov ecx,PASSIVE_LEVEL
            0xff, 0x15, 0x18, 0x12, 0x01, 0x00, // KfLowerIrql
            0xff, 0x15, 0x1c, 0x12, 0x01, 0x00, // KeRaiseIrqlToSynchLevel
            0xa3, 0x10, 0x20, 0x00, 0x00, // store old IRQL
            0xb9, 0x00, 0x00, 0x00, 0x00, // mov ecx,PASSIVE_LEVEL
            0xff, 0x15, 0x18, 0x12, 0x01, 0x00, // KfLowerIrql
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[108, 145, 138, 123, 128, 129, 161, 130]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.frame_counter = 120;
        machine.board.update_kernel_time();
        machine.run_frame(&InputState::default());

        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap()),
            0
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap()),
            1
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2008..0x200c].try_into().unwrap()),
            1
        );
        assert_eq!(machine.board.read32(0x3004), 0);
        assert_eq!(
            u64::from_le_bytes(machine.board.ram[0x3020..0x3028].try_into().unwrap()),
            20_000_000
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x200c..0x2010].try_into().unwrap()),
            0
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2010..0x2014].try_into().unwrap()),
            0
        );
        assert_eq!(machine.current_irql, 0);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_thread_event_and_critical_section_thunks_follow_single_thread_semantics() {
        let program = [
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // KeGetCurrentThread
            0xa3, 0x00, 0x20, 0x00, 0x00, // store thread
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // KeEnterCriticalRegion
            0x68, 0x00, 0x30, 0x00, 0x00, // push critical section
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // RtlInitializeCriticalSection
            0x68, 0x00, 0x30, 0x00, 0x00, // push critical section
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // RtlTryEnterCriticalSection
            0xa3, 0x04, 0x20, 0x00, 0x00, // store result
            0x68, 0x00, 0x30, 0x00, 0x00, // push critical section
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // RtlEnterCriticalSectionAndRegion
            0xa1, 0x10, 0x30, 0x00, 0x00, // mov eax,[LockCount]
            0xa3, 0x08, 0x20, 0x00, 0x00, // store lock
            0xa1, 0x14, 0x30, 0x00, 0x00, // mov eax,[RecursionCount]
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store recursion
            0xa1, 0x18, 0x30, 0x00, 0x00, // mov eax,[OwningThread]
            0xa3, 0x10, 0x20, 0x00, 0x00, // store owner
            0x68, 0x00, 0x30, 0x00, 0x00, // push critical section
            0xff, 0x15, 0x14, 0x12, 0x01, 0x00, // RtlLeaveCriticalSection
            0x68, 0x00, 0x30, 0x00, 0x00, // push critical section
            0xff, 0x15, 0x18, 0x12, 0x01, 0x00, // RtlLeaveCriticalSectionAndRegion
            0xff, 0x15, 0x1c, 0x12, 0x01, 0x00, // KeLeaveCriticalRegion
            0x6a, 0x01, // push signaled
            0x6a, 0x00, // push NotificationEvent
            0x68, 0x40, 0x30, 0x00, 0x00, // push event
            0xff, 0x15, 0x20, 0x12, 0x01, 0x00, // KeInitializeEvent
            0xf4,
        ];
        let image =
            synthetic_xbe_with_thunks(&program, &[104, 101, 291, 306, 278, 294, 295, 122, 108]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        let thread = u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap());
        assert_eq!(thread, machine.current_thread_address());
        assert_eq!(
            thread.wrapping_add(XBOX_KERNEL_THREAD_RESERVE),
            machine.board.kernel_heap.start
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap()),
            1
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2008..0x200c].try_into().unwrap()),
            1
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x200c..0x2010].try_into().unwrap()),
            2
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2010..0x2014].try_into().unwrap()),
            thread
        );
        assert_eq!(machine.board.read32(0x3010), u32::MAX);
        assert_eq!(machine.board.read32(0x3014), 0);
        assert_eq!(machine.board.read32(0x3018), 0);
        assert_eq!(machine.board.read8(0x3000), 1);
        assert_eq!(machine.board.read8(0x3002), 4);
        assert_eq!(machine.board.read32(0x3004), 0);
        assert_eq!(machine.board.read32(0x3008), 0x3008);
        assert_eq!(machine.board.read32(0x300c), 0x3008);
        assert_eq!(machine.board.read32(thread.wrapping_add(0x68)), 0);
        assert_eq!(machine.board.read8(0x3040), 0);
        assert_eq!(machine.board.read8(0x3042), 4);
        assert_eq!(machine.board.read32(0x3044), 1);
        assert_eq!(machine.board.read32(0x3048), 0x3048);
        assert_eq!(machine.board.read32(0x304c), 0x3048);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_stack_thunks_include_guard_page_and_free_the_full_allocation() {
        let program = [
            0x6a, 0x00, // push DebuggerThread=false
            0x68, 0x00, 0x20, 0x00, 0x00, // push NumberOfBytes=0x2000
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // MmCreateKernelStack
            0xa3, 0x00, 0x20, 0x00, 0x00, // store StackBase
            0xa1, 0x00, 0x20, 0x00, 0x00, // mov eax,[StackBase]
            0x2d, 0x00, 0x20, 0x00, 0x00, // sub eax,0x2000 -> StackLimit
            0x50, // push StackLimit
            0xa1, 0x00, 0x20, 0x00, 0x00, // mov eax,[StackBase]
            0x50, // push StackBase
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // MmDeleteKernelStack
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[169, 170]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        let stack_base = machine.board.read32(0x2000);
        assert_ne!(stack_base, 0);
        assert_eq!(stack_base & 0xfff, 0);
        let allocation = stack_base - 0x3000;
        assert_eq!(machine.board.kernel_heap.allocation_size(allocation), None);
        let reused = machine.board.kernel_heap.allocate(0x3000, 0x1000).unwrap();
        assert_eq!(reused, allocation);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_allocator_thunks_allocate_query_free_reuse_and_page_align() {
        let program = [
            0x6a, 0x20, // push 32
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // ExAllocatePool
            0xa3, 0x00, 0x20, 0x00, 0x00, // save pointer
            0xc7, 0x00, 0xef, 0xbe, 0xad, 0xde, // mov dword [eax],0xdeadbeef
            0x50, // push eax
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // ExQueryPoolBlockSize
            0xa3, 0x04, 0x20, 0x00, 0x00, // save size
            0xa1, 0x00, 0x20, 0x00, 0x00, // mov eax,[0x2000]
            0x50, // push eax
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // ExFreePool
            0x6a, 0x20, // push 32
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // ExAllocatePool
            0xa3, 0x08, 0x20, 0x00, 0x00, // save reused pointer
            0x68, 0x00, 0x18, 0x00, 0x00, // push 0x1800
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // MmAllocateContiguousMemory
            0xa3, 0x0c, 0x20, 0x00, 0x00, // save pointer
            0x50, // push eax
            0xff, 0x15, 0x1c, 0x12, 0x01, 0x00, // MmQueryAllocationSize
            0xa3, 0x10, 0x20, 0x00, 0x00, // save size
            0xa1, 0x0c, 0x20, 0x00, 0x00, // mov eax,[0x200c]
            0x50, // push eax
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // MmFreeContiguousMemory
            0x6a, 0x04, // push protection
            0x68, 0x00, 0x10, 0x00, 0x00, // push 0x1000
            0xff, 0x15, 0x14, 0x12, 0x01, 0x00, // MmAllocateSystemMemory
            0xa3, 0x14, 0x20, 0x00, 0x00, // save pointer
            0x68, 0x00, 0x10, 0x00, 0x00, // push NumberOfBytes
            0x50, // push eax (BaseAddress)
            0xff, 0x15, 0x18, 0x12, 0x01, 0x00, // MmFreeSystemMemory
            0xa3, 0x18, 0x20, 0x00, 0x00, // save freed pages
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[14, 23, 17, 165, 171, 167, 172, 180]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        let pool = u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap());
        let reused = u32::from_le_bytes(machine.board.ram[0x2008..0x200c].try_into().unwrap());
        let contiguous = u32::from_le_bytes(machine.board.ram[0x200c..0x2010].try_into().unwrap());
        let system = u32::from_le_bytes(machine.board.ram[0x2014..0x2018].try_into().unwrap());
        assert_ne!(pool, 0);
        assert_eq!(pool, reused);
        assert_eq!(pool & 0xf, 0);
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap()),
            32
        );
        assert_eq!(machine.board.read32(reused), 0);
        assert_ne!(contiguous, 0);
        assert_eq!(contiguous & 0xfff, 0);
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2010..0x2014].try_into().unwrap()),
            0x1800
        );
        assert_ne!(system, 0);
        assert_eq!(system & 0xfff, 0);
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2018..0x201c].try_into().unwrap()),
            1
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_low_level_nls_conversion_truncates_and_reports_byte_counts() {
        let program = [
            0x6a, 0x04, // push BytesInMultiByteString
            0x68, 0x00, 0x30, 0x00, 0x00, // push MultiByteString
            0x68, 0x00, 0x20, 0x00, 0x00, // push BytesInUnicodeString result
            0x6a, 0x06, // push MaxBytesInUnicodeString
            0x68, 0x00, 0x31, 0x00, 0x00, // push UnicodeString
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlMultiByteToUnicodeN
            0xa3, 0x10, 0x20, 0x00, 0x00, // store status
            0x6a, 0x04, // push BytesInMultiByteString
            0x68, 0x00, 0x30, 0x00, 0x00, // push MultiByteString
            0x68, 0x04, 0x20, 0x00, 0x00, // push size result
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlMultiByteToUnicodeSize
            0xa3, 0x14, 0x20, 0x00, 0x00, // store status
            0x6a, 0x08, // push BytesInUnicodeString
            0x68, 0x20, 0x31, 0x00, 0x00, // push UnicodeString
            0x68, 0x08, 0x20, 0x00, 0x00, // push BytesInMultiByteString result
            0x6a, 0x03, // push MaxBytesInMultiByteString
            0x68, 0x20, 0x30, 0x00, 0x00, // push MultiByteString
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // RtlUnicodeToMultiByteN
            0xa3, 0x18, 0x20, 0x00, 0x00, // store status
            0x6a, 0x08, // push BytesInUnicodeString
            0x68, 0x20, 0x31, 0x00, 0x00, // push UnicodeString
            0x68, 0x0c, 0x20, 0x00, 0x00, // push size result
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // RtlUnicodeToMultiByteSize
            0xa3, 0x1c, 0x20, 0x00, 0x00, // store status
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[299, 300, 310, 311]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.ram[0x3000..0x3004].copy_from_slice(b"ABCD");
        for (index, unit) in [u16::from(b'A'), 0x00fe, 0x0100, u16::from(b'D')]
            .into_iter()
            .enumerate()
        {
            let start = 0x3120 + index * 2;
            machine.board.ram[start..start + 2].copy_from_slice(&unit.to_le_bytes());
        }

        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, 6),
            (0x2004, 8),
            (0x2008, 3),
            (0x200c, 4),
            (0x2010, STATUS_SUCCESS),
            (0x2014, STATUS_SUCCESS),
            (0x2018, STATUS_SUCCESS),
            (0x201c, STATUS_SUCCESS),
        ] {
            assert_eq!(
                u32::from_le_bytes(machine.board.ram[offset..offset + 4].try_into().unwrap()),
                expected
            );
        }
        assert_eq!(
            &machine.board.ram[0x3100..0x3106],
            &[b'A', 0, b'B', 0, b'C', 0]
        );
        assert_eq!(&machine.board.ram[0x3020..0x3023], &[b'A', 0xfe, b'?']);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_ansi_unicode_conversion_handles_fixed_overflow_and_allocated_buffers() {
        let program = [
            0x6a, 0x00, // push AllocateDestinationString=false
            0x68, 0x00, 0x22, 0x00, 0x00, // push ANSI source
            0x68, 0x00, 0x21, 0x00, 0x00, // push Unicode destination
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlAnsiStringToUnicodeString
            0xa3, 0x00, 0x20, 0x00, 0x00, // status
            0x6a, 0x00, // push false
            0x68, 0x00, 0x22, 0x00, 0x00, // push ANSI source
            0x68, 0x10, 0x21, 0x00, 0x00, // push small Unicode destination
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlAnsiStringToUnicodeString
            0xa3, 0x04, 0x20, 0x00, 0x00, // overflow status
            0x6a, 0x01, // push true
            0x68, 0x10, 0x22, 0x00, 0x00, // push ANSI source2
            0x68, 0x20, 0x21, 0x00, 0x00, // push allocated Unicode destination
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlAnsiStringToUnicodeString
            0xa3, 0x08, 0x20, 0x00, 0x00, // status
            0xa1, 0x24, 0x21, 0x00, 0x00, // mov eax,[0x2124]
            0xa3, 0x20, 0x20, 0x00, 0x00, // save allocated pointer
            0x6a, 0x00, // push false
            0x68, 0x20, 0x22, 0x00, 0x00, // push Unicode source
            0x68, 0x30, 0x21, 0x00, 0x00, // push ANSI destination
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlUnicodeStringToAnsiString
            0xa3, 0x0c, 0x20, 0x00, 0x00, // status
            0x6a, 0x00, // push false
            0x68, 0x20, 0x22, 0x00, 0x00, // push Unicode source
            0x68, 0x40, 0x21, 0x00, 0x00, // push small ANSI destination
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlUnicodeStringToAnsiString
            0xa3, 0x10, 0x20, 0x00, 0x00, // overflow status
            0x6a, 0x01, // push true
            0x68, 0x30, 0x22, 0x00, 0x00, // push Unicode source2
            0x68, 0x50, 0x21, 0x00, 0x00, // push allocated ANSI destination
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlUnicodeStringToAnsiString
            0xa3, 0x14, 0x20, 0x00, 0x00, // status
            0xa1, 0x54, 0x21, 0x00, 0x00, // mov eax,[0x2154]
            0xa3, 0x24, 0x20, 0x00, 0x00, // save allocated pointer
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[260, 308]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();

        machine.board.ram[0x3000..0x3003].copy_from_slice(&[b'A', 0x80, b'Z']);
        machine.board.ram[0x3020..0x3022].copy_from_slice(b"Hi");
        machine.board.ram[0x2200..0x2202].copy_from_slice(&3u16.to_le_bytes());
        machine.board.ram[0x2202..0x2204].copy_from_slice(&3u16.to_le_bytes());
        machine.board.ram[0x2204..0x2208].copy_from_slice(&0x3000u32.to_le_bytes());
        machine.board.ram[0x2210..0x2212].copy_from_slice(&2u16.to_le_bytes());
        machine.board.ram[0x2212..0x2214].copy_from_slice(&2u16.to_le_bytes());
        machine.board.ram[0x2214..0x2218].copy_from_slice(&0x3020u32.to_le_bytes());

        machine.board.ram[0x2102..0x2104].copy_from_slice(&8u16.to_le_bytes());
        machine.board.ram[0x2104..0x2108].copy_from_slice(&0x3100u32.to_le_bytes());
        machine.board.ram[0x2112..0x2114].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2114..0x2118].copy_from_slice(&0x3120u32.to_le_bytes());

        let unicode = [u16::from(b'A'), 0x00fe, 0x0100];
        for (index, unit) in unicode.into_iter().enumerate() {
            let start = 0x3040 + index * 2;
            machine.board.ram[start..start + 2].copy_from_slice(&unit.to_le_bytes());
        }
        let unicode2 = [u16::from(b'B'), 0x0100];
        for (index, unit) in unicode2.into_iter().enumerate() {
            let start = 0x3060 + index * 2;
            machine.board.ram[start..start + 2].copy_from_slice(&unit.to_le_bytes());
        }
        machine.board.ram[0x2220..0x2222].copy_from_slice(&6u16.to_le_bytes());
        machine.board.ram[0x2222..0x2224].copy_from_slice(&6u16.to_le_bytes());
        machine.board.ram[0x2224..0x2228].copy_from_slice(&0x3040u32.to_le_bytes());
        machine.board.ram[0x2230..0x2232].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2232..0x2234].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2234..0x2238].copy_from_slice(&0x3060u32.to_le_bytes());

        machine.board.ram[0x2132..0x2134].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2134..0x2138].copy_from_slice(&0x3140u32.to_le_bytes());
        machine.board.ram[0x2142..0x2144].copy_from_slice(&3u16.to_le_bytes());
        machine.board.ram[0x2144..0x2148].copy_from_slice(&0x3160u32.to_le_bytes());

        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, STATUS_SUCCESS),
            (0x2004, STATUS_BUFFER_OVERFLOW),
            (0x2008, STATUS_SUCCESS),
            (0x200c, STATUS_SUCCESS),
            (0x2010, STATUS_BUFFER_OVERFLOW),
            (0x2014, STATUS_SUCCESS),
        ] {
            assert_eq!(
                u32::from_le_bytes(machine.board.ram[offset..offset + 4].try_into().unwrap()),
                expected
            );
        }
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2100..0x2102].try_into().unwrap()),
            6
        );
        assert_eq!(
            &machine.board.ram[0x3100..0x3108],
            &[b'A', 0, 0x80, 0xff, b'Z', 0, 0, 0]
        );
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2110..0x2112].try_into().unwrap()),
            6
        );
        assert_eq!(&machine.board.ram[0x3120..0x3124], &[0; 4]);

        let unicode_alloc =
            u32::from_le_bytes(machine.board.ram[0x2020..0x2024].try_into().unwrap());
        assert_ne!(unicode_alloc, 0);
        assert_eq!(
            machine.board.kernel_heap.allocation_size(unicode_alloc),
            Some(6)
        );
        let unicode_alloc_range = ram_range(unicode_alloc, 6).unwrap();
        assert_eq!(
            &machine.board.ram[unicode_alloc_range],
            &[b'H', 0, b'i', 0, 0, 0]
        );

        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2130..0x2132].try_into().unwrap()),
            3
        );
        assert_eq!(&machine.board.ram[0x3140..0x3144], &[b'A', 0xfe, b'?', 0]);
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2140..0x2142].try_into().unwrap()),
            2
        );
        assert_eq!(&machine.board.ram[0x3160..0x3163], &[b'A', 0xfe, 0]);

        let ansi_alloc = u32::from_le_bytes(machine.board.ram[0x2024..0x2028].try_into().unwrap());
        assert_ne!(ansi_alloc, 0);
        assert_eq!(
            machine.board.kernel_heap.allocation_size(ansi_alloc),
            Some(3)
        );
        let ansi_alloc_range = ram_range(ansi_alloc, 3).unwrap();
        assert_eq!(&machine.board.ram[ansi_alloc_range], &[b'B', b'?', 0]);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_append_string_thunks_enforce_capacity_and_unicode_termination() {
        let program = [
            0x68, 0x10, 0x21, 0x00, 0x00, // push ANSI source
            0x68, 0x00, 0x21, 0x00, 0x00, // push ANSI destination
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlAppendStringToString
            0xa3, 0x00, 0x20, 0x00, 0x00, // store status
            0x68, 0x20, 0x21, 0x00, 0x00, // push larger ANSI source
            0x68, 0x00, 0x21, 0x00, 0x00, // push ANSI destination
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlAppendStringToString
            0xa3, 0x04, 0x20, 0x00, 0x00, // store overflow
            0x68, 0x40, 0x21, 0x00, 0x00, // push Unicode source descriptor
            0x68, 0x30, 0x21, 0x00, 0x00, // push Unicode destination
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlAppendUnicodeStringToString
            0xa3, 0x08, 0x20, 0x00, 0x00, // store status
            0x68, 0x40, 0x31, 0x00, 0x00, // push direct UTF-16 source
            0x68, 0x30, 0x21, 0x00, 0x00, // push Unicode destination
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // RtlAppendUnicodeToString
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store status
            0x68, 0x60, 0x31, 0x00, 0x00, // push oversized direct source
            0x68, 0x30, 0x21, 0x00, 0x00, // push Unicode destination
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // RtlAppendUnicodeToString
            0xa3, 0x10, 0x20, 0x00, 0x00, // store overflow
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[261, 262, 263]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();

        machine.board.ram[0x3000..0x3002].copy_from_slice(b"Hi");
        machine.board.ram[0x3020..0x3022].copy_from_slice(b"!!");
        machine.board.ram[0x3040..0x3043].copy_from_slice(b"XYZ");
        machine.board.ram[0x2100..0x2102].copy_from_slice(&2u16.to_le_bytes());
        machine.board.ram[0x2102..0x2104].copy_from_slice(&6u16.to_le_bytes());
        machine.board.ram[0x2104..0x2108].copy_from_slice(&0x3000u32.to_le_bytes());
        machine.board.ram[0x2110..0x2112].copy_from_slice(&2u16.to_le_bytes());
        machine.board.ram[0x2112..0x2114].copy_from_slice(&2u16.to_le_bytes());
        machine.board.ram[0x2114..0x2118].copy_from_slice(&0x3020u32.to_le_bytes());
        machine.board.ram[0x2120..0x2122].copy_from_slice(&3u16.to_le_bytes());
        machine.board.ram[0x2122..0x2124].copy_from_slice(&3u16.to_le_bytes());
        machine.board.ram[0x2124..0x2128].copy_from_slice(&0x3040u32.to_le_bytes());

        machine.board.ram[0x3100..0x3102].copy_from_slice(&u16::from(b'A').to_le_bytes());
        machine.board.ram[0x3120..0x3122].copy_from_slice(&u16::from(b'B').to_le_bytes());
        machine.board.ram[0x3140..0x3144].copy_from_slice(&[b'C', 0, 0, 0]);
        machine.board.ram[0x3160..0x3166].copy_from_slice(&[b'D', 0, b'E', 0, 0, 0]);
        machine.board.ram[0x2130..0x2132].copy_from_slice(&2u16.to_le_bytes());
        machine.board.ram[0x2132..0x2134].copy_from_slice(&8u16.to_le_bytes());
        machine.board.ram[0x2134..0x2138].copy_from_slice(&0x3100u32.to_le_bytes());
        machine.board.ram[0x2140..0x2142].copy_from_slice(&2u16.to_le_bytes());
        machine.board.ram[0x2142..0x2144].copy_from_slice(&2u16.to_le_bytes());
        machine.board.ram[0x2144..0x2148].copy_from_slice(&0x3120u32.to_le_bytes());

        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, STATUS_SUCCESS),
            (0x2004, STATUS_BUFFER_TOO_SMALL),
            (0x2008, STATUS_SUCCESS),
            (0x200c, STATUS_SUCCESS),
            (0x2010, STATUS_BUFFER_TOO_SMALL),
        ] {
            assert_eq!(
                u32::from_le_bytes(machine.board.ram[offset..offset + 4].try_into().unwrap()),
                expected
            );
        }
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2100..0x2102].try_into().unwrap()),
            4
        );
        assert_eq!(&machine.board.ram[0x3000..0x3004], b"Hi!!");
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2130..0x2132].try_into().unwrap()),
            6
        );
        assert_eq!(
            &machine.board.ram[0x3100..0x3108],
            &[b'A', 0, b'B', 0, b'C', 0, 0, 0]
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_unicode_integer_conversion_matches_descriptor_and_status_rules() {
        let program = [
            0x68, 0x00, 0x21, 0x00, 0x00, // push Unicode descriptor
            0x6a, 0x10, // push base 16
            0x68, 0xef, 0xbe, 0x00, 0x00, // push value
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlIntegerToUnicodeString
            0xa3, 0x00, 0x20, 0x00, 0x00, // store status
            0x68, 0x10, 0x21, 0x00, 0x00, // push small descriptor
            0x6a, 0x0a, // push base 10
            0x68, 0xd2, 0x04, 0x00, 0x00, // push 1234
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlIntegerToUnicodeString
            0xa3, 0x04, 0x20, 0x00, 0x00, // store status
            0x68, 0x20, 0x20, 0x00, 0x00, // push Value pointer
            0x6a, 0x00, // push auto base
            0x68, 0x20, 0x21, 0x00, 0x00, // push Unicode string descriptor
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlUnicodeStringToInteger
            0xa3, 0x08, 0x20, 0x00, 0x00, // store status
            0x68, 0x20, 0x20, 0x00, 0x00, // push Value pointer
            0x6a, 0x03, // push invalid base
            0x68, 0x20, 0x21, 0x00, 0x00, // push Unicode string descriptor
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlUnicodeStringToInteger
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store status
            0x6a, 0x00, // push null Value pointer
            0x6a, 0x0a, // push base 10
            0x68, 0x20, 0x21, 0x00, 0x00, // push Unicode string descriptor
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlUnicodeStringToInteger
            0xa3, 0x10, 0x20, 0x00, 0x00, // store status
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[293, 309]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();

        machine.board.ram[0x2102..0x2104].copy_from_slice(&10u16.to_le_bytes());
        machine.board.ram[0x2104..0x2108].copy_from_slice(&0x3100u32.to_le_bytes());
        machine.board.ram[0x2112..0x2114].copy_from_slice(&6u16.to_le_bytes());
        machine.board.ram[0x2114..0x2118].copy_from_slice(&0x3120u32.to_le_bytes());

        let unicode_source = [
            b' ' as u16,
            b'-' as u16,
            b'0' as u16,
            b'b' as u16,
            b'1' as u16,
            b'0' as u16,
            b'1' as u16,
            b'1' as u16,
            b'!' as u16,
        ];
        for (index, unit) in unicode_source.into_iter().enumerate() {
            let start = 0x3140 + index * 2;
            machine.board.ram[start..start + 2].copy_from_slice(&unit.to_le_bytes());
        }
        machine.board.ram[0x2120..0x2122].copy_from_slice(&18u16.to_le_bytes());
        machine.board.ram[0x2122..0x2124].copy_from_slice(&18u16.to_le_bytes());
        machine.board.ram[0x2124..0x2128].copy_from_slice(&0x3140u32.to_le_bytes());

        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, STATUS_SUCCESS),
            (0x2004, STATUS_BUFFER_OVERFLOW),
            (0x2008, STATUS_SUCCESS),
            (0x200c, STATUS_INVALID_PARAMETER),
            (0x2010, STATUS_ACCESS_VIOLATION),
            (0x2020, 0xffff_fff5),
        ] {
            assert_eq!(
                u32::from_le_bytes(machine.board.ram[offset..offset + 4].try_into().unwrap()),
                expected
            );
        }
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2100..0x2102].try_into().unwrap()),
            8
        );
        assert_eq!(
            &machine.board.ram[0x3100..0x310a],
            &[b'B', 0, b'E', 0, b'E', 0, b'F', 0, 0, 0]
        );
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2110..0x2112].try_into().unwrap()),
            8
        );
        assert_eq!(&machine.board.ram[0x3120..0x3126], &[0; 6]);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_numeric_text_conversion_matches_status_and_base_rules() {
        let program = [
            0x68, 0x20, 0x20, 0x00, 0x00, // push Value pointer
            0x6a, 0x00, // push auto base
            0x68, 0x00, 0x30, 0x00, 0x00, // push String
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlCharToInteger
            0xa3, 0x00, 0x20, 0x00, 0x00, // store status
            0x68, 0x20, 0x20, 0x00, 0x00, // push Value pointer
            0x6a, 0x03, // push invalid base
            0x68, 0x00, 0x30, 0x00, 0x00, // push String
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlCharToInteger
            0xa3, 0x04, 0x20, 0x00, 0x00, // store status
            0x6a, 0x00, // push null Value pointer
            0x6a, 0x0a, // push base 10
            0x68, 0x00, 0x30, 0x00, 0x00, // push String
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlCharToInteger
            0xa3, 0x08, 0x20, 0x00, 0x00, // store status
            0x68, 0x40, 0x30, 0x00, 0x00, // push output String
            0x6a, 0x05, // push OutputLength
            0x6a, 0x10, // push base 16
            0x68, 0xef, 0xbe, 0x00, 0x00, // push Value
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlIntegerToChar
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store status
            0x68, 0x50, 0x30, 0x00, 0x00, // push output String
            0x6a, 0x03, // push too-small OutputLength
            0x6a, 0x0a, // push base 10
            0x68, 0xd2, 0x04, 0x00, 0x00, // push 1234
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlIntegerToChar
            0xa3, 0x10, 0x20, 0x00, 0x00, // store status
            0x6a, 0x00, // push null String
            0x6a, 0x05, // push OutputLength
            0x6a, 0x0a, // push base 10
            0x6a, 0x2a, // push Value
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlIntegerToChar
            0xa3, 0x14, 0x20, 0x00, 0x00, // store status
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[267, 292]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.ram[0x3000..0x3009].copy_from_slice(b"  -0x2A! ");
        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, STATUS_SUCCESS),
            (0x2004, STATUS_INVALID_PARAMETER),
            (0x2008, STATUS_ACCESS_VIOLATION),
            (0x200c, STATUS_SUCCESS),
            (0x2010, STATUS_BUFFER_OVERFLOW),
            (0x2014, STATUS_ACCESS_VIOLATION),
            (0x2020, 0xffff_ffd6),
        ] {
            assert_eq!(
                u32::from_le_bytes(machine.board.ram[offset..offset + 4].try_into().unwrap()),
                expected
            );
        }
        assert_eq!(&machine.board.ram[0x3040..0x3045], b"BEEF ");
        assert_eq!(&machine.board.ram[0x3050..0x3054], &[0; 4]);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_upper_string_and_generic_mask_follow_kernel_mappings() {
        let program = [
            0x68, 0x10, 0x22, 0x00, 0x00, // push GenericMapping
            0x68, 0x00, 0x22, 0x00, 0x00, // push AccessMask
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlMapGenericMask
            0x68, 0x00, 0x21, 0x00, 0x00, // push source descriptor
            0x68, 0x10, 0x21, 0x00, 0x00, // push destination descriptor
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlUpperString
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[297, 317]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();

        machine.board.ram[0x2200..0x2204]
            .copy_from_slice(&(GENERIC_READ | GENERIC_EXECUTE | 0x0008).to_le_bytes());
        for (index, value) in [1u32, 2, 4, 0x0f].into_iter().enumerate() {
            let start = 0x2210 + index * 4;
            machine.board.ram[start..start + 4].copy_from_slice(&value.to_le_bytes());
        }

        machine.board.ram[0x3000..0x3004].copy_from_slice(&[b'a', 0xe0, 0xff, b'b']);
        machine.board.ram[0x2100..0x2102].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2102..0x2104].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2104..0x2108].copy_from_slice(&0x3000u32.to_le_bytes());
        machine.board.ram[0x2112..0x2114].copy_from_slice(&3u16.to_le_bytes());
        machine.board.ram[0x2114..0x2118].copy_from_slice(&0x3050u32.to_le_bytes());

        machine.run_frame(&InputState::default());

        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2200..0x2204].try_into().unwrap()),
            0x000d
        );
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2110..0x2112].try_into().unwrap()),
            3
        );
        assert_eq!(&machine.board.ram[0x3050..0x3053], &[b'A', 0xc0, b'?']);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_ansi_compare_equal_and_case_thunks_follow_latin1_rules() {
        let program = [
            0x6a, 0x00, // push case-sensitive
            0x68, 0x10, 0x21, 0x00, 0x00, // push string2
            0x68, 0x00, 0x21, 0x00, 0x00, // push string1
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlCompareString
            0xa3, 0x00, 0x20, 0x00, 0x00, // store result
            0x6a, 0x01, // push case-insensitive
            0x68, 0x10, 0x21, 0x00, 0x00, // push string2
            0x68, 0x00, 0x21, 0x00, 0x00, // push string1
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlCompareString
            0xa3, 0x04, 0x20, 0x00, 0x00, // store result
            0x6a, 0x00, // push case-sensitive
            0x68, 0x10, 0x21, 0x00, 0x00, // push string2
            0x68, 0x00, 0x21, 0x00, 0x00, // push string1
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlEqualString
            0xa3, 0x08, 0x20, 0x00, 0x00, // store result
            0x6a, 0x01, // push case-insensitive
            0x68, 0x10, 0x21, 0x00, 0x00, // push string2
            0x68, 0x00, 0x21, 0x00, 0x00, // push string1
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlEqualString
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store result
            0x68, 0xc0, 0x00, 0x00, 0x00, // push Latin-1 uppercase A-grave
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // RtlLowerChar
            0xa3, 0x10, 0x20, 0x00, 0x00, // store result
            0x68, 0xff, 0x00, 0x00, 0x00, // push 0xff
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // RtlUpperChar
            0xa3, 0x14, 0x20, 0x00, 0x00, // store result
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[270, 279, 296, 316]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.ram[0x3000..0x3003].copy_from_slice(&[b'A', b'b', 0xc0]);
        machine.board.ram[0x3020..0x3023].copy_from_slice(&[b'a', b'B', 0xe0]);
        for (descriptor, buffer) in [(0x2100, 0x3000u32), (0x2110, 0x3020u32)] {
            machine.board.ram[descriptor..descriptor + 2].copy_from_slice(&3u16.to_le_bytes());
            machine.board.ram[descriptor + 2..descriptor + 4].copy_from_slice(&3u16.to_le_bytes());
            machine.board.ram[descriptor + 4..descriptor + 8]
                .copy_from_slice(&buffer.to_le_bytes());
        }
        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, 0xffff_ffe0),
            (0x2004, 0),
            (0x2008, 0),
            (0x200c, 1),
            (0x2010, 0xffff_ffe0),
            (0x2014, 0x3f),
        ] {
            assert_eq!(
                u32::from_le_bytes(machine.board.ram[offset..offset + 4].try_into().unwrap()),
                expected
            );
        }
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_unicode_case_compare_and_conversion_follow_utf16_rules() {
        let program = [
            0x6a, 0x01, // push case insensitive
            0x68, 0x10, 0x21, 0x00, 0x00, // push string2
            0x68, 0x00, 0x21, 0x00, 0x00, // push string1
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlCompareUnicodeString
            0xa3, 0x00, 0x20, 0x00, 0x00, // store compare result
            0x6a, 0x01, // push case insensitive
            0x68, 0x10, 0x21, 0x00, 0x00, // push string2
            0x68, 0x00, 0x21, 0x00, 0x00, // push string1
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlEqualUnicodeString
            0xa3, 0x04, 0x20, 0x00, 0x00, // store equality
            0x68, 0xc4, 0x00, 0x00, 0x00, // push U+00C4
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // RtlDowncaseUnicodeChar
            0xa3, 0x08, 0x20, 0x00, 0x00, // store lowercase
            0x68, 0xe4, 0x00, 0x00, 0x00, // push U+00E4
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // RtlUpcaseUnicodeChar
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store uppercase
            0x6a, 0x01, // allocate
            0x68, 0x20, 0x21, 0x00, 0x00, // push source descriptor
            0x68, 0x30, 0x21, 0x00, 0x00, // push destination descriptor
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // RtlDowncaseUnicodeString
            0xa3, 0x10, 0x20, 0x00, 0x00, // store status
            0xa1, 0x34, 0x21, 0x00, 0x00, // mov eax,[dest.Buffer]
            0xa3, 0x14, 0x20, 0x00, 0x00, // store allocated pointer
            0x6a, 0x00, // fixed destination
            0x68, 0x50, 0x21, 0x00, 0x00, // push source descriptor
            0x68, 0x40, 0x21, 0x00, 0x00, // push destination descriptor
            0xff, 0x15, 0x14, 0x12, 0x01, 0x00, // RtlUpcaseUnicodeString
            0xa3, 0x18, 0x20, 0x00, 0x00, // store status
            0x6a, 0x06, // push bytes in Unicode source
            0x68, 0xa0, 0x30, 0x00, 0x00, // push Unicode source
            0x68, 0x20, 0x20, 0x00, 0x00, // push bytes-written pointer
            0x6a, 0x03, // push max output bytes
            0x68, 0xc0, 0x30, 0x00, 0x00, // push output
            0xff, 0x15, 0x18, 0x12, 0x01, 0x00, // RtlUpcaseUnicodeToMultiByteN
            0xa3, 0x1c, 0x20, 0x00, 0x00, // store status
            0x68, 0x30, 0x21, 0x00, 0x00, // push allocated descriptor
            0xff, 0x15, 0x1c, 0x12, 0x01, 0x00, // RtlFreeUnicodeString
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[271, 280, 275, 313, 276, 314, 315, 287]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();

        machine.board.ram[0x3000..0x3004].copy_from_slice(&[0xc4, 0x00, b'b', 0]);
        machine.board.ram[0x3020..0x3024].copy_from_slice(&[0xe4, 0x00, b'B', 0]);
        for (descriptor, buffer) in [(0x2100usize, 0x3000u32), (0x2110, 0x3020)] {
            machine.board.ram[descriptor..descriptor + 2].copy_from_slice(&4u16.to_le_bytes());
            machine.board.ram[descriptor + 2..descriptor + 4].copy_from_slice(&4u16.to_le_bytes());
            machine.board.ram[descriptor + 4..descriptor + 8]
                .copy_from_slice(&buffer.to_le_bytes());
        }

        machine.board.ram[0x3040..0x3044].copy_from_slice(&[b'A', 0, 0xc4, 0]);
        machine.board.ram[0x2120..0x2122].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2122..0x2124].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2124..0x2128].copy_from_slice(&0x3040u32.to_le_bytes());

        machine.board.ram[0x3060..0x3064].fill(0);
        machine.board.ram[0x2142..0x2144].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2144..0x2148].copy_from_slice(&0x3060u32.to_le_bytes());
        machine.board.ram[0x3080..0x3084].copy_from_slice(&[0xe4, 0x00, b'z', 0]);
        machine.board.ram[0x2150..0x2152].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2152..0x2154].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2154..0x2158].copy_from_slice(&0x3080u32.to_le_bytes());

        machine.board.ram[0x30a0..0x30a6].copy_from_slice(&[b'a', 0, 0xe9, 0x00, 0xbc, 0x03]);
        machine.run_frame(&InputState::default());

        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap()),
            0
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap()),
            1
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2008..0x200c].try_into().unwrap()),
            0x00e4
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x200c..0x2010].try_into().unwrap()),
            0x00c4
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2010..0x2014].try_into().unwrap()),
            STATUS_SUCCESS
        );
        let allocated = u32::from_le_bytes(machine.board.ram[0x2014..0x2018].try_into().unwrap());
        assert_ne!(allocated, 0);
        let allocated_range = ram_range(allocated, 4).unwrap();
        assert_eq!(&machine.board.ram[allocated_range], &[b'a', 0, 0xe4, 0]);
        assert_eq!(machine.board.kernel_heap.allocation_size(allocated), None);
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2018..0x201c].try_into().unwrap()),
            STATUS_SUCCESS
        );
        assert_eq!(&machine.board.ram[0x3060..0x3064], &[0xc4, 0, b'Z', 0]);
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x201c..0x2020].try_into().unwrap()),
            STATUS_SUCCESS
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2020..0x2024].try_into().unwrap()),
            3
        );
        assert_eq!(&machine.board.ram[0x30c0..0x30c3], &[b'A', 0xc9, b'?']);
        assert_eq!(&machine.board.ram[0x2130..0x2138], &[0; 8]);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_time_conversion_matches_1601_epoch_and_validates_dates() {
        let program = [
            0x68, 0x20, 0x30, 0x00, 0x00, // push Time
            0x68, 0x00, 0x30, 0x00, 0x00, // push TimeFields
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlTimeFieldsToTime
            0xa3, 0x00, 0x20, 0x00, 0x00, // store success
            0x68, 0x40, 0x30, 0x00, 0x00, // push TimeFields output
            0x68, 0x20, 0x30, 0x00, 0x00, // push Time
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlTimeToTimeFields
            0x68, 0x80, 0x30, 0x00, 0x00, // push Time
            0x68, 0x60, 0x30, 0x00, 0x00, // push invalid TimeFields
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlTimeFieldsToTime
            0xa3, 0x04, 0x20, 0x00, 0x00, // store failure
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[304, 305]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();

        let fields = [1970u16, 1, 1, 0, 0, 0, 0, 0xffff];
        for (index, field) in fields.into_iter().enumerate() {
            let offset = 0x3000 + index * 2;
            machine.board.ram[offset..offset + 2].copy_from_slice(&field.to_le_bytes());
        }
        let invalid = [1900u16, 2, 29, 0, 0, 0, 0, 0];
        for (index, field) in invalid.into_iter().enumerate() {
            let offset = 0x3060 + index * 2;
            machine.board.ram[offset..offset + 2].copy_from_slice(&field.to_le_bytes());
        }
        machine.board.ram[0x3080..0x3088].copy_from_slice(&0x1122_3344_5566_7788u64.to_le_bytes());

        machine.run_frame(&InputState::default());

        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap()),
            1
        );
        assert_eq!(
            u64::from_le_bytes(machine.board.ram[0x3020..0x3028].try_into().unwrap()),
            116_444_736_000_000_000
        );
        let expected = [1970u16, 1, 1, 0, 0, 0, 0, 4];
        for (index, expected_field) in expected.into_iter().enumerate() {
            let offset = 0x3040 + index * 2;
            assert_eq!(
                u16::from_le_bytes(machine.board.ram[offset..offset + 2].try_into().unwrap()),
                expected_field
            );
        }
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap()),
            0
        );
        assert_eq!(
            u64::from_le_bytes(machine.board.ram[0x3080..0x3088].try_into().unwrap()),
            0x1122_3344_5566_7788
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_extended_integer_thunks_return_edx_eax_and_remainder() {
        let program = [
            0x68, 0x03, 0x00, 0x00, 0x00, // push multiplier
            0x68, 0xff, 0xff, 0xff, 0xff, // push multiplicand high
            0x68, 0xfe, 0xff, 0xff, 0xff, // push multiplicand low (-2)
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlExtendedIntegerMultiply
            0xa3, 0x00, 0x20, 0x00, 0x00, // store low
            0x52, 0x58, // mov eax,edx via push/pop
            0xa3, 0x04, 0x20, 0x00, 0x00, // store high
            0x68, 0x20, 0x20, 0x00, 0x00, // push remainder pointer
            0x68, 0x03, 0x00, 0x00, 0x00, // push divisor
            0x68, 0x01, 0x00, 0x00, 0x00, // push dividend high
            0x68, 0x06, 0x00, 0x00, 0x00, // push dividend low
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlExtendedLargeIntegerDivide
            0xa3, 0x08, 0x20, 0x00, 0x00, // store quotient low
            0x52, 0x58, // mov eax,edx
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store quotient high
            0x6a, 0x00, // push shift
            0x68, 0x00, 0x00, 0x00, 0x80, // push magic high
            0x6a, 0x00, // push magic low
            0x68, 0xff, 0xff, 0xff, 0xff, // push dividend high
            0x68, 0xf6, 0xff, 0xff, 0xff, // push dividend low (-10)
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // RtlExtendedMagicDivide
            0xa3, 0x10, 0x20, 0x00, 0x00, // store result low
            0x52, 0x58, // mov eax,edx
            0xa3, 0x14, 0x20, 0x00, 0x00, // store result high
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[281, 282, 283]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());

        for (offset, expected) in [
            (0x2000, 0xffff_fffa),
            (0x2004, 0xffff_ffff),
            (0x2008, 0x5555_5557),
            (0x200c, 0),
            (0x2010, 0xffff_fffb),
            (0x2014, 0xffff_ffff),
            (0x2020, 1),
        ] {
            assert_eq!(
                u32::from_le_bytes(machine.board.ram[offset..offset + 4].try_into().unwrap()),
                expected
            );
        }
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_copy_create_and_free_string_thunks_use_guest_descriptors() {
        let program = [
            0x68, 0x00, 0x21, 0x00, 0x00, // push ANSI source descriptor
            0x68, 0x10, 0x21, 0x00, 0x00, // push ANSI destination descriptor
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlCopyString
            0x68, 0x20, 0x21, 0x00, 0x00, // push Unicode source descriptor
            0x68, 0x30, 0x21, 0x00, 0x00, // push Unicode destination descriptor
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlCopyUnicodeString
            0x68, 0x00, 0x32, 0x00, 0x00, // push UTF-16 source
            0x68, 0x40, 0x21, 0x00, 0x00, // push destination descriptor
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // RtlCreateUnicodeString
            0xa3, 0x00, 0x20, 0x00, 0x00, // store success
            0xa1, 0x44, 0x21, 0x00, 0x00, // mov eax,[descriptor.Buffer]
            0xa3, 0x04, 0x20, 0x00, 0x00, // store allocated buffer
            0xa1, 0x40, 0x21, 0x00, 0x00, // mov eax,[Length|MaximumLength]
            0xa3, 0x08, 0x20, 0x00, 0x00, // store packed lengths
            0x68, 0x40, 0x21, 0x00, 0x00, // push descriptor
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // RtlFreeUnicodeString
            0x68, 0x50, 0x21, 0x00, 0x00, // push ANSI descriptor
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // RtlFreeAnsiString
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[272, 273, 274, 287, 286]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();

        machine.board.ram[0x3000..0x3006].copy_from_slice(b"hello ");
        machine.board.ram[0x2100..0x2102].copy_from_slice(&5u16.to_le_bytes());
        machine.board.ram[0x2102..0x2104].copy_from_slice(&6u16.to_le_bytes());
        machine.board.ram[0x2104..0x2108].copy_from_slice(&0x3000u32.to_le_bytes());
        machine.board.ram[0x2112..0x2114].copy_from_slice(&3u16.to_le_bytes());
        machine.board.ram[0x2114..0x2118].copy_from_slice(&0x3050u32.to_le_bytes());

        machine.board.ram[0x3100..0x3106].copy_from_slice(&[b'A', 0, b'B', 0, 0, 0]);
        machine.board.ram[0x2120..0x2122].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2122..0x2124].copy_from_slice(&6u16.to_le_bytes());
        machine.board.ram[0x2124..0x2128].copy_from_slice(&0x3100u32.to_le_bytes());
        machine.board.ram[0x2132..0x2134].copy_from_slice(&2u16.to_le_bytes());
        machine.board.ram[0x2134..0x2138].copy_from_slice(&0x3150u32.to_le_bytes());

        machine.board.ram[0x3200..0x3206].copy_from_slice(&[b'X', 0, b'Y', 0, 0, 0]);
        let ansi_buffer = machine.board.kernel_heap.allocate(4, 16).unwrap();
        let ansi_range = ram_range(ansi_buffer, 4).unwrap();
        machine.board.ram[ansi_range].copy_from_slice(b"abc\0");
        machine.board.ram[0x2150..0x2152].copy_from_slice(&3u16.to_le_bytes());
        machine.board.ram[0x2152..0x2154].copy_from_slice(&4u16.to_le_bytes());
        machine.board.ram[0x2154..0x2158].copy_from_slice(&ansi_buffer.to_le_bytes());
        machine.run_frame(&InputState::default());

        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2110..0x2112].try_into().unwrap()),
            3
        );
        assert_eq!(&machine.board.ram[0x3050..0x3053], b"hel");
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2130..0x2132].try_into().unwrap()),
            2
        );
        assert_eq!(&machine.board.ram[0x3150..0x3152], &[b'A', 0]);
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap()),
            1
        );
        let created_buffer =
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap());
        assert_ne!(created_buffer, 0);
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2008..0x200c].try_into().unwrap()),
            u32::from_le_bytes([4, 0, 6, 0])
        );
        let created_range = ram_range(created_buffer, 6).unwrap();
        assert_eq!(&machine.board.ram[created_range], &[b'X', 0, b'Y', 0, 0, 0]);
        assert_eq!(
            machine.board.kernel_heap.allocation_size(created_buffer),
            None
        );
        assert_eq!(&machine.board.ram[0x2140..0x2148], &[0; 8]);
        assert_eq!(machine.board.kernel_heap.allocation_size(ansi_buffer), None);
        assert_eq!(&machine.board.ram[0x2150..0x2158], &[0; 8]);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_rtl_compare_and_init_string_thunks_follow_xbox_struct_layouts() {
        let program = [
            0x6a, 0x04, // push length
            0x68, 0x20, 0x30, 0x00, 0x00, // push source2
            0x68, 0x00, 0x30, 0x00, 0x00, // push source1
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlCompareMemory
            0xa3, 0x00, 0x20, 0x00, 0x00, // store matched bytes
            0x68, 0x44, 0x33, 0x22, 0x11, // push pattern
            0x6a, 0x0c, // push length
            0x68, 0x40, 0x30, 0x00, 0x00, // push source
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlCompareMemoryUlong
            0xa3, 0x04, 0x20, 0x00, 0x00, // store matched bytes
            0x68, 0x00, 0x30, 0x00, 0x00, // push ANSI source
            0x68, 0x00, 0x21, 0x00, 0x00, // push ANSI descriptor
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // RtlInitAnsiString
            0x68, 0x00, 0x31, 0x00, 0x00, // push UTF-16 source
            0x68, 0x10, 0x21, 0x00, 0x00, // push Unicode descriptor
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // RtlInitUnicodeString
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[268, 269, 289, 290]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.ram[0x3000..0x3004].copy_from_slice(b"abc ");
        machine.board.ram[0x3020..0x3024].copy_from_slice(b"abX ");
        machine.board.ram[0x3040..0x3044].copy_from_slice(&0x1122_3344u32.to_le_bytes());
        machine.board.ram[0x3044..0x3048].copy_from_slice(&0x1122_3344u32.to_le_bytes());
        machine.board.ram[0x3048..0x304c].copy_from_slice(&0x5566_7788u32.to_le_bytes());
        machine.board.ram[0x3100..0x3106].copy_from_slice(&[b'A', 0, b'B', 0, 0, 0]);
        machine.run_frame(&InputState::default());

        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap()),
            2
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap()),
            8
        );
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2100..0x2102].try_into().unwrap()),
            3
        );
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2102..0x2104].try_into().unwrap()),
            4
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2104..0x2108].try_into().unwrap()),
            0x3000
        );
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2110..0x2112].try_into().unwrap()),
            4
        );
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2112..0x2114].try_into().unwrap()),
            6
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2114..0x2118].try_into().unwrap()),
            0x3100
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_mm_query_statistics_reports_live_modeled_memory_state() {
        let program = [
            0x68, 0x00, 0x10, 0x00, 0x00, // push 0x1000
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // ExAllocatePool
            0x68, 0x00, 0x30, 0x00, 0x00, // push statistics
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // MmQueryStatistics
            0xa3, 0x00, 0x20, 0x00, 0x00, // store success status
            0x68, 0x40, 0x30, 0x00, 0x00, // push invalid statistics
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // MmQueryStatistics
            0xa3, 0x04, 0x20, 0x00, 0x00, // store invalid status
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[14, 181]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.write32(0x3000, 9 * 4);
        machine.board.write32(0x3040, 8 * 4);
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), STATUS_SUCCESS);
        assert_eq!(machine.board.read32(0x2004), STATUS_INVALID_PARAMETER);
        assert_eq!(machine.board.read32(0x3000), 36);
        assert_eq!(machine.board.read32(0x3004), (RAM_SIZE / 0x1000) as u32);
        assert_eq!(machine.board.read32(0x3014), 16);
        assert_eq!(machine.board.read32(0x3018), 1);
        assert_eq!(machine.board.read32(0x301c), 1);

        let total_pages = machine.board.read32(0x3004);
        let available_pages = machine.board.read32(0x3008);
        let image_pages = machine.board.read32(0x3020);
        assert!(image_pages > 0);
        assert_eq!(machine.board.read32(0x300c), image_pages * 0x1000);
        assert_eq!(machine.board.read32(0x3010), image_pages * 0x1000);
        assert_eq!(
            available_pages,
            total_pages - image_pages - 1 - 16 - 1 - (XBOX_GPU_INSTANCE_BYTES / 0x1000)
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_mm_gpu_protection_and_persistence_thunks_preserve_memory_contracts() {
        let program = [
            0x68, 0x00, 0x20, 0x00, 0x00, // push NumberOfPaddingBytes
            0x68, 0x00, 0x18, 0x00, 0x00, // push NumberOfBytes=0x1800
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // MmClaimGpuInstanceMemory
            0xa3, 0x04, 0x20, 0x00, 0x00, // store returned heap ceiling
            0x6a, 0x02, // push PAGE_READONLY
            0x68, 0x00, 0x20, 0x00, 0x00, // push NumberOfBytes=0x2000
            0x68, 0x00, 0x40, 0x00, 0x00, // push BaseAddress
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // MmSetAddressProtect
            0x68, 0x00, 0x48, 0x00, 0x00, // push VirtualAddress
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // MmQueryAddressProtect
            0xa3, 0x08, 0x20, 0x00, 0x00, // store protection
            0x68, 0x00, 0x00, 0x00, 0x70, // push invalid VirtualAddress
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // MmQueryAddressProtect
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store invalid protection
            0xc7, 0x05, 0x00, 0x50, 0x00, 0x00, 0x78, 0x56, 0x34, 0x12, // [0x5000]=marker
            0x6a, 0x01, // push Persist=true
            0x68, 0x00, 0x10, 0x00, 0x00, // push NumberOfBytes=0x1000
            0x68, 0x00, 0x50, 0x00, 0x00, // push BaseAddress
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // MmPersistContiguousMemory
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[168, 182, 179, 178]);
        let mut machine = XboxMachine::from_xbe(image.clone()).unwrap();
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), XBOX_GPU_INSTANCE_BYTES);
        assert_eq!(
            machine.board.read32(0x2004),
            XBOX_KERNEL_HEAP_ALIAS + XBOX_GPU_INSTANCE_BASE + XBOX_GPU_INSTANCE_BYTES
        );
        assert_eq!(machine.mm_gpu_instance_bytes, 0x2000);
        assert_eq!(machine.board.read32(0x2008), 0x02);
        assert_eq!(machine.board.read32(0x200c), 0);
        assert_eq!(machine.board.read32(0x5000), 0x1234_5678);
        assert!(machine.mm_persistent_pages.contains(&5));
        assert_eq!(machine.page_protection(0x4800), 0x02);

        let state = machine.save_state().unwrap();
        let mut restored = XboxMachine::from_xbe(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.mm_gpu_instance_bytes, 0x2000);
        assert!(restored.mm_persistent_pages.contains(&5));
        assert_eq!(restored.page_protection(0x4800), 0x02);

        machine.quick_reboot();
        assert!(machine.powered);
        assert_eq!(machine.board.read32(0x5000), 0x1234_5678);
        assert!(machine.mm_persistent_pages.contains(&5));
        assert_eq!(machine.page_protection(0x4800), XBOX_PAGE_READWRITE);
        assert_eq!(machine.mm_gpu_instance_bytes, XBOX_GPU_INSTANCE_BYTES);
    }

    #[test]
    fn kernel_mm_map_lock_and_unmap_thunks_match_flat_xbox_addressing() {
        let program = [
            0x6a, 0x04, // push PAGE_READWRITE
            0x68, 0x00, 0x10, 0x00, 0x00, // push length 0x1000
            0x68, 0x00, 0x00, 0x00, 0xfd, // push NV2A physical address
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // MmMapIoSpace
            0xa3, 0x00, 0x20, 0x00, 0x00, // store MMIO mapping
            0x6a, 0x04, // push PAGE_READWRITE
            0x68, 0x00, 0x10, 0x00, 0x00, // push length 0x1000
            0x68, 0x00, 0x30, 0x00, 0x00, // push physical RAM 0x3000
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // MmMapIoSpace
            0xa3, 0x04, 0x20, 0x00, 0x00, // store RAM mapping
            0x6a, 0x00, // push UnlockPages=false
            0x68, 0x00, 0x10, 0x00, 0x00, // push length
            0x68, 0x00, 0x30, 0x00, 0x80, // push KSEG0 RAM mapping
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // MmLockUnlockBufferPages
            0x6a, 0x00, // push UnlockPage=false
            0x68, 0x00, 0x30, 0x00, 0x00, // push physical page
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // MmLockUnlockPhysicalPage
            0x68, 0x00, 0x10, 0x00, 0x00, // push length
            0x68, 0x00, 0x00, 0x00, 0xfd, // push NV2A mapping
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // MmUnmapIoSpace
            0x68, 0x00, 0x10, 0x00, 0x00, // push length
            0x68, 0x00, 0x30, 0x00, 0x80, // push RAM mapping
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // MmUnmapIoSpace
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[177, 175, 176, 183]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap()),
            NV2A_BASE
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap()),
            0x8000_3000
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn kernel_memory_runtime_thunks_operate_on_guest_ram_and_fastcall_registers() {
        let program = [
            0x6a, 0x7a, // push fill
            0x6a, 0x08, // push length
            0x68, 0x00, 0x30, 0x00, 0x00, // push destination
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // RtlFillMemory
            0x68, 0x44, 0x33, 0x22, 0x11, // push pattern
            0x6a, 0x08, // push length
            0x68, 0x20, 0x30, 0x00, 0x00, // push destination
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // RtlFillMemoryUlong
            0x6a, 0x08, // push length
            0x68, 0x20, 0x30, 0x00, 0x00, // push source
            0x68, 0x40, 0x30, 0x00, 0x00, // push destination
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // RtlMoveMemory
            0xb9, 0x78, 0x56, 0x34, 0x12, // mov ecx,0x12345678
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // RtlUlongByteSwap
            0xa3, 0x00, 0x20, 0x00, 0x00, // store eax
            0xb9, 0x34, 0x12, 0x00, 0x00, // mov ecx,0x1234
            0xff, 0x15, 0x10, 0x12, 0x01, 0x00, // RtlUshortByteSwap
            0xa3, 0x04, 0x20, 0x00, 0x00, // store eax
            0x6a, 0x04, // push length
            0x68, 0x04, 0x30, 0x00, 0x00, // push destination
            0xff, 0x15, 0x14, 0x12, 0x01, 0x00, // RtlZeroMemory
            0x68, 0x40, 0x30, 0x00, 0x80, // push 0x80003040
            0xff, 0x15, 0x18, 0x12, 0x01, 0x00, // MmGetPhysicalAddress
            0xa3, 0x08, 0x20, 0x00, 0x00, // store eax
            0x68, 0x00, 0x00, 0x00, 0x90, // push invalid address
            0xff, 0x15, 0x1c, 0x12, 0x01, 0x00, // MmIsAddressValid
            0xa3, 0x0c, 0x20, 0x00, 0x00, // store eax
            0x68, 0x40, 0x30, 0x00, 0x80, // push valid alias
            0xff, 0x15, 0x1c, 0x12, 0x01, 0x00, // MmIsAddressValid
            0xa3, 0x10, 0x20, 0x00, 0x00, // store eax
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[284, 285, 298, 307, 318, 320, 173, 174]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(&machine.board.ram[0x3000..0x3004], &[0x7a; 4]);
        assert_eq!(&machine.board.ram[0x3004..0x3008], &[0; 4]);
        assert_eq!(
            &machine.board.ram[0x3020..0x3028],
            &[0x44, 0x33, 0x22, 0x11, 0x44, 0x33, 0x22, 0x11]
        );
        assert_eq!(
            machine.board.ram[0x3040..0x3048],
            machine.board.ram[0x3020..0x3028]
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap()),
            0x7856_3412
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap()),
            0x3412
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2008..0x200c].try_into().unwrap()),
            0x3040
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x200c..0x2010].try_into().unwrap()),
            0
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2010..0x2014].try_into().unwrap()),
            1
        );
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn av_exports_query_capabilities_and_drive_pitch_aware_rgb565_scanout() {
        let program = [
            0x68, 0x78, 0x56, 0x34, 0x12, // push saved data address
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // AvSetSavedDataAddress
            0xff, 0x15, 0x04, 0x12, 0x01, 0x00, // AvGetSavedDataAddress
            0xa3, 0x00, 0x20, 0x00, 0x00, // store saved data address
            0x68, 0x04, 0x20, 0x00, 0x00, // push Result
            0x6a, 0x00, // push Param
            0x6a, 0x06, // push AV_QUERY_AV_CAPABILITIES
            0x6a, 0x00, // push RegisterBase
            0xff, 0x15, 0x08, 0x12, 0x01, 0x00, // AvSendTVEncoderOption
            0x68, 0x00, 0x40, 0x00, 0x00, // push FrameBuffer
            0x68, 0x00, 0x05, 0x00, 0x00, // push Pitch=1280
            0x6a, 0x11, // push X_D3DFMT_LIN_R5G6B5
            0x6a, 0x00, // push Mode
            0x6a, 0x00, // push Step
            0x6a, 0x00, // push RegisterBase
            0xff, 0x15, 0x0c, 0x12, 0x01, 0x00, // AvSetDisplayMode
            0xa3, 0x08, 0x20, 0x00, 0x00, // store status
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[4, 1, 2, 3]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.ram[0x4000..0x4002].copy_from_slice(&0xf800u16.to_le_bytes());
        machine.board.ram[0x4002..0x4004].copy_from_slice(&0x07e0u16.to_le_bytes());
        machine.run_frame(&InputState::default());

        assert_eq!(machine.board.read32(0x2000), 0x1234_5678);
        assert_eq!(machine.board.read32(0x2004), 0x0040_0104);
        assert_eq!(machine.board.read32(0x2008), STATUS_SUCCESS);
        assert_eq!(machine.board.nv2a_u32(PCRTC_START), 0x4000);
        assert_eq!(machine.board.display_format, 0x11);
        assert_eq!(machine.board.display_pitch, 1280);
        assert_eq!(&machine.video().pixels()[..4], &[255, 0, 0, 255]);
        assert_eq!(&machine.video().pixels()[4..8], &[0, 255, 0, 255]);

        let state = machine.save_state().unwrap();
        let mut restored = XboxMachine::from_xbe(synthetic_xbe(&[0xf4])).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.av_saved_data_address, 0x1234_5678);
        assert_eq!(restored.board.display_format, 0x11);
        assert_eq!(restored.board.display_pitch, 1280);
        assert_eq!(&restored.video().pixels()[..4], &[255, 0, 0, 255]);
    }

    #[test]
    fn nv2a_pcrtc_start_presents_shared_ram() {
        let program = [
            0xb8, 0x00, 0x20, 0x00, 0x00, 0xa3, 0x00, 0x08, 0x60, 0xfd, 0xb8, 0x00, 0x00, 0xff,
            0x00, 0xa3, 0x00, 0x20, 0x00, 0x00, 0xf4,
        ];
        let mut machine = XboxMachine::from_xbe(synthetic_xbe(&program)).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.board.nv2a_u32(PCRTC_START), 0x2000);
        assert_eq!(&machine.video().pixels()[..4], &[255, 0, 0, 255]);
    }

    #[test]
    fn x86_port_io_reaches_smbus_smc_registers() {
        let program = [
            0xba, 0x04, 0xc0, 0x00, 0x00, // mov edx,0xc004
            0xb0, 0x21, // mov al,SMC read address
            0xee, // out dx,al
            0xba, 0x08, 0xc0, 0x00, 0x00, // mov edx,0xc008
            0xb0, 0x04, // mov al,AV pack command
            0xee, // out dx,al
            0xba, 0x02, 0xc0, 0x00, 0x00, // mov edx,0xc002
            0xb0, 0x0a, // byte-data transaction + start
            0xee, // out dx,al
            0xba, 0x06, 0xc0, 0x00, 0x00, // mov edx,0xc006
            0xec, // in al,dx
            0xa2, 0x00, 0x20, 0x00, 0x00, // mov [0x2000],al
            0xba, 0x00, 0xc0, 0x00, 0x00, // mov edx,0xc000
            0xec, // in al,dx
            0xa2, 0x01, 0x20, 0x00, 0x00, // mov [0x2001],al
            0xf4,
        ];
        let mut machine = XboxMachine::from_xbe(synthetic_xbe(&program)).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.board.ram[0x2000], 0x01);
        assert_ne!(machine.board.ram[0x2001] & SMBUS_STATUS_CYCLE_COMPLETE, 0);
        assert!(machine.powered);
        assert!(machine.cpu.halted());
    }

    #[test]
    fn smbus_eeprom_byte_data_is_persistent_and_stateful() {
        let image = synthetic_xbe(&[0xf4]);
        let mut machine = XboxMachine::from_xbe(image.clone()).unwrap();
        machine.board.io_write8(0xc004, SMBUS_EEPROM_ADDRESS << 1);
        machine.board.io_write8(0xc008, 0x58);
        machine.board.io_write8(0xc006, 0xa5);
        machine.board.io_write8(0xc002, 0x0a);
        assert_eq!(machine.board.io_read8(0xc000), SMBUS_STATUS_CYCLE_COMPLETE);

        machine
            .board
            .io_write8(0xc004, (SMBUS_EEPROM_ADDRESS << 1) | 1);
        machine.board.io_write8(0xc008, 0x58);
        machine.board.io_write8(0xc002, 0x0a);
        assert_eq!(machine.board.io_read8(0xc006), 0xa5);
        assert_eq!(
            machine.persistent_len(ResourceKind::Storage, 0),
            EEPROM_SIZE
        );
        let mut persisted = [0; EEPROM_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut persisted)
            .unwrap();
        assert_eq!(persisted[0x58], 0xa5);

        let state = machine.save_state().unwrap();
        let mut restored = XboxMachine::from_xbe(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.board.smbus.eeprom[0x58], 0xa5);
        assert_eq!(restored.board.smbus.command, 0x58);
    }

    #[test]
    fn smbus_smc_shutdown_command_powers_machine_off() {
        let program = [
            0xba, 0x04, 0xc0, 0x00, 0x00, // mov edx,0xc004
            0xb0, 0x20, // mov al,SMC write address
            0xee, // out dx,al
            0xba, 0x08, 0xc0, 0x00, 0x00, // mov edx,0xc008
            0xb0, 0x02, // mov al,power command
            0xee, // out dx,al
            0xba, 0x06, 0xc0, 0x00, 0x00, // mov edx,0xc006
            0xb0, 0x80, // mov al,shutdown
            0xee, // out dx,al
            0xba, 0x02, 0xc0, 0x00, 0x00, // mov edx,0xc002
            0xb0, 0x0a, // byte-data transaction + start
            0xee, // out dx,al
            0x90,
        ];
        let mut machine = XboxMachine::from_xbe(synthetic_xbe(&program)).unwrap();
        machine.run_frame(&InputState::default());
        assert!(!machine.powered);
    }

    #[test]
    fn pci_config_mechanism_exposes_xbox_chipset_topology_to_x86() {
        let program = [
            0xba, 0xf8, 0x0c, 0x00, 0x00, // mov edx,0xcf8
            0xb8, 0x00, 0x09, 0x00, 0x80, // mov eax,0x80000900 (SMBus ID)
            0xef, // out dx,eax
            0xba, 0xfc, 0x0c, 0x00, 0x00, // mov edx,0xcfc
            0xed, // in eax,dx
            0xa3, 0x00, 0x20, 0x00, 0x00, // mov [0x2000],eax
            0xba, 0xf8, 0x0c, 0x00, 0x00, // mov edx,0xcf8
            0xb8, 0x08, 0x09, 0x00, 0x80, // mov eax,0x80000908 (SMBus class)
            0xef, // out dx,eax
            0xba, 0xfc, 0x0c, 0x00, 0x00, // mov edx,0xcfc
            0xed, // in eax,dx
            0xa3, 0x04, 0x20, 0x00, 0x00, // mov [0x2004],eax
            0xba, 0xf8, 0x0c, 0x00, 0x00, // mov edx,0xcf8
            0xb8, 0x00, 0x00, 0x01, 0x80, // mov eax,0x80010000 (NV2A ID)
            0xef, // out dx,eax
            0xba, 0xfc, 0x0c, 0x00, 0x00, // mov edx,0xcfc
            0xed, // in eax,dx
            0xa3, 0x08, 0x20, 0x00, 0x00, // mov [0x2008],eax
            0xba, 0xf8, 0x0c, 0x00, 0x00, // mov edx,0xcf8
            0xb8, 0x00, 0x48, 0x00, 0x80, // mov eax,0x80004800 (IDE ID)
            0xef, // out dx,eax
            0xba, 0xfc, 0x0c, 0x00, 0x00, // mov edx,0xcfc
            0xed, // in eax,dx
            0xa3, 0x0c, 0x20, 0x00, 0x00, // mov [0x200c],eax
            0xba, 0xf8, 0x0c, 0x00, 0x00, // mov edx,0xcf8
            0xb8, 0x08, 0x48, 0x00, 0x80, // mov eax,0x80004808 (IDE class)
            0xef, // out dx,eax
            0xba, 0xfc, 0x0c, 0x00, 0x00, // mov edx,0xcfc
            0xed, // in eax,dx
            0xa3, 0x10, 0x20, 0x00, 0x00, // mov [0x2010],eax
            0xba, 0xf8, 0x0c, 0x00, 0x00, // mov edx,0xcf8
            0xb8, 0x20, 0x48, 0x00, 0x80, // mov eax,0x80004820 (IDE BMIBA)
            0xef, // out dx,eax
            0xba, 0xfc, 0x0c, 0x00, 0x00, // mov edx,0xcfc
            0xed, // in eax,dx
            0xa3, 0x14, 0x20, 0x00, 0x00, // mov [0x2014],eax
            0xf4,
        ];
        let mut machine = XboxMachine::from_xbe(synthetic_xbe(&program)).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2000..0x2004].try_into().unwrap()),
            0x01b4_10de
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2004..0x2008].try_into().unwrap()),
            0x0c05_00b1
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2008..0x200c].try_into().unwrap()),
            0x02a0_10de
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x200c..0x2010].try_into().unwrap()),
            0x01bc_10de
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2010..0x2014].try_into().unwrap()),
            0x0101_80b1
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x2014..0x2018].try_into().unwrap()),
            0x0000_0001
        );
        assert!(machine.cpu.halted());
    }

    #[test]
    fn hal_pci_space_reads_identity_and_updates_writable_bar() {
        let program = [
            0x6a, 0x00, // push WritePCISpace=false
            0x6a, 0x04, // push Length=4
            0x68, 0x00, 0x30, 0x00, 0x00, // push buffer
            0x6a, 0x00, // push register 0
            0x6a, 0x21, // push slot device1/function1
            0x6a, 0x00, // push bus 0
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // HalReadWritePCISpace
            0x6a, 0x01, // push WritePCISpace=true
            0x6a, 0x04, // push Length=4
            0x68, 0x04, 0x30, 0x00, 0x00, // push buffer
            0x6a, 0x14, // push BAR register
            0x6a, 0x21, // push slot device1/function1
            0x6a, 0x00, // push bus 0
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // HalReadWritePCISpace
            0x6a, 0x00, // push WritePCISpace=false
            0x6a, 0x04, // push Length=4
            0x68, 0x08, 0x30, 0x00, 0x00, // push buffer
            0x6a, 0x14, // push BAR register
            0x6a, 0x21, // push slot device1/function1
            0x6a, 0x00, // push bus 0
            0xff, 0x15, 0x00, 0x12, 0x01, 0x00, // HalReadWritePCISpace
            0xf4,
        ];
        let image = synthetic_xbe_with_thunks(&program, &[46]);
        let mut machine = XboxMachine::from_xbe(image).unwrap();
        machine.board.ram[0x3004..0x3008].copy_from_slice(&0x0000_d001u32.to_le_bytes());
        machine.run_frame(&InputState::default());
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x3000..0x3004].try_into().unwrap()),
            0x01b4_10de
        );
        assert_eq!(
            u32::from_le_bytes(machine.board.ram[0x3008..0x300c].try_into().unwrap()),
            0x0000_d001
        );
        assert_eq!(machine.board.pci.smbus_base(), 0xd000);
        assert_eq!(machine.cpu.regs[ESP], 0x03ff_ffc0);
        assert!(machine.cpu.halted());
        assert!(machine.powered);
    }

    #[test]
    fn pci_smbus_bar_relocation_moves_guest_io_window_and_survives_state() {
        let image = synthetic_xbe(&[0xf4]);
        let mut machine = XboxMachine::from_xbe(image.clone()).unwrap();
        machine.board.io_write32(PCI_CONFIG_ADDRESS, 0x8000_0914);
        assert_eq!(machine.board.io_read32(PCI_CONFIG_DATA), 0x0000_c001);
        machine.board.io_write32(PCI_CONFIG_DATA, 0x0000_d001);
        assert_eq!(machine.board.pci.smbus_base(), 0xd000);
        assert_eq!(machine.board.io_read8(0xc000), 0xff);

        machine
            .board
            .io_write8(0xd004, (SMBUS_EEPROM_ADDRESS << 1) | 1);
        machine.board.io_write8(0xd008, 0x00);
        machine.board.io_write8(0xd002, 0x0a);
        assert_eq!(machine.board.io_read8(0xd000), SMBUS_STATUS_CYCLE_COMPLETE);

        let state = machine.save_state().unwrap();
        let mut restored = XboxMachine::from_xbe(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.board.pci.smbus_base(), 0xd000);
        assert_eq!(restored.board.pci.config_address, 0x8000_0914);
    }

    #[test]
    fn xid_input_report_matches_original_xbox_layout() {
        let mut input = InputState::default();
        input.buttons[0] = UP | RIGHT | START | SELECT | L3 | R3 | FACE_SOUTH | FACE_WEST | L1 | R1;
        input.axes[0][AXIS_LEFT_TRIGGER] = 16_384;
        input.axes[0][AXIS_RIGHT_TRIGGER] = 32_767;
        input.axes[0][AXIS_LEFT_X] = -12_345;
        input.axes[0][AXIS_LEFT_Y] = 23_456;
        input.axes[0][AXIS_RIGHT_X] = -30_000;
        input.axes[0][AXIS_RIGHT_Y] = 30_000;
        let report = XboxXid::input_report(&input);
        assert_eq!(report[0], 0);
        assert_eq!(report[1], 20);
        assert_eq!(u16::from_le_bytes(report[2..4].try_into().unwrap()), 0x00f9);
        assert_eq!(&report[4..10], &[0xff, 0, 0xff, 0, 0xff, 0xff]);
        assert!((126..=128).contains(&report[10]));
        assert_eq!(report[11], 255);
        assert_eq!(
            i16::from_le_bytes(report[12..14].try_into().unwrap()),
            -12_345
        );
        assert_eq!(
            i16::from_le_bytes(report[14..16].try_into().unwrap()),
            23_456
        );
        assert_eq!(
            i16::from_le_bytes(report[16..18].try_into().unwrap()),
            -30_000
        );
        assert_eq!(
            i16::from_le_bytes(report[18..20].try_into().unwrap()),
            30_000
        );
    }

    #[test]
    fn ohci_root_hub_exposes_front_port_and_reset_enables_xid_device() {
        let mut machine = XboxMachine::from_xbe(synthetic_xbe(&[0xf4])).unwrap();
        assert_eq!(machine.board.read32(USB0_BASE), 0x10);
        assert_eq!(machine.board.read32(USB0_BASE + 0x48) & 0xff, 4);
        let port = USB0_BASE + 0x5c;
        assert_ne!(machine.board.read32(port) & OHCI_PORT_CCS, 0);
        assert_eq!(machine.board.read32(port) & OHCI_PORT_PES, 0);
        machine.board.write32(port, OHCI_PORT_PRS);
        let status = machine.board.read32(port);
        assert_ne!(status & OHCI_PORT_CCS, 0);
        assert_ne!(status & OHCI_PORT_PES, 0);
        assert_ne!(status & OHCI_PORT_PRSC, 0);
        assert_eq!(machine.board.ohci[0].xid.address, 0);
    }

    #[test]
    fn ohci_dma_enumerates_xid_and_delivers_interrupt_input_report() {
        fn put_u32(ram: &mut [u8], address: usize, value: u32) {
            ram[address..address + 4].copy_from_slice(&value.to_le_bytes());
        }

        let mut machine = XboxMachine::from_xbe(synthetic_xbe(&[0xf4])).unwrap();
        machine.board.write32(USB0_BASE + 0x5c, OHCI_PORT_PRS);

        const HCCA: usize = 0x3000;
        const ED: usize = 0x3100;
        const SETUP_TD: usize = 0x3200;
        const DATA_TD: usize = 0x3220;
        const STATUS_TD: usize = 0x3240;
        const TAIL_TD: usize = 0x3260;
        const SETUP_BUFFER: usize = 0x3400;
        const DATA_BUFFER: usize = 0x3500;

        put_u32(&mut machine.board.ram, ED, 64 << 16);
        put_u32(&mut machine.board.ram, ED + 4, TAIL_TD as u32);
        put_u32(&mut machine.board.ram, ED + 8, SETUP_TD as u32);
        put_u32(&mut machine.board.ram, SETUP_TD, 0xe << 28);
        put_u32(&mut machine.board.ram, SETUP_TD + 4, SETUP_BUFFER as u32);
        put_u32(&mut machine.board.ram, SETUP_TD + 8, DATA_TD as u32);
        put_u32(
            &mut machine.board.ram,
            SETUP_TD + 12,
            (SETUP_BUFFER + 7) as u32,
        );
        put_u32(&mut machine.board.ram, DATA_TD, (0xe << 28) | (2 << 19));
        put_u32(&mut machine.board.ram, DATA_TD + 4, DATA_BUFFER as u32);
        put_u32(&mut machine.board.ram, DATA_TD + 8, STATUS_TD as u32);
        put_u32(
            &mut machine.board.ram,
            DATA_TD + 12,
            (DATA_BUFFER + 17) as u32,
        );
        put_u32(&mut machine.board.ram, STATUS_TD, (0xe << 28) | (1 << 19));
        put_u32(&mut machine.board.ram, STATUS_TD + 8, TAIL_TD as u32);
        machine.board.ram[SETUP_BUFFER..SETUP_BUFFER + 8]
            .copy_from_slice(&[0x80, 0x06, 0x00, 0x01, 0, 0, 18, 0]);
        machine.board.write32(USB0_BASE + 0x18, HCCA as u32);
        machine.board.write32(USB0_BASE + 0x20, ED as u32);
        machine.board.write32(
            USB0_BASE + 0x04,
            OHCI_CONTROL_OPERATIONAL | OHCI_CONTROL_CLE,
        );
        machine.board.service_usb();
        assert_eq!(
            &machine.board.ram[DATA_BUFFER + 8..DATA_BUFFER + 12],
            &[0x5e, 0x04, 0x02, 0x02]
        );
        assert_ne!(
            u32::from_le_bytes(
                machine.board.ram[HCCA + 0x84..HCCA + 0x88]
                    .try_into()
                    .unwrap()
            ),
            0
        );

        const PERIODIC_ED: usize = 0x3600;
        const INPUT_TD: usize = 0x3620;
        const INPUT_TAIL: usize = 0x3640;
        const INPUT_BUFFER: usize = 0x3700;
        machine.board.ohci[0].xid.address = 5;
        machine.board.ohci[0].xid.configuration = 1;
        let ed_flags = 5u32 | (2 << 7) | (2 << 11) | (32 << 16);
        put_u32(&mut machine.board.ram, PERIODIC_ED, ed_flags);
        put_u32(&mut machine.board.ram, PERIODIC_ED + 4, INPUT_TAIL as u32);
        put_u32(&mut machine.board.ram, PERIODIC_ED + 8, INPUT_TD as u32);
        put_u32(&mut machine.board.ram, INPUT_TD, 0xe << 28);
        put_u32(&mut machine.board.ram, INPUT_TD + 4, INPUT_BUFFER as u32);
        put_u32(&mut machine.board.ram, INPUT_TD + 8, INPUT_TAIL as u32);
        put_u32(
            &mut machine.board.ram,
            INPUT_TD + 12,
            (INPUT_BUFFER + 19) as u32,
        );
        let slot = usize::from((machine.board.ohci[0].fm_number.wrapping_add(1)) & 31);
        put_u32(&mut machine.board.ram, HCCA + slot * 4, PERIODIC_ED as u32);
        let mut input = InputState::default();
        input.buttons[0] = UP | FACE_SOUTH | START;
        input.axes[0][AXIS_LEFT_X] = 12_000;
        input.axes[0][AXIS_RIGHT_TRIGGER] = 32_767;
        machine.board.input = input;
        machine.board.write32(
            USB0_BASE + 0x04,
            OHCI_CONTROL_OPERATIONAL | OHCI_CONTROL_PLE,
        );
        machine.board.service_usb();
        assert_eq!(machine.board.ram[INPUT_BUFFER + 1], 20);
        assert_eq!(machine.board.ram[INPUT_BUFFER + 4], 0xff);
        assert_eq!(machine.board.ram[INPUT_BUFFER + 11], 255);
        assert_eq!(
            i16::from_le_bytes(
                machine.board.ram[INPUT_BUFFER + 12..INPUT_BUFFER + 14]
                    .try_into()
                    .unwrap()
            ),
            12_000
        );
    }

    #[test]
    fn ide_hdd_identify_and_security_unlock_use_16_bit_string_io() {
        let program = [
            0xba, 0xf6, 0x01, 0x00, 0x00, // mov edx,0x1f6
            0xb0, 0xa0, // mov al,master
            0xee, // out dx,al
            0xba, 0xf7, 0x01, 0x00, 0x00, // mov edx,0x1f7
            0xb0, 0xec, // mov al,IDENTIFY DEVICE
            0xee, // out dx,al
            0xba, 0xf0, 0x01, 0x00, 0x00, // mov edx,0x1f0
            0xbf, 0x00, 0x20, 0x00, 0x00, // mov edi,0x2000
            0xb9, 0x00, 0x01, 0x00, 0x00, // mov ecx,256
            0x66, 0xf3, 0x6d, // rep insw
            0xba, 0xf6, 0x01, 0x00, 0x00, // mov edx,0x1f6
            0xb0, 0xa0, // mov al,master
            0xee, // out dx,al
            0xba, 0xf7, 0x01, 0x00, 0x00, // mov edx,0x1f7
            0xb0, 0xf2, // mov al,SECURITY UNLOCK
            0xee, // out dx,al
            0xba, 0xf0, 0x01, 0x00, 0x00, // mov edx,0x1f0
            0xbe, 0x00, 0x30, 0x00, 0x00, // mov esi,0x3000
            0xb9, 0x00, 0x01, 0x00, 0x00, // mov ecx,256
            0x66, 0xf3, 0x6f, // rep outsw
            0xf4,
        ];
        let mut machine = XboxMachine::from_xbe(synthetic_xbe(&program)).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(
            u16::from_le_bytes(machine.board.ram[0x2000..0x2002].try_into().unwrap()),
            0x0040
        );
        assert_ne!(
            u16::from_le_bytes(machine.board.ram[0x2100..0x2102].try_into().unwrap()) & 0x0004,
            0
        );
        assert!(!machine.board.ide.hdd_locked);
        assert_eq!(machine.cpu.regs[7], 0x2200);
        assert_eq!(machine.cpu.regs[6], 0x3200);
        assert_eq!(machine.cpu.regs[1], 0);
        assert!(machine.cpu.halted());
    }

    #[test]
    fn ide_atapi_dvd_identifies_reports_capacity_and_reads_disc_sector() {
        fn issue_packet(board: &mut XboxBoard, packet: [u8; 12], byte_limit: u16) {
            board.io_write8(0x01f4, byte_limit as u8);
            board.io_write8(0x01f5, (byte_limit >> 8) as u8);
            board.io_write8(0x01f7, 0xa0);
            assert_ne!(board.io_read8(0x01f7) & ATA_STATUS_DRQ, 0);
            for pair in packet.as_chunks::<2>().0 {
                board.io_write16(IDE_PRIMARY_BASE, u16::from_le_bytes(*pair));
            }
        }

        fn read_words(board: &mut XboxBoard, byte_count: usize) -> Vec<u8> {
            let mut data = Vec::with_capacity(byte_count);
            for _ in 0..byte_count.div_ceil(2) {
                data.extend_from_slice(&board.io_read16(IDE_PRIMARY_BASE).to_le_bytes());
            }
            data.truncate(byte_count);
            data
        }

        let mut disc = vec![0; ATAPI_SECTOR_SIZE * 2];
        for (index, byte) in disc[ATAPI_SECTOR_SIZE..].iter_mut().enumerate() {
            *byte = (index as u8).wrapping_mul(37).wrapping_add(11);
        }
        let image = synthetic_xbe(&[0xf4]);
        let mut machine =
            XboxMachine::from_images(image, Some(ResourceBlob::from_bytes(&disc))).unwrap();
        machine.board.io_write8(0x01f6, 0xb0);
        machine.board.io_write8(0x01f7, 0xa1);
        assert_ne!(machine.board.io_read8(0x01f7) & ATA_STATUS_DRQ, 0);
        assert_eq!(machine.board.io_read16(IDE_PRIMARY_BASE), 0x8580);
        for _ in 1..256 {
            let _ = machine.board.io_read16(IDE_PRIMARY_BASE);
        }
        assert_eq!(machine.board.io_read8(0x01f7), ATA_STATUS_DRDY);

        issue_packet(
            &mut machine.board,
            [0x25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            8,
        );
        let capacity = read_words(&mut machine.board, 8);
        assert_eq!(u32::from_be_bytes(capacity[0..4].try_into().unwrap()), 1);
        assert_eq!(
            u32::from_be_bytes(capacity[4..8].try_into().unwrap()),
            ATAPI_SECTOR_SIZE as u32
        );

        issue_packet(
            &mut machine.board,
            [0x28, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0],
            ATAPI_SECTOR_SIZE as u16,
        );
        let sector = read_words(&mut machine.board, ATAPI_SECTOR_SIZE);
        assert_eq!(sector, disc[ATAPI_SECTOR_SIZE..]);
        assert_eq!(machine.board.io_read8(0x01f7), ATA_STATUS_DRDY);
    }

    #[test]
    fn kernel_heap_state_round_trip_preserves_allocations_and_free_reuse() {
        let image = synthetic_xbe(&[0xf4]);
        let mut machine = XboxMachine::from_xbe(image.clone()).unwrap();
        let first = machine.board.kernel_heap.allocate(0x40, 16).unwrap();
        let second = machine.board.kernel_heap.allocate(0x80, 16).unwrap();
        assert_eq!(machine.board.kernel_heap.free(first), Some(0x40));
        let state = machine.save_state().unwrap();

        let mut restored = XboxMachine::from_xbe(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(
            restored.board.kernel_heap.allocation_size(second),
            Some(0x80)
        );
        let reused = restored.board.kernel_heap.allocate(0x20, 16).unwrap();
        assert_eq!(reused, first);
        assert_eq!(
            restored.board.kernel_heap.allocation_size(reused),
            Some(0x20)
        );
    }

    #[test]
    fn machine_state_round_trip_preserves_cpu_ram_and_nv2a() {
        let image = synthetic_xbe(&[0xf4]);
        let mut machine = XboxMachine::from_xbe(image.clone()).unwrap();
        machine.cpu.regs[0] = 0xfeed_beef;
        machine.current_irql = 2;
        machine.board.ram[0x3000..0x3004].copy_from_slice(&0x1122_3344u32.to_le_bytes());
        machine.board.nv2a[PCRTC_START..PCRTC_START + 4].copy_from_slice(&0x3000u32.to_le_bytes());
        let state = machine.save_state().unwrap();
        let mut restored = XboxMachine::from_xbe(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.cpu.regs[0], 0xfeed_beef);
        assert_eq!(
            restored.board.ram[0x3000..0x3004],
            0x1122_3344u32.to_le_bytes()
        );
        assert_eq!(restored.board.nv2a_u32(PCRTC_START), 0x3000);
        assert_eq!(restored.current_irql, 2);
        assert_eq!(restored.platform(), PlatformId::Xbox);
    }
}
