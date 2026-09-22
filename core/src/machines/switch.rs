use crate::blueprint::GuestIsa;
use crate::clock::ClockRate;
use crate::cluster::ProcessorCluster;
use crate::cpu_aarch64::{AArch64Bus, AArch64Cpu};
use crate::input::{
    AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_X, AXIS_RIGHT_Y, DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH,
    FACE_WEST, L1, L2, L3, LEFT, R1, R2, R3, RIGHT, SELECT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::ResourceBlob;
use crate::sparse_memory::SparseMemory;
use crate::state::{StateReader, StateWriter};

use super::switch_ipc;

const STATE_VERSION: u32 = 5;
const DRAM_SIZE: u64 = 4 * 1024 * 1024 * 1024;
const NRO_BASE: u64 = 0x71_0000_0000;
const NRO_END: u64 = NRO_BASE + DRAM_SIZE;
const USER_ADDRESS_SPACE_END: u64 = 0x80_0000_0000;
const PAGE_SIZE: u64 = 0x1000;
const HEAP_ALIGNMENT: u64 = 0x20_0000;
const HEAP_GUARD_SIZE: u64 = HEAP_ALIGNMENT;
const STACK_REGION_SIZE: u64 = 0x0100_0000;
const STACK_REGION_BASE: u64 = NRO_END - STACK_REGION_SIZE;
const ENV_REGION_SIZE: u64 = PAGE_SIZE;
const ENV_REGION_BASE: u64 = STACK_REGION_BASE - ENV_REGION_SIZE;
const LOADER_RETURN_ADDRESS: u64 = ENV_REGION_BASE + 0x800;
const ALIAS_REGION_BASE: u64 = NRO_END;
const ALIAS_REGION_SIZE: u64 = 0x1_0000_0000;
const PROCESS_MEMORY_LIMIT: u64 = 3 * 1024 * 1024 * 1024;
const MAIN_STACK_SIZE: u64 = 0x0010_0000;
const MAIN_STACK_BASE: u64 = NRO_END - MAIN_STACK_SIZE;
const TLS_SLOT_SIZE: u64 = 0x1000;
const TLS_REGION_BASE: u64 = STACK_REGION_BASE;
const PROCESS_HANDLE: u32 = 0x80;
const MAIN_THREAD_HANDLE: u32 = 0x100;
const CURRENT_THREAD_HANDLE: u32 = 0xffff_8000;
const SERVICE_HANDLE_BASE: u32 = 0x1000;
const MUTEX_WAIT_FLAG: u32 = 0x4000_0000;
const CPU_COUNT: usize = 4;
const FRAME_RATE: f64 = 60.0;
const FRAMES_PER_SECOND: u64 = 60;
const AUDIO_RATE: u32 = 48_000;
const STEPS_PER_FRAME: u64 = 100_000;
const GUEST_BUDGET_RATE: u64 = STEPS_PER_FRAME * FRAMES_PER_SECOND;
const SYSTEM_TICK_RATE: u64 = 19_200_000;
const SCHEDULER_QUANTUM: u64 = 1_024;
const VIDEO_WIDTH: u32 = 1280;
const VIDEO_HEIGHT: u32 = 720;
const HID_NPAD_OFFSET: u64 = 0x9a00;
const HID_NPAD_ENTRY_SIZE: u64 = 0x5000;
const HID_NPAD_FULL_KEY_LIFO_OFFSET: u64 = 0x28;
const HID_LIFO_HEADER_SIZE: u64 = 0x20;
const HID_NPAD_STORAGE_SIZE: u64 = 0x30;
const HID_NPAD_STORAGE_COUNT: u64 = 17;
const HID_NPAD_STYLE_FULL_KEY: u32 = 1;
const HID_NPAD_ATTRIBUTE_CONNECTED: u32 = 1;

fn hid_buttons(buttons: u64) -> u64 {
    let mut out = 0u64;
    out |= u64::from(buttons & FACE_EAST != 0);
    out |= u64::from(buttons & FACE_SOUTH != 0) << 1;
    out |= u64::from(buttons & FACE_NORTH != 0) << 2;
    out |= u64::from(buttons & FACE_WEST != 0) << 3;
    out |= u64::from(buttons & L3 != 0) << 4;
    out |= u64::from(buttons & R3 != 0) << 5;
    out |= u64::from(buttons & L1 != 0) << 6;
    out |= u64::from(buttons & R1 != 0) << 7;
    out |= u64::from(buttons & L2 != 0) << 8;
    out |= u64::from(buttons & R2 != 0) << 9;
    out |= u64::from(buttons & START != 0) << 10;
    out |= u64::from(buttons & SELECT != 0) << 11;
    out |= u64::from(buttons & LEFT != 0) << 12;
    out |= u64::from(buttons & UP != 0) << 13;
    out |= u64::from(buttons & RIGHT != 0) << 14;
    out |= u64::from(buttons & DOWN != 0) << 15;
    out
}

fn hid_axis(value: i16, invert: bool) -> i32 {
    let value = if invert {
        value.saturating_neg()
    } else {
        value
    };
    i32::from(value).clamp(-0x7fff, 0x7fff)
}

fn parcel_bytes(payload: &[u8]) -> Vec<u8> {
    let payload_size = payload.len() as u32;
    let payload_offset = 16u32;
    let object_offset = payload_offset + payload_size;
    let mut parcel = vec![0u8; object_offset as usize];
    parcel[0..4].copy_from_slice(&payload_size.to_le_bytes());
    parcel[4..8].copy_from_slice(&payload_offset.to_le_bytes());
    parcel[8..12].copy_from_slice(&0u32.to_le_bytes());
    parcel[12..16].copy_from_slice(&object_offset.to_le_bytes());
    parcel[16..].copy_from_slice(payload);
    parcel
}

fn vi_native_window_parcel(binder_id: u32) -> Vec<u8> {
    let mut payload = Vec::with_capacity(12);
    payload.extend_from_slice(&0u32.to_le_bytes());
    payload.extend_from_slice(&0u32.to_le_bytes());
    payload.extend_from_slice(&binder_id.to_le_bytes());
    parcel_bytes(&payload)
}

fn binder_connect_reply() -> Vec<u8> {
    let mut payload = Vec::with_capacity(20);
    payload.extend_from_slice(&VIDEO_WIDTH.to_le_bytes());
    payload.extend_from_slice(&VIDEO_HEIGHT.to_le_bytes());
    payload.extend_from_slice(&0u32.to_le_bytes());
    payload.extend_from_slice(&0u32.to_le_bytes());
    payload.extend_from_slice(&0i32.to_le_bytes());
    parcel_bytes(&payload)
}

fn binder_status_reply() -> Vec<u8> {
    parcel_bytes(&0i32.to_le_bytes())
}

fn le32(bytes: &[u8], offset: usize) -> Result<u32, String> {
    let value = bytes
        .get(offset..offset + 4)
        .ok_or_else(|| "NRO header is truncated".to_string())?;
    Ok(u32::from_le_bytes(value.try_into().unwrap()))
}

fn guest_to_backing(address: u64) -> Option<u64> {
    (NRO_BASE..NRO_END)
        .contains(&address)
        .then_some(address - NRO_BASE)
}
#[derive(Clone, Copy)]
struct NroSegment {
    offset: u32,
    size: u32,
}

fn parse_nro(image: &ResourceBlob) -> Result<([NroSegment; 3], u32), String> {
    let mut header = [0u8; 0x80];
    image.read(0, &mut header)?;
    if &header[0x10..0x14] != b"NRO0" {
        return Err("Switch homebrew image is not an NRO".into());
    }
    let declared_size = u64::from(le32(&header, 0x18)?);
    if declared_size < 0x80 || declared_size > image.len() {
        return Err("NRO declared size is outside the staged image".into());
    }
    let mut segments = [NroSegment { offset: 0, size: 0 }; 3];
    for (index, segment) in segments.iter_mut().enumerate() {
        let base = 0x20 + index * 8;
        segment.offset = le32(&header, base)?;
        segment.size = le32(&header, base + 4)?;
    }
    Ok((segments, le32(&header, 0x38)?))
}
fn align_up(value: u64, alignment: u64) -> Result<u64, String> {
    let mask = alignment - 1;
    value
        .checked_add(mask)
        .map(|rounded| rounded & !mask)
        .ok_or_else(|| "address alignment overflow".to_string())
}

fn supported_svc_hints() -> [u64; 3] {
    let mut hints = [0u64; 3];
    for svc in [
        0x01u16, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10,
        0x13, 0x14, 0x16, 0x18, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x21, 0x24, 0x25, 0x27, 0x29,
        0x2a, 0x2b,
    ] {
        let word = usize::from(svc / 64);
        hints[word] |= 1u64 << (svc % 64);
    }
    hints
}

fn write_config_entry(page: &mut [u8], index: usize, key: u32, value0: u64, value1: u64) {
    let offset = index * 24;
    page[offset..offset + 4].copy_from_slice(&key.to_le_bytes());
    page[offset + 4..offset + 8].copy_from_slice(&0u32.to_le_bytes());
    page[offset + 8..offset + 16].copy_from_slice(&value0.to_le_bytes());
    page[offset + 16..offset + 24].copy_from_slice(&value1.to_le_bytes());
}

fn load_nro(image: &ResourceBlob, dram: &mut SparseMemory) -> Result<(u64, u64), String> {
    let (segments, bss_size) = parse_nro(image)?;
    dram.clear();
    let mut previous_end = 0u64;
    let mut program_end = 0u64;
    for segment in segments {
        let offset = u64::from(segment.offset);
        let size = u64::from(segment.size);
        let end = offset
            .checked_add(size)
            .filter(|end| *end <= image.len() && *end <= DRAM_SIZE)
            .ok_or_else(|| "NRO segment exceeds image or Switch DRAM".to_string())?;
        if size != 0 && offset < previous_end {
            return Err("NRO segments overlap or are out of order".into());
        }
        if size != 0 {
            let mut bytes = vec![0; size as usize];
            image.read(offset, &mut bytes)?;
            dram.write(offset, &bytes)?;
            previous_end = end;
            program_end = program_end.max(end);
        }
    }
    let data = segments[2];
    let bss_start = u64::from(data.offset) + u64::from(data.size);
    let bss_end = bss_start
        .checked_add(u64::from(bss_size))
        .filter(|end| *end <= DRAM_SIZE)
        .ok_or_else(|| "NRO BSS exceeds Switch DRAM".to_string())?;
    dram.zero(bss_start, u64::from(bss_size))?;
    program_end = program_end.max(bss_end);
    let program_size = align_up(program_end, PAGE_SIZE)?;
    Ok((NRO_BASE + u64::from(segments[0].offset), program_size))
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct HorizonMemoryInfo {
    address: u64,
    size: u64,
    memory_type: u32,
    attributes: u32,
    permissions: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct MirrorMapping {
    destination: u64,
    source: u64,
    size: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct SharedMapping {
    handle: u32,
    address: u64,
    size: u64,
    permissions: u32,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct HorizonThread {
    handle: u32,
    cpu_index: u8,
    entry: u64,
    argument: u64,
    stack_top: u64,
    priority: i32,
    preferred_core: i32,
    affinity_mask: u64,
    tls_base: u64,
    started: bool,
    exited: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct MutexWaiter {
    address: u64,
    cpu_index: u8,
    self_tag: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CondvarWaiter {
    mutex_address: u64,
    condvar_address: u64,
    cpu_index: u8,
    self_tag: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ServiceKind {
    Sm = 1,
    TimeRoot = 2,
    TimeUserClock = 3,
    TimeNetworkClock = 4,
    TimeSteadyClock = 5,
    TimeZone = 6,
    TimeLocalClock = 7,
    FsProxy = 8,
    AppletAe = 9,
    Hid = 10,
    HidAppletResource = 11,
    HidSharedMemory = 12,
    Event = 13,
    ViRoot = 14,
    ViApplicationDisplay = 15,
    ViBinderRelay = 16,
}

impl ServiceKind {
    fn from_u8(value: u8) -> Option<Self> {
        Some(match value {
            1 => Self::Sm,
            2 => Self::TimeRoot,
            3 => Self::TimeUserClock,
            4 => Self::TimeNetworkClock,
            5 => Self::TimeSteadyClock,
            6 => Self::TimeZone,
            7 => Self::TimeLocalClock,
            8 => Self::FsProxy,
            9 => Self::AppletAe,
            10 => Self::Hid,
            11 => Self::HidAppletResource,
            12 => Self::HidSharedMemory,
            13 => Self::Event,
            14 => Self::ViRoot,
            15 => Self::ViApplicationDisplay,
            16 => Self::ViBinderRelay,
            _ => return None,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct ServiceSession {
    handle: u32,
    kind: ServiceKind,
}

#[derive(Clone)]
struct HorizonKernel {
    program_size: u64,
    heap_base: u64,
    heap_size: u64,
    mappings: Vec<MirrorMapping>,
    shared_mappings: Vec<SharedMapping>,
    threads: Vec<HorizonThread>,
    waiting_on: [Option<u32>; CPU_COUNT],
    mutex_waiters: Vec<MutexWaiter>,
    condvar_waiters: Vec<CondvarWaiter>,
    services: Vec<ServiceSession>,
    next_service_handle: u32,
    next_thread_handle: u32,
    last_unsupported_svc: Option<u16>,
}

impl HorizonKernel {
    fn new(program_size: u64) -> Result<Self, String> {
        let program_end = NRO_BASE
            .checked_add(program_size)
            .ok_or_else(|| "Switch program extent overflows".to_string())?;
        let heap_base = align_up(
            program_end
                .checked_add(HEAP_GUARD_SIZE)
                .ok_or_else(|| "Switch heap placement overflows".to_string())?,
            HEAP_ALIGNMENT,
        )?;
        if heap_base >= STACK_REGION_BASE {
            return Err("Switch program leaves no room for the heap".into());
        }
        Ok(Self {
            program_size,
            heap_base,
            heap_size: 0,
            mappings: Vec::new(),
            shared_mappings: Vec::new(),
            threads: vec![HorizonThread {
                handle: MAIN_THREAD_HANDLE,
                cpu_index: 0,
                entry: 0,
                argument: 0,
                stack_top: NRO_END - 0x10_000,
                priority: 0x2c,
                preferred_core: 0,
                affinity_mask: (1u64 << CPU_COUNT) - 1,
                tls_base: TLS_REGION_BASE,
                started: true,
                exited: false,
            }],
            waiting_on: [None; CPU_COUNT],
            mutex_waiters: Vec::new(),
            condvar_waiters: Vec::new(),
            services: Vec::new(),
            next_service_handle: SERVICE_HANDLE_BASE,
            next_thread_handle: MAIN_THREAD_HANDLE + 1,
            last_unsupported_svc: None,
        })
    }

    fn reset(&mut self, program_size: u64) -> Result<(), String> {
        *self = Self::new(program_size)?;
        Ok(())
    }

    fn heap_capacity(&self) -> u64 {
        ENV_REGION_BASE - self.heap_base
    }

    fn set_heap_size(&mut self, size: u64) -> Result<u64, u32> {
        if !size.is_multiple_of(HEAP_ALIGNMENT) || size > 4 * 1024 * 1024 * 1024 {
            return Err(0xCA01);
        }
        if size > self.heap_capacity() {
            return Err(0xD001);
        }
        let resource_limit = PROCESS_MEMORY_LIMIT
            .saturating_sub(self.program_size)
            .saturating_sub(STACK_REGION_SIZE)
            .saturating_sub(ENV_REGION_SIZE);
        if size > resource_limit {
            return Err(0x10801);
        }
        self.heap_size = size;
        Ok(self.heap_base)
    }

    fn used_memory(&self) -> u64 {
        self.program_size
            .saturating_add(self.heap_size)
            .saturating_add(STACK_REGION_SIZE)
            .saturating_add(ENV_REGION_SIZE)
    }

    fn get_info(&self, id: u64, sub_id: u64) -> Option<u64> {
        match id {
            0 => Some((1u64 << CPU_COUNT) - 1),
            1 => Some(u64::MAX),
            2 => Some(ALIAS_REGION_BASE),
            3 => Some(ALIAS_REGION_SIZE),
            4 => Some(self.heap_base),
            5 => Some(self.heap_capacity()),
            6 | 21 => Some(PROCESS_MEMORY_LIMIT),
            7 | 22 => Some(self.used_memory()),
            8 => Some(0),
            11 if sub_id < 4 => Some(
                [
                    0x4f4d_4e49_454d_5531,
                    0x485a_4f4e_4b45_524e,
                    0x4445_5445_524d_494e,
                    0x4953_5449_435f_5231,
                ][sub_id as usize],
            ),
            12 => Some(NRO_BASE),
            13 => Some(USER_ADDRESS_SPACE_END - NRO_BASE),
            14 => Some(STACK_REGION_BASE),
            15 => Some(STACK_REGION_SIZE),
            23 => Some(1),
            24 => Some((CPU_COUNT - 1) as u64),
            28 => Some(0),
            _ => None,
        }
    }

    fn thread_handle_for_cpu(&self, cpu_index: usize) -> Option<u32> {
        self.threads
            .iter()
            .find(|thread| usize::from(thread.cpu_index) == cpu_index && !thread.exited)
            .map(|thread| thread.handle)
    }

    fn resolve_thread_handle(&self, handle: u32, cpu_index: usize) -> Option<u32> {
        if handle == CURRENT_THREAD_HANDLE {
            self.thread_handle_for_cpu(cpu_index)
        } else {
            self.threads
                .iter()
                .any(|thread| thread.handle == handle)
                .then_some(handle)
        }
    }

    fn thread(&self, handle: u32) -> Option<&HorizonThread> {
        self.threads.iter().find(|thread| thread.handle == handle)
    }

    fn thread_mut(&mut self, handle: u32) -> Option<&mut HorizonThread> {
        self.threads
            .iter_mut()
            .find(|thread| thread.handle == handle)
    }

    fn allocate_service(&mut self, kind: ServiceKind) -> u32 {
        let handle = self.next_service_handle;
        self.next_service_handle = self
            .next_service_handle
            .wrapping_add(1)
            .max(SERVICE_HANDLE_BASE);
        self.services.push(ServiceSession { handle, kind });
        handle
    }
    fn service_kind(&self, handle: u32) -> Option<ServiceKind> {
        self.services
            .iter()
            .find(|session| session.handle == handle)
            .map(|session| session.kind)
    }

    fn close_service(&mut self, handle: u32) -> bool {
        if let Some(index) = self
            .services
            .iter()
            .position(|session| session.handle == handle)
        {
            self.services.remove(index);
            true
        } else {
            false
        }
    }

    fn service_for_name(name: &[u8]) -> Option<ServiceKind> {
        let end = name
            .iter()
            .position(|byte| *byte == 0)
            .unwrap_or(name.len());
        match &name[..end] {
            b"time:u" => Some(ServiceKind::TimeRoot),
            b"fsp-srv" => Some(ServiceKind::FsProxy),
            b"appletAE" => Some(ServiceKind::AppletAe),
            b"hid" => Some(ServiceKind::Hid),
            b"vi:u" => Some(ServiceKind::ViRoot),
            _ => None,
        }
    }

    fn hid_shared_memory_address(&self) -> Option<u64> {
        self.shared_mappings.iter().find_map(|mapping| {
            (self.service_kind(mapping.handle) == Some(ServiceKind::HidSharedMemory))
                .then_some(mapping.address)
        })
    }

    fn ranges_overlap(a: u64, a_size: u64, b: u64, b_size: u64) -> bool {
        let Some(a_end) = a.checked_add(a_size) else {
            return true;
        };
        let Some(b_end) = b.checked_add(b_size) else {
            return true;
        };
        a < b_end && b < a_end
    }

    fn map_memory(&mut self, destination: u64, source: u64, size: u64) -> Result<(), u32> {
        let heap_end = self.heap_base.saturating_add(self.heap_size);
        let destination_end = destination.saturating_add(size);
        let source_end = source.saturating_add(size);
        let overlaps_tls = Self::ranges_overlap(
            destination,
            size,
            TLS_REGION_BASE,
            TLS_SLOT_SIZE * CPU_COUNT as u64,
        );
        if size == 0
            || !destination.is_multiple_of(PAGE_SIZE)
            || !source.is_multiple_of(PAGE_SIZE)
            || !size.is_multiple_of(PAGE_SIZE)
            || destination < STACK_REGION_BASE
            || destination_end > MAIN_STACK_BASE
            || source < self.heap_base
            || source_end > heap_end
            || guest_to_backing(destination).is_none()
            || guest_to_backing(source).is_none()
            || overlaps_tls
            || self.mappings.iter().any(|mapping| {
                Self::ranges_overlap(destination, size, mapping.destination, mapping.size)
            })
        {
            return Err(0xD401);
        }
        self.mappings.push(MirrorMapping {
            destination,
            source,
            size,
        });
        Ok(())
    }

    fn unmap_memory(&mut self, destination: u64, source: u64, size: u64) -> Result<(), u32> {
        let Some(index) = self.mappings.iter().position(|mapping| {
            mapping.destination == destination && mapping.source == source && mapping.size == size
        }) else {
            return Err(0xD401);
        };
        self.mappings.remove(index);
        Ok(())
    }

    fn map_shared_memory(
        &mut self,
        handle: u32,
        address: u64,
        size: u64,
        permissions: u32,
    ) -> Result<(), u32> {
        if self.service_kind(handle) != Some(ServiceKind::HidSharedMemory)
            || size != 0x40000
            || permissions != 1
            || !address.is_multiple_of(PAGE_SIZE)
            || !size.is_multiple_of(PAGE_SIZE)
        {
            return Err(0xD401);
        }
        let Some(info) = self.query_memory(address) else {
            return Err(0xD401);
        };
        if info.memory_type != 0 || info.size < size {
            return Err(0xD401);
        }
        self.shared_mappings.push(SharedMapping {
            handle,
            address,
            size,
            permissions,
        });
        Ok(())
    }

    fn unmap_shared_memory(&mut self, handle: u32, address: u64, size: u64) -> Result<(), u32> {
        let Some(index) = self.shared_mappings.iter().position(|mapping| {
            mapping.handle == handle && mapping.address == address && mapping.size == size
        }) else {
            return Err(0xD401);
        };
        self.shared_mappings.remove(index);
        Ok(())
    }

    fn create_thread(
        &mut self,
        entry: u64,
        argument: u64,
        stack_top: u64,
        priority: i32,
        requested_core: i32,
    ) -> Result<(u32, usize, u64), u32> {
        if !(NRO_BASE..NRO_END).contains(&entry)
            || stack_top <= STACK_REGION_BASE
            || stack_top > NRO_END
            || !(0..=63).contains(&priority)
            || !(-2..CPU_COUNT as i32).contains(&requested_core)
        {
            return Err(0xCA01);
        }
        let cpu_index = (1..CPU_COUNT)
            .find(|candidate| {
                self.threads
                    .iter()
                    .all(|thread| usize::from(thread.cpu_index) != *candidate || thread.exited)
            })
            .ok_or(0x10801u32)?;
        let handle = self.next_thread_handle;
        self.next_thread_handle = self
            .next_thread_handle
            .wrapping_add(1)
            .max(MAIN_THREAD_HANDLE + 1);
        let preferred_core = if requested_core >= 0 {
            requested_core
        } else {
            cpu_index as i32
        };
        let affinity_mask = if requested_core >= 0 {
            1u64 << requested_core
        } else {
            (1u64 << CPU_COUNT) - 1
        };
        let tls_base = TLS_REGION_BASE + cpu_index as u64 * TLS_SLOT_SIZE;
        self.threads.push(HorizonThread {
            handle,
            cpu_index: cpu_index as u8,
            entry,
            argument,
            stack_top,
            priority,
            preferred_core,
            affinity_mask,
            tls_base,
            started: false,
            exited: false,
        });
        Ok((handle, cpu_index, tls_base))
    }

    fn query_memory(&self, address: u64) -> Option<HorizonMemoryInfo> {
        if address >= USER_ADDRESS_SPACE_END {
            return None;
        }
        let program_end = NRO_BASE + self.program_size;
        let heap_end = self.heap_base + self.heap_size;

        if let Some(mapping) = self
            .shared_mappings
            .iter()
            .find(|mapping| (mapping.address..mapping.address + mapping.size).contains(&address))
        {
            return Some(HorizonMemoryInfo {
                address: mapping.address,
                size: mapping.size,
                memory_type: 6,
                attributes: 0,
                permissions: mapping.permissions,
            });
        }

        if (ENV_REGION_BASE..STACK_REGION_BASE).contains(&address) {
            return Some(HorizonMemoryInfo {
                address: ENV_REGION_BASE,
                size: ENV_REGION_SIZE,
                memory_type: 2,
                attributes: 0,
                permissions: 1,
            });
        }

        if (STACK_REGION_BASE..NRO_END).contains(&address) {
            let tls_region_end = TLS_REGION_BASE + TLS_SLOT_SIZE * CPU_COUNT as u64;
            if (TLS_REGION_BASE..tls_region_end).contains(&address) {
                let slot = (address - TLS_REGION_BASE) / TLS_SLOT_SIZE;
                return Some(HorizonMemoryInfo {
                    address: TLS_REGION_BASE + slot * TLS_SLOT_SIZE,
                    size: TLS_SLOT_SIZE,
                    memory_type: 0x0c,
                    attributes: 0,
                    permissions: 3,
                });
            }
            if (MAIN_STACK_BASE..NRO_END).contains(&address) {
                return Some(HorizonMemoryInfo {
                    address: MAIN_STACK_BASE,
                    size: MAIN_STACK_SIZE,
                    memory_type: 2,
                    attributes: 0,
                    permissions: 3,
                });
            }
            for mapping in &self.mappings {
                if (mapping.destination..mapping.destination + mapping.size).contains(&address) {
                    return Some(HorizonMemoryInfo {
                        address: mapping.destination,
                        size: mapping.size,
                        memory_type: 0x0b,
                        attributes: 0,
                        permissions: 3,
                    });
                }
            }

            let mut base = STACK_REGION_BASE;
            let mut end = NRO_END;
            let mut consider = |occupied_base: u64, occupied_size: u64| {
                let occupied_end = occupied_base.saturating_add(occupied_size);
                if occupied_end <= address {
                    base = base.max(occupied_end);
                } else if occupied_base > address {
                    end = end.min(occupied_base);
                }
            };
            consider(TLS_REGION_BASE, TLS_SLOT_SIZE * CPU_COUNT as u64);
            consider(MAIN_STACK_BASE, MAIN_STACK_SIZE);
            for mapping in &self.mappings {
                consider(mapping.destination, mapping.size);
            }
            return Some(HorizonMemoryInfo {
                address: base,
                size: end.saturating_sub(base),
                memory_type: 0,
                attributes: 0,
                permissions: 0,
            });
        }

        let (mut base, mut end, memory_type, permissions) = if address < NRO_BASE {
            (0, NRO_BASE, 0, 0)
        } else if address < program_end {
            (NRO_BASE, program_end, 3, 5)
        } else if address < self.heap_base {
            (program_end, self.heap_base, 0, 0)
        } else if self.heap_size != 0 && address < heap_end {
            (self.heap_base, heap_end, 5, 3)
        } else if address < ENV_REGION_BASE {
            (heap_end.max(self.heap_base), ENV_REGION_BASE, 0, 0)
        } else {
            (NRO_END, USER_ADDRESS_SPACE_END, 0, 0)
        };
        if memory_type == 0 {
            for mapping in &self.shared_mappings {
                let mapping_end = mapping.address.saturating_add(mapping.size);
                if mapping_end <= address {
                    base = base.max(mapping_end);
                } else if mapping.address > address {
                    end = end.min(mapping.address);
                }
            }
        }
        Some(HorizonMemoryInfo {
            address: base,
            size: end.saturating_sub(base),
            memory_type,
            attributes: 0,
            permissions,
        })
    }

    fn save(&self, out: &mut StateWriter) {
        out.u64(self.program_size);
        out.u64(self.heap_base);
        out.u64(self.heap_size);
        out.u32(self.mappings.len() as u32);
        for mapping in &self.mappings {
            out.u64(mapping.destination);
            out.u64(mapping.source);
            out.u64(mapping.size);
        }
        out.u32(self.threads.len() as u32);
        for thread in &self.threads {
            out.u32(thread.handle);
            out.u8(thread.cpu_index);
            out.u64(thread.entry);
            out.u64(thread.argument);
            out.u64(thread.stack_top);
            out.u32(thread.priority as u32);
            out.u32(thread.preferred_core as u32);
            out.u64(thread.affinity_mask);
            out.u64(thread.tls_base);
            out.u8(u8::from(thread.started));
            out.u8(u8::from(thread.exited));
        }
        for waiting in self.waiting_on {
            match waiting {
                Some(handle) => {
                    out.u8(1);
                    out.u32(handle);
                }
                None => out.u8(0),
            }
        }
        out.u32(self.mutex_waiters.len() as u32);
        for waiter in &self.mutex_waiters {
            out.u64(waiter.address);
            out.u8(waiter.cpu_index);
            out.u32(waiter.self_tag);
        }
        out.u32(self.condvar_waiters.len() as u32);
        for waiter in &self.condvar_waiters {
            out.u64(waiter.mutex_address);
            out.u64(waiter.condvar_address);
            out.u8(waiter.cpu_index);
            out.u32(waiter.self_tag);
        }
        out.u32(self.services.len() as u32);
        for service in &self.services {
            out.u32(service.handle);
            out.u8(service.kind as u8);
        }
        out.u32(self.shared_mappings.len() as u32);
        for mapping in &self.shared_mappings {
            out.u32(mapping.handle);
            out.u64(mapping.address);
            out.u64(mapping.size);
            out.u32(mapping.permissions);
        }
        out.u32(self.next_service_handle);
        out.u32(self.next_thread_handle);
        match self.last_unsupported_svc {
            Some(value) => {
                out.u8(1);
                out.u16(value);
            }
            None => out.u8(0),
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let program_size = input.u64()?;
        let mut restored = Self::new(program_size)?;
        let heap_base = input.u64()?;
        if heap_base != restored.heap_base {
            return Err("Switch kernel state has an invalid heap base".into());
        }
        let heap_size = input.u64()?;
        restored
            .set_heap_size(heap_size)
            .map_err(|_| "Switch kernel state has an invalid heap size".to_string())?;

        let mapping_count = input.u32()? as usize;
        if mapping_count > 256 {
            return Err("Switch kernel state has too many memory mappings".into());
        }
        let mut mappings = Vec::with_capacity(mapping_count);
        for _ in 0..mapping_count {
            mappings.push(MirrorMapping {
                destination: input.u64()?,
                source: input.u64()?,
                size: input.u64()?,
            });
        }

        let thread_count = input.u32()? as usize;
        if thread_count == 0 || thread_count > 256 {
            return Err("Switch kernel state has an invalid thread count".into());
        }
        let mut threads = Vec::with_capacity(thread_count);
        for _ in 0..thread_count {
            let thread = HorizonThread {
                handle: input.u32()?,
                cpu_index: input.u8()?,
                entry: input.u64()?,
                argument: input.u64()?,
                stack_top: input.u64()?,
                priority: input.u32()? as i32,
                preferred_core: input.u32()? as i32,
                affinity_mask: input.u64()?,
                tls_base: input.u64()?,
                started: input.u8()? != 0,
                exited: input.u8()? != 0,
            };
            let cpu_index = usize::from(thread.cpu_index);
            if cpu_index >= CPU_COUNT
                || thread.tls_base != TLS_REGION_BASE + cpu_index as u64 * TLS_SLOT_SIZE
                || threads
                    .iter()
                    .any(|other: &HorizonThread| other.handle == thread.handle)
                || (!thread.exited
                    && threads.iter().any(|other: &HorizonThread| {
                        usize::from(other.cpu_index) == cpu_index && !other.exited
                    }))
            {
                return Err("Switch kernel state has invalid thread metadata".into());
            }
            threads.push(thread);
        }
        if !threads
            .iter()
            .any(|thread| thread.handle == MAIN_THREAD_HANDLE && thread.cpu_index == 0)
        {
            return Err("Switch kernel state is missing the main thread".into());
        }
        restored.threads = threads;
        restored.mappings.clear();
        for mapping in mappings {
            restored
                .map_memory(mapping.destination, mapping.source, mapping.size)
                .map_err(|_| "Switch kernel state has an invalid memory mapping".to_string())?;
        }
        for slot in &mut restored.waiting_on {
            *slot = if input.u8()? != 0 {
                Some(input.u32()?)
            } else {
                None
            };
        }
        if restored.waiting_on.iter().flatten().any(|handle| {
            !restored
                .threads
                .iter()
                .any(|thread| thread.handle == *handle)
        }) {
            return Err("Switch kernel state waits on an unknown thread handle".into());
        }

        let mutex_waiter_count = input.u32()? as usize;
        if mutex_waiter_count > 256 {
            return Err("Switch kernel state has too many mutex waiters".into());
        }
        restored.mutex_waiters.clear();
        for _ in 0..mutex_waiter_count {
            let waiter = MutexWaiter {
                address: input.u64()?,
                cpu_index: input.u8()?,
                self_tag: input.u32()?,
            };
            if usize::from(waiter.cpu_index) >= CPU_COUNT
                || guest_to_backing(waiter.address).is_none()
            {
                return Err("Switch kernel state has invalid mutex waiter metadata".into());
            }
            restored.mutex_waiters.push(waiter);
        }

        let condvar_waiter_count = input.u32()? as usize;
        if condvar_waiter_count > 256 {
            return Err("Switch kernel state has too many condition waiters".into());
        }
        restored.condvar_waiters.clear();
        for _ in 0..condvar_waiter_count {
            let waiter = CondvarWaiter {
                mutex_address: input.u64()?,
                condvar_address: input.u64()?,
                cpu_index: input.u8()?,
                self_tag: input.u32()?,
            };
            if usize::from(waiter.cpu_index) >= CPU_COUNT
                || guest_to_backing(waiter.mutex_address).is_none()
                || guest_to_backing(waiter.condvar_address).is_none()
            {
                return Err("Switch kernel state has invalid condition waiter metadata".into());
            }
            restored.condvar_waiters.push(waiter);
        }
        let service_count = input.u32()? as usize;
        if service_count > 256 {
            return Err("Switch kernel state has too many service sessions".into());
        }
        restored.services.clear();
        for _ in 0..service_count {
            let handle = input.u32()?;
            let kind = ServiceKind::from_u8(input.u8()?)
                .ok_or_else(|| "Switch kernel state has an invalid service type".to_string())?;
            if handle < SERVICE_HANDLE_BASE || restored.services.iter().any(|s| s.handle == handle)
            {
                return Err("Switch kernel state has invalid service metadata".into());
            }
            restored.services.push(ServiceSession { handle, kind });
        }
        let shared_mapping_count = input.u32()? as usize;
        if shared_mapping_count > 64 {
            return Err("Switch kernel state has too many shared-memory mappings".into());
        }
        restored.shared_mappings.clear();
        for _ in 0..shared_mapping_count {
            let handle = input.u32()?;
            let address = input.u64()?;
            let size = input.u64()?;
            let permissions = input.u32()?;
            restored
                .map_shared_memory(handle, address, size, permissions)
                .map_err(|_| {
                    "Switch kernel state has an invalid shared-memory mapping".to_string()
                })?;
        }
        restored.next_service_handle = input.u32()?;
        let max_service = restored
            .services
            .iter()
            .map(|s| s.handle)
            .max()
            .unwrap_or(SERVICE_HANDLE_BASE - 1);
        if restored.next_service_handle <= max_service {
            return Err("Switch kernel state has an invalid next service handle".into());
        }
        restored.next_thread_handle = input.u32()?;
        let max_handle = restored
            .threads
            .iter()
            .map(|thread| thread.handle)
            .max()
            .unwrap_or(MAIN_THREAD_HANDLE);
        if restored.next_thread_handle <= max_handle {
            return Err("Switch kernel state has an invalid next thread handle".into());
        }
        restored.last_unsupported_svc = if input.u8()? != 0 {
            Some(input.u16()?)
        } else {
            None
        };
        *self = restored;
        Ok(())
    }
}

struct SwitchBoard {
    dram: SparseMemory,
    image: ResourceBlob,
    video: VideoBuffer,
    audio: AudioBuffer,
    memory_epoch: u64,
}

impl SwitchBoard {
    fn new(image: ResourceBlob) -> Result<(Self, u64, u64), String> {
        let mut dram = SparseMemory::new(DRAM_SIZE)?;
        let (entry, program_size) = load_nro(&image, &mut dram)?;
        let mut board = Self {
            dram,
            image,
            video: {
                let mut video = VideoBuffer::new(VIDEO_WIDTH, VIDEO_HEIGHT);
                video.clear([0, 0, 0, 255]);
                video
            },
            audio: AudioBuffer::new(AUDIO_RATE, 2),
            memory_epoch: 0,
        };
        board.install_homebrew_environment()?;
        Ok((board, entry, program_size))
    }

    fn install_homebrew_environment(&mut self) -> Result<(), String> {
        let mut page = vec![0u8; ENV_REGION_SIZE as usize];
        let hints = supported_svc_hints();
        write_config_entry(&mut page, 0, 1, u64::from(MAIN_THREAD_HANDLE), 0);
        write_config_entry(&mut page, 1, 6, hints[0], hints[1]);
        write_config_entry(&mut page, 2, 7, 2, 0);
        write_config_entry(&mut page, 3, 10, u64::from(PROCESS_HANDLE), 0);
        write_config_entry(
            &mut page,
            4,
            14,
            0x4f4d_4e49_454d_5531,
            0x484f_5249_5a4f_4e31,
        );
        write_config_entry(&mut page, 5, 16, 0x0005_0100, 0);
        write_config_entry(&mut page, 6, 17, hints[2], 0);
        write_config_entry(&mut page, 7, 0, 0, 0);
        let trampoline = (LOADER_RETURN_ADDRESS - ENV_REGION_BASE) as usize;
        page[trampoline..trampoline + 4].copy_from_slice(&0xd400_00e1u32.to_le_bytes());
        self.dram.write(ENV_REGION_BASE - NRO_BASE, &page)
    }

    fn reset(&mut self) -> Result<(u64, u64), String> {
        let layout = load_nro(&self.image, &mut self.dram)?;
        self.install_homebrew_environment()?;
        self.video.clear([0, 0, 0, 255]);
        self.audio.begin_frame();
        self.memory_epoch = 0;
        Ok(layout)
    }
    fn end_frame(&mut self) {
        self.audio.begin_frame();
        for _ in 0..(AUDIO_RATE / 60) {
            self.audio.push_stereo(0.0, 0.0);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        self.dram.save(out);
        out.u64(self.memory_epoch);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.dram.load(input)?;
        self.memory_epoch = input.u64()?;
        self.video.clear([0, 0, 0, 255]);
        self.audio.begin_frame();
        Ok(())
    }

    fn read_guest(&self, address: u64, bytes: &mut [u8]) -> Result<(), String> {
        let offset = guest_to_backing(address)
            .ok_or_else(|| "Switch guest read address is outside DRAM".to_string())?;
        self.dram.read(offset, bytes)
    }

    fn write_guest(&mut self, address: u64, bytes: &[u8]) -> Result<(), String> {
        let end = address
            .checked_add(bytes.len() as u64)
            .ok_or_else(|| "Switch guest write address overflows".to_string())?;
        if !bytes.is_empty() && address < STACK_REGION_BASE && end > ENV_REGION_BASE {
            return Err("Switch homebrew environment is read-only".into());
        }
        let offset = guest_to_backing(address)
            .ok_or_else(|| "Switch guest write address is outside DRAM".to_string())?;
        self.dram.write(offset, bytes)?;
        if !bytes.is_empty() {
            self.memory_epoch = self.memory_epoch.wrapping_add(1);
        }
        Ok(())
    }

    fn read_guest_cstr(&self, address: u64, limit: usize) -> Result<Vec<u8>, String> {
        let mut out = Vec::new();
        for index in 0..limit {
            let mut byte = [0u8; 1];
            self.read_guest(address + index as u64, &mut byte)?;
            if byte[0] == 0 {
                return Ok(out);
            }
            out.push(byte[0]);
        }
        Err("Switch guest string is not terminated".into())
    }

    fn read_guest_u32(&self, address: u64) -> Result<u32, String> {
        let mut bytes = [0u8; 4];
        self.read_guest(address, &mut bytes)?;
        Ok(u32::from_le_bytes(bytes))
    }

    fn write_guest_u32(&mut self, address: u64, value: u32) -> Result<(), String> {
        self.write_guest(address, &value.to_le_bytes())
    }

    fn copy_guest(&mut self, destination: u64, source: u64, size: u64) -> Result<(), String> {
        let length =
            usize::try_from(size).map_err(|_| "Switch guest copy is too large".to_string())?;
        let mut bytes = vec![0; length];
        self.read_guest(source, &mut bytes)?;
        self.write_guest(destination, &bytes)
    }

    fn zero_guest(&mut self, address: u64, size: u64) -> Result<(), String> {
        let offset = guest_to_backing(address)
            .ok_or_else(|| "Switch guest zero address is outside DRAM".to_string())?;
        self.dram.zero(offset, size)?;
        if size != 0 {
            self.memory_epoch = self.memory_epoch.wrapping_add(1);
        }
        Ok(())
    }

    fn write_memory_info(&mut self, address: u64, info: HorizonMemoryInfo) -> Result<(), String> {
        let mut bytes = [0u8; 40];
        bytes[0..8].copy_from_slice(&info.address.to_le_bytes());
        bytes[8..16].copy_from_slice(&info.size.to_le_bytes());
        bytes[16..20].copy_from_slice(&info.memory_type.to_le_bytes());
        bytes[20..24].copy_from_slice(&info.attributes.to_le_bytes());
        bytes[24..28].copy_from_slice(&info.permissions.to_le_bytes());
        self.write_guest(address, &bytes)
    }
}

impl AArch64Bus for SwitchBoard {
    fn read8(&mut self, address: u64) -> u8 {
        guest_to_backing(address).map_or(0, |offset| self.dram.read8(offset))
    }

    fn write8(&mut self, address: u64, value: u8) {
        if (ENV_REGION_BASE..STACK_REGION_BASE).contains(&address) {
            return;
        }
        if let Some(offset) = guest_to_backing(address) {
            self.dram.write8(offset, value);
            self.memory_epoch = self.memory_epoch.wrapping_add(1);
        }
    }

    fn exclusive_epoch(&self) -> u64 {
        self.memory_epoch
    }
}
pub struct SwitchMachine {
    cpus: [AArch64Cpu; CPU_COUNT],
    board: SwitchBoard,
    kernel: HorizonKernel,
    scheduler: ProcessorCluster,
    input: InputState,
    powered: bool,
}

impl SwitchMachine {
    fn configured_cpu() -> AArch64Cpu {
        let mut cpu = AArch64Cpu::new();
        cpu.configure_system_counter(SYSTEM_TICK_RATE, GUEST_BUDGET_RATE)
            .expect("Switch system-counter constants are valid");
        cpu
    }

    pub fn from_nro(image: ResourceBlob) -> Result<Self, String> {
        let (board, entry, program_size) = SwitchBoard::new(image)?;
        let mut machine = Self {
            cpus: std::array::from_fn(|_| Self::configured_cpu()),
            board,
            kernel: HorizonKernel::new(program_size)?,
            scheduler: Self::new_scheduler(),
            input: InputState::default(),
            powered: true,
        };
        machine.boot_primary(entry);
        Ok(machine)
    }

    fn new_scheduler() -> ProcessorCluster {
        let mut scheduler =
            ProcessorCluster::new(ClockRate::hz(FRAMES_PER_SECOND), SCHEDULER_QUANTUM)
                .expect("Switch scheduler constants are valid");
        for index in 0..CPU_COUNT {
            let id = scheduler.add_processor(
                GuestIsa::ArmV8,
                ClockRate::hz(STEPS_PER_FRAME * FRAMES_PER_SECOND),
            );
            if index != 0 {
                scheduler.set_halted(id, true).unwrap();
            }
        }
        scheduler
    }

    fn boot_primary(&mut self, entry: u64) {
        self.cpus = std::array::from_fn(|_| Self::configured_cpu());
        let cpu = &mut self.cpus[0];
        cpu.reset_to(entry);
        cpu.sp = NRO_END - 0x10_000;
        cpu.tpidrro_el0 = TLS_REGION_BASE;
        cpu.x[0] = ENV_REGION_BASE;
        cpu.x[1] = u64::MAX;
        cpu.x[30] = LOADER_RETURN_ADDRESS;
        let _ = self.board.zero_guest(TLS_REGION_BASE, TLS_SLOT_SIZE);
        if let Some(thread) = self.kernel.thread_mut(MAIN_THREAD_HANDLE) {
            thread.entry = entry;
            thread.stack_top = cpu.sp;
            thread.tls_base = TLS_REGION_BASE;
            thread.started = true;
            thread.exited = false;
        }
    }

    fn processor_id(index: usize) -> crate::cluster::ProcessorId {
        crate::cluster::ProcessorId(index as u16)
    }

    fn sync_hid_input(&mut self) -> Result<(), String> {
        let Some(shared_base) = self.kernel.hid_shared_memory_address() else {
            return Ok(());
        };

        for player in 0..self.input.buttons.len() {
            let entry_base = shared_base + HID_NPAD_OFFSET + player as u64 * HID_NPAD_ENTRY_SIZE;
            self.board
                .write_guest_u32(entry_base, HID_NPAD_STYLE_FULL_KEY)?;

            let lifo = entry_base + HID_NPAD_FULL_KEY_LIFO_OFFSET;
            let mut header = [0u8; HID_LIFO_HEADER_SIZE as usize];
            self.board.read_guest(lifo, &mut header)?;
            let count =
                u64::from_le_bytes(header[24..32].try_into().unwrap()).min(HID_NPAD_STORAGE_COUNT);
            let tail = if count == 0 {
                0
            } else {
                u64::from_le_bytes(header[16..24].try_into().unwrap()) % HID_NPAD_STORAGE_COUNT
            };
            let next_tail = if count == 0 {
                0
            } else {
                (tail + 1) % HID_NPAD_STORAGE_COUNT
            };
            let sampling_number = if count == 0 {
                1
            } else {
                let current = lifo + HID_LIFO_HEADER_SIZE + tail * HID_NPAD_STORAGE_SIZE;
                let mut bytes = [0u8; 8];
                self.board.read_guest(current, &mut bytes)?;
                u64::from_le_bytes(bytes).wrapping_add(1).max(1)
            };

            header[8..16].copy_from_slice(&HID_NPAD_STORAGE_COUNT.to_le_bytes());
            header[16..24].copy_from_slice(&next_tail.to_le_bytes());
            header[24..32].copy_from_slice(&(count + 1).min(HID_NPAD_STORAGE_COUNT).to_le_bytes());
            self.board.write_guest(lifo, &header)?;

            let axes = self.input.axes[player];
            let mut storage = [0u8; HID_NPAD_STORAGE_SIZE as usize];
            storage[0..8].copy_from_slice(&sampling_number.to_le_bytes());
            storage[8..16].copy_from_slice(&sampling_number.to_le_bytes());
            storage[16..24].copy_from_slice(&hid_buttons(self.input.buttons[player]).to_le_bytes());
            storage[24..28].copy_from_slice(&hid_axis(axes[AXIS_LEFT_X], false).to_le_bytes());
            storage[28..32].copy_from_slice(&hid_axis(axes[AXIS_LEFT_Y], true).to_le_bytes());
            storage[32..36].copy_from_slice(&hid_axis(axes[AXIS_RIGHT_X], false).to_le_bytes());
            storage[36..40].copy_from_slice(&hid_axis(axes[AXIS_RIGHT_Y], true).to_le_bytes());
            storage[40..44].copy_from_slice(&HID_NPAD_ATTRIBUTE_CONNECTED.to_le_bytes());
            let storage_address = lifo + HID_LIFO_HEADER_SIZE + next_tail * HID_NPAD_STORAGE_SIZE;
            self.board.write_guest(storage_address, &storage)?;
        }
        Ok(())
    }

    fn wake_waiters_for(&mut self, handle: u32) {
        for cpu_index in 0..CPU_COUNT {
            if self.kernel.waiting_on[cpu_index] == Some(handle) {
                self.kernel.waiting_on[cpu_index] = None;
                self.cpus[cpu_index].x[0] = 0;
                self.cpus[cpu_index].x[1] = 0;
                let _ = self
                    .scheduler
                    .set_halted(Self::processor_id(cpu_index), false);
            }
        }
    }

    fn wake_cpu(&mut self, cpu_index: usize, result: u32) {
        self.cpus[cpu_index].x[0] = u64::from(result);
        let _ = self
            .scheduler
            .set_halted(Self::processor_id(cpu_index), false);
    }

    fn queue_mutex_waiter(&mut self, waiter: MutexWaiter) -> Result<(), u32> {
        let value = self
            .board
            .read_guest_u32(waiter.address)
            .map_err(|_| 0xD401u32)?;
        self.board
            .write_guest_u32(waiter.address, value | MUTEX_WAIT_FLAG)
            .map_err(|_| 0xD401u32)?;
        if !self.kernel.mutex_waiters.iter().any(|existing| {
            existing.address == waiter.address && existing.cpu_index == waiter.cpu_index
        }) {
            self.kernel.mutex_waiters.push(waiter);
        }
        let _ = self
            .scheduler
            .set_halted(Self::processor_id(usize::from(waiter.cpu_index)), true);
        Ok(())
    }

    fn release_mutex(&mut self, address: u64) -> Result<(), u32> {
        if let Some(position) = self
            .kernel
            .mutex_waiters
            .iter()
            .position(|waiter| waiter.address == address)
        {
            let waiter = self.kernel.mutex_waiters.remove(position);
            let has_more = self
                .kernel
                .mutex_waiters
                .iter()
                .any(|other| other.address == address);
            let value = waiter.self_tag | if has_more { MUTEX_WAIT_FLAG } else { 0 };
            self.board
                .write_guest_u32(address, value)
                .map_err(|_| 0xD401u32)?;
            self.wake_cpu(usize::from(waiter.cpu_index), 0);
        } else {
            self.board
                .write_guest_u32(address, 0)
                .map_err(|_| 0xD401u32)?;
        }
        Ok(())
    }

    fn acquire_mutex_or_queue(&mut self, waiter: MutexWaiter) -> Result<(), u32> {
        let value = self
            .board
            .read_guest_u32(waiter.address)
            .map_err(|_| 0xD401u32)?;
        if value & !MUTEX_WAIT_FLAG == 0 {
            self.board
                .write_guest_u32(waiter.address, waiter.self_tag)
                .map_err(|_| 0xD401u32)?;
            self.wake_cpu(usize::from(waiter.cpu_index), 0);
            Ok(())
        } else {
            self.queue_mutex_waiter(waiter)
        }
    }

    fn signal_condition(&mut self, address: u64, count: i32) -> Result<(), u32> {
        if count == 0 {
            return Ok(());
        }
        let mut remaining = if count < 0 {
            usize::MAX
        } else {
            count as usize
        };
        let mut selected = Vec::new();
        let mut position = 0;
        while position < self.kernel.condvar_waiters.len() && remaining != 0 {
            if self.kernel.condvar_waiters[position].condvar_address == address {
                selected.push(self.kernel.condvar_waiters.remove(position));
                remaining = remaining.saturating_sub(1);
            } else {
                position += 1;
            }
        }
        for waiter in selected {
            self.acquire_mutex_or_queue(MutexWaiter {
                address: waiter.mutex_address,
                cpu_index: waiter.cpu_index,
                self_tag: waiter.self_tag,
            })?;
        }
        Ok(())
    }

    fn initialize_thread_cpu(&mut self, handle: u32) -> Result<usize, u32> {
        let thread = self.kernel.thread(handle).cloned().ok_or(0xE401u32)?;
        let index = usize::from(thread.cpu_index);
        self.cpus[index] = Self::configured_cpu();
        self.cpus[index].reset_to(thread.entry);
        self.cpus[index].x[0] = thread.argument;
        self.cpus[index].sp = thread.stack_top;
        self.cpus[index].tpidrro_el0 = thread.tls_base;
        self.board
            .zero_guest(thread.tls_base, TLS_SLOT_SIZE)
            .map_err(|_| 0xD401u32)?;
        Ok(index)
    }

    fn handle_service_request(&mut self, index: usize, handle: u32) -> Result<(), u32> {
        let kind = self.kernel.service_kind(handle).ok_or(0xE401u32)?;
        let tls = self.cpus[index].tpidrro_el0;
        let mut buffer = vec![0u8; 0x100];
        self.board
            .read_guest(tls, &mut buffer)
            .map_err(|_| 0xD401u32)?;
        let request = switch_ipc::parse_request(&buffer).map_err(|_| 0xF601u32)?;
        if request.close {
            return Ok(());
        }
        let domain = request.domain_object.is_some();
        let mut result = 0u32;
        let mut data = Vec::new();
        let mut copy_handles = Vec::new();
        let mut move_handles = Vec::new();
        let mut domain_objects = Vec::new();

        if matches!(request.command_type, 5 | 7) {
            match request.command_id {
                0 => data.extend_from_slice(&1u32.to_le_bytes()),
                2 => {
                    let clone = self.kernel.allocate_service(kind);
                    move_handles.push(clone);
                }
                3 => data.extend_from_slice(&0x500u16.to_le_bytes()),
                _ => result = 0xF601,
            }
        } else if matches!(request.command_type, 4 | 6) {
            match kind {
                ServiceKind::Sm => match request.command_id {
                    0 => {}
                    1 => {
                        let name = request.payload.get(..8).ok_or(0xD401u32)?;
                        if let Some(service) = HorizonKernel::service_for_name(name) {
                            move_handles.push(self.kernel.allocate_service(service));
                        } else {
                            result = 0xF601;
                        }
                    }
                    _ => result = 0xF601,
                },
                ServiceKind::AppletAe => match (request.domain_object, request.command_id) {
                    (Some(1), 200 | 201) => domain_objects.push(2),
                    (Some(2), 20) => domain_objects.push(3),
                    (Some(2), 10) => domain_objects.push(4),
                    (Some(2), 11) => domain_objects.push(5),
                    (Some(2), 0) => domain_objects.push(6),
                    (Some(2), 1) => domain_objects.push(7),
                    (Some(2), 2) => domain_objects.push(8),
                    (Some(2), 3) => domain_objects.push(9),
                    (Some(2), 4) => domain_objects.push(10),
                    (Some(2), 1000) => domain_objects.push(11),
                    (Some(3), 11) => data.extend_from_slice(&[0; 8]),
                    (Some(4), 0) => data.extend_from_slice(&[0; 4]),
                    (Some(6), 0) => {
                        copy_handles.push(self.kernel.allocate_service(ServiceKind::Event));
                    }
                    (Some(6), 5) => data.push(0),
                    (Some(6), 6) => data.extend_from_slice(&0u32.to_le_bytes()),
                    (Some(6), 9) => data.push(1),
                    (Some(7), 11 | 12) => {}
                    (Some(7), 40) => data.extend_from_slice(&1u64.to_le_bytes()),
                    (Some(8), 1) => data.extend_from_slice(&1u64.to_le_bytes()),
                    _ => result = 0xF601,
                },
                ServiceKind::Hid => match request.command_id {
                    0 => move_handles
                        .push(self.kernel.allocate_service(ServiceKind::HidAppletResource)),
                    100 | 102 | 103 | 109 => {}
                    101 => data.extend_from_slice(&HID_NPAD_STYLE_FULL_KEY.to_le_bytes()),
                    _ => result = 0xF601,
                },
                ServiceKind::HidAppletResource => match request.command_id {
                    0 => copy_handles
                        .push(self.kernel.allocate_service(ServiceKind::HidSharedMemory)),
                    _ => result = 0xF601,
                },
                ServiceKind::HidSharedMemory | ServiceKind::Event => result = 0xF601,
                ServiceKind::ViRoot => match request.command_id {
                    0 => move_handles.push(
                        self.kernel
                            .allocate_service(ServiceKind::ViApplicationDisplay),
                    ),
                    _ => result = 0xF601,
                },
                ServiceKind::ViApplicationDisplay => match request.command_id {
                    100 => {
                        move_handles.push(self.kernel.allocate_service(ServiceKind::ViBinderRelay))
                    }
                    1010 | 1011 => data.extend_from_slice(&1u64.to_le_bytes()),
                    1020 | 2021 | 2031 | 2101 => {}
                    1102 => {
                        data.extend_from_slice(&1280i64.to_le_bytes());
                        data.extend_from_slice(&720i64.to_le_bytes());
                    }
                    2020 | 2030 => {
                        let parcel = vi_native_window_parcel(1);
                        if let Some(output) = request.recv_buffers.first() {
                            if output.size < parcel.len() as u64 {
                                result = 0xF601;
                            } else if self.board.write_guest(output.address, &parcel).is_err() {
                                result = 0xD401;
                            } else if request.command_id == 2020 {
                                data.extend_from_slice(&(parcel.len() as u64).to_le_bytes());
                            } else {
                                data.extend_from_slice(&1u64.to_le_bytes());
                                data.extend_from_slice(&(parcel.len() as u64).to_le_bytes());
                            }
                        } else {
                            result = 0xF601;
                        }
                    }
                    5202 => copy_handles.push(self.kernel.allocate_service(ServiceKind::Event)),
                    _ => result = 0xF601,
                },
                ServiceKind::ViBinderRelay => match request.command_id {
                    0 | 3 => {
                        let code = request
                            .payload
                            .get(4..8)
                            .map(|bytes| u32::from_le_bytes(bytes.try_into().unwrap()));
                        let reply = match code {
                            Some(10) => Some(binder_connect_reply()),
                            Some(11) => Some(binder_status_reply()),
                            _ => None,
                        };
                        match (request.recv_buffers.first(), reply) {
                            (Some(output), Some(reply)) if output.size >= reply.len() as u64 => {
                                if self.board.write_guest(output.address, &reply).is_err() {
                                    result = 0xD401;
                                }
                            }
                            _ => result = 0xF601,
                        }
                    }
                    1 => {}
                    2 => copy_handles.push(self.kernel.allocate_service(ServiceKind::Event)),
                    _ => result = 0xF601,
                },
                ServiceKind::TimeRoot => {
                    let child = match request.command_id {
                        0 => Some(ServiceKind::TimeUserClock),
                        1 => Some(ServiceKind::TimeNetworkClock),
                        2 => Some(ServiceKind::TimeSteadyClock),
                        3 => Some(ServiceKind::TimeZone),
                        4 => Some(ServiceKind::TimeLocalClock),
                        _ => None,
                    };
                    if let Some(child) = child {
                        move_handles.push(self.kernel.allocate_service(child));
                    } else {
                        result = 0xF601;
                    }
                }
                ServiceKind::TimeUserClock
                | ServiceKind::TimeNetworkClock
                | ServiceKind::TimeLocalClock => {
                    if request.command_id == 0 {
                        let seconds =
                            1_700_000_000u64 + self.cpus[index].cycles / GUEST_BUDGET_RATE;
                        data.extend_from_slice(&seconds.to_le_bytes());
                    } else {
                        result = 0xF601;
                    }
                }
                ServiceKind::TimeSteadyClock | ServiceKind::TimeZone => {
                    result = 0xF601;
                }
                ServiceKind::FsProxy => match (request.domain_object, request.command_id) {
                    (Some(1), 1) => {}
                    (Some(1), 18) => domain_objects.push(2),
                    _ => result = 0xF601,
                },
            }
        } else {
            result = 0xF601;
        }

        let response = switch_ipc::encode_response(
            switch_ipc::Response {
                result,
                data: &data,
                copy_handles: &copy_handles,
                move_handles: &move_handles,
                domain_objects: &domain_objects,
            },
            domain,
        )
        .map_err(|_| 0xF601u32)?;
        self.board
            .write_guest(tls, &response)
            .map_err(|_| 0xD401u32)
    }

    fn connect_named_port(&mut self, index: usize) -> Result<u32, u32> {
        let name = self
            .board
            .read_guest_cstr(self.cpus[index].x[1], 12)
            .map_err(|_| 0xD401u32)?;
        if name.as_slice() != b"sm:" {
            return Err(0xF601);
        }
        Ok(self.kernel.allocate_service(ServiceKind::Sm))
    }

    fn handle_svc(&mut self, index: usize, svc: u16) -> bool {
        match svc {
            0x01 => {
                let size = self.cpus[index].x[1];
                match self.kernel.set_heap_size(size) {
                    Ok(address) => {
                        self.cpus[index].x[0] = 0;
                        self.cpus[index].x[1] = address;
                    }
                    Err(result) => self.cpus[index].x[0] = u64::from(result),
                }
            }
            0x04 => {
                let destination = self.cpus[index].x[0];
                let source = self.cpus[index].x[1];
                let size = self.cpus[index].x[2];
                match self.kernel.map_memory(destination, source, size) {
                    Ok(()) => match self.board.copy_guest(destination, source, size) {
                        Ok(()) => self.cpus[index].x[0] = 0,
                        Err(_) => {
                            let _ = self.kernel.unmap_memory(destination, source, size);
                            self.cpus[index].x[0] = 0xD401;
                        }
                    },
                    Err(result) => self.cpus[index].x[0] = u64::from(result),
                }
            }
            0x05 => {
                let destination = self.cpus[index].x[0];
                let source = self.cpus[index].x[1];
                let size = self.cpus[index].x[2];
                let exists = self.kernel.mappings.iter().any(|mapping| {
                    mapping.destination == destination
                        && mapping.source == source
                        && mapping.size == size
                });
                if exists
                    && self.board.copy_guest(source, destination, size).is_ok()
                    && self.board.zero_guest(destination, size).is_ok()
                    && self.kernel.unmap_memory(destination, source, size).is_ok()
                {
                    self.cpus[index].x[0] = 0;
                } else {
                    self.cpus[index].x[0] = 0xD401;
                }
            }
            0x06 => {
                let output = self.cpus[index].x[0];
                let address = self.cpus[index].x[2];
                let Some(info) = self.kernel.query_memory(address) else {
                    self.cpus[index].x[0] = 0xCC01;
                    return true;
                };
                if self.board.write_memory_info(output, info).is_ok() {
                    self.cpus[index].x[0] = 0;
                    self.cpus[index].x[1] = 0;
                } else {
                    self.cpus[index].x[0] = 0xCC01;
                }
            }
            0x07 => {
                for cpu in &mut self.cpus {
                    cpu.halt();
                }
                for processor in 0..CPU_COUNT {
                    let _ = self
                        .scheduler
                        .set_halted(Self::processor_id(processor), true);
                }
                for thread in &mut self.kernel.threads {
                    thread.exited = true;
                }
                self.powered = false;
                return false;
            }
            0x08 => {
                let entry = self.cpus[index].x[1];
                let argument = self.cpus[index].x[2];
                let stack_top = self.cpus[index].x[3];
                let priority = self.cpus[index].x[4] as u32 as i32;
                let requested_core = self.cpus[index].x[5] as u32 as i32;
                match self.kernel.create_thread(
                    entry,
                    argument,
                    stack_top,
                    priority,
                    requested_core,
                ) {
                    Ok((handle, _, _)) => match self.initialize_thread_cpu(handle) {
                        Ok(_) => {
                            self.cpus[index].x[0] = 0;
                            self.cpus[index].x[1] = u64::from(handle);
                        }
                        Err(result) => {
                            if let Some(thread) = self.kernel.thread_mut(handle) {
                                thread.exited = true;
                            }
                            self.cpus[index].x[0] = u64::from(result);
                        }
                    },
                    Err(result) => self.cpus[index].x[0] = u64::from(result),
                }
            }
            0x09 => {
                let handle = self.cpus[index].x[0] as u32;
                let Some(thread) = self.kernel.thread(handle).cloned() else {
                    self.cpus[index].x[0] = 0xE401;
                    return true;
                };
                if thread.started || thread.exited {
                    self.cpus[index].x[0] = 0xE401;
                } else {
                    self.kernel.thread_mut(handle).unwrap().started = true;
                    let _ = self
                        .scheduler
                        .set_halted(Self::processor_id(usize::from(thread.cpu_index)), false);
                    self.cpus[index].x[0] = 0;
                }
            }
            0x0A => {
                if let Some(handle) = self.kernel.thread_handle_for_cpu(index) {
                    if let Some(thread) = self.kernel.thread_mut(handle) {
                        thread.exited = true;
                    }
                    self.wake_waiters_for(handle);
                }
                self.cpus[index].halt();
                let _ = self.scheduler.set_halted(Self::processor_id(index), true);
                if index == 0 {
                    self.powered = false;
                }
                return false;
            }
            0x0B => return false,
            0x0C => {
                let raw_handle = self.cpus[index].x[1] as u32;
                let Some(handle) = self.kernel.resolve_thread_handle(raw_handle, index) else {
                    self.cpus[index].x[0] = 0xE401;
                    return true;
                };
                let priority = self.kernel.thread(handle).unwrap().priority;
                self.cpus[index].x[0] = 0;
                self.cpus[index].x[1] = u64::from(priority as u32);
            }
            0x0D => {
                let raw_handle = self.cpus[index].x[0] as u32;
                let priority = self.cpus[index].x[1] as u32 as i32;
                let Some(handle) = self.kernel.resolve_thread_handle(raw_handle, index) else {
                    self.cpus[index].x[0] = 0xE401;
                    return true;
                };
                if !(0..=63).contains(&priority) {
                    self.cpus[index].x[0] = 0xCA01;
                } else {
                    self.kernel.thread_mut(handle).unwrap().priority = priority;
                    self.cpus[index].x[0] = 0;
                }
            }
            0x0E => {
                let raw_handle = self.cpus[index].x[2] as u32;
                let Some(handle) = self.kernel.resolve_thread_handle(raw_handle, index) else {
                    self.cpus[index].x[0] = 0xE401;
                    return true;
                };
                let thread = self.kernel.thread(handle).unwrap();
                self.cpus[index].x[0] = 0;
                self.cpus[index].x[1] = u64::from(thread.preferred_core as u32);
                self.cpus[index].x[2] = thread.affinity_mask;
            }
            0x0F => {
                let raw_handle = self.cpus[index].x[0] as u32;
                let preferred_core = self.cpus[index].x[1] as u32 as i32;
                let affinity_mask = self.cpus[index].x[2];
                let Some(handle) = self.kernel.resolve_thread_handle(raw_handle, index) else {
                    self.cpus[index].x[0] = 0xE401;
                    return true;
                };
                if !(-1..CPU_COUNT as i32).contains(&preferred_core)
                    || affinity_mask == 0
                    || affinity_mask & !((1u64 << CPU_COUNT) - 1) != 0
                {
                    self.cpus[index].x[0] = 0xCA01;
                } else {
                    let thread = self.kernel.thread_mut(handle).unwrap();
                    thread.preferred_core = preferred_core;
                    thread.affinity_mask = affinity_mask;
                    self.cpus[index].x[0] = 0;
                }
            }
            0x10 => self.cpus[index].x[0] = index as u64,
            0x13 => {
                let handle = self.cpus[index].x[0] as u32;
                let address = self.cpus[index].x[1];
                let size = self.cpus[index].x[2];
                let permissions = self.cpus[index].x[3] as u32;
                match self
                    .kernel
                    .map_shared_memory(handle, address, size, permissions)
                {
                    Ok(()) => match self.board.zero_guest(address, size) {
                        Ok(()) => match self.sync_hid_input() {
                            Ok(()) => self.cpus[index].x[0] = 0,
                            Err(_) => {
                                let _ = self.kernel.unmap_shared_memory(handle, address, size);
                                self.cpus[index].x[0] = 0xD401;
                            }
                        },
                        Err(_) => {
                            let _ = self.kernel.unmap_shared_memory(handle, address, size);
                            self.cpus[index].x[0] = 0xD401;
                        }
                    },
                    Err(result) => self.cpus[index].x[0] = u64::from(result),
                }
            }
            0x14 => {
                let handle = self.cpus[index].x[0] as u32;
                let address = self.cpus[index].x[1];
                let size = self.cpus[index].x[2];
                match self.kernel.unmap_shared_memory(handle, address, size) {
                    Ok(()) => self.cpus[index].x[0] = 0,
                    Err(result) => self.cpus[index].x[0] = u64::from(result),
                }
            }
            0x16 => {
                let handle = self.cpus[index].x[0] as u32;
                if self.kernel.close_service(handle) {
                    self.cpus[index].x[0] = 0;
                } else if handle == MAIN_THREAD_HANDLE {
                    self.cpus[index].x[0] = 0xE401;
                } else if let Some(position) = self
                    .kernel
                    .threads
                    .iter()
                    .position(|thread| thread.handle == handle && thread.exited)
                {
                    self.kernel.threads.remove(position);
                    self.cpus[index].x[0] = 0;
                } else {
                    self.cpus[index].x[0] = 0xE401;
                }
            }
            0x18 => {
                let handles = self.cpus[index].x[1];
                let count = self.cpus[index].x[2] as u32;
                let timeout = self.cpus[index].x[3] as i64;
                if count != 1 {
                    self.cpus[index].x[0] = 0xCA01;
                    return true;
                }
                let mut bytes = [0u8; 4];
                if self.board.read_guest(handles, &mut bytes).is_err() {
                    self.cpus[index].x[0] = 0xD401;
                    return true;
                }
                let handle = u32::from_le_bytes(bytes);
                let Some(thread) = self.kernel.thread(handle) else {
                    self.cpus[index].x[0] = 0xE401;
                    return true;
                };
                if thread.exited {
                    self.cpus[index].x[0] = 0;
                    self.cpus[index].x[1] = 0;
                } else if timeout == 0 {
                    self.cpus[index].x[0] = 0xEA01;
                } else {
                    self.kernel.waiting_on[index] = Some(handle);
                    self.cpus[index].x[0] = 0;
                    self.cpus[index].x[1] = 0;
                    let _ = self.scheduler.set_halted(Self::processor_id(index), true);
                    return false;
                }
            }
            0x1A => {
                let wait_tag = self.cpus[index].x[0] as u32;
                let address = self.cpus[index].x[1];
                let self_tag = self.cpus[index].x[2] as u32;
                let current = match self.board.read_guest_u32(address) {
                    Ok(value) => value,
                    Err(_) => {
                        self.cpus[index].x[0] = 0xD401;
                        return true;
                    }
                };
                if current & !MUTEX_WAIT_FLAG == 0 {
                    if self.board.write_guest_u32(address, self_tag).is_ok() {
                        self.cpus[index].x[0] = 0;
                    } else {
                        self.cpus[index].x[0] = 0xD401;
                    }
                } else if current & !MUTEX_WAIT_FLAG != wait_tag {
                    self.cpus[index].x[0] = 0xE401;
                } else {
                    let waiter = MutexWaiter {
                        address,
                        cpu_index: index as u8,
                        self_tag,
                    };
                    match self.queue_mutex_waiter(waiter) {
                        Ok(()) => {
                            self.cpus[index].x[0] = 0;
                            return false;
                        }
                        Err(result) => self.cpus[index].x[0] = u64::from(result),
                    }
                }
            }
            0x1B => {
                let address = self.cpus[index].x[0];
                match self.release_mutex(address) {
                    Ok(()) => self.cpus[index].x[0] = 0,
                    Err(result) => self.cpus[index].x[0] = u64::from(result),
                }
            }
            0x1C => {
                let mutex_address = self.cpus[index].x[0];
                let condvar_address = self.cpus[index].x[1];
                let self_tag = self.cpus[index].x[2] as u32;
                let timeout = self.cpus[index].x[3] as i64;
                let current = match self.board.read_guest_u32(mutex_address) {
                    Ok(value) => value,
                    Err(_) => {
                        self.cpus[index].x[0] = 0xD401;
                        return true;
                    }
                };
                if current & !MUTEX_WAIT_FLAG != self_tag {
                    self.cpus[index].x[0] = 0xE401;
                    return true;
                }
                if let Err(result) = self.release_mutex(mutex_address) {
                    self.cpus[index].x[0] = u64::from(result);
                    return true;
                }
                if timeout == 0 {
                    self.cpus[index].x[0] = 0xEA01;
                } else {
                    self.kernel.condvar_waiters.push(CondvarWaiter {
                        mutex_address,
                        condvar_address,
                        cpu_index: index as u8,
                        self_tag,
                    });
                    self.cpus[index].x[0] = 0;
                    let _ = self.scheduler.set_halted(Self::processor_id(index), true);
                    return false;
                }
            }
            0x1D => {
                let address = self.cpus[index].x[0];
                let count = self.cpus[index].x[1] as u32 as i32;
                match self.signal_condition(address, count) {
                    Ok(()) => self.cpus[index].x[0] = 0,
                    Err(result) => self.cpus[index].x[0] = u64::from(result),
                }
            }
            0x1E => {
                self.cpus[index].x[0] =
                    self.cpus[index].cycles.saturating_mul(SYSTEM_TICK_RATE) / GUEST_BUDGET_RATE;
            }
            0x1F => match self.connect_named_port(index) {
                Ok(handle) => {
                    self.cpus[index].x[0] = 0;
                    self.cpus[index].x[1] = u64::from(handle);
                }
                Err(result) => self.cpus[index].x[0] = u64::from(result),
            },
            0x21 => {
                let handle = self.cpus[index].x[0] as u32;
                self.cpus[index].x[0] = match self.handle_service_request(index, handle) {
                    Ok(()) => 0,
                    Err(result) => u64::from(result),
                };
            }
            0x24 => {
                self.cpus[index].x[0] = 0;
                self.cpus[index].x[1] = 1;
            }
            0x25 => {
                let raw_handle = self.cpus[index].x[1] as u32;
                let Some(handle) = self.kernel.resolve_thread_handle(raw_handle, index) else {
                    self.cpus[index].x[0] = 0xE401;
                    return true;
                };
                self.cpus[index].x[0] = 0;
                self.cpus[index].x[1] = u64::from(handle);
            }
            0x27 | 0x2A | 0x2B => self.cpus[index].x[0] = 0,
            0x29 => {
                let id = self.cpus[index].x[1];
                let sub_id = self.cpus[index].x[3];
                if let Some(value) = self.kernel.get_info(id, sub_id) {
                    self.cpus[index].x[0] = 0;
                    self.cpus[index].x[1] = value;
                } else {
                    self.cpus[index].x[0] = 0xF001;
                }
            }
            _ => {
                self.kernel.last_unsupported_svc = Some(svc);
                self.cpus[index].halt();
                self.powered = false;
                return false;
            }
        }
        true
    }

    fn run_cores(&mut self) {
        for batch in self.scheduler.advance_batches(1) {
            for slice in batch.slices {
                let index = slice.processor.0 as usize;
                let target = self.cpus[index].cycles.saturating_add(slice.cycles);
                while self.powered
                    && self.cpus[index].cycles < target
                    && !self.cpus[index].invalid_instruction()
                    && !self.cpus[index].halted()
                {
                    self.cpus[index].step(&mut self.board);
                    if let Some(svc) = self.cpus[index].take_pending_svc() {
                        if !self.handle_svc(index, svc) {
                            break;
                        }
                    }
                }
                if self.cpus[index].invalid_instruction() || self.cpus[index].halted() {
                    self.scheduler.set_halted(slice.processor, true).unwrap();
                    if self.cpus[index].invalid_instruction() && index == 0 {
                        self.powered = false;
                    }
                }
            }
        }
    }
}

impl Machine for SwitchMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Switch
    }

    fn reset(&mut self) {
        match self.board.reset() {
            Ok((entry, program_size)) => {
                if self.kernel.reset(program_size).is_err() {
                    self.powered = false;
                    return;
                }
                self.boot_primary(entry);
                self.scheduler = Self::new_scheduler();
                self.input = InputState::default();
                self.powered = true;
            }
            Err(_) => self.powered = false,
        }
    }

    fn run_frame(&mut self, input: &InputState) {
        self.input = input.clone();
        let _ = self.sync_hid_input();
        if self.powered {
            self.run_cores();
        }
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
        let mut out = StateWriter::new(PlatformId::Switch, STATE_VERSION);
        for cpu in &self.cpus {
            cpu.save(&mut out);
        }
        self.kernel.save(&mut out);
        self.scheduler.save(&mut out);
        self.board.save(&mut out);
        out.u8(u8::from(self.powered));
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Switch, STATE_VERSION)?;
        for cpu in &mut self.cpus {
            cpu.load(&mut input)?;
        }
        self.kernel.load(&mut input)?;
        self.scheduler.load(&mut input)?;
        self.board.load(&mut input)?;
        self.powered = input.u8()? != 0;
        input.finish()?;
        self.input = InputState::default();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn movz_x(rd: u32, immediate: u16) -> u32 {
        0xd280_0000 | (u32::from(immediate) << 5) | rd
    }

    fn str_x(rt: u32, rn: u32) -> u32 {
        0xf900_0000 | (rn << 5) | rt
    }

    fn svc(id: u16) -> u32 {
        0xd400_0001 | (u32::from(id) << 5)
    }

    fn nro_bytes(program: &[u32]) -> Vec<u8> {
        let text_size = 0x1000usize;
        let mut image = vec![0u8; text_size];
        image[0..4].copy_from_slice(&0x1400_0020u32.to_le_bytes());
        image[0x10..0x14].copy_from_slice(b"NRO0");
        image[0x18..0x1c].copy_from_slice(&(text_size as u32).to_le_bytes());
        image[0x20..0x24].copy_from_slice(&0u32.to_le_bytes());
        image[0x24..0x28].copy_from_slice(&(text_size as u32).to_le_bytes());
        image[0x28..0x2c].copy_from_slice(&(text_size as u32).to_le_bytes());
        image[0x30..0x34].copy_from_slice(&(text_size as u32).to_le_bytes());
        for (index, instruction) in program.iter().enumerate() {
            let start = 0x80 + index * 4;
            image[start..start + 4].copy_from_slice(&instruction.to_le_bytes());
        }
        image
    }

    fn install_cmif_request(
        machine: &mut SwitchMachine,
        index: usize,
        command_type: u16,
        command_id: u32,
        payload: &[u8],
        send_pid: bool,
    ) {
        let mut bytes = vec![0u8; 0x100];
        let data_start = if send_pid { 20usize } else { 8usize };
        let cmif_start = (data_start + 15) & !15;
        let end = cmif_start + 16 + payload.len();
        let data_words = (end - data_start).div_ceil(4);
        bytes[0..4].copy_from_slice(&u32::from(command_type).to_le_bytes());
        let word1 = data_words as u32 | if send_pid { 1u32 << 31 } else { 0 };
        bytes[4..8].copy_from_slice(&word1.to_le_bytes());
        if send_pid {
            bytes[8..12].copy_from_slice(&1u32.to_le_bytes());
        }
        bytes[cmif_start..cmif_start + 4].copy_from_slice(&0x4943_4653u32.to_le_bytes());
        bytes[cmif_start + 8..cmif_start + 12].copy_from_slice(&command_id.to_le_bytes());
        bytes[cmif_start + 16..end].copy_from_slice(payload);
        machine
            .board
            .write_guest(machine.cpus[index].tpidrro_el0, &bytes)
            .unwrap();
    }

    fn install_cmif_request_with_recv_buffer(
        machine: &mut SwitchMachine,
        index: usize,
        command_id: u32,
        payload: &[u8],
        output_address: u64,
        output_size: u64,
        send_pid: bool,
    ) {
        let mut bytes = vec![0u8; 0x100];
        let packed = (((output_address >> 36) as u32 & 0x3f_ffff) << 2)
            | (((output_size >> 32) as u32 & 0xf) << 24)
            | (((output_address >> 32) as u32 & 0xf) << 28);
        bytes[0..4].copy_from_slice(&(4u32 | (1 << 24)).to_le_bytes());
        let descriptor = if send_pid {
            bytes[8..12].copy_from_slice(&1u32.to_le_bytes());
            20usize
        } else {
            8usize
        };
        bytes[descriptor..descriptor + 4].copy_from_slice(&(output_size as u32).to_le_bytes());
        bytes[descriptor + 4..descriptor + 8]
            .copy_from_slice(&(output_address as u32).to_le_bytes());
        bytes[descriptor + 8..descriptor + 12].copy_from_slice(&packed.to_le_bytes());
        let data_start = descriptor + 12;
        let cmif_start = (data_start + 15) & !15;
        let end = cmif_start + 16 + payload.len();
        let data_words = (end - data_start).div_ceil(4);
        let word1 = data_words as u32 | if send_pid { 1u32 << 31 } else { 0 };
        bytes[4..8].copy_from_slice(&word1.to_le_bytes());
        bytes[cmif_start..cmif_start + 4].copy_from_slice(&0x4943_4653u32.to_le_bytes());
        bytes[cmif_start + 8..cmif_start + 12].copy_from_slice(&command_id.to_le_bytes());
        bytes[cmif_start + 16..end].copy_from_slice(payload);
        machine
            .board
            .write_guest(machine.cpus[index].tpidrro_el0, &bytes)
            .unwrap();
    }

    fn tls_u32(machine: &SwitchMachine, index: usize, offset: u64) -> u32 {
        let mut bytes = [0u8; 4];
        machine
            .board
            .read_guest(machine.cpus[index].tpidrro_el0 + offset, &mut bytes)
            .unwrap();
        u32::from_le_bytes(bytes)
    }

    #[test]
    fn nro_loads_and_primary_core_executes_aarch64() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[movz_x(1, 0x1234), str_x(1, 2), svc(7)]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        machine.cpus[0].x[2] = NRO_BASE + 0x2000;
        machine.run_frame(&InputState::default());
        let mut bytes = [0u8; 8];
        machine.board.dram.read(0x2000, &mut bytes).unwrap();
        assert_eq!(u64::from_le_bytes(bytes), 0x1234);
        assert_eq!(machine.cpus[0].last_svc, Some(7));
        assert!(!machine.cpus[0].invalid_instruction());
        assert!(!machine.powered);
    }
    #[test]
    fn horizon_memory_syscalls_follow_libnx_register_abi() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();

        machine.cpus[0].x[1] = 0x40_0000;
        assert!(machine.handle_svc(0, 0x01));
        assert_eq!(machine.cpus[0].x[0], 0);
        let heap_base = machine.cpus[0].x[1];
        assert_eq!(heap_base, machine.kernel.heap_base);
        assert_eq!(machine.kernel.heap_size, 0x40_0000);

        machine.cpus[0].x[1] = 4;
        machine.cpus[0].x[3] = 0;
        assert!(machine.handle_svc(0, 0x29));
        assert_eq!(machine.cpus[0].x[0], 0);
        assert_eq!(machine.cpus[0].x[1], heap_base);

        machine.cpus[0].x[1] = 0x1000;
        assert!(machine.handle_svc(0, 0x01));
        assert_eq!(machine.cpus[0].x[0], 0xCA01);
        assert_eq!(machine.kernel.heap_size, 0x40_0000);
    }

    #[test]
    fn query_memory_writes_libnx_memory_info_layout() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        machine.kernel.set_heap_size(0x20_0000).unwrap();
        let output = STACK_REGION_BASE + 0x1000;
        machine.cpus[0].x[0] = output;
        machine.cpus[0].x[2] = machine.kernel.heap_base;
        assert!(machine.handle_svc(0, 0x06));
        assert_eq!(machine.cpus[0].x[0], 0);
        assert_eq!(machine.cpus[0].x[1], 0);

        let mut bytes = [0u8; 40];
        machine
            .board
            .dram
            .read(output - NRO_BASE, &mut bytes)
            .unwrap();
        assert_eq!(
            u64::from_le_bytes(bytes[0..8].try_into().unwrap()),
            machine.kernel.heap_base
        );
        assert_eq!(
            u64::from_le_bytes(bytes[8..16].try_into().unwrap()),
            0x20_0000
        );
        assert_eq!(u32::from_le_bytes(bytes[16..20].try_into().unwrap()), 5);
        assert_eq!(u32::from_le_bytes(bytes[24..28].try_into().unwrap()), 3);
    }

    #[test]
    fn horizon_regions_cover_modern_libnx_virtual_memory_setup() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let machine = SwitchMachine::from_nro(image).unwrap();
        for id in [2, 3, 4, 5, 12, 13, 14, 15, 28] {
            assert!(
                machine.kernel.get_info(id, 0).is_some(),
                "missing GetInfo id {id}"
            );
        }
        assert_eq!(machine.kernel.get_info(2, 0), Some(ALIAS_REGION_BASE));
        assert_eq!(machine.kernel.get_info(3, 0), Some(ALIAS_REGION_SIZE));
        assert_eq!(machine.kernel.get_info(28, 0), Some(0));
    }

    #[test]
    fn nro_boot_uses_homebrew_loader_environment_abi() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        assert_eq!(machine.cpus[0].x[0], ENV_REGION_BASE);
        assert_eq!(machine.cpus[0].x[1], u64::MAX);
        assert_eq!(machine.cpus[0].x[30], LOADER_RETURN_ADDRESS);

        let mut entries = [0u8; 8 * 24];
        machine
            .board
            .read_guest(ENV_REGION_BASE, &mut entries)
            .unwrap();
        let key = |index: usize| {
            u32::from_le_bytes(entries[index * 24..index * 24 + 4].try_into().unwrap())
        };
        let value = |index: usize, word: usize| {
            let start = index * 24 + 8 + word * 8;
            u64::from_le_bytes(entries[start..start + 8].try_into().unwrap())
        };
        assert_eq!((key(0), value(0, 0)), (1, u64::from(MAIN_THREAD_HANDLE)));
        assert_eq!(key(1), 6);
        let hints = supported_svc_hints();
        assert_eq!((value(1, 0), value(1, 1)), (hints[0], hints[1]));
        assert_eq!((key(2), value(2, 0)), (7, 2));
        assert_eq!((key(3), value(3, 0)), (10, u64::from(PROCESS_HANDLE)));
        assert_eq!(key(4), 14);
        assert_eq!((key(5), value(5, 0)), (16, 0x0005_0100));
        assert_eq!((key(6), value(6, 0)), (17, hints[2]));
        assert_eq!(key(7), 0);
        assert_eq!(
            machine.board.read_guest_u32(LOADER_RETURN_ADDRESS).unwrap(),
            0xd400_00e1
        );

        let info = machine.kernel.query_memory(ENV_REGION_BASE).unwrap();
        assert_eq!((info.memory_type, info.permissions), (2, 1));
        assert!(machine.board.write_guest(ENV_REGION_BASE, &[0xff]).is_err());
        AArch64Bus::write8(&mut machine.board, ENV_REGION_BASE, 0xff);
        let mut first = [0u8; 1];
        machine
            .board
            .read_guest(ENV_REGION_BASE, &mut first)
            .unwrap();
        assert_eq!(first[0], 1);
    }

    fn install_domain_request(
        machine: &mut SwitchMachine,
        index: usize,
        object_id: u32,
        command_id: u32,
        payload: &[u8],
    ) {
        let mut bytes = vec![0u8; 0x100];
        let cmif_start = 16usize;
        let sfci_start = cmif_start + 16;
        let end = sfci_start + 16 + payload.len();
        let data_words = (end - 8).div_ceil(4);
        bytes[0..4].copy_from_slice(&4u32.to_le_bytes());
        bytes[4..8].copy_from_slice(&(data_words as u32).to_le_bytes());
        bytes[cmif_start] = 1;
        bytes[cmif_start + 4..cmif_start + 8].copy_from_slice(&object_id.to_le_bytes());
        bytes[sfci_start..sfci_start + 4].copy_from_slice(&0x4943_4653u32.to_le_bytes());
        bytes[sfci_start + 8..sfci_start + 12].copy_from_slice(&command_id.to_le_bytes());
        bytes[sfci_start + 16..end].copy_from_slice(payload);
        machine
            .board
            .write_guest(machine.cpus[index].tpidrro_el0, &bytes)
            .unwrap();
    }

    fn connect_sm(machine: &mut SwitchMachine) -> u32 {
        let name = NRO_BASE + 0x3000;
        machine.board.write_guest(name, b"sm:\0").unwrap();
        machine.cpus[0].x[1] = name;
        assert!(machine.handle_svc(0, 0x1f));
        assert_eq!(machine.cpus[0].x[0], 0);
        let handle = machine.cpus[0].x[1] as u32;
        assert_eq!(machine.kernel.service_kind(handle), Some(ServiceKind::Sm));
        handle
    }

    fn send_request(machine: &mut SwitchMachine, handle: u32) {
        machine.cpus[0].x[0] = u64::from(handle);
        assert!(machine.handle_svc(0, 0x21));
        assert_eq!(machine.cpus[0].x[0], 0);
    }

    #[test]
    fn sm_ipc_registers_client_and_acquires_default_services() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        let sm = connect_sm(&mut machine);

        install_cmif_request(&mut machine, 0, 5, 3, &[], false);
        send_request(&mut machine, sm);
        assert_eq!(tls_u32(&machine, 0, 32) & 0xffff, 0x500);

        install_cmif_request(&mut machine, 0, 4, 0, &[], true);
        send_request(&mut machine, sm);
        assert_eq!(tls_u32(&machine, 0, 24), 0);

        install_cmif_request(&mut machine, 0, 4, 1, b"time:u\0\0", false);
        send_request(&mut machine, sm);
        let time = tls_u32(&machine, 0, 12);
        assert_eq!(
            machine.kernel.service_kind(time),
            Some(ServiceKind::TimeRoot)
        );
        install_cmif_request(&mut machine, 0, 4, 1, b"fsp-srv\0", false);
        send_request(&mut machine, sm);
        let fs = tls_u32(&machine, 0, 12);
        assert_eq!(machine.kernel.service_kind(fs), Some(ServiceKind::FsProxy));
    }

    #[test]
    fn time_and_filesystem_service_boot_protocol_executes() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        let sm = connect_sm(&mut machine);

        install_cmif_request(&mut machine, 0, 4, 1, b"time:u\0\0", false);
        send_request(&mut machine, sm);
        let time = tls_u32(&machine, 0, 12);
        let expected = [
            ServiceKind::TimeUserClock,
            ServiceKind::TimeNetworkClock,
            ServiceKind::TimeSteadyClock,
            ServiceKind::TimeZone,
            ServiceKind::TimeLocalClock,
        ];
        let mut clocks = Vec::new();
        for (command, kind) in expected.into_iter().enumerate() {
            install_cmif_request(&mut machine, 0, 4, command as u32, &[], false);
            send_request(&mut machine, time);
            let handle = tls_u32(&machine, 0, 12);
            assert_eq!(machine.kernel.service_kind(handle), Some(kind));
            clocks.push(handle);
        }
        install_cmif_request(&mut machine, 0, 4, 0, &[], false);
        send_request(&mut machine, clocks[0]);
        let mut now_bytes = [0u8; 8];
        machine
            .board
            .read_guest(machine.cpus[0].tpidrro_el0 + 32, &mut now_bytes)
            .unwrap();
        assert_eq!(u64::from_le_bytes(now_bytes), 1_700_000_000);

        install_cmif_request(&mut machine, 0, 4, 1, b"fsp-srv\0", false);
        send_request(&mut machine, sm);
        let fs = tls_u32(&machine, 0, 12);
        install_cmif_request(&mut machine, 0, 5, 0, &[], false);
        send_request(&mut machine, fs);
        assert_eq!(tls_u32(&machine, 0, 32), 1);

        install_domain_request(&mut machine, 0, 1, 1, &[]);
        send_request(&mut machine, fs);
        install_domain_request(&mut machine, 0, 1, 18, &[]);
        send_request(&mut machine, fs);
        assert_eq!(tls_u32(&machine, 0, 48), 2);

        install_cmif_request(&mut machine, 0, 5, 2, &[], false);
        send_request(&mut machine, fs);
        let clone = tls_u32(&machine, 0, 12);
        assert_eq!(
            machine.kernel.service_kind(clone),
            Some(ServiceKind::FsProxy)
        );
    }

    #[test]
    fn library_applet_boot_protocol_exposes_required_domain_objects() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        let sm = connect_sm(&mut machine);

        install_cmif_request(&mut machine, 0, 4, 1, b"appletAE", false);
        send_request(&mut machine, sm);
        let applet = tls_u32(&machine, 0, 12);
        assert_eq!(
            machine.kernel.service_kind(applet),
            Some(ServiceKind::AppletAe)
        );

        install_cmif_request(&mut machine, 0, 5, 0, &[], false);
        send_request(&mut machine, applet);
        assert_eq!(tls_u32(&machine, 0, 32), 1);

        install_domain_request(&mut machine, 0, 1, 201, &[]);
        send_request(&mut machine, applet);
        assert_eq!(tls_u32(&machine, 0, 48), 2);

        for (command, object) in [
            (20, 3),
            (10, 4),
            (11, 5),
            (0, 6),
            (1, 7),
            (2, 8),
            (3, 9),
            (4, 10),
            (1000, 11),
        ] {
            install_domain_request(&mut machine, 0, 2, command, &[]);
            send_request(&mut machine, applet);
            assert_eq!(tls_u32(&machine, 0, 48), object);
        }

        install_domain_request(&mut machine, 0, 6, 0, &[]);
        send_request(&mut machine, applet);
        let event = tls_u32(&machine, 0, 12);
        assert_eq!(machine.kernel.service_kind(event), Some(ServiceKind::Event));
        install_domain_request(&mut machine, 0, 8, 1, &[]);
        send_request(&mut machine, applet);
        let mut aruid = [0u8; 8];
        machine
            .board
            .read_guest(machine.cpus[0].tpidrro_el0 + 48, &mut aruid)
            .unwrap();
        assert_eq!(u64::from_le_bytes(aruid), 1);

        install_domain_request(&mut machine, 0, 7, 40, &[]);
        send_request(&mut machine, applet);
        let mut layer_id = [0u8; 8];
        machine
            .board
            .read_guest(machine.cpus[0].tpidrro_el0 + 48, &mut layer_id)
            .unwrap();
        assert_eq!(u64::from_le_bytes(layer_id), 1);
    }

    #[test]
    fn vi_application_display_boot_protocol_executes() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        let sm = connect_sm(&mut machine);

        install_cmif_request(&mut machine, 0, 4, 1, b"vi:u\0\0\0\0", false);
        send_request(&mut machine, sm);
        let vi = tls_u32(&machine, 0, 12);
        assert_eq!(machine.kernel.service_kind(vi), Some(ServiceKind::ViRoot));

        install_cmif_request(&mut machine, 0, 4, 0, &0u32.to_le_bytes(), false);
        send_request(&mut machine, vi);
        let display_service = tls_u32(&machine, 0, 12);
        assert_eq!(
            machine.kernel.service_kind(display_service),
            Some(ServiceKind::ViApplicationDisplay)
        );

        install_cmif_request(&mut machine, 0, 4, 100, &[], false);
        send_request(&mut machine, display_service);
        let relay = tls_u32(&machine, 0, 12);
        assert_eq!(
            machine.kernel.service_kind(relay),
            Some(ServiceKind::ViBinderRelay)
        );

        let mut display_name = [0u8; 0x40];
        display_name[..7].copy_from_slice(b"Default");
        install_cmif_request(&mut machine, 0, 4, 1010, &display_name, false);
        send_request(&mut machine, display_service);
        let mut display_id = [0u8; 8];
        machine
            .board
            .read_guest(machine.cpus[0].tpidrro_el0 + 32, &mut display_id)
            .unwrap();
        assert_eq!(u64::from_le_bytes(display_id), 1);

        install_cmif_request(&mut machine, 0, 4, 1102, &1u64.to_le_bytes(), false);
        send_request(&mut machine, display_service);
        let mut resolution = [0u8; 16];
        machine
            .board
            .read_guest(machine.cpus[0].tpidrro_el0 + 32, &mut resolution)
            .unwrap();
        assert_eq!(
            i64::from_le_bytes(resolution[0..8].try_into().unwrap()),
            1280
        );
        assert_eq!(
            i64::from_le_bytes(resolution[8..16].try_into().unwrap()),
            720
        );

        let native_window = STACK_REGION_BASE + 0x2000;
        let mut open_layer = vec![0u8; 0x50];
        open_layer[..7].copy_from_slice(b"Default");
        open_layer[0x40..0x48].copy_from_slice(&1u64.to_le_bytes());
        open_layer[0x48..0x50].copy_from_slice(&1u64.to_le_bytes());
        install_cmif_request_with_recv_buffer(
            &mut machine,
            0,
            2020,
            &open_layer,
            native_window,
            0x100,
            true,
        );
        send_request(&mut machine, display_service);
        let mut native_window_size = [0u8; 8];
        machine
            .board
            .read_guest(machine.cpus[0].tpidrro_el0 + 32, &mut native_window_size)
            .unwrap();
        assert_eq!(u64::from_le_bytes(native_window_size), 28);
        let mut parcel = [0u8; 28];
        machine
            .board
            .read_guest(native_window, &mut parcel)
            .unwrap();
        assert_eq!(u32::from_le_bytes(parcel[0..4].try_into().unwrap()), 12);
        assert_eq!(u32::from_le_bytes(parcel[4..8].try_into().unwrap()), 16);
        assert_eq!(u32::from_le_bytes(parcel[24..28].try_into().unwrap()), 1);

        let binder_reply = STACK_REGION_BASE + 0x2400;
        let mut connect = Vec::with_capacity(12);
        connect.extend_from_slice(&1i32.to_le_bytes());
        connect.extend_from_slice(&10u32.to_le_bytes());
        connect.extend_from_slice(&0u32.to_le_bytes());
        install_cmif_request_with_recv_buffer(
            &mut machine,
            0,
            3,
            &connect,
            binder_reply,
            0x400,
            false,
        );
        send_request(&mut machine, relay);
        let mut binder_parcel = [0u8; 36];
        machine
            .board
            .read_guest(binder_reply, &mut binder_parcel)
            .unwrap();
        assert_eq!(
            u32::from_le_bytes(binder_parcel[0..4].try_into().unwrap()),
            20
        );
        assert_eq!(
            u32::from_le_bytes(binder_parcel[16..20].try_into().unwrap()),
            VIDEO_WIDTH
        );
        assert_eq!(
            u32::from_le_bytes(binder_parcel[20..24].try_into().unwrap()),
            VIDEO_HEIGHT
        );
        assert_eq!(
            i32::from_le_bytes(binder_parcel[32..36].try_into().unwrap()),
            0
        );

        install_cmif_request(&mut machine, 0, 4, 5202, &1u64.to_le_bytes(), false);
        send_request(&mut machine, display_service);
        let vsync = tls_u32(&machine, 0, 12);
        assert_eq!(machine.kernel.service_kind(vsync), Some(ServiceKind::Event));

        install_cmif_request(&mut machine, 0, 4, 1, &[0; 12], false);
        send_request(&mut machine, relay);
        assert_eq!(tls_u32(&machine, 0, 24), 0);
        install_cmif_request(&mut machine, 0, 4, 2, &[0; 8], false);
        send_request(&mut machine, relay);
        let binder_event = tls_u32(&machine, 0, 12);
        assert_eq!(
            machine.kernel.service_kind(binder_event),
            Some(ServiceKind::Event)
        );
    }

    #[test]
    fn hid_shared_memory_maps_through_horizon_svc() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        let sm = connect_sm(&mut machine);

        install_cmif_request(&mut machine, 0, 4, 1, b"hid\0\0\0\0\0", false);
        send_request(&mut machine, sm);
        let hid = tls_u32(&machine, 0, 12);
        assert_eq!(machine.kernel.service_kind(hid), Some(ServiceKind::Hid));
        install_cmif_request(&mut machine, 0, 4, 0, &[], true);
        send_request(&mut machine, hid);
        let resource = tls_u32(&machine, 0, 12);
        assert_eq!(
            machine.kernel.service_kind(resource),
            Some(ServiceKind::HidAppletResource)
        );
        install_cmif_request(&mut machine, 0, 4, 0, &[], false);
        send_request(&mut machine, resource);
        let shared = tls_u32(&machine, 0, 12);
        assert_eq!(
            machine.kernel.service_kind(shared),
            Some(ServiceKind::HidSharedMemory)
        );

        let address = machine.kernel.heap_base + 0x20_0000;
        machine.cpus[0].x[0] = u64::from(shared);
        machine.cpus[0].x[1] = address;
        machine.cpus[0].x[2] = 0x40000;
        machine.cpus[0].x[3] = 1;
        assert!(machine.handle_svc(0, 0x13));
        assert_eq!(machine.cpus[0].x[0], 0);
        let info = machine.kernel.query_memory(address).unwrap();
        assert_eq!(
            (info.memory_type, info.permissions, info.size),
            (6, 1, 0x40000)
        );

        for command in [109, 102, 100] {
            install_cmif_request(&mut machine, 0, 4, command, &[], false);
            send_request(&mut machine, hid);
            assert_eq!(tls_u32(&machine, 0, 24), 0);
        }
        install_cmif_request(&mut machine, 0, 4, 101, &[], false);
        send_request(&mut machine, hid);
        assert_eq!(tls_u32(&machine, 0, 24), 0);
        assert_eq!(tls_u32(&machine, 0, 32), HID_NPAD_STYLE_FULL_KEY);

        machine.input.buttons[0] = FACE_EAST | FACE_SOUTH | L1 | START | LEFT;
        machine.input.axes[0][AXIS_LEFT_X] = i16::MAX;
        machine.input.axes[0][AXIS_LEFT_Y] = i16::MIN;
        machine.input.axes[0][AXIS_RIGHT_X] = i16::MIN;
        machine.input.axes[0][AXIS_RIGHT_Y] = 16_384;
        machine.sync_hid_input().unwrap();

        let entry = address + HID_NPAD_OFFSET;
        assert_eq!(
            machine.board.read_guest_u32(entry).unwrap(),
            HID_NPAD_STYLE_FULL_KEY
        );
        let lifo = entry + HID_NPAD_FULL_KEY_LIFO_OFFSET;
        let mut header = [0u8; HID_LIFO_HEADER_SIZE as usize];
        machine.board.read_guest(lifo, &mut header).unwrap();
        assert_eq!(u64::from_le_bytes(header[8..16].try_into().unwrap()), 17);
        assert_eq!(u64::from_le_bytes(header[24..32].try_into().unwrap()), 2);
        let tail = u64::from_le_bytes(header[16..24].try_into().unwrap());
        let storage_address = lifo + HID_LIFO_HEADER_SIZE + tail * HID_NPAD_STORAGE_SIZE;
        let mut storage = [0u8; HID_NPAD_STORAGE_SIZE as usize];
        machine
            .board
            .read_guest(storage_address, &mut storage)
            .unwrap();
        assert_eq!(u64::from_le_bytes(storage[0..8].try_into().unwrap()), 2);
        assert_eq!(u64::from_le_bytes(storage[8..16].try_into().unwrap()), 2);
        assert_eq!(
            u64::from_le_bytes(storage[16..24].try_into().unwrap()),
            (1 << 0) | (1 << 1) | (1 << 6) | (1 << 10) | (1 << 12)
        );
        assert_eq!(
            i32::from_le_bytes(storage[24..28].try_into().unwrap()),
            0x7fff
        );
        assert_eq!(
            i32::from_le_bytes(storage[28..32].try_into().unwrap()),
            0x7fff
        );
        assert_eq!(
            i32::from_le_bytes(storage[32..36].try_into().unwrap()),
            -0x7fff
        );
        assert_eq!(
            i32::from_le_bytes(storage[36..40].try_into().unwrap()),
            -16_384
        );
        assert_eq!(
            u32::from_le_bytes(storage[40..44].try_into().unwrap()),
            HID_NPAD_ATTRIBUTE_CONNECTED
        );

        let state = machine.save_state().unwrap();
        machine.load_state(&state).unwrap();
        assert_eq!(machine.kernel.query_memory(address).unwrap().memory_type, 6);

        machine.cpus[0].x[0] = u64::from(shared);
        machine.cpus[0].x[1] = address;
        machine.cpus[0].x[2] = 0x40000;
        assert!(machine.handle_svc(0, 0x14));
        assert_eq!(machine.cpus[0].x[0], 0);
        assert_eq!(machine.kernel.query_memory(address).unwrap().memory_type, 0);
    }

    #[test]
    fn mapped_thread_stack_copies_back_on_unmap() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        machine.kernel.set_heap_size(0x40_0000).unwrap();
        let source = machine.kernel.heap_base + 0x1000;
        let destination = STACK_REGION_BASE + 0x8000;
        machine.board.write_guest(source, &[1, 2, 3, 4]).unwrap();

        machine.cpus[0].x[0] = destination;
        machine.cpus[0].x[1] = source;
        machine.cpus[0].x[2] = 0x1000;
        assert!(machine.handle_svc(0, 0x04));
        assert_eq!(machine.cpus[0].x[0], 0);
        assert_eq!(
            machine
                .kernel
                .query_memory(destination)
                .unwrap()
                .memory_type,
            0x0b
        );
        let mut mapped = [0u8; 4];
        machine.board.read_guest(destination, &mut mapped).unwrap();
        assert_eq!(mapped, [1, 2, 3, 4]);

        machine
            .board
            .write_guest(destination, &[9, 8, 7, 6])
            .unwrap();
        machine.cpus[0].x[0] = destination;
        machine.cpus[0].x[1] = source;
        machine.cpus[0].x[2] = 0x1000;
        assert!(machine.handle_svc(0, 0x05));
        let mut copied_back = [0u8; 4];
        machine.board.read_guest(source, &mut copied_back).unwrap();
        assert_eq!(copied_back, [9, 8, 7, 6]);
        assert_eq!(
            machine
                .kernel
                .query_memory(destination)
                .unwrap()
                .memory_type,
            0
        );
    }

    #[test]
    fn thread_lifecycle_runs_on_secondary_core_with_horizon_tls() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[svc(0x0a)]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        machine.kernel.set_heap_size(0x40_0000).unwrap();
        let stack_source = machine.kernel.heap_base + 0x20_000;
        let stack_destination = STACK_REGION_BASE + 0x10_000;
        let stack_size = 0x20_000;
        machine
            .kernel
            .map_memory(stack_destination, stack_source, stack_size)
            .unwrap();
        machine
            .board
            .copy_guest(stack_destination, stack_source, stack_size)
            .unwrap();

        machine.cpus[0].x[1] = NRO_BASE + 0x80;
        machine.cpus[0].x[2] = 0x1234_5678;
        machine.cpus[0].x[3] = stack_destination + stack_size - 0x40;
        machine.cpus[0].x[4] = 0x3b;
        machine.cpus[0].x[5] = (-2i64) as u64;
        assert!(machine.handle_svc(0, 0x08));
        assert_eq!(machine.cpus[0].x[0], 0);
        let handle = machine.cpus[0].x[1] as u32;
        let thread = machine.kernel.thread(handle).unwrap().clone();
        assert_eq!(thread.cpu_index, 1);
        assert_eq!(machine.cpus[1].x[0], 0x1234_5678);
        assert_eq!(machine.cpus[1].tpidrro_el0, TLS_REGION_BASE + TLS_SLOT_SIZE);
        assert_eq!(
            machine
                .kernel
                .query_memory(machine.cpus[1].tpidrro_el0)
                .unwrap()
                .memory_type,
            0x0c
        );

        machine.cpus[0].x[1] = u64::from(handle);
        assert!(machine.handle_svc(0, 0x0c));
        assert_eq!(machine.cpus[0].x[1], 0x3b);
        machine.cpus[0].x[0] = u64::from(handle);
        machine.cpus[0].x[1] = 0x20;
        assert!(machine.handle_svc(0, 0x0d));
        machine.cpus[0].x[0] = u64::from(handle);
        assert!(machine.handle_svc(0, 0x09));
        assert_eq!(machine.cpus[0].x[0], 0);

        machine
            .scheduler
            .set_halted(SwitchMachine::processor_id(0), true)
            .unwrap();
        machine.run_cores();
        assert!(machine.kernel.thread(handle).unwrap().exited);
        assert!(machine.cpus[1].halted());

        let handle_address = machine.kernel.heap_base + 0x1000;
        machine
            .board
            .write_guest(handle_address, &handle.to_le_bytes())
            .unwrap();
        machine.cpus[0].x[1] = handle_address;
        machine.cpus[0].x[2] = 1;
        machine.cpus[0].x[3] = 0;
        assert!(machine.handle_svc(0, 0x18));
        assert_eq!(machine.cpus[0].x[0], 0);
        machine.cpus[0].x[0] = u64::from(handle);
        assert!(machine.handle_svc(0, 0x16));
        assert!(machine.kernel.thread(handle).is_none());
    }

    #[test]
    fn wait_synchronization_wakes_when_child_exits() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0xd503_201f, svc(0x0a)]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        machine.kernel.set_heap_size(0x40_0000).unwrap();
        let stack_destination = STACK_REGION_BASE + 0x40_000;
        machine.cpus[0].x[1] = NRO_BASE + 0x80;
        machine.cpus[0].x[2] = 0;
        machine.cpus[0].x[3] = stack_destination + 0x20_000 - 0x40;
        machine.cpus[0].x[4] = 0x30;
        machine.cpus[0].x[5] = (-1i64) as u64;
        assert!(machine.handle_svc(0, 0x08));
        let handle = machine.cpus[0].x[1] as u32;
        machine.cpus[0].x[0] = u64::from(handle);
        assert!(machine.handle_svc(0, 0x09));

        let handle_address = machine.kernel.heap_base + 0x2000;
        machine
            .board
            .write_guest(handle_address, &handle.to_le_bytes())
            .unwrap();
        machine.cpus[0].x[1] = handle_address;
        machine.cpus[0].x[2] = 1;
        machine.cpus[0].x[3] = u64::MAX;
        assert!(!machine.handle_svc(0, 0x18));
        assert_eq!(machine.kernel.waiting_on[0], Some(handle));
        machine.run_cores();
        assert_eq!(machine.kernel.waiting_on[0], None);
        assert!(machine.kernel.thread(handle).unwrap().exited);
    }

    #[test]
    fn mutex_arbitration_transfers_ownership_to_waiter() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        machine.kernel.set_heap_size(0x20_0000).unwrap();
        let mutex = machine.kernel.heap_base + 0x4000;
        machine
            .board
            .write_guest_u32(mutex, MAIN_THREAD_HANDLE)
            .unwrap();

        machine.cpus[1].x[0] = u64::from(MAIN_THREAD_HANDLE);
        machine.cpus[1].x[1] = mutex;
        machine.cpus[1].x[2] = 0x101;
        assert!(!machine.handle_svc(1, 0x1a));
        assert_eq!(
            machine.board.read_guest_u32(mutex).unwrap(),
            MAIN_THREAD_HANDLE | MUTEX_WAIT_FLAG
        );
        assert_eq!(machine.kernel.mutex_waiters.len(), 1);

        machine.cpus[0].x[0] = mutex;
        assert!(machine.handle_svc(0, 0x1b));
        assert_eq!(machine.board.read_guest_u32(mutex).unwrap(), 0x101);
        assert!(machine.kernel.mutex_waiters.is_empty());
        assert_eq!(machine.cpus[1].x[0], 0);
    }

    #[test]
    fn condition_signal_reacquires_mutex_before_waking_waiter() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        machine.kernel.set_heap_size(0x20_0000).unwrap();
        let mutex = machine.kernel.heap_base + 0x5000;
        let condition = machine.kernel.heap_base + 0x6000;
        machine.board.write_guest_u32(mutex, 0x101).unwrap();

        machine.cpus[1].x[0] = mutex;
        machine.cpus[1].x[1] = condition;
        machine.cpus[1].x[2] = 0x101;
        machine.cpus[1].x[3] = u64::MAX;
        assert!(!machine.handle_svc(1, 0x1c));
        assert_eq!(machine.board.read_guest_u32(mutex).unwrap(), 0);
        assert_eq!(machine.kernel.condvar_waiters.len(), 1);

        machine.cpus[0].x[0] = condition;
        machine.cpus[0].x[1] = 1;
        assert!(machine.handle_svc(0, 0x1d));
        assert!(machine.kernel.condvar_waiters.is_empty());
        assert_eq!(machine.board.read_guest_u32(mutex).unwrap(), 0x101);
        assert_eq!(machine.cpus[1].x[0], 0);

        machine.board.write_guest_u32(mutex, 0x101).unwrap();
        machine.cpus[1].x[0] = mutex;
        machine.cpus[1].x[1] = condition;
        machine.cpus[1].x[2] = 0x101;
        machine.cpus[1].x[3] = 0;
        assert!(machine.handle_svc(1, 0x1c));
        assert_eq!(machine.cpus[1].x[0], 0xEA01);
        assert!(machine.kernel.condvar_waiters.is_empty());
        assert_eq!(machine.board.read_guest_u32(mutex).unwrap(), 0);
    }

    #[test]
    fn non_exit_svc_returns_to_guest_before_exit_process() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[svc(0x10), svc(0x07)]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.cpus[0].cycles, 3);
        assert_eq!(machine.cpus[0].x[0], 0);
        assert_eq!(machine.cpus[0].last_svc, Some(0x07));
        assert!(machine.cpus[0].halted());
        assert!(!machine.cpus[0].invalid_instruction());
        assert!(!machine.powered);
    }

    #[test]
    fn released_secondary_core_receives_deterministic_frame_budget() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[0x1400_0000]));
        let mut machine = SwitchMachine::from_nro(image).unwrap();
        let entry = machine.cpus[0].pc;
        machine.cpus[1].reset_to(entry);
        machine
            .scheduler
            .set_halted(crate::cluster::ProcessorId(1), false)
            .unwrap();
        machine.run_frame(&InputState::default());
        assert_eq!(machine.cpus[0].cycles, STEPS_PER_FRAME);
        assert_eq!(machine.cpus[1].cycles, STEPS_PER_FRAME);
        assert_eq!(machine.cpus[2].cycles, 0);
        assert_eq!(machine.cpus[3].cycles, 0);
        assert!(machine.powered);
    }

    #[test]
    fn switch_state_preserves_all_cores_and_sparse_dram() {
        let image = ResourceBlob::from_bytes(&nro_bytes(&[svc(7)]));
        let mut machine = SwitchMachine::from_nro(image.clone()).unwrap();
        machine.cpus[1].x[4] = 0x1122_3344_5566_7788;
        machine.cpus[3].sp = NRO_BASE + 0x3000;
        machine.kernel.set_heap_size(0x40_0000).unwrap();
        let stack_source = machine.kernel.heap_base + 0x10_000;
        let stack_destination = STACK_REGION_BASE + 0x80_000;
        machine
            .kernel
            .map_memory(stack_destination, stack_source, 0x1000)
            .unwrap();
        let (thread_handle, _, _) = machine
            .kernel
            .create_thread(NRO_BASE + 0x80, 0x55aa, stack_destination + 0xfc0, 0x30, -1)
            .unwrap();
        machine.initialize_thread_cpu(thread_handle).unwrap();
        machine.kernel.thread_mut(thread_handle).unwrap().started = true;
        machine.cpus[1].x[4] = 0x1122_3344_5566_7788;
        machine
            .board
            .dram
            .write(0x3456_7000, &[0xaa, 0xbb, 0xcc])
            .unwrap();
        let state = machine.save_state().unwrap();

        let mut restored = SwitchMachine::from_nro(image).unwrap();
        restored.load_state(&state).unwrap();
        assert_eq!(restored.cpus[1].x[4], 0x1122_3344_5566_7788);
        assert_eq!(restored.cpus[3].sp, NRO_BASE + 0x3000);
        assert_eq!(restored.kernel.heap_base, machine.kernel.heap_base);
        assert_eq!(restored.kernel.heap_size, 0x40_0000);
        assert_eq!(restored.kernel.mappings, machine.kernel.mappings);
        assert_eq!(restored.kernel.threads, machine.kernel.threads);
        assert_eq!(
            restored.kernel.next_thread_handle,
            machine.kernel.next_thread_handle
        );
        let mut bytes = [0u8; 3];
        restored.board.dram.read(0x3456_7000, &mut bytes).unwrap();
        assert_eq!(bytes, [0xaa, 0xbb, 0xcc]);
        assert_eq!(restored.platform(), PlatformId::Switch);
    }

    #[test]
    fn nro_loader_rejects_invalid_magic_and_overlapping_segments() {
        let mut invalid = nro_bytes(&[svc(7)]);
        invalid[0x10..0x14].copy_from_slice(b"BAD0");
        assert!(SwitchMachine::from_nro(ResourceBlob::from_bytes(&invalid)).is_err());

        let mut overlap = nro_bytes(&[svc(7)]);
        overlap[0x28..0x2c].copy_from_slice(&0x800u32.to_le_bytes());
        overlap[0x2c..0x30].copy_from_slice(&0x100u32.to_le_bytes());
        assert!(SwitchMachine::from_nro(ResourceBlob::from_bytes(&overlap)).is_err());
    }
}
