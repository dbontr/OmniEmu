use std::collections::VecDeque;

use crate::cd_image::{DiscImage, TrackKind};
use crate::cpu68000::{Bus68000, M68000};
use crate::cpu_sh2::{Sh2, Sh2Bus, Sh2Divu, Sh2Dmac, Sh2Frt, Sh2Intc, Sh2Sci, Sh2Wdt};
use crate::input::{
    DOWN, FACE_EAST, FACE_NORTH, FACE_SOUTH, FACE_WEST, L1, LEFT, R1, RIGHT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::{ResourceBlob, ResourceKind, RESOURCE_PENDING};
use crate::state::{StateReader, StateWriter};

const SH2_HZ: u64 = 28_636_360;
const SOUND_68K_HZ: u64 = 11_289_600;
const AUDIO_RATE: u64 = 44_100;
const FRAME_RATE_NUM: u64 = 60_000;
const FRAME_RATE_DEN: u64 = 1_001;
const WIDTH: usize = 320;
const HEIGHT: usize = 224;
const STATE_VERSION: u32 = 16;
const BIOS_SIZE: usize = 512 * 1024;
const WORK_RAM_SIZE: usize = 1024 * 1024;
const SOUND_RAM_SIZE: usize = 512 * 1024;
const VDP1_VRAM_SIZE: usize = 512 * 1024;
const VDP1_FB_SIZE: usize = 256 * 1024;
const VDP2_VRAM_SIZE: usize = 512 * 1024;
const VDP2_CRAM_SIZE: usize = 4 * 1024;
const BACKUP_RAM_SIZE: usize = 32 * 1024;
const SECTORS_PER_SECOND: u64 = 150;
const CDDA_FRAMES_PER_SECTOR: usize = 588;
const CD_BUFFER_COUNT: usize = 200;
const CD_PARTITION_COUNT: usize = 24;

fn be16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_be_bytes([bytes[offset], bytes[offset + 1]])
}

fn be16_wrapped(bytes: &[u8], offset: usize) -> u16 {
    let mask = bytes.len() - 1;
    u16::from_be_bytes([bytes[offset & mask], bytes[(offset + 1) & mask]])
}

fn put_be16(bytes: &mut [u8], offset: usize, value: u16) {
    let raw = value.to_be_bytes();
    bytes[offset] = raw[0];
    bytes[offset + 1] = raw[1];
}

#[cfg(test)]
fn put_be32(bytes: &mut [u8], offset: usize, value: u32) {
    let raw = value.to_be_bytes();
    bytes[offset..offset + 4].copy_from_slice(&raw);
}

fn be32(bytes: &[u8], offset: usize) -> u32 {
    u32::from_be_bytes([
        bytes[offset],
        bytes[offset + 1],
        bytes[offset + 2],
        bytes[offset + 3],
    ])
}

fn be32_wrapped(bytes: &[u8], offset: usize) -> u32 {
    let mask = bytes.len() - 1;
    u32::from_be_bytes([
        bytes[offset & mask],
        bytes[(offset + 1) & mask],
        bytes[(offset + 2) & mask],
        bytes[(offset + 3) & mask],
    ])
}

#[derive(Clone)]
struct SaturnCdBufferedSector {
    fad: u32,
    raw: Box<[u8; 2352]>,
}

impl SaturnCdBufferedSector {
    fn transfer_view(&self, format: u8) -> &[u8] {
        match format {
            1 => &self.raw[16..],
            2 => &self.raw[12..],
            3 => self.raw.as_slice(),
            _ if self.raw[15] == 2 => {
                if self.raw[18] & 0x20 != 0 {
                    &self.raw[24..2348]
                } else {
                    &self.raw[24..2072]
                }
            }
            _ => &self.raw[16..2064],
        }
    }
}

#[derive(Clone, Copy)]
struct SaturnCdFilter {
    mode: u8,
    true_conn: u8,
    false_conn: u8,
    fad: u32,
    range: u32,
    channel: u8,
    file: u8,
    submode: u8,
    submode_mask: u8,
    coding_info: u8,
    coding_info_mask: u8,
}

impl SaturnCdFilter {
    fn new(index: usize) -> Self {
        Self {
            mode: 0,
            true_conn: index as u8,
            false_conn: 0xff,
            fad: 0,
            range: 0,
            channel: 0,
            file: 0,
            submode: 0,
            submode_mask: 0,
            coding_info: 0,
            coding_info_mask: 0,
        }
    }

    fn reset_condition(&mut self) {
        self.mode = 0;
        self.fad = 0;
        self.range = 0;
        self.channel = 0;
        self.file = 0;
        self.submode = 0;
        self.submode_mask = 0;
        self.coding_info = 0;
        self.coding_info_mask = 0;
    }

    fn matches(&self, sector: &SaturnCdBufferedSector) -> bool {
        if self.mode & 0x40 != 0
            && (sector.fad < self.fad || sector.fad >= self.fad.saturating_add(self.range))
        {
            return false;
        }

        let (file, channel, submode, coding_info) = if sector.raw[15] == 2 {
            (
                sector.raw[16],
                sector.raw[17],
                sector.raw[18],
                sector.raw[19],
            )
        } else {
            (0, 0, 0, 0)
        };
        let reverse = self.mode & 0x10 != 0 && self.mode & 0x0f != 0;
        let mismatch = (self.mode & 0x01 != 0 && file != self.file)
            || (self.mode & 0x02 != 0 && channel != self.channel)
            || (self.mode & 0x04 != 0 && submode & self.submode_mask != self.submode)
            || (self.mode & 0x08 != 0 && coding_info & self.coding_info_mask != self.coding_info);
        if mismatch {
            reverse
        } else {
            !reverse
        }
    }
}

struct SaturnCdBlock {
    disc: Option<DiscImage>,
    hirq: u16,
    hirq_mask: u16,
    cr: [u16; 4],
    target_lba: u32,
    current_lba: u32,
    reading: bool,
    data: VecDeque<u8>,
    partitions: [VecDeque<SaturnCdBufferedSector>; CD_PARTITION_COUNT],
    filters: [SaturnCdFilter; CD_PARTITION_COUNT],
    cd_device_connection: u8,
    last_buffer_destination: u8,
    get_sector_length: u8,
    put_sector_length: u8,
    actual_size_words: u32,
    fad_search_fad: u32,
    fad_search_pos: u16,
    fad_search_partition: u8,
    transfer_bytes_read: u32,
    phase: u64,
    cdda_sector: Box<[u8; 2352]>,
    cdda_sample_index: usize,
    cdda_phase: u64,
    cdda_samples: VecDeque<[i16; 2]>,
}

impl SaturnCdBlock {
    fn new(disc: Option<ResourceBlob>) -> Result<Self, String> {
        Ok(Self {
            disc: disc.map(DiscImage::new).transpose()?,
            hirq: 0x0001,
            hirq_mask: 0,
            cr: [0; 4],
            target_lba: 0,
            current_lba: 0,
            reading: false,
            data: VecDeque::with_capacity(4096),
            partitions: std::array::from_fn(|_| VecDeque::new()),
            filters: std::array::from_fn(SaturnCdFilter::new),
            cd_device_connection: 0xff,
            last_buffer_destination: 0xff,
            get_sector_length: 0,
            put_sector_length: 0,
            actual_size_words: 0,
            fad_search_fad: 0,
            fad_search_pos: 0xffff,
            fad_search_partition: 0,
            transfer_bytes_read: 0,
            phase: 0,
            cdda_sector: Box::new([0; 2352]),
            cdda_sample_index: CDDA_FRAMES_PER_SECTOR,
            cdda_phase: 0,
            cdda_samples: VecDeque::with_capacity(1024),
        })
    }

    fn reset(&mut self) {
        self.hirq = 0x0001;
        self.hirq_mask = 0;
        self.cr = [0; 4];
        self.target_lba = 0;
        self.current_lba = 0;
        self.reading = false;
        self.data.clear();
        self.reset_buffers_and_selectors();
        self.phase = 0;
        self.reset_cdda_position();
        self.cdda_samples.clear();
    }

    fn reset_buffers_and_selectors(&mut self) {
        for partition in &mut self.partitions {
            partition.clear();
        }
        self.filters = std::array::from_fn(SaturnCdFilter::new);
        self.cd_device_connection = 0xff;
        self.last_buffer_destination = 0xff;
        self.get_sector_length = 0;
        self.put_sector_length = 0;
        self.actual_size_words = 0;
        self.fad_search_fad = 0;
        self.fad_search_pos = 0xffff;
        self.fad_search_partition = 0;
        self.transfer_bytes_read = 0;
    }

    fn total_buffered_sectors(&self) -> usize {
        self.partitions.iter().map(VecDeque::len).sum()
    }

    fn route_sector(
        &self,
        start_filter: u8,
        sector: &SaturnCdBufferedSector,
    ) -> Option<(u8, usize)> {
        let mut current = start_filter;
        for _ in 0..CD_PARTITION_COUNT {
            let index = usize::from(current);
            let filter = self.filters.get(index)?;
            if filter.matches(sector) {
                let destination = usize::from(filter.true_conn);
                return (destination < CD_PARTITION_COUNT).then_some((current, destination));
            }
            current = filter.false_conn;
            if current == 0xff {
                return None;
            }
        }
        None
    }

    fn selection_range(len: usize, offset: u16, count: u16) -> Option<std::ops::Range<usize>> {
        if len == 0 {
            return None;
        }
        let start = if offset == 0xffff {
            len - 1
        } else {
            usize::from(offset)
        };
        let count = if count == 0xffff {
            len.checked_sub(start)?
        } else {
            usize::from(count)
        };
        (count != 0 && start < len && start.checked_add(count)? <= len)
            .then_some(start..start + count)
    }

    fn reset_cdda_position(&mut self) {
        self.cdda_sector.fill(0);
        self.cdda_sample_index = CDDA_FRAMES_PER_SECTOR;
        self.cdda_phase = 0;
        self.cdda_samples.clear();
    }

    fn audio_path_active(&self) -> bool {
        if !self.reading {
            return false;
        }
        if self.cdda_sample_index < CDDA_FRAMES_PER_SECTOR {
            return true;
        }
        self.disc
            .as_ref()
            .and_then(|disc| disc.track_for_lba(self.current_lba))
            .is_some_and(|track| track.kind == TrackKind::Audio)
    }

    fn load_cdda_sector(&mut self) -> bool {
        let Some(disc) = &self.disc else {
            self.reading = false;
            return false;
        };
        if self.current_lba >= disc.sectors || self.current_lba >= self.target_lba {
            self.reading = false;
            self.hirq |= 0x0010;
            return false;
        }
        if disc
            .track_for_lba(self.current_lba)
            .is_none_or(|track| track.kind != TrackKind::Audio)
        {
            self.reading = false;
            self.hirq |= 0x0010;
            return false;
        }
        match disc.read_raw_sector(self.current_lba, self.cdda_sector.as_mut()) {
            Ok(()) => {
                self.current_lba = self.current_lba.saturating_add(1);
                self.cdda_sample_index = 0;
                true
            }
            Err(error) if error == RESOURCE_PENDING => false,
            Err(_) => {
                self.reading = false;
                self.hirq |= 0x0010;
                false
            }
        }
    }

    fn next_cdda_sample(&mut self) -> Option<[i16; 2]> {
        if self.cdda_sample_index >= CDDA_FRAMES_PER_SECTOR && !self.load_cdda_sector() {
            return None;
        }
        let offset = self.cdda_sample_index * 4;
        let sample = [
            i16::from_le_bytes([self.cdda_sector[offset], self.cdda_sector[offset + 1]]),
            i16::from_le_bytes([self.cdda_sector[offset + 2], self.cdda_sector[offset + 3]]),
        ];
        self.cdda_sample_index += 1;
        Some(sample)
    }

    fn tick_cdda(&mut self, sh_cycles: u32) {
        self.cdda_phase = self
            .cdda_phase
            .saturating_add(u64::from(sh_cycles).saturating_mul(AUDIO_RATE));
        while self.cdda_phase >= SH2_HZ {
            self.cdda_phase -= SH2_HZ;
            let Some(sample) = self.next_cdda_sample() else {
                break;
            };
            self.cdda_samples.push_back(sample);
        }
    }

    fn irq_pending(&self) -> bool {
        self.hirq & self.hirq_mask != 0
    }

    fn drive_status(&self) -> u16 {
        if self.disc.is_none() {
            0x0700
        } else if self.reading {
            0x0300
        } else {
            0x0100
        }
    }

    fn current_fad(&self) -> u32 {
        self.current_lba.saturating_add(150)
    }

    fn standard_return(&mut self, status: u16) {
        let fad = self.current_fad();
        self.cr[0] = status;
        if self.disc.is_some() {
            self.cr[1] = 0x4101;
            self.cr[2] = 0x0100 | ((fad >> 16) as u16 & 0x00ff);
            self.cr[3] = fad as u16;
        } else {
            self.cr[1] = 0xffff;
            self.cr[2] = 0xffff;
            self.cr[3] = 0xffff;
        }
    }

    fn queue_toc(&mut self) {
        self.data.clear();
        let mut toc = [0xffu8; 102 * 4];
        let Some(disc) = &self.disc else {
            self.data.extend(toc);
            return;
        };
        let metadata = 99 * 4;
        for track in &disc.tracks {
            let offset = usize::from(track.number.saturating_sub(1)) * 4;
            if offset + 3 >= metadata {
                continue;
            }
            let fad = track.start_lba.saturating_add(150);
            toc[offset] = if track.kind == TrackKind::Data {
                0x41
            } else {
                0x01
            };
            toc[offset + 1] = (fad >> 16) as u8;
            toc[offset + 2] = (fad >> 8) as u8;
            toc[offset + 3] = fad as u8;
        }

        let leadout_fad = disc.sectors.saturating_add(150);
        toc[metadata] = 0x41;
        toc[metadata + 1] = disc.first_track();
        toc[metadata + 2] = 0;
        toc[metadata + 3] = 0;
        toc[metadata + 4] = 0x41;
        toc[metadata + 5] = disc.last_track();
        toc[metadata + 6] = 0;
        toc[metadata + 7] = 0;
        toc[metadata + 8] = 0x41;
        toc[metadata + 9] = (leadout_fad >> 16) as u8;
        toc[metadata + 10] = (leadout_fad >> 8) as u8;
        toc[metadata + 11] = leadout_fad as u8;
        self.data.extend(toc);
    }

    fn pop_data_byte(&mut self) -> u8 {
        let value = self.data.pop_front().unwrap_or(0);
        if self.cr[0] & 0x4000 != 0 {
            self.transfer_bytes_read = self.transfer_bytes_read.saturating_add(1);
        }
        if self.data.is_empty() {
            self.hirq &= !0x0002;
            self.cr[0] &= !0x4000;
        }
        value
    }

    fn queue_partition_transfer(
        &mut self,
        partition: usize,
        range: std::ops::Range<usize>,
        delete: bool,
    ) {
        let sectors = self.partitions[partition]
            .range(range.clone())
            .cloned()
            .collect::<Vec<_>>();
        self.data.clear();
        self.transfer_bytes_read = 0;
        for sector in &sectors {
            self.data
                .extend(sector.transfer_view(self.get_sector_length));
        }
        if delete {
            self.partitions[partition].drain(range);
        }
        self.cr = [self.drive_status() | 0x4000, 0, 0, 0];
        self.hirq |= 0x0003;
    }

    fn issue_command(&mut self) {
        let request = self.cr;
        let command = (request[0] >> 8) as u8;
        match command {
            0x00 => {
                self.standard_return(self.drive_status());
                self.hirq |= 0x0001;
            }
            0x01 => {
                self.cr = [self.drive_status(), 0x0201, 0x0000, 0x0400];
                self.hirq |= 0x0001;
            }
            0x02 => {
                self.queue_toc();
                self.cr = [0x4100, (102 * 2) as u16, 0, 0];
                self.hirq |= 0x0003;
            }
            0x03 => {
                let leadout_fad = self
                    .disc
                    .as_ref()
                    .map_or(150, |disc| disc.sectors.saturating_add(150));
                match request[0] & 0x00ff {
                    0 => {
                        self.cr = [
                            self.drive_status(),
                            0,
                            0x0100 | ((leadout_fad >> 16) as u16 & 0x00ff),
                            leadout_fad as u16,
                        ];
                    }
                    1 => {
                        self.cr = [self.drive_status(), 0, 0x0100, 0];
                    }
                    _ => {
                        self.cr = [self.drive_status(), 0, 0, 0];
                    }
                }
                self.hirq |= 0x0001;
            }
            0x04 => {
                self.current_lba = 0;
                self.target_lba = 0;
                self.reading = false;
                self.phase = 0;
                self.data.clear();
                self.reset_buffers_and_selectors();
                self.reset_cdda_position();
                self.standard_return(self.drive_status());
                self.hirq = (self.hirq & 0xffe5) | 0x00c1;
            }
            0x06 => {
                let transferred_words = self.transfer_bytes_read / 2;
                self.data.clear();
                self.transfer_bytes_read = 0;
                self.hirq &= !0x0002;
                self.cr = [
                    self.drive_status() | ((transferred_words >> 16) as u16 & 0x00ff),
                    transferred_words as u16,
                    0,
                    0,
                ];
                self.hirq |= 0x0081;
            }
            0x10 => {
                if self.disc.is_none() {
                    self.reading = false;
                    self.standard_return(0x0700);
                    self.hirq |= 0x0001;
                    return;
                }
                let start = (u32::from(request[0] & 0x00ff) << 16) | u32::from(request[1]);
                if start != 0x00ff_ffff {
                    if start & 0x0080_0000 != 0 {
                        let fad = start & 0x000f_ffff;
                        self.current_lba = fad.saturating_sub(150);
                    } else {
                        let track = ((start >> 8) & 0xff) as u8;
                        if track == 0 {
                            self.current_lba = 0;
                        } else if let Some(lba) =
                            self.disc.as_ref().and_then(|disc| disc.track_start(track))
                        {
                            self.current_lba = lba;
                        }
                    }
                }
                let end = (u32::from(request[2] & 0x00ff) << 16) | u32::from(request[3]);
                if let Some(disc) = &self.disc {
                    self.current_lba = self.current_lba.min(disc.sectors);
                    self.target_lba = if end == 0 || end == 0x00ff_ffff {
                        disc.sectors
                    } else if end & 0x0080_0000 != 0 {
                        if start & 0x0080_0000 != 0 {
                            self.current_lba
                                .saturating_add(end & 0x007f_ffff)
                                .min(disc.sectors)
                        } else {
                            (end & 0x007f_ffff).saturating_sub(150).min(disc.sectors)
                        }
                    } else {
                        let end_track = ((end >> 8) & 0xff) as u8;
                        disc.track_start(end_track.saturating_add(1))
                            .unwrap_or(disc.sectors)
                    };
                } else {
                    self.target_lba = self.current_lba;
                }
                self.reading = self.current_lba < self.target_lba;
                self.phase = 0;
                self.reset_cdda_position();
                self.standard_return(if self.reading { 0x0300 } else { 0x0100 });
                self.hirq |= 0x0001;
            }
            0x11 => {
                self.reading = false;
                if request[0] & 0x0080 != 0 {
                    let target = (u32::from(request[0] & 0x00ff) << 16) | u32::from(request[1]);
                    if target != 0x00ff_ffff {
                        let fad = target & 0x007f_ffff;
                        self.current_lba = fad.saturating_sub(150);
                    }
                } else {
                    let track = (request[1] >> 8) as u8;
                    if track == 0 {
                        self.current_lba = 0;
                    } else if let Some(lba) =
                        self.disc.as_ref().and_then(|disc| disc.track_start(track))
                    {
                        self.current_lba = lba;
                    }
                }
                if let Some(disc) = &self.disc {
                    self.current_lba = self.current_lba.min(disc.sectors);
                }
                self.target_lba = self.current_lba;
                self.reset_cdda_position();
                self.standard_return(self.drive_status());
                self.hirq |= 0x0001;
            }
            0x30 => {
                let connection = (request[2] >> 8) as u8;
                if connection == 0xff || usize::from(connection) < CD_PARTITION_COUNT {
                    self.cd_device_connection = connection;
                    self.standard_return(self.drive_status());
                    self.hirq |= 0x0041;
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                    self.hirq |= 0x0001;
                }
            }
            0x31 => {
                self.cr = [
                    self.drive_status(),
                    0,
                    u16::from(self.cd_device_connection) << 8,
                    0,
                ];
                self.hirq |= 0x0001;
            }
            0x32 => {
                self.cr = [
                    self.drive_status(),
                    0,
                    u16::from(self.last_buffer_destination) << 8,
                    0,
                ];
                self.hirq |= 0x0001;
            }
            0x40 => {
                let filter = usize::from((request[2] >> 8) as u8);
                if let Some(filter) = self.filters.get_mut(filter) {
                    filter.fad = (u32::from(request[0] as u8) << 16) | u32::from(request[1]);
                    filter.range = (u32::from(request[2] as u8) << 16) | u32::from(request[3]);
                    self.standard_return(self.drive_status());
                    self.hirq |= 0x0041;
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                    self.hirq |= 0x0001;
                }
            }
            0x41 => {
                let index = usize::from((request[2] >> 8) as u8);
                if let Some(filter) = self.filters.get(index) {
                    self.cr = [
                        self.drive_status() | ((filter.fad >> 16) as u16 & 0x00ff),
                        filter.fad as u16,
                        ((index as u16) << 8) | ((filter.range >> 16) as u16 & 0x00ff),
                        filter.range as u16,
                    ];
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                }
                self.hirq |= 0x0001;
            }
            0x42 => {
                let index = usize::from((request[2] >> 8) as u8);
                if let Some(filter) = self.filters.get_mut(index) {
                    filter.channel = request[0] as u8;
                    filter.submode_mask = (request[1] >> 8) as u8;
                    filter.coding_info_mask = request[1] as u8;
                    filter.file = request[2] as u8;
                    filter.submode = (request[3] >> 8) as u8;
                    filter.coding_info = request[3] as u8;
                    self.standard_return(self.drive_status());
                    self.hirq |= 0x0041;
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                    self.hirq |= 0x0001;
                }
            }
            0x43 => {
                let index = usize::from((request[2] >> 8) as u8);
                if let Some(filter) = self.filters.get(index) {
                    self.cr = [
                        self.drive_status() | u16::from(filter.channel),
                        (u16::from(filter.submode_mask) << 8) | u16::from(filter.coding_info_mask),
                        ((index as u16) << 8) | u16::from(filter.file),
                        (u16::from(filter.submode) << 8) | u16::from(filter.coding_info),
                    ];
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                }
                self.hirq |= 0x0001;
            }
            0x44 => {
                let index = usize::from((request[2] >> 8) as u8);
                if let Some(filter) = self.filters.get_mut(index) {
                    filter.mode = request[0] as u8;
                    if filter.mode & 0x80 != 0 {
                        filter.reset_condition();
                    }
                    self.standard_return(self.drive_status());
                    self.hirq |= 0x0041;
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                    self.hirq |= 0x0001;
                }
            }
            0x45 => {
                let index = usize::from((request[2] >> 8) as u8);
                if let Some(filter) = self.filters.get(index) {
                    self.cr = [
                        self.drive_status() | u16::from(filter.mode),
                        0,
                        (index as u16) << 8,
                        0,
                    ];
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                }
                self.hirq |= 0x0001;
            }
            0x46 => {
                let index = usize::from((request[2] >> 8) as u8);
                let flags = request[0] as u8;
                let true_conn = (request[1] >> 8) as u8;
                let false_conn = request[1] as u8;
                let valid_true = flags & 0x01 == 0
                    || true_conn == 0xff
                    || usize::from(true_conn) < CD_PARTITION_COUNT;
                let valid_false = flags & 0x02 == 0
                    || false_conn == 0xff
                    || usize::from(false_conn) < CD_PARTITION_COUNT;
                if index < CD_PARTITION_COUNT && valid_true && valid_false {
                    if flags & 0x01 != 0 {
                        self.filters[index].true_conn = true_conn;
                    }
                    if flags & 0x02 != 0 {
                        self.filters[index].false_conn = false_conn;
                    }
                    self.standard_return(self.drive_status());
                    self.hirq |= 0x0041;
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                    self.hirq |= 0x0001;
                }
            }
            0x47 => {
                let index = usize::from((request[2] >> 8) as u8);
                if let Some(filter) = self.filters.get(index) {
                    self.cr = [
                        self.drive_status(),
                        (u16::from(filter.true_conn) << 8) | u16::from(filter.false_conn),
                        (index as u16) << 8,
                        0,
                    ];
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                }
                self.hirq |= 0x0001;
            }
            0x48 => {
                let flags = request[0] as u8;
                let partition = usize::from((request[2] >> 8) as u8);
                if flags == 0 {
                    if partition < CD_PARTITION_COUNT {
                        self.partitions[partition].clear();
                    } else {
                        self.cr = [0xff00, 0, 0, 0];
                        self.hirq |= 0x0001;
                        return;
                    }
                } else {
                    if flags & 0x04 != 0 {
                        for partition in &mut self.partitions {
                            partition.clear();
                        }
                    }
                    for (index, filter) in self.filters.iter_mut().enumerate() {
                        if flags & 0x10 != 0 {
                            filter.reset_condition();
                        }
                        if flags & 0x20 != 0 {
                            if self.cd_device_connection == index as u8 {
                                self.cd_device_connection = 0xff;
                            }
                            if usize::from(filter.false_conn) < CD_PARTITION_COUNT {
                                filter.false_conn = 0xff;
                            }
                        }
                        if flags & 0x40 != 0 {
                            filter.true_conn = index as u8;
                        }
                        if flags & 0x80 != 0 {
                            filter.false_conn = 0xff;
                        }
                    }
                }
                self.hirq &= !0x0008;
                self.standard_return(self.drive_status());
                self.hirq |= 0x0041;
            }
            0x50 => {
                let free = CD_BUFFER_COUNT.saturating_sub(self.total_buffered_sectors());
                self.cr = [
                    self.drive_status(),
                    free as u16,
                    (CD_PARTITION_COUNT as u16) << 8,
                    CD_BUFFER_COUNT as u16,
                ];
                self.hirq |= 0x0001;
            }
            0x51 => {
                let partition = usize::from((request[2] >> 8) as u8);
                if partition < CD_PARTITION_COUNT {
                    self.cr = [
                        self.drive_status(),
                        0,
                        0,
                        self.partitions[partition].len() as u16,
                    ];
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                }
                self.hirq |= 0x0001;
            }
            0x52 => {
                let partition = usize::from((request[2] >> 8) as u8);
                let range = (partition < CD_PARTITION_COUNT)
                    .then(|| {
                        Self::selection_range(
                            self.partitions[partition].len(),
                            request[1],
                            request[3],
                        )
                    })
                    .flatten();
                if let Some(range) = range {
                    self.actual_size_words = self.partitions[partition]
                        .range(range)
                        .map(|sector| {
                            (sector.transfer_view(self.get_sector_length).len() / 2) as u32
                        })
                        .sum();
                    self.standard_return(self.drive_status());
                    self.hirq |= 0x0041;
                } else {
                    self.standard_return(0x0500);
                    self.hirq |= 0x0001;
                }
            }
            0x53 => {
                self.cr = [
                    self.drive_status() | ((self.actual_size_words >> 16) as u16 & 0x00ff),
                    self.actual_size_words as u16,
                    0,
                    0,
                ];
                self.hirq |= 0x0001;
            }
            0x54 => {
                let partition = usize::from((request[2] >> 8) as u8);
                let offset = if request[1] == 0xffff {
                    self.partitions
                        .get(partition)
                        .and_then(|part| part.len().checked_sub(1))
                } else {
                    Some(usize::from(request[1]))
                };
                let sector = offset
                    .and_then(|offset| self.partitions.get(partition)?.get(offset))
                    .cloned();
                if let Some(sector) = sector {
                    let (file, channel, submode, coding) = if sector.raw[15] == 2 {
                        (
                            sector.raw[16],
                            sector.raw[17],
                            sector.raw[18],
                            sector.raw[19],
                        )
                    } else {
                        (0, 0, 0, 0)
                    };
                    self.cr = [
                        self.drive_status() | ((sector.fad >> 16) as u16 & 0x00ff),
                        sector.fad as u16,
                        (u16::from(file) << 8) | u16::from(channel),
                        (u16::from(submode) << 8) | u16::from(coding),
                    ];
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                }
                self.hirq |= 0x0001;
            }
            0x55 => {
                let partition = usize::from((request[2] >> 8) as u8);
                let search_fad = (u32::from(request[2] as u8) << 16) | u32::from(request[3]);
                let start = if partition < CD_PARTITION_COUNT {
                    if request[1] == 0xffff {
                        self.partitions[partition].len().checked_sub(1)
                    } else {
                        Some(usize::from(request[1]))
                    }
                } else {
                    None
                };
                if let Some(start) = start.filter(|&start| start < self.partitions[partition].len())
                {
                    self.fad_search_fad = 0;
                    self.fad_search_pos = 0xffff;
                    self.fad_search_partition = partition as u8;
                    for (index, sector) in self.partitions[partition].iter().enumerate().skip(start)
                    {
                        if sector.fad <= search_fad
                            && (self.fad_search_pos == 0xffff || sector.fad >= self.fad_search_fad)
                        {
                            self.fad_search_pos = index as u16;
                            self.fad_search_fad = sector.fad;
                        }
                    }
                    self.standard_return(self.drive_status());
                    self.hirq |= 0x0041;
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                    self.hirq |= 0x0001;
                }
            }
            0x56 => {
                self.cr = [
                    self.drive_status(),
                    self.fad_search_pos,
                    (u16::from(self.fad_search_partition) << 8)
                        | ((self.fad_search_fad >> 16) as u16 & 0x00ff),
                    self.fad_search_fad as u16,
                ];
                self.hirq |= 0x0001;
            }
            0x60 => {
                let get_length = request[0] as u8;
                let put_length = (request[1] >> 8) as u8;
                let valid_get = get_length == 0xff || get_length <= 3;
                let valid_put = put_length == 0xff || put_length <= 3;
                if valid_get && valid_put {
                    if get_length != 0xff {
                        self.get_sector_length = get_length;
                    }
                    if put_length != 0xff {
                        self.put_sector_length = put_length;
                    }
                    self.standard_return(self.drive_status());
                    self.hirq |= 0x0041;
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                    self.hirq |= 0x0001;
                }
            }
            0x61..=0x63 => {
                let partition = usize::from((request[2] >> 8) as u8);
                let range = (partition < CD_PARTITION_COUNT)
                    .then(|| {
                        Self::selection_range(
                            self.partitions[partition].len(),
                            request[1],
                            request[3],
                        )
                    })
                    .flatten();
                if let Some(range) = range {
                    if command == 0x62 {
                        self.partitions[partition].drain(range);
                        self.hirq &= !0x0008;
                        self.standard_return(self.drive_status());
                        self.hirq |= 0x0081;
                    } else {
                        self.queue_partition_transfer(partition, range, command == 0x63);
                        if command == 0x63 {
                            self.hirq &= !0x0008;
                        }
                    }
                } else {
                    self.standard_return(0x0500);
                    self.hirq |= 0x0001;
                }
            }
            0x65 | 0x66 => {
                let destination_filter = request[0] as u8;
                let source_partition = usize::from((request[2] >> 8) as u8);
                let range = (source_partition < CD_PARTITION_COUNT)
                    .then(|| {
                        Self::selection_range(
                            self.partitions[source_partition].len(),
                            request[1],
                            request[3],
                        )
                    })
                    .flatten();
                let valid_destination = usize::from(destination_filter) < CD_PARTITION_COUNT;
                if let Some(range) = range.filter(|_| valid_destination) {
                    let count = range.len();
                    if command == 0x65
                        && self.total_buffered_sectors().saturating_add(count) > CD_BUFFER_COUNT
                    {
                        self.standard_return(0x0600);
                        self.hirq |= 0x0001;
                        return;
                    }
                    let sectors = if command == 0x66 {
                        self.partitions[source_partition]
                            .drain(range)
                            .collect::<Vec<_>>()
                    } else {
                        self.partitions[source_partition]
                            .range(range)
                            .cloned()
                            .collect::<Vec<_>>()
                    };
                    if command == 0x66 {
                        self.hirq &= !0x0008;
                    }
                    for sector in sectors {
                        if let Some((filter, partition)) =
                            self.route_sector(destination_filter, &sector)
                        {
                            self.partitions[partition].push_back(sector);
                            self.last_buffer_destination = filter;
                        } else {
                            self.last_buffer_destination = 0xff;
                        }
                    }
                    if self.total_buffered_sectors() >= CD_BUFFER_COUNT {
                        self.hirq |= 0x0008;
                    }
                    self.standard_return(self.drive_status());
                    self.hirq |= 0x0101;
                } else {
                    self.cr = [0xff00, 0, 0, 0];
                    self.hirq |= 0x0001;
                }
            }
            0x67 => {
                self.cr = [self.drive_status(), 0, 0, 0];
                self.hirq |= 0x0001;
            }
            _ => {
                self.cr = [0xff00, 0, 0, 0];
                self.hirq |= 0x0001;
            }
        }
    }

    fn tick(&mut self, sh_cycles: u32) {
        if self.audio_path_active() {
            self.tick_cdda(sh_cycles);
            return;
        }
        if !self.reading {
            return;
        }
        self.phase = self
            .phase
            .saturating_add(u64::from(sh_cycles) * SECTORS_PER_SECOND);
        while self.phase >= SH2_HZ {
            self.phase -= SH2_HZ;
            if self.total_buffered_sectors() >= CD_BUFFER_COUNT {
                self.hirq |= 0x0008;
                break;
            }
            let Some(disc) = &self.disc else {
                self.reading = false;
                self.hirq |= 0x0010;
                break;
            };
            if self.current_lba >= disc.sectors || self.current_lba >= self.target_lba {
                self.reading = false;
                self.hirq |= 0x0010;
                break;
            }
            let mut raw = Box::new([0u8; 2352]);
            match disc.read_raw_sector(self.current_lba, raw.as_mut()) {
                Ok(()) => {
                    let sector = SaturnCdBufferedSector {
                        fad: self.current_fad(),
                        raw,
                    };
                    if let Some((filter, partition)) =
                        self.route_sector(self.cd_device_connection, &sector)
                    {
                        self.partitions[partition].push_back(sector);
                        self.last_buffer_destination = filter;
                    } else {
                        self.last_buffer_destination = 0xff;
                    }
                    self.current_lba = self.current_lba.wrapping_add(1);
                    self.hirq |= 0x0004;
                    if self.total_buffered_sectors() >= CD_BUFFER_COUNT {
                        self.hirq |= 0x0008;
                    }
                }
                Err(_) => break,
            }
        }
    }

    fn register16(&self, address: u32) -> u16 {
        let offset = address.wrapping_sub(0x0580_0000);
        match offset {
            0x0009_0008 => self.hirq,
            0x0009_000c => self.hirq_mask,
            0x0009_0018 => self.cr[0],
            0x0009_001c => self.cr[1],
            0x0009_0020 => self.cr[2],
            0x0009_0024 => self.cr[3],
            0x0009_0028 => self.data.len().min(u16::MAX as usize) as u16,
            _ => 0,
        }
    }

    fn read8(&mut self, address: u32) -> u8 {
        let offset = address.wrapping_sub(0x0580_0000);
        if matches!(offset, 0x0001_8000 | 0x0001_8001) {
            return self.pop_data_byte();
        }
        let aligned = address & !1;
        let value = self.register16(aligned).to_be_bytes();
        value[(address & 1) as usize]
    }

    fn read16(&mut self, address: u32) -> u16 {
        let offset = address.wrapping_sub(0x0580_0000);
        if offset == 0x0001_8000 {
            let high = self.pop_data_byte();
            let low = self.pop_data_byte();
            return u16::from_be_bytes([high, low]);
        }
        self.register16(address & !1)
    }

    fn write8(&mut self, address: u32, value: u8) {
        let aligned = address & !1;
        let mut bytes = self.register16(aligned).to_be_bytes();
        bytes[(address & 1) as usize] = value;
        self.write16(aligned, u16::from_be_bytes(bytes));
    }

    fn write16(&mut self, address: u32, value: u16) {
        let offset = address.wrapping_sub(0x0580_0000);
        match offset {
            0x0009_0008 => self.hirq &= !value,
            0x0009_000c => self.hirq_mask = value,
            0x0009_0018 => self.cr[0] = value,
            0x0009_001c => self.cr[1] = value,
            0x0009_0020 => self.cr[2] = value,
            0x0009_0024 => {
                self.cr[3] = value;
                self.issue_command();
            }
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u16(self.hirq);
        out.u16(self.hirq_mask);
        for value in self.cr {
            out.u16(value);
        }
        out.u32(self.target_lba);
        out.u32(self.current_lba);
        out.u8(u8::from(self.reading));
        out.u64(self.phase);
        out.u32(self.data.len() as u32);
        for byte in &self.data {
            out.u8(*byte);
        }
        out.u8(self.cd_device_connection);
        out.u8(self.last_buffer_destination);
        for filter in self.filters {
            out.u8(filter.mode);
            out.u8(filter.true_conn);
            out.u8(filter.false_conn);
            out.u32(filter.fad);
            out.u32(filter.range);
            out.u8(filter.channel);
            out.u8(filter.file);
            out.u8(filter.submode);
            out.u8(filter.submode_mask);
            out.u8(filter.coding_info);
            out.u8(filter.coding_info_mask);
        }
        out.u8(self.get_sector_length);
        out.u8(self.put_sector_length);
        out.u32(self.actual_size_words);
        out.u32(self.fad_search_fad);
        out.u16(self.fad_search_pos);
        out.u8(self.fad_search_partition);
        out.u32(self.transfer_bytes_read);
        for partition in &self.partitions {
            out.u16(partition.len() as u16);
            for sector in partition {
                out.u32(sector.fad);
                out.blob(sector.raw.as_slice());
            }
        }
        out.blob(self.cdda_sector.as_slice());
        out.u16(self.cdda_sample_index as u16);
        out.u64(self.cdda_phase);
        out.u32(self.cdda_samples.len() as u32);
        for sample in &self.cdda_samples {
            out.u16(sample[0] as u16);
            out.u16(sample[1] as u16);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.hirq = input.u16()?;
        self.hirq_mask = input.u16()?;
        for value in &mut self.cr {
            *value = input.u16()?;
        }
        self.target_lba = input.u32()?;
        self.current_lba = input.u32()?;
        self.reading = input.u8()? != 0;
        self.phase = input.u64()? % SH2_HZ;
        self.data.clear();
        let len = input.u32()? as usize;
        if len > CD_BUFFER_COUNT * 2352 + 4096 {
            return Err("Saturn CD host-transfer state is too large".into());
        }
        for _ in 0..len {
            self.data.push_back(input.u8()?);
        }
        self.cd_device_connection = input.u8()?;
        if self.cd_device_connection != 0xff
            && usize::from(self.cd_device_connection) >= CD_PARTITION_COUNT
        {
            return Err("Saturn CD device connection is invalid".into());
        }
        self.last_buffer_destination = input.u8()?;
        if self.last_buffer_destination != 0xff
            && usize::from(self.last_buffer_destination) >= CD_PARTITION_COUNT
        {
            return Err("Saturn CD last buffer destination is invalid".into());
        }
        for filter in &mut self.filters {
            filter.mode = input.u8()?;
            filter.true_conn = input.u8()?;
            filter.false_conn = input.u8()?;
            if filter.true_conn != 0xff && usize::from(filter.true_conn) >= CD_PARTITION_COUNT {
                return Err("Saturn CD filter true connection is invalid".into());
            }
            if filter.false_conn != 0xff && usize::from(filter.false_conn) >= CD_PARTITION_COUNT {
                return Err("Saturn CD filter false connection is invalid".into());
            }
            filter.fad = input.u32()?;
            filter.range = input.u32()?;
            filter.channel = input.u8()?;
            filter.file = input.u8()?;
            filter.submode = input.u8()?;
            filter.submode_mask = input.u8()?;
            filter.coding_info = input.u8()?;
            filter.coding_info_mask = input.u8()?;
        }
        self.get_sector_length = input.u8()?;
        self.put_sector_length = input.u8()?;
        if self.get_sector_length > 3 || self.put_sector_length > 3 {
            return Err("Saturn CD sector-length state is invalid".into());
        }
        self.actual_size_words = input.u32()?;
        self.fad_search_fad = input.u32()?;
        self.fad_search_pos = input.u16()?;
        self.fad_search_partition = input.u8()?;
        if usize::from(self.fad_search_partition) >= CD_PARTITION_COUNT {
            return Err("Saturn CD FAD-search partition is invalid".into());
        }
        self.transfer_bytes_read = input.u32()?;
        let mut buffered = 0usize;
        for partition in &mut self.partitions {
            partition.clear();
            let count = usize::from(input.u16()?);
            buffered = buffered
                .checked_add(count)
                .ok_or_else(|| "Saturn CD buffer count overflow".to_string())?;
            if buffered > CD_BUFFER_COUNT {
                return Err("Saturn CD buffer state exceeds hardware capacity".into());
            }
            for _ in 0..count {
                let fad = input.u32()?;
                let raw = input.blob()?;
                if raw.len() != 2352 {
                    return Err("Saturn CD buffered sector has invalid length".into());
                }
                let mut sector = Box::new([0u8; 2352]);
                sector.copy_from_slice(raw);
                partition.push_back(SaturnCdBufferedSector { fad, raw: sector });
            }
        }
        let cdda_sector = input.blob()?;
        if cdda_sector.len() != self.cdda_sector.len() {
            return Err("Saturn CDDA sector state has invalid length".into());
        }
        self.cdda_sector.copy_from_slice(cdda_sector);
        self.cdda_sample_index = usize::from(input.u16()?);
        if self.cdda_sample_index > CDDA_FRAMES_PER_SECTOR {
            return Err("Saturn CDDA sample index is invalid".into());
        }
        self.cdda_phase = input.u64()? % SH2_HZ;
        self.cdda_samples.clear();
        let cdda_len = input.u32()?.min(4096);
        for _ in 0..cdda_len {
            self.cdda_samples
                .push_back([input.u16()? as i16, input.u16()? as i16]);
        }
        Ok(())
    }
}

#[derive(Clone)]
struct SaturnSmpc {
    regs: [u8; 0x80],
    slave_on: bool,
    sound_on: bool,
    interrupt_pending: bool,
    pad: [u16; 2],
}

impl Default for SaturnSmpc {
    fn default() -> Self {
        Self {
            regs: [0; 0x80],
            slave_on: false,
            sound_on: false,
            interrupt_pending: false,
            pad: [0xffff; 2],
        }
    }
}

impl SaturnSmpc {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn set_input(&mut self, input: &InputState) {
        for player in 0..2 {
            let buttons = input.buttons[player];
            let mut value = 0xffffu16;
            let mapping = [
                (UP, 0),
                (DOWN, 1),
                (LEFT, 2),
                (RIGHT, 3),
                (START, 4),
                (FACE_SOUTH, 5),
                (FACE_EAST, 6),
                (FACE_NORTH, 7),
                (FACE_WEST, 8),
                (L1, 9),
                (R1, 10),
            ];
            for (button, bit) in mapping {
                if buttons & button != 0 {
                    value &= !(1 << bit);
                }
            }
            self.pad[player] = value;
        }
    }

    fn command(&mut self, value: u8) {
        self.regs[0x1f] = value;
        self.regs[0x61] = 0x40;
        match value {
            0x02 => self.slave_on = true,
            0x03 => self.slave_on = false,
            0x06 => self.sound_on = true,
            0x07 => self.sound_on = false,
            0x0d => {
                self.slave_on = false;
                self.sound_on = false;
            }
            0x10 => {
                let [p1h, p1l] = self.pad[0].to_be_bytes();
                let [p2h, p2l] = self.pad[1].to_be_bytes();
                self.regs[0x21] = 0x10;
                self.regs[0x23] = p1h;
                self.regs[0x25] = p1l;
                self.regs[0x27] = p2h;
                self.regs[0x29] = p2l;
            }
            _ => {}
        }
        self.regs[0x63] = 0;
        self.interrupt_pending = true;
    }

    fn read8(&self, offset: usize) -> u8 {
        self.regs[offset & 0x7f]
    }

    fn write8(&mut self, offset: usize, value: u8) {
        let offset = offset & 0x7f;
        self.regs[offset] = value;
        if offset == 0x1f {
            self.command(value);
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
        out.u8(u8::from(self.slave_on));
        out.u8(u8::from(self.sound_on));
        out.u8(u8::from(self.interrupt_pending));
        out.u16(self.pad[0]);
        out.u16(self.pad[1]);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("Saturn SMPC state size mismatch".into());
        }
        self.regs.copy_from_slice(regs);
        self.slave_on = input.u8()? != 0;
        self.sound_on = input.u8()? != 0;
        self.interrupt_pending = input.u8()? != 0;
        self.pad[0] = input.u16()?;
        self.pad[1] = input.u16()?;
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct ScuDmaRequest {
    source: u32,
    destination: u32,
    count: u32,
    source_step: u32,
    destination_step: u32,
    indirect: bool,
    read_update: bool,
    write_update: bool,
}

struct SaturnScu {
    regs: [u8; 0xd0],
    irq_status: u32,
    irq_mask: u32,
    dma_pending: [bool; 3],
    hblank_phase: u64,
    timer0_counter: u16,
    timer1_countdown: u32,
    timer1_armed: bool,
}

impl Default for SaturnScu {
    fn default() -> Self {
        Self {
            regs: [0; 0xd0],
            irq_status: 0,
            irq_mask: 0x0000_bfff,
            dma_pending: [false; 3],
            hblank_phase: 0,
            timer0_counter: 0,
            timer1_countdown: 0,
            timer1_armed: false,
        }
    }
}

impl SaturnScu {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn reg32(&self, offset: usize) -> u32 {
        let offset = offset & !3;
        u32::from_be_bytes([
            self.regs[offset],
            self.regs[offset + 1],
            self.regs[offset + 2],
            self.regs[offset + 3],
        ])
    }

    fn dma_status(&self) -> u32 {
        let mut status = 0u32;
        for channel in 0..3 {
            if self.dma_pending[channel] {
                status |= 0x20u32 << (channel * 4);
            }
        }
        status
    }

    fn read8(&self, offset: usize) -> u8 {
        match offset {
            0x5c..=0x5f => self.dma_status().to_be_bytes()[offset - 0x5c],
            0x7c..=0x7f => self.dma_status().to_be_bytes()[offset - 0x7c],
            0xa0..=0xa3 => self.irq_mask.to_be_bytes()[offset - 0xa0],
            0xa4..=0xa7 => self.irq_status.to_be_bytes()[offset - 0xa4],
            0xc8..=0xcb => 4u32.to_be_bytes()[offset - 0xc8],
            _ => self.regs[offset.min(0xcf)],
        }
    }

    fn write8(&mut self, offset: usize, value: u8) {
        if offset >= self.regs.len() {
            return;
        }
        self.regs[offset] = value;
        if (0xa0..=0xa3).contains(&offset) {
            self.irq_mask = u32::from_be_bytes([
                self.regs[0xa0],
                self.regs[0xa1],
                self.regs[0xa2],
                self.regs[0xa3],
            ]);
        } else if (0xa4..=0xa7).contains(&offset) {
            let clear = u32::from_be_bytes([
                self.regs[0xa4],
                self.regs[0xa5],
                self.regs[0xa6],
                self.regs[0xa7],
            ]);
            self.irq_status &= !clear;
        }
        if matches!(offset, 0x9a | 0x9b) && be16(&self.regs, 0x9a) & 1 == 0 {
            self.timer0_counter = 0;
            self.timer1_countdown = 0;
            self.timer1_armed = false;
        }
        for channel in 0..3 {
            let base = channel * 0x20;
            if offset == base + 0x13
                && self.dma_enabled(channel)
                && self.dma_start_factor(channel) == 7
                && self.reg32(base + 0x10) & 1 != 0
            {
                self.dma_pending[channel] = true;
            }
        }
    }

    fn dma_enabled(&self, channel: usize) -> bool {
        channel < 3 && self.reg32(channel * 0x20 + 0x10) & 0x0100 != 0
    }

    fn dma_start_factor(&self, channel: usize) -> u8 {
        if channel >= 3 {
            return 7;
        }
        (self.reg32(channel * 0x20 + 0x14) & 7) as u8
    }

    fn dma_event(&mut self, event: u8) {
        for channel in 0..3 {
            if self.dma_enabled(channel) && self.dma_start_factor(channel) == event {
                self.dma_pending[channel] = true;
            }
        }
    }

    fn raise(&mut self, bit: u8) {
        self.irq_status |= 1u32 << bit.min(31);
    }

    fn timers_enabled(&self) -> bool {
        be16(&self.regs, 0x9a) & 1 != 0
    }

    fn timer1_mode(&self) -> bool {
        be16(&self.regs, 0x9a) & 0x0100 != 0
    }

    fn timer0_compare(&self) -> u16 {
        (self.reg32(0x90) & 0x03ff) as u16
    }

    fn timer1_set(&self) -> u16 {
        (self.reg32(0x94) & 0x01ff) as u16
    }

    fn arm_timer1(&mut self) {
        let cycles = u32::from(self.timer1_set()).saturating_mul(4);
        if cycles == 0 {
            self.dma_event(4);
            self.raise(4);
            self.timer1_countdown = 0;
            self.timer1_armed = false;
        } else {
            self.timer1_countdown = cycles;
            self.timer1_armed = true;
        }
    }

    fn vblank_out(&mut self) {
        self.dma_event(1);
        self.raise(1);
        self.timer0_counter = 0;
    }

    fn vblank_in(&mut self) {
        self.dma_event(0);
        self.raise(0);
    }

    fn hblank_in(&mut self) {
        self.dma_event(2);
        self.raise(2);
        if self.timers_enabled() {
            let compare = self.timer0_compare();
            let timer0_hit = compare & 0x0200 == 0 && self.timer0_counter == (compare & 0x01ff);
            if timer0_hit {
                self.dma_event(3);
                self.raise(3);
            }
            if timer0_hit || !self.timer1_mode() {
                self.arm_timer1();
            }
        }
        self.timer0_counter = self.timer0_counter.wrapping_add(1) & 0x01ff;
    }

    fn tick_timing(&mut self, sh_cycles: u32) {
        if self.timer1_armed {
            if self.timer1_countdown <= sh_cycles {
                self.timer1_countdown = 0;
                self.timer1_armed = false;
                self.dma_event(4);
                self.raise(4);
            } else {
                self.timer1_countdown -= sh_cycles;
            }
        }

        const NTSC_TOTAL_LINES: u64 = 263;
        let threshold = SH2_HZ.saturating_mul(FRAME_RATE_DEN);
        self.hblank_phase = self.hblank_phase.saturating_add(
            u64::from(sh_cycles)
                .saturating_mul(FRAME_RATE_NUM)
                .saturating_mul(NTSC_TOTAL_LINES),
        );
        while self.hblank_phase >= threshold {
            self.hblank_phase -= threshold;
            self.hblank_in();
        }
    }

    fn interrupt(&self) -> Option<(u8, u8)> {
        let pending = self.irq_status & !self.irq_mask;
        if pending == 0 {
            return None;
        }
        let bit = pending.trailing_zeros() as u8;
        const LEVELS: [u8; 32] = [
            15, 14, 13, 12, 11, 10, 9, 8, 8, 6, 6, 5, 3, 2, 0, 0, 7, 7, 7, 7, 4, 4, 4, 4, 1, 1, 1,
            1, 1, 1, 1, 1,
        ];
        let level = LEVELS[usize::from(bit)];
        (level != 0).then_some((level, 0x40u8.wrapping_add(bit)))
    }

    fn take_dma(&mut self, channel: usize) -> Option<ScuDmaRequest> {
        if channel >= 3 || !self.dma_pending[channel] {
            return None;
        }
        self.dma_pending[channel] = false;
        let base = channel * 0x20;
        let address_control = self.reg32(base + 0x0c);
        let mode = self.reg32(base + 0x14);
        let count_mask = if channel == 0 {
            0x000f_ffff
        } else {
            0x0000_0fff
        };
        let maximum = if channel == 0 {
            0x0010_0000
        } else {
            0x0000_1000
        };
        let raw_count = self.reg32(base + 8) & count_mask;
        let count = if raw_count == 0 { maximum } else { raw_count };
        let destination_shift = address_control & 7;
        let destination_step = if destination_shift == 0 {
            0
        } else {
            1u32 << destination_shift
        };
        Some(ScuDmaRequest {
            source: self.reg32(base) & 0x27ff_ffff,
            destination: self.reg32(base + 4) & 0x27ff_ffff,
            count,
            source_step: if address_control & 0x0100 != 0 { 2 } else { 0 },
            destination_step,
            indirect: mode & 0x0100_0000 != 0,
            read_update: mode & 0x0001_0000 != 0,
            write_update: mode & 0x0000_0100 != 0,
        })
    }

    fn complete_dma(
        &mut self,
        channel: usize,
        final_source: u32,
        final_destination: u32,
        read_update: bool,
        write_update: bool,
    ) {
        if channel >= 3 {
            return;
        }
        let base = channel * 0x20;
        if read_update {
            self.regs[base..base + 4].copy_from_slice(&final_source.to_be_bytes());
        }
        if write_update {
            self.regs[base + 4..base + 8].copy_from_slice(&final_destination.to_be_bytes());
        }
        let enable = self.reg32(base + 0x10) & !1;
        self.regs[base + 0x10..base + 0x14].copy_from_slice(&enable.to_be_bytes());
        self.raise(11 - channel as u8);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(&self.regs);
        out.u32(self.irq_status);
        out.u32(self.irq_mask);
        for pending in self.dma_pending {
            out.u8(u8::from(pending));
        }
        out.u64(self.hblank_phase);
        out.u16(self.timer0_counter);
        out.u32(self.timer1_countdown);
        out.u8(u8::from(self.timer1_armed));
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("Saturn SCU state size mismatch".into());
        }
        self.regs.copy_from_slice(regs);
        self.irq_status = input.u32()?;
        self.irq_mask = input.u32()?;
        for pending in &mut self.dma_pending {
            *pending = input.u8()? != 0;
        }
        self.hblank_phase = input.u64()? % SH2_HZ.saturating_mul(FRAME_RATE_DEN);
        self.timer0_counter = input.u16()? & 0x01ff;
        self.timer1_countdown = input.u32()?;
        self.timer1_armed = input.u8()? != 0;
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct Vdp1Texture {
    source: usize,
    width: usize,
    height: usize,
    ctrl: u16,
    pmod: u16,
    color: u16,
}

impl Vdp1Texture {
    fn from_command(source: usize, size: u16, ctrl: u16, pmod: u16, color: u16) -> Self {
        Self {
            source,
            width: usize::from((size >> 8) & 0x3f) * 8,
            height: usize::from(size & 0xff),
            ctrl,
            pmod,
            color,
        }
    }
}

struct SaturnVdp1 {
    vram: Box<[u8; VDP1_VRAM_SIZE]>,
    framebuffers: [Box<[u8; VDP1_FB_SIZE]>; 2],
    regs: [u8; 0x20],
    draw: usize,
    display: usize,
    local: [i32; 2],
    system_clip: [i32; 4],
    user_clip: [i32; 4],
}

impl Default for SaturnVdp1 {
    fn default() -> Self {
        Self {
            vram: Box::new([0; VDP1_VRAM_SIZE]),
            framebuffers: [Box::new([0; VDP1_FB_SIZE]), Box::new([0; VDP1_FB_SIZE])],
            regs: [0; 0x20],
            draw: 0,
            display: 1,
            local: [0; 2],
            system_clip: [0, 0, 511, 255],
            user_clip: [0, 0, 511, 255],
        }
    }
}

impl SaturnVdp1 {
    fn reset(&mut self) {
        self.vram.fill(0);
        for framebuffer in &mut self.framebuffers {
            framebuffer.fill(0);
        }
        self.regs.fill(0);
        self.draw = 0;
        self.display = 1;
        self.local = [0; 2];
        self.system_clip = [0, 0, 511, 255];
        self.user_clip = [0, 0, 511, 255];
    }

    fn read8(&self, address: u32) -> u8 {
        match address {
            0x05c0_0000..=0x05c7_ffff => {
                self.vram[(address as usize - 0x05c0_0000) & (VDP1_VRAM_SIZE - 1)]
            }
            0x05c8_0000..=0x05cb_ffff => {
                self.framebuffers[self.draw][(address as usize - 0x05c8_0000) & (VDP1_FB_SIZE - 1)]
            }
            0x05d0_0000..=0x05d0_001f => self.regs[address as usize - 0x05d0_0000],
            _ => 0,
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        match address {
            0x05c0_0000..=0x05c7_ffff => {
                self.vram[(address as usize - 0x05c0_0000) & (VDP1_VRAM_SIZE - 1)] = value
            }
            0x05c8_0000..=0x05cb_ffff => {
                self.framebuffers[self.draw]
                    [(address as usize - 0x05c8_0000) & (VDP1_FB_SIZE - 1)] = value
            }
            0x05d0_0000..=0x05d0_001f => self.regs[address as usize - 0x05d0_0000] = value,
            _ => {}
        }
    }

    fn in_rect(rect: [i32; 4], x: i32, y: i32) -> bool {
        x >= rect[0] && x <= rect[2] && y >= rect[1] && y <= rect[3]
    }

    fn pixel_allowed(&self, x: i32, y: i32, pmod: u16) -> bool {
        if x < 0 || y < 0 || x >= 512 || y >= 256 || !Self::in_rect(self.system_clip, x, y) {
            return false;
        }
        if pmod & 0x0400 != 0 {
            let inside = Self::in_rect(self.user_clip, x, y);
            if (pmod & 0x0200 == 0 && !inside) || (pmod & 0x0200 != 0 && inside) {
                return false;
            }
        }
        pmod & 0x0100 == 0 || ((x ^ y) & 1) == 0
    }

    fn put_pixel(&mut self, x: i32, y: i32, color: u16, pmod: u16) {
        if !self.pixel_allowed(x, y, pmod) {
            return;
        }
        let offset = (y as usize * 512 + x as usize) * 2;
        if offset + 1 < VDP1_FB_SIZE {
            put_be16(self.framebuffers[self.draw].as_mut_slice(), offset, color);
        }
    }

    fn line(&mut self, mut x0: i32, mut y0: i32, x1: i32, y1: i32, color: u16, pmod: u16) {
        let dx = (x1 - x0).abs();
        let sx = if x0 < x1 { 1 } else { -1 };
        let dy = -(y1 - y0).abs();
        let sy = if y0 < y1 { 1 } else { -1 };
        let mut error = dx + dy;
        loop {
            self.put_pixel(x0, y0, color, pmod);
            if x0 == x1 && y0 == y1 {
                break;
            }
            let doubled = error * 2;
            if doubled >= dy {
                error += dy;
                x0 += sx;
            }
            if doubled <= dx {
                error += dx;
                y0 += sy;
            }
        }
    }

    fn texture_pixel(&self, texture: Vdp1Texture, sx: usize, sy: usize) -> Option<u16> {
        let Vdp1Texture {
            source,
            width,
            height,
            ctrl,
            pmod,
            color,
        } = texture;
        if width == 0 {
            return None;
        }
        let dir = (ctrl >> 4) & 3;
        let x = if dir & 1 != 0 {
            width.saturating_sub(1).saturating_sub(sx)
        } else {
            sx
        };
        let y = if dir & 2 != 0 {
            height.saturating_sub(1).saturating_sub(sy)
        } else {
            sy
        };
        let mode = (pmod >> 3) & 7;
        let transparent_zero = pmod & 0x0040 == 0;
        let end_codes = pmod & 0x0080 == 0;
        let mask = VDP1_VRAM_SIZE - 1;
        match mode {
            0 | 1 => {
                let pixel_index = y.saturating_mul(width).saturating_add(x);
                let byte = self.vram[(source + pixel_index / 2) & mask];
                let value = if pixel_index & 1 == 0 {
                    byte >> 4
                } else {
                    byte & 0x0f
                };
                if (transparent_zero && value == 0) || (end_codes && value == 0x0f) {
                    return None;
                }
                if mode == 0 {
                    Some((color & 0xfff0) | u16::from(value))
                } else {
                    let table = (usize::from(color) * 8 + usize::from(value) * 2) & mask;
                    Some(be16_wrapped(self.vram.as_slice(), table))
                }
            }
            2..=4 => {
                let value = self.vram[(source + y.saturating_mul(width) + x) & mask];
                if (transparent_zero && value == 0) || (end_codes && value == 0xff) {
                    return None;
                }
                let bits = match mode {
                    2 => 6,
                    3 => 7,
                    _ => 8,
                };
                let pixel_mask = (1u16 << bits) - 1;
                Some((color & !pixel_mask) | (u16::from(value) & pixel_mask))
            }
            5 => {
                let pixel_index = y.saturating_mul(width).saturating_add(x);
                let value = be16_wrapped(self.vram.as_slice(), source + pixel_index * 2);
                if (transparent_zero && value & 0x7fff == 0)
                    || (end_codes && value & 0x7fff == 0x7fff)
                {
                    None
                } else {
                    Some(value | 0x8000)
                }
            }
            _ => None,
        }
    }

    fn draw_textured_rect(&mut self, texture: Vdp1Texture, rect: [i32; 4]) {
        let [x0, y0, x1, y1] = rect;
        let width = texture.width;
        let height = texture.height;
        if width == 0 || height == 0 {
            return;
        }
        let dest_width = (x1 - x0).unsigned_abs() as usize + 1;
        let dest_height = (y1 - y0).unsigned_abs() as usize + 1;
        for dy in 0..dest_height {
            let y = if y1 >= y0 {
                y0 + dy as i32
            } else {
                y0 - dy as i32
            };
            let sy = (dy * height / dest_height).min(height - 1);
            for dx in 0..dest_width {
                let x = if x1 >= x0 {
                    x0 + dx as i32
                } else {
                    x0 - dx as i32
                };
                let sx = (dx * width / dest_width).min(width - 1);
                if let Some(pixel) = self.texture_pixel(texture, sx, sy) {
                    self.put_pixel(x, y, pixel, texture.pmod);
                }
            }
        }
    }

    fn triangle_weights(
        px: f32,
        py: f32,
        a: (f32, f32),
        b: (f32, f32),
        c: (f32, f32),
    ) -> Option<[f32; 3]> {
        let denominator = (b.1 - c.1) * (a.0 - c.0) + (c.0 - b.0) * (a.1 - c.1);
        if denominator.abs() < f32::EPSILON {
            return None;
        }
        let wa = ((b.1 - c.1) * (px - c.0) + (c.0 - b.0) * (py - c.1)) / denominator;
        let wb = ((c.1 - a.1) * (px - c.0) + (a.0 - c.0) * (py - c.1)) / denominator;
        let wc = 1.0 - wa - wb;
        (wa >= -0.001 && wb >= -0.001 && wc >= -0.001).then_some([wa, wb, wc])
    }

    fn fill_triangle(&mut self, points: [(i32, i32); 3], color: u16, pmod: u16) {
        let min_x = points.iter().map(|point| point.0).min().unwrap_or(0).max(0);
        let max_x = points
            .iter()
            .map(|point| point.0)
            .max()
            .unwrap_or(-1)
            .min(511);
        let min_y = points.iter().map(|point| point.1).min().unwrap_or(0).max(0);
        let max_y = points
            .iter()
            .map(|point| point.1)
            .max()
            .unwrap_or(-1)
            .min(255);
        let [a, b, c] = points.map(|point| (point.0 as f32, point.1 as f32));
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                if Self::triangle_weights(x as f32 + 0.5, y as f32 + 0.5, a, b, c).is_some() {
                    self.put_pixel(x, y, color, pmod);
                }
            }
        }
    }

    fn draw_textured_triangle(
        &mut self,
        points: [(i32, i32); 3],
        uv: [(f32, f32); 3],
        texture: Vdp1Texture,
    ) {
        let width = texture.width;
        let height = texture.height;
        if width == 0 || height == 0 {
            return;
        }
        let min_x = points.iter().map(|point| point.0).min().unwrap_or(0).max(0);
        let max_x = points
            .iter()
            .map(|point| point.0)
            .max()
            .unwrap_or(-1)
            .min(511);
        let min_y = points.iter().map(|point| point.1).min().unwrap_or(0).max(0);
        let max_y = points
            .iter()
            .map(|point| point.1)
            .max()
            .unwrap_or(-1)
            .min(255);
        let [a, b, c] = points.map(|point| (point.0 as f32, point.1 as f32));
        for y in min_y..=max_y {
            for x in min_x..=max_x {
                let Some(weights) = Self::triangle_weights(x as f32 + 0.5, y as f32 + 0.5, a, b, c)
                else {
                    continue;
                };
                let u = weights[0] * uv[0].0 + weights[1] * uv[1].0 + weights[2] * uv[2].0;
                let v = weights[0] * uv[0].1 + weights[1] * uv[1].1 + weights[2] * uv[2].1;
                let sx = (u.clamp(0.0, 1.0) * (width - 1) as f32).round() as usize;
                let sy = (v.clamp(0.0, 1.0) * (height - 1) as f32).round() as usize;
                if let Some(pixel) = self.texture_pixel(texture, sx, sy) {
                    self.put_pixel(x, y, pixel, texture.pmod);
                }
            }
        }
    }

    fn render_commands(&mut self) {
        self.framebuffers[self.draw].fill(0);
        let mut address = 0usize;
        let mut call_stack = Vec::with_capacity(8);
        for _ in 0..4096 {
            if address + 0x20 > self.vram.len() {
                break;
            }
            let ctrl = be16(self.vram.as_slice(), address);
            if ctrl & 0x8000 != 0 {
                break;
            }
            let jump = (ctrl >> 12) & 7;
            let skip = jump & 4 != 0;
            let command = ctrl & 0x000f;
            let link = usize::from(be16(self.vram.as_slice(), address + 2)) * 8;
            if !skip {
                let pmod = be16(self.vram.as_slice(), address + 4);
                let color = be16(self.vram.as_slice(), address + 6);
                let source = usize::from(be16(self.vram.as_slice(), address + 8)) * 8;
                let size = be16(self.vram.as_slice(), address + 0x0a);
                let texture = Vdp1Texture::from_command(source, size, ctrl, pmod, color);
                let raw_xa = i32::from(be16(self.vram.as_slice(), address + 0x0c) as i16);
                let raw_ya = i32::from(be16(self.vram.as_slice(), address + 0x0e) as i16);
                let raw_xb = i32::from(be16(self.vram.as_slice(), address + 0x10) as i16);
                let raw_yb = i32::from(be16(self.vram.as_slice(), address + 0x12) as i16);
                let raw_xc = i32::from(be16(self.vram.as_slice(), address + 0x14) as i16);
                let raw_yc = i32::from(be16(self.vram.as_slice(), address + 0x16) as i16);
                let raw_xd = i32::from(be16(self.vram.as_slice(), address + 0x18) as i16);
                let raw_yd = i32::from(be16(self.vram.as_slice(), address + 0x1a) as i16);
                let xa = raw_xa + self.local[0];
                let ya = raw_ya + self.local[1];
                let xb = raw_xb + self.local[0];
                let yb = raw_yb + self.local[1];
                let xc = raw_xc + self.local[0];
                let yc = raw_yc + self.local[1];
                let xd = raw_xd + self.local[0];
                let yd = raw_yd + self.local[1];
                match command {
                    0 => {
                        let width = i32::from((size >> 8) & 0x3f) * 8;
                        let height = i32::from(size & 0xff);
                        if width > 0 && height > 0 {
                            self.draw_textured_rect(
                                texture,
                                [xa, ya, xa + width - 1, ya + height - 1],
                            );
                        }
                    }
                    1 => {
                        let zoom = (ctrl >> 8) & 0x0f;
                        let (end_x, end_y) = if zoom == 0 {
                            (xc, yc)
                        } else {
                            let width = raw_xb.unsigned_abs().max(1) as i32;
                            let height = raw_yb.unsigned_abs().max(1) as i32;
                            (xa + width - 1, ya + height - 1)
                        };
                        self.draw_textured_rect(texture, [xa, ya, end_x, end_y]);
                    }
                    2 => {
                        let points = [(xa, ya), (xb, yb), (xc, yc), (xd, yd)];
                        self.draw_textured_triangle(
                            [points[0], points[1], points[2]],
                            [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)],
                            texture,
                        );
                        self.draw_textured_triangle(
                            [points[0], points[2], points[3]],
                            [(0.0, 0.0), (1.0, 1.0), (0.0, 1.0)],
                            texture,
                        );
                    }
                    4 => {
                        self.fill_triangle([(xa, ya), (xb, yb), (xc, yc)], color, pmod);
                        self.fill_triangle([(xa, ya), (xc, yc), (xd, yd)], color, pmod);
                    }
                    5 => {
                        self.line(xa, ya, xb, yb, color, pmod);
                        self.line(xb, yb, xc, yc, color, pmod);
                        self.line(xc, yc, xd, yd, color, pmod);
                        self.line(xd, yd, xa, ya, color, pmod);
                    }
                    6 => self.line(xa, ya, xb, yb, color, pmod),
                    8 => self.user_clip = [raw_xa, raw_ya, raw_xc, raw_yc],
                    9 => self.system_clip = [0, 0, raw_xc, raw_yc],
                    10 => self.local = [raw_xa, raw_ya],
                    _ => {}
                }
            }

            let next = address + 0x20;
            address = match jump & 3 {
                0 => next,
                1 => {
                    if link == 0 {
                        next
                    } else {
                        link
                    }
                }
                2 => {
                    call_stack.push(next);
                    if link == 0 {
                        next
                    } else {
                        link
                    }
                }
                _ => call_stack.pop().unwrap_or(next),
            };
        }
    }

    fn end_frame(&mut self) {
        self.render_commands();
        self.display = self.draw;
        self.draw ^= 1;
    }

    fn display_buffer(&self) -> &[u8] {
        self.framebuffers[self.display].as_slice()
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.vram.as_slice());
        out.blob(self.framebuffers[0].as_slice());
        out.blob(self.framebuffers[1].as_slice());
        out.blob(&self.regs);
        out.u8(self.draw as u8);
        out.u8(self.display as u8);
        out.u32(self.local[0] as u32);
        out.u32(self.local[1] as u32);
        for value in self.system_clip {
            out.u32(value as u32);
        }
        for value in self.user_clip {
            out.u32(value as u32);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let vram = input.blob()?;
        if vram.len() != self.vram.len() {
            return Err("Saturn VDP1 VRAM state size mismatch".into());
        }
        self.vram.copy_from_slice(vram);
        for framebuffer in &mut self.framebuffers {
            let data = input.blob()?;
            if data.len() != framebuffer.len() {
                return Err("Saturn VDP1 framebuffer state size mismatch".into());
            }
            framebuffer.copy_from_slice(data);
        }
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("Saturn VDP1 register state size mismatch".into());
        }
        self.regs.copy_from_slice(regs);
        self.draw = usize::from(input.u8()? & 1);
        self.display = usize::from(input.u8()? & 1);
        self.local = [input.u32()? as i32, input.u32()? as i32];
        for value in &mut self.system_clip {
            *value = input.u32()? as i32;
        }
        for value in &mut self.user_clip {
            *value = input.u32()? as i32;
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct NormalBgFormat {
    color_mode: u16,
    palette: usize,
    color_ram_offset: usize,
    transparent_code_disabled: bool,
}

#[derive(Clone, Copy)]
struct NormalBgCharacter {
    index: usize,
    palette: usize,
    size: usize,
    flip_x: bool,
    flip_y: bool,
}

#[derive(Clone, Copy)]
struct RotationParameters {
    xst: i64,
    yst: i64,
    zst: i64,
    dxst: i64,
    dyst: i64,
    dx: i64,
    dy: i64,
    a: i64,
    b: i64,
    c: i64,
    d: i64,
    e: i64,
    f: i64,
    px: i64,
    py: i64,
    pz: i64,
    cx: i64,
    cy: i64,
    cz: i64,
    mx: i64,
    my: i64,
    kx: i64,
    ky: i64,
}

struct SaturnVdp2 {
    vram: Box<[u8; VDP2_VRAM_SIZE]>,
    cram: Box<[u8; VDP2_CRAM_SIZE]>,
    regs: Box<[u8; 0x200]>,
    video: VideoBuffer,
    priority: Vec<u8>,
    frame: u64,
}

impl Default for SaturnVdp2 {
    fn default() -> Self {
        Self {
            vram: Box::new([0; VDP2_VRAM_SIZE]),
            cram: Box::new([0; VDP2_CRAM_SIZE]),
            regs: Box::new([0; 0x200]),
            video: VideoBuffer::new(WIDTH as u32, HEIGHT as u32),
            priority: vec![0; WIDTH * HEIGHT],
            frame: 0,
        }
    }
}

impl SaturnVdp2 {
    fn reset(&mut self) {
        self.vram.fill(0);
        self.cram.fill(0);
        self.regs.fill(0);
        self.video.clear([0, 0, 0, 255]);
        self.priority.fill(0);
        self.frame = 0;
    }

    fn color(word: u16) -> [u8; 4] {
        let r = ((word & 0x1f) * 255 / 31) as u8;
        let g = (((word >> 5) & 0x1f) * 255 / 31) as u8;
        let b = (((word >> 10) & 0x1f) * 255 / 31) as u8;
        [r, g, b, 255]
    }

    fn cram_color(&self, dot_color: u16, address_offset: usize) -> [u8; 4] {
        let mode = (be16(self.regs.as_slice(), 0x000e) >> 12) & 3;
        let mut index = (usize::from(dot_color & 0x07ff) + (address_offset << 8)) & 0x07ff;
        if mode != 1 {
            index &= 0x03ff;
        }
        if mode == 2 {
            let offset = (index * 4) & (VDP2_CRAM_SIZE - 1);
            let blue = self.cram[(offset + 1) & (VDP2_CRAM_SIZE - 1)];
            let green = self.cram[(offset + 2) & (VDP2_CRAM_SIZE - 1)];
            let red = self.cram[(offset + 3) & (VDP2_CRAM_SIZE - 1)];
            [red, green, blue, 255]
        } else {
            let offset = (index * 2) & (VDP2_CRAM_SIZE - 1);
            Self::color(be16_wrapped(self.cram.as_slice(), offset))
        }
    }

    fn read8(&self, address: u32) -> u8 {
        match address {
            0x05e0_0000..=0x05ef_ffff => {
                self.vram[(address as usize - 0x05e0_0000) & (VDP2_VRAM_SIZE - 1)]
            }
            0x05f0_0000..=0x05f7_ffff => {
                self.cram[(address as usize - 0x05f0_0000) & (VDP2_CRAM_SIZE - 1)]
            }
            0x05f8_0000..=0x05fb_ffff => self.regs[(address as usize - 0x05f8_0000) & 0x1ff],
            _ => 0,
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        match address {
            0x05e0_0000..=0x05ef_ffff => {
                self.vram[(address as usize - 0x05e0_0000) & (VDP2_VRAM_SIZE - 1)] = value
            }
            0x05f0_0000..=0x05f7_ffff => {
                self.cram[(address as usize - 0x05f0_0000) & (VDP2_CRAM_SIZE - 1)] = value
            }
            0x05f8_0000..=0x05fb_ffff => {
                self.regs[(address as usize - 0x05f8_0000) & 0x1ff] = value
            }
            _ => {}
        }
    }

    fn normal_bg_bitmap_pixel(
        &self,
        base: usize,
        dimensions: [usize; 2],
        position: [usize; 2],
        format: NormalBgFormat,
    ) -> Option<[u8; 4]> {
        let [width, height] = dimensions;
        let [x, y] = position;
        let NormalBgFormat {
            color_mode,
            palette,
            color_ram_offset,
            transparent_code_disabled,
        } = format;
        let x = x % width;
        let y = y % height;
        let pixel = y.saturating_mul(width).saturating_add(x);
        let mask = VDP2_VRAM_SIZE - 1;
        match color_mode {
            0 => {
                let byte = self.vram[(base + pixel / 2) & mask];
                let value = if pixel & 1 == 0 {
                    byte >> 4
                } else {
                    byte & 0x0f
                };
                if value == 0 && !transparent_code_disabled {
                    return None;
                }
                let dot_color = ((palette << 4) | usize::from(value)) as u16;
                Some(self.cram_color(dot_color, color_ram_offset))
            }
            1 => {
                let value = self.vram[(base + pixel) & mask];
                if value == 0 && !transparent_code_disabled {
                    return None;
                }
                let dot_color = ((palette << 8) | usize::from(value)) as u16;
                Some(self.cram_color(dot_color, color_ram_offset))
            }
            2 => {
                let value = be16_wrapped(self.vram.as_slice(), base + pixel * 2) & 0x07ff;
                if value == 0 && !transparent_code_disabled {
                    return None;
                }
                Some(self.cram_color(value, color_ram_offset))
            }
            3 => {
                let value = be16_wrapped(self.vram.as_slice(), base + pixel * 2);
                if value & 0x8000 == 0 && !transparent_code_disabled {
                    None
                } else {
                    Some(Self::color(value & 0x7fff))
                }
            }
            4 => {
                let offset = base + pixel * 4;
                let control = self.vram[offset & mask];
                let blue = self.vram[(offset + 1) & mask];
                let green = self.vram[(offset + 2) & mask];
                let red = self.vram[(offset + 3) & mask];
                if control & 0x80 == 0 && !transparent_code_disabled {
                    None
                } else {
                    Some([red, green, blue, 255])
                }
            }
            _ => None,
        }
    }

    fn render_normal_bg_bitmap(&mut self, index: usize) {
        let bgon = be16(self.regs.as_slice(), 0x0020);
        let chctla = be16(self.regs.as_slice(), 0x0028);
        let (
            enable_mask,
            transparency_mask,
            bitmap_mask,
            size_shift,
            color_shift,
            color_mask,
            palette,
            map_shift,
            scroll_x_offset,
            scroll_y_offset,
            color_ram_shift,
        ) = match index {
            0 => (
                0x0001u16,
                0x0100u16,
                0x0002u16,
                2u32,
                4u32,
                7u16,
                usize::from(be16(self.regs.as_slice(), 0x002c) & 7),
                0u32,
                0x0070usize,
                0x0074usize,
                0u32,
            ),
            1 => (
                0x0002,
                0x0200,
                0x0200,
                10,
                12,
                3,
                usize::from((be16(self.regs.as_slice(), 0x002c) >> 8) & 7),
                4,
                0x0080,
                0x0084,
                4,
            ),
            _ => return,
        };
        if bgon & enable_mask == 0 || chctla & bitmap_mask == 0 {
            return;
        }
        let layer_priority = self.normal_bg_priority(index);

        let (width, height) = match (chctla >> size_shift) & 3 {
            0 => (512usize, 256usize),
            1 => (512, 512),
            2 => (1024, 256),
            _ => (1024, 512),
        };
        let color_mode = (chctla >> color_shift) & color_mask;
        let map_offset = usize::from((be16(self.regs.as_slice(), 0x003c) >> map_shift) & 7);
        let base = (map_offset * 0x20000) & (VDP2_VRAM_SIZE - 1);
        let color_ram_offset =
            usize::from((be16(self.regs.as_slice(), 0x00e4) >> color_ram_shift) & 7);
        let transparent_code_disabled = bgon & transparency_mask != 0;

        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                if !self.normal_bg_window_allows(index, x, y) {
                    continue;
                }
                let (source_x, source_y) =
                    self.normal_bg_source_position(index, x, y, scroll_x_offset, scroll_y_offset);
                let Some(rgba) = self.normal_bg_bitmap_pixel(
                    base,
                    [width, height],
                    [source_x, source_y],
                    NormalBgFormat {
                        color_mode,
                        palette,
                        color_ram_offset,
                        transparent_code_disabled,
                    },
                ) else {
                    continue;
                };
                let rgba = self.apply_color_offset(index, rgba);
                let pixel = y * WIDTH + x;
                let target = pixel * 4;
                self.video.pixels_mut()[target..target + 4].copy_from_slice(&rgba);
                self.priority[pixel] = layer_priority;
            }
        }
    }

    fn normal_bg_plane_base(
        &self,
        map_register: u16,
        map_offset: u16,
        one_word: bool,
        plane_size: u16,
        character_size: usize,
    ) -> usize {
        let combined = ((map_offset & 7) << 6) | (map_register & 0x3f);
        let large_character = character_size == 16;
        let address = match (one_word, plane_size, large_character) {
            (true, 0, false) => usize::from(combined & 0x007f) * 0x2000,
            (true, 1, false) => usize::from((combined >> 1) & 0x003f) * 0x4000,
            (true, 2, false) => usize::from((combined >> 2) & 0x001f) * 0x8000,
            (false, 0, false) => usize::from(combined & 0x003f) * 0x4000,
            (false, 1, false) => usize::from((combined >> 1) & 0x001f) * 0x8000,
            (false, 2, false) => usize::from((combined >> 2) & 0x000f) * 0x10000,
            (true, 0, true) => usize::from(combined & 0x01ff) * 0x0800,
            (true, 1, true) => usize::from((combined >> 1) & 0x00ff) * 0x1000,
            (true, 2, true) => usize::from((combined >> 2) & 0x007f) * 0x2000,
            (false, 0, true) => usize::from(combined & 0x00ff) * 0x1000,
            (false, 1, true) => usize::from((combined >> 1) & 0x007f) * 0x2000,
            (false, 2, true) => usize::from((combined >> 2) & 0x003f) * 0x4000,
            _ => 0,
        };
        address & (VDP2_VRAM_SIZE - 1)
    }

    fn normal_bg_pattern_name(
        &self,
        address: usize,
        one_word: bool,
        color_mode: u16,
        pnc: u16,
        character_size: usize,
    ) -> (usize, usize, bool, bool) {
        if !one_word {
            let control = be16_wrapped(self.vram.as_slice(), address);
            let character = be16_wrapped(self.vram.as_slice(), address + 2) & 0x7fff;
            return (
                usize::from(character),
                usize::from(control & 0x007f),
                control & 0x4000 != 0,
                control & 0x8000 != 0,
            );
        }

        let entry = be16_wrapped(self.vram.as_slice(), address);
        let auxiliary_mode = pnc & 0x4000 != 0;
        let palette = if color_mode == 0 {
            (usize::from((pnc >> 5) & 7) << 4) | usize::from((entry >> 12) & 0x0f)
        } else {
            usize::from((entry >> 12) & 7)
        };
        if character_size == 16 {
            if auxiliary_mode {
                let character = (usize::from((pnc >> 4) & 1) << 14)
                    | (usize::from(entry & 0x0fff) << 2)
                    | usize::from(pnc & 3);
                (character, palette, false, false)
            } else {
                let character = (usize::from((pnc >> 2) & 7) << 12)
                    | (usize::from(entry & 0x03ff) << 2)
                    | usize::from(pnc & 3);
                (character, palette, entry & 0x0400 != 0, entry & 0x0800 != 0)
            }
        } else if auxiliary_mode {
            let character = (usize::from((pnc >> 2) & 7) << 12) | usize::from(entry & 0x0fff);
            (character, palette, false, false)
        } else {
            let character = (usize::from(pnc & 0x001f) << 10) | usize::from(entry & 0x03ff);
            (character, palette, entry & 0x0400 != 0, entry & 0x0800 != 0)
        }
    }

    fn normal_bg_character_pixel(
        &self,
        character: NormalBgCharacter,
        position: [usize; 2],
        format: NormalBgFormat,
    ) -> Option<[u8; 4]> {
        let NormalBgCharacter {
            index: character,
            palette,
            size: character_size,
            flip_x,
            flip_y,
        } = character;
        let [mut x, mut y] = position;
        let NormalBgFormat {
            color_mode,
            palette: _,
            color_ram_offset,
            transparent_code_disabled,
        } = format;
        if !matches!(character_size, 8 | 16) {
            return None;
        }
        if flip_x {
            x = character_size.saturating_sub(1).saturating_sub(x);
        }
        if flip_y {
            y = character_size.saturating_sub(1).saturating_sub(y);
        }

        let cells_per_row = character_size / 8;
        let cell_index = (y / 8) * cells_per_row + (x / 8);
        x &= 7;
        y &= 7;
        let cell_bytes = match color_mode {
            0 => 0x20usize,
            1 => 0x40,
            2 | 3 => 0x80,
            4 => 0x100,
            _ => return None,
        };
        let base = character
            .saturating_mul(0x20)
            .saturating_add(cell_index.saturating_mul(cell_bytes));
        let mask = VDP2_VRAM_SIZE - 1;
        match color_mode {
            0 => {
                let pixel = y * 8 + x;
                let byte = self.vram[(base + pixel / 2) & mask];
                let value = if pixel & 1 == 0 {
                    byte >> 4
                } else {
                    byte & 0x0f
                };
                if value == 0 && !transparent_code_disabled {
                    return None;
                }
                let dot_color = ((palette << 4) | usize::from(value)) as u16;
                Some(self.cram_color(dot_color, color_ram_offset))
            }
            1 => {
                let value = self.vram[(base + y * 8 + x) & mask];
                if value == 0 && !transparent_code_disabled {
                    return None;
                }
                let dot_color = (((palette & 7) << 8) | usize::from(value)) as u16;
                Some(self.cram_color(dot_color, color_ram_offset))
            }
            2 => {
                let value = be16_wrapped(self.vram.as_slice(), base + (y * 8 + x) * 2) & 0x07ff;
                if value == 0 && !transparent_code_disabled {
                    None
                } else {
                    Some(self.cram_color(value, color_ram_offset))
                }
            }
            3 => {
                let value = be16_wrapped(self.vram.as_slice(), base + (y * 8 + x) * 2);
                if value & 0x8000 == 0 && !transparent_code_disabled {
                    None
                } else {
                    Some(Self::color(value & 0x7fff))
                }
            }
            4 => {
                let offset = base + (y * 8 + x) * 4;
                let control = self.vram[offset & mask];
                let blue = self.vram[(offset + 1) & mask];
                let green = self.vram[(offset + 2) & mask];
                let red = self.vram[(offset + 3) & mask];
                if control & 0x80 == 0 && !transparent_code_disabled {
                    None
                } else {
                    Some([red, green, blue, 255])
                }
            }
            _ => None,
        }
    }

    fn render_normal_bg_cell(&mut self, index: usize) {
        let bgon = be16(self.regs.as_slice(), 0x0020);
        let (
            chctl_offset,
            enable_mask,
            transparency_mask,
            bitmap_mask,
            character_size_mask,
            color_shift,
            color_mask,
            pnc_offset,
            plane_size_shift,
            map_offset_shift,
            map_ab_offset,
            map_cd_offset,
            scroll_x_offset,
            scroll_y_offset,
            color_ram_shift,
        ) = match index {
            0 => (
                0x0028usize,
                0x0001u16,
                0x0100u16,
                0x0002u16,
                0x0001u16,
                4u32,
                7u16,
                0x0030usize,
                0u32,
                0u32,
                0x0040usize,
                0x0042usize,
                0x0070usize,
                0x0074usize,
                0u32,
            ),
            1 => (
                0x0028, 0x0002, 0x0200, 0x0200, 0x0100, 12, 3, 0x0032, 2, 4, 0x0044, 0x0046,
                0x0080, 0x0084, 4,
            ),
            2 => (
                0x002a, 0x0004, 0x0400, 0, 0x0001, 1, 1, 0x0034, 4, 8, 0x0048, 0x004a, 0x0090,
                0x0092, 8,
            ),
            3 => (
                0x002a, 0x0008, 0x0800, 0, 0x0010, 5, 1, 0x0036, 6, 12, 0x004c, 0x004e, 0x0094,
                0x0096, 12,
            ),
            _ => return,
        };
        let chctl = be16(self.regs.as_slice(), chctl_offset);
        if bgon & enable_mask == 0 || chctl & bitmap_mask != 0 {
            return;
        }
        let layer_priority = self.normal_bg_priority(index);
        let character_size = if chctl & character_size_mask != 0 {
            16usize
        } else {
            8usize
        };
        let color_mode = (chctl >> color_shift) & color_mask;
        let plane_size = (be16(self.regs.as_slice(), 0x003a) >> plane_size_shift) & 3;
        let (pages_x, pages_y) = match plane_size {
            0 => (1usize, 1usize),
            1 => (2, 1),
            2 => return,
            _ => (2, 2),
        };
        let pnc = be16(self.regs.as_slice(), pnc_offset);
        let one_word = pnc & 0x8000 != 0;
        let entry_size = if one_word { 2usize } else { 4usize };
        let patterns_per_page = 512 / character_size;
        let page_capacity = patterns_per_page * patterns_per_page * entry_size;
        let map_ab = be16(self.regs.as_slice(), map_ab_offset);
        let map_cd = be16(self.regs.as_slice(), map_cd_offset);
        let map_registers = [
            map_ab & 0x003f,
            (map_ab >> 8) & 0x003f,
            map_cd & 0x003f,
            (map_cd >> 8) & 0x003f,
        ];
        let map_offset = (be16(self.regs.as_slice(), 0x003c) >> map_offset_shift) & 7;
        let plane_width = pages_x * 512;
        let plane_height = pages_y * 512;
        let map_width = plane_width * 2;
        let map_height = plane_height * 2;
        let color_ram_offset =
            usize::from((be16(self.regs.as_slice(), 0x00e4) >> color_ram_shift) & 7);
        let transparent_code_disabled = bgon & transparency_mask != 0;

        for screen_y in 0..HEIGHT {
            let (_, source_y) = self.normal_bg_source_position(
                index,
                0,
                screen_y,
                scroll_x_offset,
                scroll_y_offset,
            );
            let world_y = source_y % map_height;
            let plane_row = world_y / plane_height;
            let within_plane_y = world_y % plane_height;
            let page_row = within_plane_y / 512;
            let within_page_y = within_plane_y % 512;
            for screen_x in 0..WIDTH {
                if !self.normal_bg_window_allows(index, screen_x, screen_y) {
                    continue;
                }
                let (source_x, _) = self.normal_bg_source_position(
                    index,
                    screen_x,
                    screen_y,
                    scroll_x_offset,
                    scroll_y_offset,
                );
                let world_x = source_x % map_width;
                let plane_column = world_x / plane_width;
                let plane = plane_row * 2 + plane_column;
                let within_plane_x = world_x % plane_width;
                let page_column = within_plane_x / 512;
                let within_page_x = within_plane_x % 512;
                let page = page_row * pages_x + page_column;
                let table_base = self.normal_bg_plane_base(
                    map_registers[plane],
                    map_offset,
                    one_word,
                    plane_size,
                    character_size,
                );
                let cell_x = within_page_x / character_size;
                let cell_y = within_page_y / character_size;
                let entry = cell_y * patterns_per_page + cell_x;
                let address = table_base
                    .wrapping_add(page * page_capacity)
                    .wrapping_add(entry * entry_size);
                let (character, palette, flip_x, flip_y) =
                    self.normal_bg_pattern_name(address, one_word, color_mode, pnc, character_size);
                let Some(rgba) = self.normal_bg_character_pixel(
                    NormalBgCharacter {
                        index: character,
                        palette,
                        size: character_size,
                        flip_x,
                        flip_y,
                    },
                    [
                        within_page_x % character_size,
                        within_page_y % character_size,
                    ],
                    NormalBgFormat {
                        color_mode,
                        palette,
                        color_ram_offset,
                        transparent_code_disabled,
                    },
                ) else {
                    continue;
                };
                let rgba = self.apply_color_offset(index, rgba);
                let pixel = screen_y * WIDTH + screen_x;
                let target = pixel * 4;
                self.video.pixels_mut()[target..target + 4].copy_from_slice(&rgba);
                self.priority[pixel] = layer_priority;
            }
        }
    }

    fn back_screen_color(&self, line: usize) -> [u8; 4] {
        let upper = be16(self.regs.as_slice(), 0x00ac);
        let lower = be16(self.regs.as_slice(), 0x00ae);
        let word_address = (usize::from(upper & 7) << 16) | usize::from(lower);
        let line_offset = if upper & 0x8000 != 0 { line * 2 } else { 0 };
        let address = (word_address * 2 + line_offset) & (VDP2_VRAM_SIZE - 1);
        self.apply_color_offset(
            5,
            Self::color(be16_wrapped(self.vram.as_slice(), address) & 0x7fff),
        )
    }

    fn render_back_screen(&mut self) {
        let per_line = be16(self.regs.as_slice(), 0x00ac) & 0x8000 != 0;
        if !per_line {
            self.video.clear(self.back_screen_color(0));
            return;
        }
        for y in 0..HEIGHT {
            let rgba = self.back_screen_color(y);
            let start = y * WIDTH * 4;
            let end = start + WIDTH * 4;
            let (pixels, _) = self.video.pixels_mut()[start..end].as_chunks_mut::<4>();
            for pixel in pixels {
                pixel.copy_from_slice(&rgba);
            }
        }
    }

    fn window_contains(&self, window: usize, x: usize, y: usize) -> bool {
        let position_base = if window == 0 {
            0x00c0usize
        } else {
            0x00c8usize
        };
        let start_x = usize::from(be16(self.regs.as_slice(), position_base) & 0x03ff);
        let start_y = usize::from(be16(self.regs.as_slice(), position_base + 2) & 0x03ff);
        let end_x = usize::from(be16(self.regs.as_slice(), position_base + 4) & 0x03ff);
        let end_y = usize::from(be16(self.regs.as_slice(), position_base + 6) & 0x03ff);
        if start_y > end_y || y < start_y || y > end_y || start_x > end_x {
            return false;
        }

        let line_register = if window == 0 {
            0x00d8usize
        } else {
            0x00dcusize
        };
        let upper = be16(self.regs.as_slice(), line_register);
        if upper & 0x8000 == 0 {
            return x >= start_x && x <= end_x;
        }
        let lower = be16(self.regs.as_slice(), line_register + 2);
        let address_value = (usize::from(upper & 7) << 15) | usize::from((lower >> 1) & 0x7fff);
        let table = (address_value * 4) & (VDP2_VRAM_SIZE - 1);
        let entry = (table + y * 4) & (VDP2_VRAM_SIZE - 1);
        let line_start = usize::from(be16_wrapped(self.vram.as_slice(), entry) & 0x03ff);
        let line_end = usize::from(be16_wrapped(self.vram.as_slice(), entry + 2) & 0x03ff);
        line_start <= line_end && x >= line_start && x <= line_end
    }

    fn window_control_allows(&self, control: u8, x: usize, y: usize) -> bool {
        let mut values = [false; 2];
        let mut count = 0usize;
        for window in 0..2 {
            let enable_bit = if window == 0 { 0x02 } else { 0x08 };
            if control & enable_bit == 0 {
                continue;
            }
            let area_bit = if window == 0 { 0x01 } else { 0x04 };
            let inside = self.window_contains(window, x, y);
            values[count] = if control & area_bit != 0 {
                !inside
            } else {
                inside
            };
            count += 1;
        }
        if count == 0 {
            return true;
        }
        if control & 0x80 != 0 {
            values[..count].iter().all(|value| *value)
        } else {
            values[..count].iter().any(|value| *value)
        }
    }

    fn normal_bg_window_allows(&self, index: usize, x: usize, y: usize) -> bool {
        let register = if index < 2 { 0x00d0usize } else { 0x00d2usize };
        let word = be16(self.regs.as_slice(), register);
        let control = if index & 1 == 0 {
            word as u8
        } else {
            (word >> 8) as u8
        };
        self.window_control_allows(control, x, y)
    }

    fn sprite_window_allows(&self, x: usize, y: usize) -> bool {
        let control = (be16(self.regs.as_slice(), 0x00d4) >> 8) as u8;
        self.window_control_allows(control, x, y)
    }

    fn normal_bg_mosaic_sample(&self, index: usize, x: usize, y: usize) -> (usize, usize) {
        let control = be16(self.regs.as_slice(), 0x0022);
        if index > 3 || control & (1 << index) == 0 {
            return (x, y);
        }
        let width = usize::from(((control >> 8) & 0x0f) + 1);
        let height = usize::from(((control >> 12) & 0x0f) + 1);
        (x - x % width, y - y % height)
    }

    fn normal_bg_source_position(
        &self,
        index: usize,
        x: usize,
        y: usize,
        scroll_x_offset: usize,
        scroll_y_offset: usize,
    ) -> (usize, usize) {
        let (sample_x, sample_y) = self.normal_bg_mosaic_sample(index, x, y);
        let fractional_scroll = index <= 1;
        let mut scroll_x = (i64::from(be16(self.regs.as_slice(), scroll_x_offset) & 0x07ff) << 16)
            | if fractional_scroll {
                i64::from((be16(self.regs.as_slice(), scroll_x_offset + 2) >> 8) & 0xff) << 8
            } else {
                0
            };
        let mut scroll_y = (i64::from(be16(self.regs.as_slice(), scroll_y_offset) & 0x07ff) << 16)
            | if fractional_scroll {
                i64::from((be16(self.regs.as_slice(), scroll_y_offset + 2) >> 8) & 0xff) << 8
            } else {
                0
            };
        let (zoom_x_offset, zoom_y_offset) = match index {
            0 => (Some(0x0078usize), Some(0x007cusize)),
            1 => (Some(0x0088usize), Some(0x008cusize)),
            _ => (None, None),
        };
        let zoom = |offset: Option<usize>| -> i64 {
            let Some(offset) = offset else {
                return 0x1_0000;
            };
            let integer = i64::from(be16(self.regs.as_slice(), offset) & 7) << 16;
            let fraction = i64::from((be16(self.regs.as_slice(), offset + 2) >> 8) & 0xff) << 8;
            let raw = integer | fraction;
            if raw == 0 {
                0x1_0000
            } else {
                raw
            }
        };
        let mut zoom_x = zoom(zoom_x_offset);
        let zoom_y = zoom(zoom_y_offset);
        if index <= 1 {
            let control = be16(self.regs.as_slice(), 0x009a);
            let (horizontal, vertical, line_zoom, interval_shift, table_offset) = if index == 0 {
                (
                    control & 0x0002 != 0,
                    control & 0x0004 != 0,
                    control & 0x0008 != 0,
                    (control >> 4) & 3,
                    0x00a0usize,
                )
            } else {
                (
                    control & 0x0200 != 0,
                    control & 0x0400 != 0,
                    control & 0x0800 != 0,
                    (control >> 12) & 3,
                    0x00a4usize,
                )
            };
            let active = usize::from(horizontal) + usize::from(vertical) + usize::from(line_zoom);
            if active != 0 {
                let interval = 1usize << interval_shift;
                let group = sample_y / interval;
                let upper = be16(self.regs.as_slice(), table_offset);
                let lower = be16(self.regs.as_slice(), table_offset + 2);
                let word_address = ((usize::from(upper & 7) << 16) | usize::from(lower)) & 0x03ffff;
                let mut address = (word_address * 2 + group * active * 4) & (VDP2_VRAM_SIZE - 1);
                let sign_extend = |value: u32, bits: u32| -> i64 {
                    let shift = 64 - bits;
                    (i64::from(value) << shift) >> shift
                };
                if horizontal {
                    let value = sign_extend(
                        be32_wrapped(self.vram.as_slice(), address) & 0x07ff_ff00,
                        27,
                    );
                    scroll_x += value;
                    address = (address + 4) & (VDP2_VRAM_SIZE - 1);
                }
                if vertical {
                    let value = sign_extend(
                        be32_wrapped(self.vram.as_slice(), address) & 0x07ff_ff00,
                        27,
                    );
                    scroll_y += value - (group * interval) as i64 * zoom_y;
                    address = (address + 4) & (VDP2_VRAM_SIZE - 1);
                }
                if line_zoom {
                    let value = sign_extend(
                        be32_wrapped(self.vram.as_slice(), address) & 0x0007_ff00,
                        19,
                    );
                    if value != 0 {
                        zoom_x = value;
                    }
                }
            }
        }
        let source_x = (scroll_x + sample_x as i64 * zoom_x) >> 16;
        let source_y = (scroll_y + sample_y as i64 * zoom_y) >> 16;
        (source_x as usize, source_y as usize)
    }

    fn signed_color_offset(value: u16) -> i16 {
        let raw = (value & 0x01ff) as i16;
        if raw & 0x0100 != 0 {
            raw - 0x0200
        } else {
            raw
        }
    }

    fn apply_color_offset(&self, layer_bit: usize, mut rgba: [u8; 4]) -> [u8; 4] {
        let enable = be16(self.regs.as_slice(), 0x0110);
        if layer_bit > 6 || enable & (1 << layer_bit) == 0 {
            return rgba;
        }
        let select = be16(self.regs.as_slice(), 0x0112);
        let base = if select & (1 << layer_bit) != 0 {
            0x011ausize
        } else {
            0x0114usize
        };
        for (component, offset) in rgba[..3].iter_mut().zip([base, base + 2, base + 4]) {
            let delta = i32::from(Self::signed_color_offset(be16(
                self.regs.as_slice(),
                offset,
            )));
            *component = (i32::from(*component) + delta).clamp(0, 255) as u8;
        }
        rgba
    }

    fn sign_extend(value: u32, bits: u32) -> i64 {
        let shift = 64 - bits;
        (i64::from(value) << shift) >> shift
    }

    fn fixed_mul(left: i64, right: i64) -> i64 {
        (left * right) >> 16
    }

    fn rotation_parameters(&self, parameter: usize) -> RotationParameters {
        let upper = usize::from(be16(self.regs.as_slice(), 0x00bc) & 7);
        let lower = usize::from(be16(self.regs.as_slice(), 0x00be));
        let mut base = ((upper << 16) | lower) << 1;
        if parameter == 0 {
            base &= !0x80;
        } else {
            base |= 0x80;
        }
        let word = |index: usize| be32_wrapped(self.vram.as_slice(), base + index * 4);
        let signed =
            |index: usize, mask: u32, bits: u32| Self::sign_extend(word(index) & mask, bits);
        let packed_high = |index: usize| Self::sign_extend(word(index) & 0x3fff_0000, 30);
        let packed_low = |index: usize| Self::sign_extend((word(index) & 0x0000_3fff) << 16, 30);
        RotationParameters {
            xst: signed(0, 0x1fff_ffc0, 29),
            yst: signed(1, 0x1fff_ffc0, 29),
            zst: signed(2, 0x1fff_ffc0, 29),
            dxst: signed(3, 0x0007_ffc0, 19),
            dyst: signed(4, 0x0007_ffc0, 19),
            dx: signed(5, 0x0007_ffc0, 19),
            dy: signed(6, 0x0007_ffc0, 19),
            a: signed(7, 0x000f_ffc0, 20),
            b: signed(8, 0x000f_ffc0, 20),
            c: signed(9, 0x000f_ffc0, 20),
            d: signed(10, 0x000f_ffc0, 20),
            e: signed(11, 0x000f_ffc0, 20),
            f: signed(12, 0x000f_ffc0, 20),
            px: packed_high(13),
            py: packed_low(13),
            pz: packed_high(14),
            cx: packed_high(15),
            cy: packed_low(15),
            cz: packed_high(16),
            mx: signed(17, 0x3fff_ffc0, 30),
            my: signed(18, 0x3fff_ffc0, 30),
            kx: signed(19, 0x00ff_ffff, 24),
            ky: signed(20, 0x00ff_ffff, 24),
        }
    }

    fn rotation_source_position(
        parameters: RotationParameters,
        screen_x: usize,
        screen_y: usize,
    ) -> (i64, i64) {
        let y = (screen_y as i64) << 16;
        let line_x = parameters.xst + Self::fixed_mul(parameters.dxst, y) - parameters.px;
        let line_y = parameters.yst + Self::fixed_mul(parameters.dyst, y) - parameters.py;
        let line_z = parameters.zst - parameters.pz;
        let xsp = Self::fixed_mul(parameters.a, line_x)
            + Self::fixed_mul(parameters.b, line_y)
            + Self::fixed_mul(parameters.c, line_z);
        let ysp = Self::fixed_mul(parameters.d, line_x)
            + Self::fixed_mul(parameters.e, line_y)
            + Self::fixed_mul(parameters.f, line_z);
        let xp = Self::fixed_mul(parameters.a, parameters.px - parameters.cx)
            + Self::fixed_mul(parameters.b, parameters.py - parameters.cy)
            + Self::fixed_mul(parameters.c, parameters.pz - parameters.cz)
            + parameters.cx
            + parameters.mx;
        let yp = Self::fixed_mul(parameters.d, parameters.px - parameters.cx)
            + Self::fixed_mul(parameters.e, parameters.py - parameters.cy)
            + Self::fixed_mul(parameters.f, parameters.pz - parameters.cz)
            + parameters.cy
            + parameters.my;
        let dx = Self::fixed_mul(parameters.a, parameters.dx)
            + Self::fixed_mul(parameters.b, parameters.dy);
        let dy = Self::fixed_mul(parameters.d, parameters.dx)
            + Self::fixed_mul(parameters.e, parameters.dy);
        let xs = Self::fixed_mul(parameters.kx, xsp) + xp;
        let ys = Self::fixed_mul(parameters.ky, ysp) + yp;
        let dxs = Self::fixed_mul(parameters.kx, dx);
        let dys = Self::fixed_mul(parameters.ky, dy);
        (
            (xs + dxs * screen_x as i64) >> 16,
            (ys + dys * screen_x as i64) >> 16,
        )
    }

    fn rbg0_priority(&self) -> u8 {
        (be16(self.regs.as_slice(), 0x00fc) & 7) as u8
    }

    fn rbg0_window_allows(&self, x: usize, y: usize) -> bool {
        self.window_control_allows(be16(self.regs.as_slice(), 0x00d4) as u8, x, y)
    }

    fn rbg0_mosaic_sample(&self, x: usize, y: usize) -> (usize, usize) {
        let control = be16(self.regs.as_slice(), 0x0022);
        if control & 0x0010 == 0 {
            return (x, y);
        }
        let width = usize::from(((control >> 8) & 0x0f) + 1);
        let height = usize::from(((control >> 12) & 0x0f) + 1);
        (x - x % width, y - y % height)
    }

    fn normal_bg_priority(&self, index: usize) -> u8 {
        let (offset, shift) = match index {
            0 => (0x00f8usize, 0u32),
            1 => (0x00f8, 8),
            2 => (0x00fa, 0),
            3 => (0x00fa, 8),
            _ => return 0,
        };
        ((be16(self.regs.as_slice(), offset) >> shift) & 7) as u8
    }

    fn render_normal_bg(&mut self, index: usize) {
        if index <= 1 {
            self.render_normal_bg_bitmap(index);
        }
        self.render_normal_bg_cell(index);
    }

    fn render_rbg0_bitmap_parameter(&mut self, parameter: usize) {
        let bgon = be16(self.regs.as_slice(), 0x0020);
        let chctl = be16(self.regs.as_slice(), 0x002a);
        if bgon & 0x0010 == 0 || chctl & 0x0200 == 0 {
            return;
        }
        let priority = self.rbg0_priority();
        if priority == 0 {
            return;
        }
        let width = 512usize;
        let height = if chctl & 0x0400 != 0 {
            512usize
        } else {
            256usize
        };
        let color_mode = (chctl >> 12) & 7;
        if color_mode > 4 {
            return;
        }
        let map_offset = usize::from((be16(self.regs.as_slice(), 0x003e) >> 4) & 3);
        let base = (map_offset * 0x20000) & (VDP2_VRAM_SIZE - 1);
        let format = NormalBgFormat {
            color_mode,
            palette: usize::from(be16(self.regs.as_slice(), 0x002e) & 7),
            color_ram_offset: usize::from(be16(self.regs.as_slice(), 0x00e6) & 7),
            transparent_code_disabled: bgon & 0x1000 != 0,
        };
        let parameters = self.rotation_parameters(parameter);
        let over_mode = if parameter == 0 {
            (be16(self.regs.as_slice(), 0x003a) >> 10) & 3
        } else {
            (be16(self.regs.as_slice(), 0x003a) >> 14) & 3
        };

        for screen_y in 0..HEIGHT {
            for screen_x in 0..WIDTH {
                if !self.rbg0_window_allows(screen_x, screen_y) {
                    continue;
                }
                let (sample_x, sample_y) = self.rbg0_mosaic_sample(screen_x, screen_y);
                let (source_x, source_y) =
                    Self::rotation_source_position(parameters, sample_x, sample_y);
                let (source_x, source_y) = match over_mode {
                    0 => (
                        source_x.rem_euclid(width as i64) as usize,
                        source_y.rem_euclid(height as i64) as usize,
                    ),
                    3 => {
                        if !(0..512).contains(&source_x) || !(0..512).contains(&source_y) {
                            continue;
                        }
                        (source_x as usize, source_y as usize)
                    }
                    _ => {
                        if source_x < 0
                            || source_y < 0
                            || source_x >= width as i64
                            || source_y >= height as i64
                        {
                            continue;
                        }
                        (source_x as usize, source_y as usize)
                    }
                };
                let Some(rgba) = self.normal_bg_bitmap_pixel(
                    base,
                    [width, height],
                    [source_x, source_y],
                    format,
                ) else {
                    continue;
                };
                let rgba = self.apply_color_offset(4, rgba);
                let pixel = screen_y * WIDTH + screen_x;
                let target = pixel * 4;
                self.video.pixels_mut()[target..target + 4].copy_from_slice(&rgba);
                self.priority[pixel] = priority;
            }
        }
    }

    fn rbg0_map_registers(&self, parameter: usize) -> [u16; 16] {
        let base = if parameter == 0 {
            0x0050usize
        } else {
            0x0060usize
        };
        let mut maps = [0u16; 16];
        for pair in 0..8 {
            let word = be16(self.regs.as_slice(), base + pair * 2);
            maps[pair * 2] = word & 0x003f;
            maps[pair * 2 + 1] = (word >> 8) & 0x003f;
        }
        maps
    }

    fn render_rbg0_cell_parameter(&mut self, parameter: usize) {
        let bgon = be16(self.regs.as_slice(), 0x0020);
        let chctl = be16(self.regs.as_slice(), 0x002a);
        if bgon & 0x0010 == 0 || chctl & 0x0200 != 0 {
            return;
        }
        let priority = self.rbg0_priority();
        if priority == 0 {
            return;
        }

        let character_size = if chctl & 0x0100 != 0 { 16usize } else { 8usize };
        let color_mode = (chctl >> 12) & 7;
        if color_mode > 4 {
            return;
        }
        let pnc = be16(self.regs.as_slice(), 0x0038);
        let one_word = pnc & 0x8000 != 0;
        let entry_size = if one_word { 2usize } else { 4usize };
        let plane_shift = if parameter == 0 { 8 } else { 12 };
        let plane_size = (be16(self.regs.as_slice(), 0x003a) >> plane_shift) & 3;
        let (pages_x, pages_y) = match plane_size {
            0 => (1usize, 1usize),
            1 => (2, 1),
            2 => return,
            _ => (2, 2),
        };
        let map_offset = if parameter == 0 {
            be16(self.regs.as_slice(), 0x003e) & 3
        } else {
            (be16(self.regs.as_slice(), 0x003e) >> 4) & 3
        };
        let maps = self.rbg0_map_registers(parameter);
        let patterns_per_page = 512 / character_size;
        let page_capacity = patterns_per_page * patterns_per_page * entry_size;
        let plane_width = pages_x * 512;
        let plane_height = pages_y * 512;
        let map_width = plane_width * 4;
        let map_height = plane_height * 4;
        let over_mode = if parameter == 0 {
            (be16(self.regs.as_slice(), 0x003a) >> 10) & 3
        } else {
            (be16(self.regs.as_slice(), 0x003a) >> 14) & 3
        };
        let format = NormalBgFormat {
            color_mode,
            palette: 0,
            color_ram_offset: usize::from(be16(self.regs.as_slice(), 0x00e6) & 7),
            transparent_code_disabled: bgon & 0x1000 != 0,
        };
        let parameters = self.rotation_parameters(parameter);

        for screen_y in 0..HEIGHT {
            for screen_x in 0..WIDTH {
                if !self.rbg0_window_allows(screen_x, screen_y) {
                    continue;
                }
                let (sample_x, sample_y) = self.rbg0_mosaic_sample(screen_x, screen_y);
                let (source_x, source_y) =
                    Self::rotation_source_position(parameters, sample_x, sample_y);
                let (source_x, source_y) = match over_mode {
                    0 => (
                        source_x.rem_euclid(map_width as i64) as usize,
                        source_y.rem_euclid(map_height as i64) as usize,
                    ),
                    3 => {
                        if !(0..512).contains(&source_x) || !(0..512).contains(&source_y) {
                            continue;
                        }
                        (source_x as usize, source_y as usize)
                    }
                    _ => {
                        if source_x < 0
                            || source_y < 0
                            || source_x >= map_width as i64
                            || source_y >= map_height as i64
                        {
                            continue;
                        }
                        (source_x as usize, source_y as usize)
                    }
                };

                let plane_column = source_x / plane_width;
                let plane_row = source_y / plane_height;
                let plane = plane_row * 4 + plane_column;
                let within_plane_x = source_x % plane_width;
                let within_plane_y = source_y % plane_height;
                let page_column = within_plane_x / 512;
                let page_row = within_plane_y / 512;
                let page = page_row * pages_x + page_column;
                let within_page_x = within_plane_x % 512;
                let within_page_y = within_plane_y % 512;
                let table_base = self.normal_bg_plane_base(
                    maps[plane],
                    map_offset,
                    one_word,
                    plane_size,
                    character_size,
                );
                let cell_x = within_page_x / character_size;
                let cell_y = within_page_y / character_size;
                let entry = cell_y * patterns_per_page + cell_x;
                let address = table_base
                    .wrapping_add(page * page_capacity)
                    .wrapping_add(entry * entry_size);
                let (character, palette, flip_x, flip_y) =
                    self.normal_bg_pattern_name(address, one_word, color_mode, pnc, character_size);
                let Some(rgba) = self.normal_bg_character_pixel(
                    NormalBgCharacter {
                        index: character,
                        palette,
                        size: character_size,
                        flip_x,
                        flip_y,
                    },
                    [
                        within_page_x % character_size,
                        within_page_y % character_size,
                    ],
                    format,
                ) else {
                    continue;
                };
                let rgba = self.apply_color_offset(4, rgba);
                let pixel = screen_y * WIDTH + screen_x;
                let target = pixel * 4;
                self.video.pixels_mut()[target..target + 4].copy_from_slice(&rgba);
                self.priority[pixel] = priority;
            }
        }
    }

    fn render_rbg0_parameter(&mut self, parameter: usize) {
        if be16(self.regs.as_slice(), 0x002a) & 0x0200 != 0 {
            self.render_rbg0_bitmap_parameter(parameter);
        } else {
            self.render_rbg0_cell_parameter(parameter);
        }
    }

    fn render_rbg0(&mut self) {
        match be16(self.regs.as_slice(), 0x00b0) & 3 {
            0 => self.render_rbg0_parameter(0),
            1 => self.render_rbg0_parameter(1),
            2 | 3 => {
                self.render_rbg0_parameter(1);
                self.render_rbg0_parameter(0);
            }
            _ => {}
        }
    }

    fn sprite_priority_selector(word: u16, sprite_type: u8, rgb: bool) -> u8 {
        if rgb {
            return 0;
        }
        match sprite_type {
            0 => ((word >> 14) & 3) as u8,
            1 => ((word >> 13) & 7) as u8,
            2 => ((word >> 14) & 1) as u8,
            3 | 4 => ((word >> 13) & 3) as u8,
            5..=7 => ((word >> 12) & 7) as u8,
            8 | 9 | 12 | 13 => ((word >> 7) & 1) as u8,
            10 | 14 => ((word >> 6) & 3) as u8,
            _ => 0,
        }
    }

    fn sprite_priority(&self, selector: u8) -> u8 {
        let selector = usize::from(selector.min(7));
        let register = 0x00f0 + (selector / 2) * 2;
        let shift = if selector & 1 == 0 { 0 } else { 8 };
        ((be16(self.regs.as_slice(), register) >> shift) & 7) as u8
    }

    fn compose(&mut self, vdp1: &SaturnVdp1) {
        let display_enabled = be16(self.regs.as_slice(), 0) & 0x8000 != 0;
        if !display_enabled {
            self.video.clear([0, 0, 0, 255]);
            self.priority.fill(0);
            return;
        }
        self.render_back_screen();
        self.priority.fill(0);
        let mut scroll_layers = [
            (self.normal_bg_priority(3), 0usize),
            (self.normal_bg_priority(2), 1usize),
            (self.normal_bg_priority(1), 2usize),
            (self.normal_bg_priority(0), 3usize),
            (self.rbg0_priority(), 4usize),
        ];
        scroll_layers.sort_by_key(|(priority, order)| (*priority, *order));
        for (priority, order) in scroll_layers {
            if priority == 0 {
                continue;
            }
            if order == 4 {
                self.render_rbg0();
            } else {
                self.render_normal_bg(3 - order);
            }
        }
        let sprite_color_ram_offset = usize::from((be16(self.regs.as_slice(), 0x00e6) >> 4) & 7);
        let spctl = be16(self.regs.as_slice(), 0x00e0);
        let sprite_type = (spctl & 0x000f) as u8;
        let mixed_rgb = spctl & 0x0020 != 0;
        let dot_mask = match sprite_type {
            0 | 1 | 2 | 3 | 5 => 0x07ff,
            4 | 6 => 0x03ff,
            7 => 0x01ff,
            8 | 12 => 0x007f,
            _ => 0x003f,
        };
        let framebuffer = vdp1.display_buffer();
        for y in 0..HEIGHT {
            for x in 0..WIDTH {
                if !self.sprite_window_allows(x, y) {
                    continue;
                }
                let source = (y * 512 + x) * 2;
                if source + 1 >= framebuffer.len() {
                    continue;
                }
                let word = be16(framebuffer, source);
                let rgb = mixed_rgb && word & 0x8000 != 0;
                let selector = Self::sprite_priority_selector(word, sprite_type, rgb);
                let sprite_priority = self.sprite_priority(selector);
                let pixel = y * WIDTH + x;
                if sprite_priority == 0 || sprite_priority < self.priority[pixel] {
                    continue;
                }
                let rgba = if rgb {
                    Self::color(word & 0x7fff)
                } else {
                    let index = word & dot_mask;
                    if index == 0 {
                        continue;
                    }
                    self.cram_color(index, sprite_color_ram_offset)
                };
                let rgba = self.apply_color_offset(6, rgba);
                let target = pixel * 4;
                self.video.pixels_mut()[target..target + 4].copy_from_slice(&rgba);
                self.priority[pixel] = sprite_priority;
            }
        }
    }

    fn render(&mut self, vdp1: &SaturnVdp1) {
        self.compose(vdp1);
        self.frame = self.frame.wrapping_add(1);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.vram.as_slice());
        out.blob(self.cram.as_slice());
        out.blob(self.regs.as_slice());
        out.u64(self.frame);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let vram = input.blob()?;
        if vram.len() != self.vram.len() {
            return Err("Saturn VDP2 VRAM state size mismatch".into());
        }
        self.vram.copy_from_slice(vram);
        let cram = input.blob()?;
        if cram.len() != self.cram.len() {
            return Err("Saturn VDP2 CRAM state size mismatch".into());
        }
        self.cram.copy_from_slice(cram);
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("Saturn VDP2 register state size mismatch".into());
        }
        self.regs.copy_from_slice(regs);
        self.frame = input.u64()?;
        Ok(())
    }
}

#[derive(Clone)]
struct SaturnScspDsp {
    coef: [i16; 64],
    madrs: [u16; 32],
    mpro: [u16; 512],
    temp: [i32; 128],
    mems: [i32; 32],
    mixs: [i32; 16],
    exts: [i16; 2],
    efreg: [i16; 16],
    dec: u32,
    rbp: u32,
    rbl: usize,
    stopped: bool,
    last_step: usize,
}

impl Default for SaturnScspDsp {
    fn default() -> Self {
        Self {
            coef: [0; 64],
            madrs: [0; 32],
            mpro: [0; 512],
            temp: [0; 128],
            mems: [0; 32],
            mixs: [0; 16],
            exts: [0; 2],
            efreg: [0; 16],
            dec: 0,
            rbp: 0,
            rbl: 8 * 1024,
            stopped: true,
            last_step: 0,
        }
    }
}

impl SaturnScspDsp {
    fn sign_extend(value: i32, bits: u32) -> i32 {
        let shift = 32 - bits;
        (value << shift) >> shift
    }

    fn pack(value: i32) -> u16 {
        let sign = ((value >> 23) & 1) as u16;
        let mut temp = ((value as u32) ^ (value as u32).wrapping_shl(1)) & 0x00ff_ffff;
        let mut exponent = 0u16;
        while exponent < 12 && temp & 0x0080_0000 == 0 {
            temp <<= 1;
            exponent += 1;
        }
        let shifted = if exponent < 12 {
            value.wrapping_shl(u32::from(exponent)) & 0x003f_ffff
        } else {
            value.wrapping_shl(11)
        };
        (((shifted >> 11) as u16) & 0x07ff) | (exponent << 11) | (sign << 15)
    }

    fn unpack(value: u16) -> i32 {
        let sign = i32::from((value >> 15) & 1);
        let mut exponent = u32::from((value >> 11) & 0x0f);
        let mantissa = i32::from(value & 0x07ff);
        let mut unpacked = mantissa << 11;
        if exponent > 11 {
            exponent = 11;
            unpacked |= sign << 22;
        } else {
            unpacked |= (sign ^ 1) << 22;
        }
        unpacked |= sign << 23;
        Self::sign_extend(unpacked, 24) >> exponent
    }

    fn configure_ring(&mut self, value: u16) {
        self.rbl = (8 * 1024) << usize::from((value >> 7) & 3);
        self.rbp = u32::from(value & 0x003f);
    }

    fn add_sample(&mut self, sample: i32, selector: usize) {
        if selector < self.mixs.len() {
            self.mixs[selector] = self.mixs[selector].wrapping_add(sample);
        }
    }

    fn start(&mut self) {
        self.stopped = false;
        self.last_step = self
            .mpro
            .as_chunks::<4>()
            .0
            .iter()
            .rposition(|instruction| instruction.iter().any(|&word| word != 0))
            .map_or(0, |index| index + 1);
    }

    fn read_word(&self, address: usize) -> u16 {
        match address {
            0x700..=0x77f => self.coef[(address - 0x700) / 2] as u16,
            0x780..=0x7bf => self.madrs[(address - 0x780) / 2],
            0x7c0..=0x7ff => self.madrs[(address - 0x7c0) / 2],
            0x800..=0xbff => self.mpro[(address - 0x800) / 2],
            0xc00..=0xdff => {
                let value = self.temp[(address >> 2) & 0x7f] as u32;
                if address & 2 != 0 {
                    value as u16
                } else {
                    (value >> 16) as u16
                }
            }
            0xe00..=0xe7f => {
                let value = self.mems[(address >> 2) & 0x1f] as u32;
                if address & 2 != 0 {
                    value as u16
                } else {
                    (value >> 16) as u16
                }
            }
            0xe80..=0xebf => {
                let value = self.mixs[(address >> 2) & 0x0f] as u32;
                if address & 2 != 0 {
                    value as u16
                } else {
                    (value >> 16) as u16
                }
            }
            0xec0..=0xedf => self.efreg[(address - 0xec0) / 2] as u16,
            0xee0..=0xee3 => self.exts[(address - 0xee0) / 2] as u16,
            _ => 0,
        }
    }

    fn write_word(&mut self, address: usize, value: u16) {
        match address {
            0x700..=0x77f => self.coef[(address - 0x700) / 2] = value as i16,
            0x780..=0x7bf => self.madrs[(address - 0x780) / 2] = value,
            0x7c0..=0x7ff => self.madrs[(address - 0x7c0) / 2] = value,
            0x800..=0xbff => {
                self.mpro[(address - 0x800) / 2] = value;
                if address == 0xbf0 {
                    self.start();
                }
            }
            _ => {}
        }
    }

    fn read_ram_word(sound_ram: &[u8], address: usize) -> u16 {
        if sound_ram.is_empty() {
            return 0;
        }
        let index = address % sound_ram.len();
        u16::from_be_bytes([sound_ram[index], sound_ram[(index + 1) % sound_ram.len()]])
    }

    fn write_ram_word(sound_ram: &mut [u8], address: usize, value: u16) {
        if sound_ram.is_empty() {
            return;
        }
        let index = address % sound_ram.len();
        let [high, low] = value.to_be_bytes();
        sound_ram[index] = high;
        let next = (index + 1) % sound_ram.len();
        sound_ram[next] = low;
    }

    fn step(&mut self, sound_ram: &mut [u8]) {
        if self.stopped {
            return;
        }
        self.efreg.fill(0);

        let mut acc = 0i32;
        let mut memval = 0i32;
        let mut frc_reg = 0i32;
        let mut y_reg = 0i32;
        let mut adrs_reg = 0u32;

        for step in 0..self.last_step {
            let instruction = &self.mpro[step * 4..step * 4 + 4];
            let tra = usize::from((instruction[0] >> 8) & 0x7f);
            let twt = instruction[0] & 0x0080 != 0;
            let twa = usize::from(instruction[0] & 0x007f);
            let xsel = instruction[1] & 0x8000 != 0;
            let ysel = (instruction[1] >> 13) & 3;
            let ira = usize::from((instruction[1] >> 6) & 0x3f);
            let iwt = instruction[1] & 0x0020 != 0;
            let iwa = usize::from(instruction[1] & 0x001f);
            let table = instruction[2] & 0x8000 != 0;
            let mwt = instruction[2] & 0x4000 != 0;
            let mrd = instruction[2] & 0x2000 != 0;
            let ewt = instruction[2] & 0x1000 != 0;
            let ewa = usize::from((instruction[2] >> 8) & 0x0f);
            let adrl = instruction[2] & 0x0080 != 0;
            let frcl = instruction[2] & 0x0040 != 0;
            let shift = (instruction[2] >> 4) & 3;
            let yrl = instruction[2] & 0x0008 != 0;
            let negb = instruction[2] & 0x0004 != 0;
            let zero = instruction[2] & 0x0002 != 0;
            let bsel = instruction[2] & 0x0001 != 0;
            let nofl = instruction[3] & 0x8000 != 0;
            let coef = usize::from((instruction[3] >> 9) & 0x3f);
            let masa = usize::from((instruction[3] >> 2) & 0x1f);
            let adreb = instruction[3] & 0x0002 != 0;
            let nxadr = instruction[3] & 0x0001 != 0;

            let mut inputs = if ira <= 0x1f {
                self.mems[ira]
            } else if ira <= 0x2f {
                self.mixs[ira - 0x20] << 4
            } else if ira <= 0x31 {
                i32::from(self.exts[ira - 0x30]) << 8
            } else {
                break;
            };
            inputs = Self::sign_extend(inputs, 24);

            if iwt {
                self.mems[iwa] = memval;
                if ira == iwa {
                    inputs = memval;
                }
            }

            let temp = Self::sign_extend(self.temp[(tra + self.dec as usize) & 0x7f], 24);
            let mut b = if zero {
                0
            } else if bsel {
                acc
            } else {
                temp
            };
            if negb {
                b = b.wrapping_neg();
            }
            let x = if xsel { inputs } else { temp };
            let mut y = match ysel {
                0 => frc_reg,
                1 => i32::from(self.coef[coef]) >> 3,
                2 => (y_reg >> 11) & 0x1fff,
                _ => (y_reg >> 4) & 0x0fff,
            };
            if yrl {
                y_reg = inputs;
            }

            let shifted = match shift {
                0 => acc.clamp(-0x0080_0000, 0x007f_ffff),
                1 => acc.wrapping_mul(2).clamp(-0x0080_0000, 0x007f_ffff),
                2 => Self::sign_extend(acc.wrapping_mul(2), 24),
                _ => Self::sign_extend(acc, 24),
            };

            y = Self::sign_extend(y, 13);
            acc = (((i64::from(x) * i64::from(y)) >> 12) + i64::from(b)) as i32;

            if twt {
                self.temp[(twa + self.dec as usize) & 0x7f] = shifted;
            }
            if frcl {
                frc_reg = if shift == 3 {
                    shifted & 0x0fff
                } else {
                    (shifted >> 11) & 0x1fff
                };
            }

            if mrd || mwt {
                let mut address = u32::from(self.madrs[masa]);
                if !table {
                    address = address.wrapping_add(self.dec);
                }
                if adreb {
                    address = address.wrapping_add(adrs_reg & 0x0fff);
                }
                if nxadr {
                    address = address.wrapping_add(1);
                }
                if !table {
                    address &= (self.rbl as u32).saturating_sub(1);
                } else {
                    address &= 0xffff;
                }
                address = address.wrapping_add(self.rbp << 12);
                let byte_address = (address as usize) << 1;

                if mrd && step & 1 != 0 {
                    let value = Self::read_ram_word(sound_ram, byte_address);
                    memval = if nofl {
                        i32::from(value) << 8
                    } else {
                        Self::unpack(value)
                    };
                }
                if mwt && step & 1 != 0 {
                    let value = if nofl {
                        (shifted >> 8) as u16
                    } else {
                        Self::pack(shifted)
                    };
                    Self::write_ram_word(sound_ram, byte_address, value);
                }
            }

            if adrl {
                adrs_reg = if shift == 3 {
                    ((shifted >> 12) & 0x0fff) as u32
                } else {
                    ((inputs >> 16) & 0x0fff) as u32
                };
            }
            if ewt {
                self.efreg[ewa] = self.efreg[ewa].wrapping_add((shifted >> 8) as i16);
            }
        }

        self.dec = self.dec.wrapping_sub(1);
        self.mixs.fill(0);
    }

    fn save(&self, out: &mut StateWriter) {
        for value in self.coef {
            out.u16(value as u16);
        }
        for value in self.madrs {
            out.u16(value);
        }
        for value in self.mpro {
            out.u16(value);
        }
        for value in self.temp {
            out.u32(value as u32);
        }
        for value in self.mems {
            out.u32(value as u32);
        }
        for value in self.mixs {
            out.u32(value as u32);
        }
        for value in self.exts {
            out.u16(value as u16);
        }
        for value in self.efreg {
            out.u16(value as u16);
        }
        out.u32(self.dec);
        out.u32(self.rbp);
        out.u32(self.rbl as u32);
        out.u8(u8::from(self.stopped));
        out.u16(self.last_step as u16);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in &mut self.coef {
            *value = input.u16()? as i16;
        }
        for value in &mut self.madrs {
            *value = input.u16()?;
        }
        for value in &mut self.mpro {
            *value = input.u16()?;
        }
        for value in &mut self.temp {
            *value = input.u32()? as i32;
        }
        for value in &mut self.mems {
            *value = input.u32()? as i32;
        }
        for value in &mut self.mixs {
            *value = input.u32()? as i32;
        }
        for value in &mut self.exts {
            *value = input.u16()? as i16;
        }
        for value in &mut self.efreg {
            *value = input.u16()? as i16;
        }
        self.dec = input.u32()?;
        self.rbp = input.u32()? & 0x3f;
        self.rbl = (input.u32()? as usize).clamp(8 * 1024, 64 * 1024);
        self.stopped = input.u8()? != 0;
        self.last_step = usize::from(input.u16()?).min(128);
        Ok(())
    }
}

struct SaturnScsp {
    regs: Box<[u8; 0x1000]>,
    phase: [u32; 32],
    keyed: [bool; 32],
    backwards: [bool; 32],
    envelope_level: [f32; 32],
    envelope_state: [u8; 32],
    pitch_lfo_phase: [u32; 32],
    amplitude_lfo_phase: [u32; 32],
    ring_buffer: [i16; 64],
    ring_ptr: u8,
    monitor_slot: u8,
    noise_state: u32,
    dma_pending: bool,
    dma_irq_pending: bool,
    dsp: SaturnScspDsp,
    midi_in: [u8; 32],
    midi_in_read: u8,
    midi_in_write: u8,
    midi_in_count: u8,
    midi_out: [u8; 32],
    midi_out_read: u8,
    midi_out_write: u8,
    midi_out_count: u8,
    midi_overflow: bool,
    midi_out_phase: u64,
    timer_phase: [u32; 3],
    sample_phase: u64,
    samples: Vec<(f32, f32)>,
}

impl Default for SaturnScsp {
    fn default() -> Self {
        Self {
            regs: Box::new([0; 0x1000]),
            phase: [0; 32],
            keyed: [false; 32],
            backwards: [false; 32],
            envelope_level: [0.0; 32],
            envelope_state: [3; 32],
            pitch_lfo_phase: [0; 32],
            amplitude_lfo_phase: [0; 32],
            ring_buffer: [0; 64],
            ring_ptr: 0,
            monitor_slot: 0,
            noise_state: 0x5343_5350,
            dma_pending: false,
            dma_irq_pending: false,
            dsp: SaturnScspDsp::default(),
            midi_in: [0; 32],
            midi_in_read: 0,
            midi_in_write: 0,
            midi_in_count: 0,
            midi_out: [0; 32],
            midi_out_read: 0,
            midi_out_write: 0,
            midi_out_count: 0,
            midi_overflow: false,
            midi_out_phase: 0,
            timer_phase: [0; 3],
            sample_phase: 0,
            samples: Vec::with_capacity(1024),
        }
    }
}

impl SaturnScsp {
    fn reset(&mut self) {
        *self = Self::default();
    }
    fn begin_frame(&mut self) {
        self.samples.clear();
    }

    fn midi_status_word(&self) -> u16 {
        let mut value = 0u16;
        if self.midi_out_count == 32 {
            value |= 0x1000;
        }
        if self.midi_out_count == 0 {
            value |= 0x0800;
        }
        if self.midi_overflow {
            value |= 0x0400;
        }
        if self.midi_in_count == 32 {
            value |= 0x0200;
        }
        if self.midi_in_count == 0 {
            value |= 0x0100;
        } else {
            value |= u16::from(self.midi_in[usize::from(self.midi_in_read)]);
        }
        value
    }

    #[cfg(test)]
    fn push_midi_input(&mut self, value: u8) {
        if self.midi_in_count == 32 {
            self.midi_overflow = true;
            return;
        }
        self.midi_in[usize::from(self.midi_in_write)] = value;
        self.midi_in_write = self.midi_in_write.wrapping_add(1) & 31;
        self.midi_in_count += 1;
        let pending = be16(self.regs.as_slice(), 0x420) | 0x0008;
        put_be16(self.regs.as_mut_slice(), 0x420, pending);
    }

    fn pop_midi_input(&mut self) -> u8 {
        if self.midi_in_count == 0 {
            return 0;
        }
        let value = self.midi_in[usize::from(self.midi_in_read)];
        self.midi_in_read = self.midi_in_read.wrapping_add(1) & 31;
        self.midi_in_count -= 1;
        if self.midi_in_count == 0 {
            let pending = be16(self.regs.as_slice(), 0x420) & !0x0008;
            put_be16(self.regs.as_mut_slice(), 0x420, pending);
        }
        value
    }

    fn push_midi_output(&mut self, value: u8) {
        if self.midi_out_count == 32 {
            self.midi_overflow = true;
            return;
        }
        if self.midi_out_count == 0 {
            self.midi_out_phase = 0;
        }
        self.midi_out[usize::from(self.midi_out_write)] = value;
        self.midi_out_write = self.midi_out_write.wrapping_add(1) & 31;
        self.midi_out_count += 1;
    }

    fn tick_midi(&mut self, sh_cycles: u32) {
        if self.midi_out_count == 0 {
            self.midi_out_phase = 0;
            return;
        }
        const MIDI_BYTES_PER_SECOND: u64 = 3_125;
        self.midi_out_phase = self
            .midi_out_phase
            .saturating_add(u64::from(sh_cycles).saturating_mul(MIDI_BYTES_PER_SECOND));
        while self.midi_out_count != 0 && self.midi_out_phase >= SH2_HZ {
            self.midi_out_phase -= SH2_HZ;
            self.midi_out_read = self.midi_out_read.wrapping_add(1) & 31;
            self.midi_out_count -= 1;
        }
        if self.midi_out_count == 0 {
            self.midi_out_phase = 0;
        }
    }

    fn refresh_slot_monitor(&mut self) {
        let slot = usize::from(self.monitor_slot & 31);
        let current_address = ((self.phase[slot] >> 28) & 0x0f) as u16;
        let state = u16::from(self.envelope_state[slot] & 3);
        let envelope = (self.envelope_level[slot].clamp(0.0, 1.0) * 1023.0).round() as u16;
        let envelope = 31u16.saturating_sub((envelope >> 5).min(31));
        put_be16(
            self.regs.as_mut_slice(),
            0x408,
            (current_address << 7) | (state << 5) | envelope,
        );
    }

    fn read8(&mut self, offset: usize) -> u8 {
        let offset = offset & 0xfff;
        if (0x600..0x700).contains(&offset) {
            let word = self.ring_buffer[((offset - 0x600) / 2) & 63].to_be_bytes();
            return word[offset & 1];
        }
        if offset >= 0x700 {
            let word = self.dsp.read_word(offset & !1).to_be_bytes();
            return word[offset & 1];
        }
        if (0x404..=0x405).contains(&offset) {
            let word = self.midi_status_word();
            if offset == 0x405 {
                let value = word as u8;
                self.pop_midi_input();
                return value;
            }
            return (word >> 8) as u8;
        }
        if (0x408..=0x409).contains(&offset) {
            self.refresh_slot_monitor();
        }
        self.regs[offset]
    }

    fn decode_sci_level(&self, source: u8) -> u8 {
        if source >= 8 {
            return 0;
        }
        let mask = 1u16 << source;
        let mut level = 0u8;
        for (bit, offset) in [0x424usize, 0x426, 0x428].into_iter().enumerate() {
            if be16(self.regs.as_slice(), offset) & mask != 0 {
                level |= 1 << bit;
            }
        }
        level
    }

    fn interrupt_source(&self) -> Option<(u8, u8, u8)> {
        let enabled = be16(self.regs.as_slice(), 0x41e);
        let active = enabled & be16(self.regs.as_slice(), 0x420);
        let source = if active & 0x0020 != 0 {
            Some(5)
        } else if self.dma_irq_pending && enabled & 0x0010 != 0 {
            Some(4)
        } else if active & 0x0040 != 0 {
            Some(6)
        } else if active & 0x0180 != 0 {
            Some(7)
        } else if active & 0x0008 != 0 {
            Some(3)
        } else {
            None
        }?;
        let level = self.decode_sci_level(source);
        (level != 0).then_some((source, level, 24 + level))
    }

    #[cfg(test)]
    fn interrupt(&self) -> Option<(u8, u8)> {
        self.interrupt_source()
            .map(|(_, level, vector)| (level, vector))
    }

    fn acknowledge_interrupt(&mut self, source: u8) {
        if source == 4 {
            self.dma_irq_pending = false;
        }
    }

    fn main_interrupt_pending(&self) -> bool {
        be16(self.regs.as_slice(), 0x42a) & be16(self.regs.as_slice(), 0x42c) != 0
    }

    fn handle_global_write(&mut self, offset: usize) {
        let aligned = offset & !1;
        match aligned {
            0x402 => {
                self.dsp.configure_ring(be16(self.regs.as_slice(), 0x402));
            }
            0x416 => {
                if be16(self.regs.as_slice(), 0x416) & 0x1000 != 0 {
                    self.dma_pending = true;
                }
            }
            0x418 | 0x41a | 0x41c => {
                self.timer_phase[(aligned - 0x418) / 2] = 0;
            }
            0x422 => {
                let reset = be16(self.regs.as_slice(), 0x422);
                let mut pending = be16(self.regs.as_slice(), 0x420) & !reset;
                for (timer, bit) in [0x0040u16, 0x0080, 0x0100].into_iter().enumerate() {
                    if reset & bit != 0
                        && be16(self.regs.as_slice(), 0x418 + timer * 2) & 0x00ff == 0x00ff
                    {
                        pending |= bit;
                    }
                }
                put_be16(self.regs.as_mut_slice(), 0x420, pending);
            }
            0x42e => {
                let reset = be16(self.regs.as_slice(), 0x42e);
                let pending = be16(self.regs.as_slice(), 0x42c) & !reset;
                put_be16(self.regs.as_mut_slice(), 0x42c, pending);
            }
            _ => {}
        }
    }

    fn write8(&mut self, offset: usize, value: u8) {
        let offset = offset & 0xfff;
        if (0x600..0x700).contains(&offset) {
            let index = ((offset - 0x600) / 2) & 63;
            let mut word = self.ring_buffer[index].to_be_bytes();
            word[offset & 1] = value;
            self.ring_buffer[index] = i16::from_be_bytes(word);
            return;
        }
        if offset >= 0x700 {
            let aligned = offset & !1;
            let mut word = self.dsp.read_word(aligned).to_be_bytes();
            word[offset & 1] = value;
            self.dsp.write_word(aligned, u16::from_be_bytes(word));
            return;
        }
        if offset == 0x407 {
            self.regs[offset] = value;
            self.push_midi_output(value);
            return;
        }
        if (0x404..=0x405).contains(&offset) {
            return;
        }
        self.regs[offset] = value;
        if (0x408..=0x409).contains(&offset) {
            self.monitor_slot = ((be16(self.regs.as_slice(), 0x408) >> 11) & 31) as u8;
        }
        if offset < 0x400 && offset % 0x20 < 2 {
            let base = (offset / 0x20) * 0x20;
            let control = be16(self.regs.as_slice(), base);
            if control & 0x1000 != 0 {
                for slot in 0..32 {
                    let slot_base = slot * 0x20;
                    let word = be16(self.regs.as_slice(), slot_base);
                    let next = word & 0x0800 != 0;
                    if next && (!self.keyed[slot] || self.envelope_state[slot] == 3) {
                        self.phase[slot] = 0;
                        self.backwards[slot] = false;
                        self.envelope_level[slot] = 383.0 / 1023.0;
                        self.envelope_state[slot] = 0;
                        self.keyed[slot] = true;
                    } else if !next && self.keyed[slot] {
                        self.envelope_state[slot] = 3;
                    }
                }
                let cleared = control & !0x1000;
                put_be16(self.regs.as_mut_slice(), base, cleared);
            }
        } else if (0x400..0x430).contains(&offset) {
            self.handle_global_write(offset);
        }
    }

    fn read_word(&mut self, offset: usize) -> u16 {
        u16::from_be_bytes([self.read8(offset), self.read8(offset.wrapping_add(1))])
    }

    fn write_word(&mut self, offset: usize, value: u16) {
        let [high, low] = value.to_be_bytes();
        self.write8(offset.wrapping_add(1), low);
        self.write8(offset, high);
    }

    fn execute_dma(&mut self, sound_ram: &mut [u8]) {
        if !self.dma_pending {
            return;
        }
        self.dma_pending = false;

        let dmea_low = usize::from(be16(self.regs.as_slice(), 0x412) & 0xfffe);
        let dmea_high = usize::from(be16(self.regs.as_slice(), 0x414) & 0xf000) << 4;
        let mut dmea = dmea_high | dmea_low;
        let mut drga = usize::from(be16(self.regs.as_slice(), 0x414) & 0x0ffe);
        let control = be16(self.regs.as_slice(), 0x416);
        let length = usize::from(control & 0x0ffe);
        let direction_to_ram = control & 0x2000 != 0;
        let gate = control & 0x4000 != 0;
        let saved_parameters = (!direction_to_ram).then(|| {
            let mut saved = [0u8; 6];
            saved.copy_from_slice(&self.regs[0x412..0x418]);
            saved
        });

        if !sound_ram.is_empty() {
            for _ in (0..length).step_by(2) {
                if direction_to_ram {
                    let value = if gate { 0 } else { self.read_word(drga) };
                    let [high, low] = value.to_be_bytes();
                    let index = dmea % sound_ram.len();
                    sound_ram[index] = high;
                    sound_ram[(index + 1) % sound_ram.len()] = low;
                    dmea = dmea.wrapping_add(2) & 0x000f_ffff;
                    if !gate {
                        drga = drga.wrapping_add(2) & 0x0ffe;
                    }
                } else {
                    let value = if gate {
                        0
                    } else {
                        let index = dmea % sound_ram.len();
                        u16::from_be_bytes([
                            sound_ram[index],
                            sound_ram[(index + 1) % sound_ram.len()],
                        ])
                    };
                    self.write_word(drga, value);
                    drga = drga.wrapping_add(2) & 0x0ffe;
                    if !gate {
                        dmea = dmea.wrapping_add(2) & 0x000f_ffff;
                    }
                }
            }
        }

        if let Some(saved) = saved_parameters {
            self.regs[0x412..0x418].copy_from_slice(&saved);
        }
        let control = be16(self.regs.as_slice(), 0x416) & !0x1000;
        put_be16(self.regs.as_mut_slice(), 0x416, control);
        self.dma_pending = false;
        if be16(self.regs.as_slice(), 0x41e) & 0x0010 != 0 {
            self.dma_irq_pending = true;
        }
    }

    fn lfo_phase_step(rate: usize) -> u32 {
        const FREQUENCIES: [f32; 32] = [
            0.17, 0.19, 0.23, 0.27, 0.34, 0.39, 0.45, 0.55, 0.68, 0.78, 0.92, 1.10, 1.39, 1.60,
            1.87, 2.27, 2.87, 3.31, 3.92, 4.79, 6.15, 7.18, 8.60, 10.8, 14.4, 17.2, 21.5, 28.7,
            43.1, 57.4, 86.1, 172.3,
        ];
        (FREQUENCIES[rate.min(31)] * 256.0 * 65_536.0 / AUDIO_RATE as f32).round() as u32
    }

    fn lfo_noise(index: u8) -> u8 {
        let mut value = u32::from(index).wrapping_add(1).wrapping_mul(0x045d_9f3b);
        value ^= value >> 16;
        value = value.wrapping_mul(0x045d_9f3b);
        value ^= value >> 16;
        value as u8
    }

    fn pitch_lfo_wave(index: u8, waveform: u16) -> i16 {
        let index = i16::from(index);
        match waveform & 3 {
            0 => {
                if index < 128 {
                    index
                } else {
                    index - 256
                }
            }
            1 => {
                if index < 128 {
                    127
                } else {
                    -128
                }
            }
            2 => {
                if index < 64 {
                    index * 2
                } else if index < 128 {
                    255 - index * 2
                } else if index < 192 {
                    256 - index * 2
                } else {
                    index * 2 - 511
                }
            }
            _ => 128 - i16::from(Self::lfo_noise(index as u8)),
        }
    }

    fn amplitude_lfo_wave(index: u8, waveform: u16) -> u16 {
        let index = u16::from(index);
        match waveform & 3 {
            0 => 255 - index,
            1 => {
                if index < 128 {
                    255
                } else {
                    0
                }
            }
            2 => {
                if index < 128 {
                    255 - index * 2
                } else {
                    index * 2 - 256
                }
            }
            _ => u16::from(Self::lfo_noise(index as u8)),
        }
    }

    fn advance_lfo_phase(phase: &mut u32, rate: usize) -> u8 {
        *phase = phase.wrapping_add(Self::lfo_phase_step(rate)) & 0x00ff_ffff;
        (*phase >> 16) as u8
    }

    fn pitch_lfo_gain(&mut self, slot: usize, register: u16) -> f32 {
        const DEPTH_CENTS: [f32; 8] = [0.0, 7.0, 13.5, 27.0, 55.0, 112.0, 230.0, 494.0];
        let depth = usize::from((register >> 5) & 7);
        if depth == 0 {
            return 1.0;
        }
        let rate = usize::from((register >> 10) & 0x1f);
        let index = Self::advance_lfo_phase(&mut self.pitch_lfo_phase[slot], rate);
        let wave = f32::from(Self::pitch_lfo_wave(index, (register >> 8) & 3));
        2.0f32.powf(DEPTH_CENTS[depth] * wave / (128.0 * 1200.0))
    }

    fn amplitude_lfo_gain(&mut self, slot: usize, register: u16) -> f32 {
        const DEPTH_DB: [f32; 8] = [0.0, 0.4, 0.8, 1.5, 3.0, 6.0, 12.0, 24.0];
        let depth = usize::from(register & 7);
        if depth == 0 {
            return 1.0;
        }
        let rate = usize::from((register >> 10) & 0x1f);
        let index = Self::advance_lfo_phase(&mut self.amplitude_lfo_phase[slot], rate);
        let wave = f32::from(Self::amplitude_lfo_wave(index, (register >> 3) & 3));
        10.0f32.powf((-DEPTH_DB[depth] * wave / 256.0) / 20.0)
    }

    fn next_noise_sample(&mut self) -> i16 {
        let mut value = self.noise_state;
        if value == 0 {
            value = 0x5343_5350;
        }
        value ^= value << 13;
        value ^= value >> 17;
        value ^= value << 5;
        self.noise_state = value;
        (value >> 16) as u16 as i16
    }

    fn ring_modulation_offset(&self, slot: usize, pcm8: bool) -> i32 {
        let modulation = be16(self.regs.as_slice(), slot * 0x20 + 0x0e);
        let depth = u32::from((modulation >> 12) & 0x0f);
        let source_x = usize::from((modulation >> 6) & 0x3f);
        let source_y = usize::from(modulation & 0x3f);
        if depth == 0 && source_x == 0 && source_y == 0 {
            return 0;
        }
        let pointer = usize::from(self.ring_ptr);
        let average = (i32::from(self.ring_buffer[(pointer + source_x) & 63])
            + i32::from(self.ring_buffer[(pointer + source_y) & 63]))
            / 2;
        let mut offset = (average << 10) >> (26 - depth);
        if !pcm8 {
            offset <<= 1;
        }
        offset
    }

    fn pitch_step(&mut self, slot: usize) -> u32 {
        let word = be16(self.regs.as_slice(), slot * 0x20 + 0x10);
        let fns = u32::from(word & 0x03ff);
        let octave = ((word >> 11) & 0x0f) as i8;
        let signed_octave = if octave & 0x08 != 0 {
            octave - 16
        } else {
            octave
        };
        let mut step = 65_536u32.saturating_add(fns << 6);
        if signed_octave > 0 {
            step = step.checked_shl(signed_octave as u32).unwrap_or(u32::MAX);
        } else if signed_octave < 0 {
            step >>= (-signed_octave) as u32;
        }
        let lfo = be16(self.regs.as_slice(), slot * 0x20 + 0x12);
        let gain = self.pitch_lfo_gain(slot, lfo);
        ((step.max(1) as f32 * gain).round() as u64).clamp(1, u64::from(u32::MAX)) as u32
    }

    fn envelope_rate_index(&self, slot: usize, rate: u16) -> usize {
        let pitch = be16(self.regs.as_slice(), slot * 0x20 + 0x10);
        let octave = ((pitch >> 11) & 0x0f) as i32;
        let octave = if octave & 8 != 0 { octave - 16 } else { octave };
        let krs = i32::from((be16(self.regs.as_slice(), slot * 0x20 + 0x0a) >> 10) & 0x0f);
        let base = if krs == 0x0f {
            0
        } else {
            octave + 2 * krs + i32::from((pitch >> 9) & 1)
        };
        (base + i32::from(rate) * 2).clamp(0, 63) as usize
    }

    fn envelope_step(&self, slot: usize, rate: u16, attack: bool) -> f32 {
        const ATTACK_MS: [f32; 64] = [
            0.0, 0.0, 8100.0, 6900.0, 6000.0, 4800.0, 4000.0, 3400.0, 3000.0, 2400.0, 2000.0,
            1700.0, 1500.0, 1200.0, 1000.0, 860.0, 760.0, 600.0, 500.0, 430.0, 380.0, 300.0, 250.0,
            220.0, 190.0, 150.0, 130.0, 110.0, 95.0, 76.0, 63.0, 55.0, 47.0, 38.0, 31.0, 27.0,
            24.0, 19.0, 15.0, 13.0, 12.0, 9.4, 7.9, 6.8, 6.0, 4.7, 3.8, 3.4, 3.0, 2.4, 2.0, 1.8,
            1.6, 1.3, 1.1, 0.93, 0.85, 0.65, 0.53, 0.44, 0.40, 0.35, 0.0, 0.0,
        ];
        const DECAY_MS: [f32; 64] = [
            0.0, 0.0, 118200.0, 101300.0, 88600.0, 70900.0, 59100.0, 50700.0, 44300.0, 35500.0,
            29600.0, 25300.0, 22200.0, 17700.0, 14800.0, 12700.0, 11100.0, 8900.0, 7400.0, 6300.0,
            5500.0, 4400.0, 3700.0, 3200.0, 2800.0, 2200.0, 1800.0, 1600.0, 1400.0, 1100.0, 920.0,
            790.0, 690.0, 550.0, 460.0, 390.0, 340.0, 270.0, 230.0, 200.0, 170.0, 140.0, 110.0,
            98.0, 85.0, 68.0, 57.0, 49.0, 43.0, 34.0, 28.0, 25.0, 22.0, 18.0, 14.0, 12.0, 11.0,
            8.5, 7.1, 6.1, 5.4, 4.3, 3.6, 3.1,
        ];
        let index = self.envelope_rate_index(slot, rate);
        if index <= 1 {
            return 0.0;
        }
        if attack && index >= 62 {
            return 1.0;
        }
        let milliseconds = if attack {
            ATTACK_MS[index]
        } else {
            DECAY_MS[index]
        };
        if milliseconds <= 0.0 {
            return 1.0;
        }
        (1000.0 / (AUDIO_RATE as f32 * milliseconds)).min(1.0)
    }

    fn update_envelope(&mut self, slot: usize) -> f32 {
        let base = slot * 0x20;
        let attack_decay = be16(self.regs.as_slice(), base + 0x08);
        let release_level = be16(self.regs.as_slice(), base + 0x0a);
        let attack_rate = attack_decay & 0x001f;
        let decay1_rate = (attack_decay >> 6) & 0x001f;
        let decay2_rate = (attack_decay >> 11) & 0x001f;
        let release_rate = release_level & 0x001f;
        let decay_level = (release_level >> 5) & 0x001f;
        let link_to_loop = release_level & 0x4000 != 0;
        let hold_attack = attack_decay & 0x0020 != 0;

        match self.envelope_state[slot] {
            0 => {
                self.envelope_level[slot] = (self.envelope_level[slot]
                    + self.envelope_step(slot, attack_rate, true))
                .min(1.0);
                if self.envelope_level[slot] >= 1.0 && !link_to_loop {
                    self.envelope_state[slot] = 1;
                }
                if hold_attack {
                    return 1.0;
                }
            }
            1 => {
                self.envelope_level[slot] = (self.envelope_level[slot]
                    - self.envelope_step(slot, decay1_rate, false))
                .max(0.0);
                let threshold = f32::from(31 - decay_level) * 32.0 / 1023.0;
                if self.envelope_level[slot] <= threshold {
                    self.envelope_state[slot] = 2;
                }
            }
            2 => {
                if decay2_rate != 0 {
                    self.envelope_level[slot] = (self.envelope_level[slot]
                        - self.envelope_step(slot, decay2_rate, false))
                    .max(0.0);
                }
            }
            3 => {
                self.envelope_level[slot] = (self.envelope_level[slot]
                    - self.envelope_step(slot, release_rate, false))
                .max(0.0);
                if self.envelope_level[slot] <= 0.0 {
                    self.stop_slot(slot);
                    return 0.0;
                }
            }
            _ => {
                self.envelope_state[slot] = 3;
            }
        }
        self.envelope_level[slot]
    }

    fn stop_slot(&mut self, slot: usize) {
        self.keyed[slot] = false;
        self.backwards[slot] = false;
        self.envelope_level[slot] = 0.0;
        self.envelope_state[slot] = 3;
        let base = slot * 0x20;
        let control = be16(self.regs.as_slice(), base) & !0x0800;
        put_be16(self.regs.as_mut_slice(), base, control);
    }

    fn advance_slot_phase(
        &mut self,
        slot: usize,
        loop_start: usize,
        loop_end: usize,
        loop_mode: u16,
    ) {
        let step = i64::from(self.pitch_step(slot));
        let mut phase = i64::from(self.phase[slot]);
        if self.backwards[slot] {
            phase -= step;
        } else {
            phase += step;
        }

        let start = (loop_start as i64) << 16;
        let end = (loop_end as i64) << 16;
        let span = (end - start).max(1);
        let link_to_loop = be16(self.regs.as_slice(), slot * 0x20 + 0x0a) & 0x4000 != 0;
        if !self.backwards[slot] && phase >= start && link_to_loop && self.envelope_state[slot] == 0
        {
            self.envelope_state[slot] = 1;
        }

        match loop_mode {
            0 => {
                if phase >= end {
                    self.phase[slot] = end.clamp(0, i64::from(u32::MAX)) as u32;
                    self.stop_slot(slot);
                    return;
                }
            }
            1 => {
                if phase >= end {
                    phase = start + (phase - end).rem_euclid(span);
                }
            }
            2 => {
                if !self.backwards[slot] && phase >= start {
                    phase = end - (phase - start);
                    self.backwards[slot] = true;
                }
                if self.backwards[slot] && phase < start {
                    phase = end - (start - phase).rem_euclid(span);
                }
            }
            3 => {
                for _ in 0..4 {
                    if !self.backwards[slot] && phase >= end {
                        phase = end - (phase - end);
                        self.backwards[slot] = true;
                    } else if self.backwards[slot] && phase < start {
                        phase = start + (start - phase);
                        self.backwards[slot] = false;
                    } else {
                        break;
                    }
                }
                phase = phase.clamp(start, end);
            }
            _ => {}
        }

        self.phase[slot] = phase.clamp(0, i64::from(u32::MAX)) as u32;
    }

    fn attenuation_gain(value: u8) -> f32 {
        let weights = [0.4f32, 0.8, 1.5, 3.0, 6.0, 12.0, 24.0, 48.0];
        let mut db = 0.0f32;
        for (bit, weight) in weights.into_iter().enumerate() {
            if value & (1 << bit) != 0 {
                db -= weight;
            }
        }
        10.0f32.powf(db / 20.0)
    }

    fn pan_gain(value: u8) -> f32 {
        let low = value & 0x0f;
        if low == 0x0f {
            return 0.0;
        }
        let mut db = 0.0f32;
        for (bit, weight) in [3.0f32, 6.0, 12.0, 24.0].into_iter().enumerate() {
            if low & (1 << bit) != 0 {
                db -= weight;
            }
        }
        10.0f32.powf(db / 20.0)
    }

    fn direct_send_gain(level: u8) -> f32 {
        let db = match level & 7 {
            0 => return 0.0,
            1 => -36.0,
            2 => -30.0,
            3 => -24.0,
            4 => -18.0,
            5 => -12.0,
            6 => -6.0,
            _ => 0.0,
        };
        10.0f32.powf(db / 20.0)
    }

    fn sample_slot(&mut self, slot: usize, sound_ram: &[u8]) -> f32 {
        if !self.keyed[slot] {
            return 0.0;
        }
        let base = slot * 0x20;
        let control = be16(self.regs.as_slice(), base);
        let pcm8 = control & 0x0010 != 0;
        let source_control = (control >> 7) & 3;
        let source_bit_control = (control >> 9) & 3;
        let loop_mode = (control >> 5) & 3;
        let start = ((usize::from(control & 0x000f)) << 16)
            | usize::from(be16(self.regs.as_slice(), base + 2));
        let loop_start = usize::from(be16(self.regs.as_slice(), base + 4));
        let loop_end = usize::from(be16(self.regs.as_slice(), base + 6)).max(loop_start + 1);
        let position = (self.phase[slot] >> 16) as usize;

        if loop_mode == 0 && position >= loop_end {
            self.stop_slot(slot);
            return 0.0;
        }

        let map_position = |position: usize| match loop_mode {
            1 if position >= loop_end => {
                loop_start + (position - loop_end) % (loop_end - loop_start)
            }
            2 | 3 => position.min(loop_end.saturating_sub(1)),
            _ => position.min(loop_end.saturating_sub(1)),
        };
        let modulation = i64::from(self.ring_modulation_offset(slot, pcm8));
        let sample_bits = match source_control {
            0 if !sound_ram.is_empty() => {
                let read_sample = |position: usize| {
                    let position = map_position(position);
                    let byte_offset = if pcm8 {
                        position as i64
                    } else {
                        (position as i64) * 2
                    };
                    let index = (start as i64 + byte_offset + modulation)
                        .rem_euclid(sound_ram.len() as i64)
                        as usize;
                    if pcm8 {
                        i16::from(sound_ram[index] as i8) << 8
                    } else {
                        let next = (index + 1) % sound_ram.len();
                        i16::from_be_bytes([sound_ram[index], sound_ram[next]])
                    }
                };
                let first = f32::from(read_sample(position));
                let second = f32::from(read_sample(position.saturating_add(1)));
                let fraction = (self.phase[slot] & 0xffff) as f32 / 65_536.0;
                (first + (second - first) * fraction)
                    .round()
                    .clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16
            }
            1 => self.next_noise_sample(),
            _ => 0,
        };
        let mut sample_bits = sample_bits as u16;
        if source_bit_control & 1 != 0 {
            sample_bits ^= 0x7fff;
        }
        if source_bit_control & 2 != 0 {
            sample_bits ^= 0x8000;
        }
        let mut sample = f32::from(sample_bits as i16) / 32768.0;

        self.advance_slot_phase(slot, loop_start, loop_end, loop_mode);
        let level = be16(self.regs.as_slice(), base + 0x0c);
        let direct = level & 0x0100 != 0;
        if !direct {
            let lfo = be16(self.regs.as_slice(), base + 0x12);
            sample *= self.amplitude_lfo_gain(slot, lfo);
            sample *= self.update_envelope(slot);
        }

        if level & 0x0200 == 0 {
            let ring_level = if direct {
                sample
            } else {
                sample * Self::attenuation_gain((level & 0x00ff) as u8)
            };
            self.ring_buffer[usize::from(self.ring_ptr)] = (ring_level * 16_384.0)
                .round()
                .clamp(f32::from(i16::MIN), f32::from(i16::MAX))
                as i16;
        }
        sample
    }

    fn set_external_inputs(&mut self, left: i16, right: i16) {
        self.dsp.exts = [left, right];
    }

    fn mix_sample(&mut self, sound_ram: &mut [u8]) -> (f32, f32) {
        let mut left = 0.0f32;
        let mut right = 0.0f32;
        for slot in 0..32 {
            let value = self.sample_slot(slot, sound_ram);
            self.ring_ptr = self.ring_ptr.wrapping_add(1) & 63;
            if value == 0.0 {
                continue;
            }

            let level = be16(self.regs.as_slice(), slot * 0x20 + 0x0c);
            let direct = level & 0x0100 != 0;
            let total_level = if direct {
                1.0
            } else {
                Self::attenuation_gain((level & 0x00ff) as u8)
            };

            let dsp_routing = be16(self.regs.as_slice(), slot * 0x20 + 0x14);
            let dsp_send = Self::direct_send_gain((dsp_routing & 7) as u8);
            if dsp_send != 0.0 {
                let selector = usize::from((dsp_routing >> 3) & 0x0f);
                let input = (value * total_level * dsp_send * 32768.0)
                    .round()
                    .clamp(i32::MIN as f32, i32::MAX as f32) as i32;
                self.dsp.add_sample(input, selector);
            }

            let value = value * total_level;
            let routing = be16(self.regs.as_slice(), slot * 0x20 + 0x16);
            let send = Self::direct_send_gain((routing >> 13) as u8);
            if send == 0.0 {
                continue;
            }
            let pan = ((routing >> 8) & 0x1f) as u8;
            let attenuation = Self::pan_gain(pan);
            if pan < 0x10 {
                left += value * send * attenuation;
                right += value * send;
            } else {
                left += value * send;
                right += value * send * attenuation;
            }
        }

        self.dsp.step(sound_ram);
        for effect in 0..16 {
            let routing = be16(self.regs.as_slice(), effect * 0x20 + 0x16);
            let send = Self::direct_send_gain(((routing >> 5) & 7) as u8);
            if send == 0.0 {
                continue;
            }
            let value = f32::from(self.dsp.efreg[effect]) / 32768.0 * send;
            let pan = (routing & 0x1f) as u8;
            let attenuation = Self::pan_gain(pan);
            if pan < 0x10 {
                left += value * attenuation;
                right += value;
            } else {
                left += value;
                right += value * attenuation;
            }
        }

        for external in 0..2 {
            let routing = be16(self.regs.as_slice(), (external + 16) * 0x20 + 0x16);
            let effect_send = ((routing >> 5) & 7) as u8;
            let (send, pan) = if effect_send != 0 {
                (Self::direct_send_gain(effect_send), (routing & 0x1f) as u8)
            } else {
                (
                    Self::direct_send_gain((routing >> 13) as u8),
                    ((routing >> 8) & 0x1f) as u8,
                )
            };
            if send == 0.0 {
                continue;
            }
            let value = f32::from(self.dsp.exts[external]) / 32768.0 * send;
            let attenuation = Self::pan_gain(pan);
            if pan < 0x10 {
                left += value * attenuation;
                right += value;
            } else {
                left += value;
                right += value * attenuation;
            }
        }

        let master = f32::from((be16(self.regs.as_slice(), 0x400) & 0x000f) as u8) / 15.0;
        (
            (left * master).clamp(-1.0, 1.0),
            (right * master).clamp(-1.0, 1.0),
        )
    }

    fn tick_timers(&mut self) {
        let mut pending = be16(self.regs.as_slice(), 0x420);
        let mut main_pending = be16(self.regs.as_slice(), 0x42c);
        for timer in 0..3 {
            let offset = 0x418 + timer * 2;
            let word = be16(self.regs.as_slice(), offset);
            let mut count = word & 0x00ff;
            if count == 0x00ff {
                continue;
            }
            let divider = 1u32 << ((word >> 8) & 7);
            self.timer_phase[timer] = self.timer_phase[timer].saturating_add(1);
            if self.timer_phase[timer] < divider {
                continue;
            }
            self.timer_phase[timer] -= divider;
            count = count.saturating_add(1).min(0x00ff);
            put_be16(self.regs.as_mut_slice(), offset, (word & 0xff00) | count);
            if count == 0x00ff {
                let bit = [0x0040u16, 0x0080, 0x0100][timer];
                pending |= bit;
                if timer == 0 {
                    main_pending |= 0x0040;
                }
            }
        }
        put_be16(self.regs.as_mut_slice(), 0x420, pending);
        put_be16(self.regs.as_mut_slice(), 0x42c, main_pending);
    }

    fn tick_with_external_source<F>(
        &mut self,
        sh_cycles: u32,
        sound_ram: &mut [u8],
        mut next_external: F,
    ) where
        F: FnMut() -> [i16; 2],
    {
        self.tick_midi(sh_cycles);
        self.sample_phase = self
            .sample_phase
            .saturating_add(u64::from(sh_cycles).saturating_mul(AUDIO_RATE));
        while self.sample_phase >= SH2_HZ {
            self.sample_phase -= SH2_HZ;
            let external = next_external();
            self.set_external_inputs(external[0], external[1]);
            self.tick_timers();
            let sample = self.mix_sample(sound_ram);
            self.samples.push(sample);
        }
    }

    #[cfg(test)]
    fn tick(&mut self, sh_cycles: u32, sound_ram: &mut [u8]) {
        self.tick_with_external_source(sh_cycles, sound_ram, || [0; 2]);
    }

    fn tick_with_external_queue(
        &mut self,
        sh_cycles: u32,
        sound_ram: &mut [u8],
        external: &mut VecDeque<[i16; 2]>,
    ) {
        self.tick_with_external_source(sh_cycles, sound_ram, || {
            external.pop_front().unwrap_or([0; 2])
        });
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.regs.as_slice());
        for phase in self.phase {
            out.u32(phase);
        }
        for keyed in self.keyed {
            out.u8(u8::from(keyed));
        }
        for backwards in self.backwards {
            out.u8(u8::from(backwards));
        }
        for level in self.envelope_level {
            out.u32(level.to_bits());
        }
        for state in self.envelope_state {
            out.u8(state);
        }
        for phase in self.pitch_lfo_phase {
            out.u32(phase);
        }
        for phase in self.amplitude_lfo_phase {
            out.u32(phase);
        }
        for sample in self.ring_buffer {
            out.u16(sample as u16);
        }
        out.u8(self.ring_ptr);
        out.u8(self.monitor_slot);
        out.u32(self.noise_state);
        out.u8(u8::from(self.dma_pending));
        out.u8(u8::from(self.dma_irq_pending));
        self.dsp.save(out);
        for value in self.midi_in {
            out.u8(value);
        }
        out.u8(self.midi_in_read);
        out.u8(self.midi_in_write);
        out.u8(self.midi_in_count);
        for value in self.midi_out {
            out.u8(value);
        }
        out.u8(self.midi_out_read);
        out.u8(self.midi_out_write);
        out.u8(self.midi_out_count);
        out.u8(u8::from(self.midi_overflow));
        out.u64(self.midi_out_phase);
        for phase in self.timer_phase {
            out.u32(phase);
        }
        out.u64(self.sample_phase);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("Saturn SCSP state size mismatch".into());
        }
        self.regs.copy_from_slice(regs);
        for phase in &mut self.phase {
            *phase = input.u32()?;
        }
        for keyed in &mut self.keyed {
            *keyed = input.u8()? != 0;
        }
        for backwards in &mut self.backwards {
            *backwards = input.u8()? != 0;
        }
        for level in &mut self.envelope_level {
            *level = f32::from_bits(input.u32()?).clamp(0.0, 1.0);
        }
        for state in &mut self.envelope_state {
            *state = input.u8()?.min(3);
        }
        for phase in &mut self.pitch_lfo_phase {
            *phase = input.u32()? & 0x00ff_ffff;
        }
        for phase in &mut self.amplitude_lfo_phase {
            *phase = input.u32()? & 0x00ff_ffff;
        }
        for sample in &mut self.ring_buffer {
            *sample = input.u16()? as i16;
        }
        self.ring_ptr = input.u8()? & 63;
        self.monitor_slot = input.u8()? & 31;
        self.noise_state = input.u32()?;
        if self.noise_state == 0 {
            self.noise_state = 0x5343_5350;
        }
        self.dma_pending = input.u8()? != 0;
        self.dma_irq_pending = input.u8()? != 0;
        self.dsp.load(input)?;
        for value in &mut self.midi_in {
            *value = input.u8()?;
        }
        self.midi_in_read = input.u8()? & 31;
        self.midi_in_write = input.u8()? & 31;
        self.midi_in_count = input.u8()?.min(32);
        for value in &mut self.midi_out {
            *value = input.u8()?;
        }
        self.midi_out_read = input.u8()? & 31;
        self.midi_out_write = input.u8()? & 31;
        self.midi_out_count = input.u8()?.min(32);
        self.midi_overflow = input.u8()? != 0;
        self.midi_out_phase = input.u64()? % SH2_HZ;
        for phase in &mut self.timer_phase {
            *phase = input.u32()? & 0x7f;
        }
        self.sample_phase = input.u64()? % SH2_HZ;
        self.samples.clear();
        Ok(())
    }
}

struct SaturnBoard {
    bios: Box<[u8]>,
    work_ram_l: Box<[u8]>,
    work_ram_h: Box<[u8]>,
    backup_ram: Box<[u8]>,
    sound_ram: Box<[u8]>,
    smpc: SaturnSmpc,
    scu: SaturnScu,
    cd: SaturnCdBlock,
    vdp1: SaturnVdp1,
    vdp2: SaturnVdp2,
    scsp: SaturnScsp,
    sh_divu: [Sh2Divu; 2],
    sh_dmac: [Sh2Dmac; 2],
    sh_frt: [Sh2Frt; 2],
    sh_intc: [Sh2Intc; 2],
    sh_sci: [Sh2Sci; 2],
    sh_wdt: [Sh2Wdt; 2],
}

impl SaturnBoard {
    fn new(bios: &[u8], disc: Option<ResourceBlob>) -> Result<Self, String> {
        if bios.len() != BIOS_SIZE {
            return Err(format!("Saturn BIOS must be exactly {BIOS_SIZE} bytes"));
        }
        Ok(Self {
            bios: bios.to_vec().into_boxed_slice(),
            work_ram_l: vec![0; WORK_RAM_SIZE].into_boxed_slice(),
            work_ram_h: vec![0; WORK_RAM_SIZE].into_boxed_slice(),
            backup_ram: vec![0; BACKUP_RAM_SIZE].into_boxed_slice(),
            sound_ram: vec![0; SOUND_RAM_SIZE].into_boxed_slice(),
            smpc: SaturnSmpc::default(),
            scu: SaturnScu::default(),
            cd: SaturnCdBlock::new(disc)?,
            vdp1: SaturnVdp1::default(),
            vdp2: SaturnVdp2::default(),
            scsp: SaturnScsp::default(),
            sh_divu: [Sh2Divu::default(); 2],
            sh_dmac: [Sh2Dmac::default(); 2],
            sh_frt: [Sh2Frt::default(); 2],
            sh_intc: [Sh2Intc::default(); 2],
            sh_sci: [Sh2Sci::default(); 2],
            sh_wdt: [Sh2Wdt::default(); 2],
        })
    }

    fn reset_devices(&mut self) {
        self.work_ram_l.fill(0);
        self.work_ram_h.fill(0);
        self.sound_ram.fill(0);
        self.smpc.reset();
        self.scu.reset();
        self.cd.reset();
        self.vdp1.reset();
        self.vdp2.reset();
        self.scsp.reset();
        self.sh_divu = [Sh2Divu::default(); 2];
        self.sh_dmac = [Sh2Dmac::default(); 2];
        self.sh_frt = [Sh2Frt::default(); 2];
        self.sh_intc = [Sh2Intc::default(); 2];
        self.sh_sci = [Sh2Sci::default(); 2];
        self.sh_wdt = [Sh2Wdt::default(); 2];
    }

    fn normalize(address: u32) -> u32 {
        address & 0x1fff_ffff
    }

    fn read8(&mut self, address: u32) -> u8 {
        let address = Self::normalize(address);
        match address {
            0x0000_0000..=0x0007_ffff => self.bios[address as usize & (BIOS_SIZE - 1)],
            0x0010_0000..=0x0010_007f => self.smpc.read8(address as usize - 0x0010_0000),
            0x0018_0000..=0x0018_ffff => {
                self.backup_ram[(address as usize - 0x0018_0000) & (BACKUP_RAM_SIZE - 1)]
            }
            0x0020_0000..=0x002f_ffff => {
                self.work_ram_l[(address as usize - 0x0020_0000) & (WORK_RAM_SIZE - 1)]
            }
            0x0580_0000..=0x0589_ffff => self.cd.read8(address),
            0x05a0_0000..=0x05a7_ffff => {
                self.sound_ram[(address as usize - 0x05a0_0000) & (SOUND_RAM_SIZE - 1)]
            }
            0x05b0_0000..=0x05b0_0fff => self.scsp.read8(address as usize - 0x05b0_0000),
            0x05c0_0000..=0x05d0_001f => self.vdp1.read8(address),
            0x05e0_0000..=0x05fb_ffff => self.vdp2.read8(address),
            0x05fe_0000..=0x05fe_00cf => self.scu.read8(address as usize - 0x05fe_0000),
            0x0600_0000..=0x060f_ffff => {
                self.work_ram_h[(address as usize - 0x0600_0000) & (WORK_RAM_SIZE - 1)]
            }
            _ => 0xff,
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        let address = Self::normalize(address);
        match address {
            0x0010_0000..=0x0010_007f => self.smpc.write8(address as usize - 0x0010_0000, value),
            0x0018_0000..=0x0018_ffff => {
                self.backup_ram[(address as usize - 0x0018_0000) & (BACKUP_RAM_SIZE - 1)] = value
            }
            0x0020_0000..=0x002f_ffff => {
                self.work_ram_l[(address as usize - 0x0020_0000) & (WORK_RAM_SIZE - 1)] = value
            }
            0x0580_0000..=0x0589_ffff => self.cd.write8(address, value),
            0x05a0_0000..=0x05a7_ffff => {
                self.sound_ram[(address as usize - 0x05a0_0000) & (SOUND_RAM_SIZE - 1)] = value
            }
            0x05b0_0000..=0x05b0_0fff => self.scsp.write8(address as usize - 0x05b0_0000, value),
            0x05c0_0000..=0x05d0_001f => self.vdp1.write8(address, value),
            0x05e0_0000..=0x05fb_ffff => self.vdp2.write8(address, value),
            0x05fe_0000..=0x05fe_00cf => self.scu.write8(address as usize - 0x05fe_0000, value),
            0x0600_0000..=0x060f_ffff => {
                self.work_ram_h[(address as usize - 0x0600_0000) & (WORK_RAM_SIZE - 1)] = value
            }
            _ => {}
        }
    }

    fn read16(&mut self, address: u32) -> u16 {
        let address = Self::normalize(address);
        if (0x0580_0000..=0x0589_ffff).contains(&address) {
            return self.cd.read16(address & !1);
        }
        u16::from_be_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
    }

    fn write16(&mut self, address: u32, value: u16) {
        let address = Self::normalize(address);
        if (0x0580_0000..=0x0589_ffff).contains(&address) {
            self.cd.write16(address & !1, value);
            return;
        }
        let [high, low] = value.to_be_bytes();
        self.write8(address, high);
        self.write8(address.wrapping_add(1), low);
    }

    fn sh_internal_interrupt(&self, side: usize) -> Option<(u8, u8)> {
        let mut selected: Option<(u8, u8)> = None;
        for candidate in [
            self.sh_divu[side].interrupt(&self.sh_intc[side]),
            self.sh_dmac[side].interrupt(&self.sh_intc[side]),
            self.sh_frt[side].interrupt(&self.sh_intc[side]),
            self.sh_sci[side].interrupt(&self.sh_intc[side]),
            self.sh_wdt[side].interrupt(&self.sh_intc[side]),
        ]
        .into_iter()
        .flatten()
        {
            if selected.is_none_or(|current| candidate.0 > current.0) {
                selected = Some(candidate);
            }
        }
        selected
    }

    fn service_scsp_dma(&mut self) {
        self.scsp.execute_dma(self.sound_ram.as_mut());
    }

    fn tick(&mut self, sh_cycles: u32) {
        self.scu.tick_timing(sh_cycles);
        self.cd.tick(sh_cycles);
        self.service_scsp_dma();
        self.scsp.tick_with_external_queue(
            sh_cycles,
            self.sound_ram.as_mut(),
            &mut self.cd.cdda_samples,
        );
        if self.scsp.main_interrupt_pending() {
            if self.scu.irq_status & (1 << 6) == 0 {
                self.scu.dma_event(5);
            }
            self.scu.raise(6);
        }
        if self.cd.irq_pending() {
            self.scu.raise(5);
        }
        if self.smpc.interrupt_pending {
            self.smpc.interrupt_pending = false;
            self.scu.raise(7);
        }
    }

    fn scu_dma_read32(&mut self, address: u32) -> u32 {
        (u32::from(self.read16(address)) << 16) | u32::from(self.read16(address.wrapping_add(2)))
    }

    fn transfer_scu_dma_block(
        &mut self,
        mut source: u32,
        mut destination: u32,
        count: u32,
        source_step: u32,
        destination_step: u32,
    ) -> (u32, u32) {
        let c_bus_destination = (destination & 0x0600_0000) == 0x0600_0000;
        let destination_step = if c_bus_destination {
            2
        } else {
            destination_step
        };
        let mut remaining = count;

        while remaining >= 2 {
            let value = self.read16(source & !1);
            self.write16(destination & !1, value);
            source = source.wrapping_add(source_step);
            destination = destination.wrapping_add(destination_step);
            remaining -= 2;
        }

        if remaining != 0 {
            let value = self.read8(source);
            self.write8(destination, value);
            source = source.wrapping_add(source_step);
            destination = destination.wrapping_add(destination_step);
        }

        (source, destination)
    }

    fn service_dma(&mut self) {
        for channel in 0..3 {
            let Some(request) = self.scu.take_dma(channel) else {
                continue;
            };

            let mut final_source = request.source;
            let mut final_destination = request.destination;
            if request.indirect {
                let mut descriptor = request.destination & 0x07ff_fffc;
                let mut ended = false;
                for _ in 0..4096 {
                    let raw_count = self.scu_dma_read32(descriptor);
                    let destination = self.scu_dma_read32(descriptor.wrapping_add(4)) & 0x07ff_ffff;
                    let raw_source = self.scu_dma_read32(descriptor.wrapping_add(8));
                    let source = raw_source & 0x07ff_ffff;
                    let count_mask = if channel == 0 {
                        0x000f_ffff
                    } else {
                        0x0003_ffff
                    };
                    let count = raw_count & count_mask;
                    let (source_end, destination_end) = self.transfer_scu_dma_block(
                        source,
                        destination,
                        count,
                        request.source_step,
                        request.destination_step,
                    );
                    final_source = source_end;
                    descriptor = descriptor.wrapping_add(12);
                    final_destination = if request.write_update {
                        descriptor
                    } else {
                        destination_end
                    };
                    if raw_source & 0x8000_0000 != 0 {
                        ended = true;
                        break;
                    }
                }
                if !ended {
                    self.scu.raise(12);
                }
            } else {
                (final_source, final_destination) = self.transfer_scu_dma_block(
                    request.source,
                    request.destination,
                    request.count,
                    request.source_step,
                    request.destination_step,
                );
            }

            self.scu.complete_dma(
                channel,
                final_source,
                final_destination,
                request.read_update,
                request.write_update,
            );
        }
    }

    fn end_frame(&mut self) {
        self.vdp1.end_frame();
        self.scu.dma_event(6);
        self.scu.raise(13);
        self.vdp2.render(&self.vdp1);
        self.scu.vblank_in();
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.work_ram_l.as_ref());
        out.blob(self.work_ram_h.as_ref());
        out.blob(self.backup_ram.as_ref());
        out.blob(self.sound_ram.as_ref());
        self.smpc.save(out);
        self.scu.save(out);
        self.cd.save(out);
        self.vdp1.save(out);
        self.vdp2.save(out);
        self.scsp.save(out);
        self.sh_divu[0].save(out);
        self.sh_divu[1].save(out);
        self.sh_dmac[0].save(out);
        self.sh_dmac[1].save(out);
        self.sh_frt[0].save(out);
        self.sh_frt[1].save(out);
        self.sh_intc[0].save(out);
        self.sh_intc[1].save(out);
        self.sh_sci[0].save(out);
        self.sh_sci[1].save(out);
        self.sh_wdt[0].save(out);
        self.sh_wdt[1].save(out);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for (name, target) in [
            ("low work RAM", &mut self.work_ram_l),
            ("high work RAM", &mut self.work_ram_h),
            ("backup RAM", &mut self.backup_ram),
            ("sound RAM", &mut self.sound_ram),
        ] {
            let data = input.blob()?;
            if data.len() != target.len() {
                return Err(format!("Saturn {name} state size mismatch"));
            }
            target.copy_from_slice(data);
        }
        self.smpc.load(input)?;
        self.scu.load(input)?;
        self.cd.load(input)?;
        self.vdp1.load(input)?;
        self.vdp2.load(input)?;
        self.scsp.load(input)?;
        self.sh_divu[0].load(input)?;
        self.sh_divu[1].load(input)?;
        self.sh_dmac[0].load(input)?;
        self.sh_dmac[1].load(input)?;
        self.sh_frt[0].load(input)?;
        self.sh_frt[1].load(input)?;
        self.sh_intc[0].load(input)?;
        self.sh_intc[1].load(input)?;
        self.sh_sci[0].load(input)?;
        self.sh_sci[1].load(input)?;
        self.sh_wdt[0].load(input)?;
        self.sh_wdt[1].load(input)?;
        Ok(())
    }
}

struct SaturnShBus<'a> {
    board: &'a mut SaturnBoard,
    side: usize,
}

impl SaturnShBus<'_> {
    fn peripheral_read32(&self, address: u32) -> Option<u32> {
        self.board.sh_divu[self.side]
            .read32(address)
            .or_else(|| self.board.sh_dmac[self.side].read32(address))
    }

    fn peripheral_write32(&mut self, address: u32, value: u32) -> bool {
        if self.board.sh_divu[self.side].write32(address, value) {
            true
        } else {
            self.board.sh_dmac[self.side].write32(address, value)
        }
    }

    fn service_auto_dma(&mut self) -> u32 {
        for index in 0..2 {
            let Some(size) = self.board.sh_dmac[self.side].auto_transfer_size(index) else {
                continue;
            };
            let channel = self.board.sh_dmac[self.side].channels[index];
            match size {
                1 => {
                    let value = self.read8(channel.sar);
                    self.write8(channel.dar, value);
                }
                2 => {
                    let value = self.read16(channel.sar);
                    self.write16(channel.dar, value);
                }
                4 => {
                    let value = self.read32(channel.sar);
                    self.write32(channel.dar, value);
                }
                16 => {
                    for offset in [0u32, 4, 8, 12] {
                        let value = self.read32(channel.sar.wrapping_add(offset));
                        self.write32(channel.dar.wrapping_add(offset), value);
                    }
                }
                _ => unreachable!(),
            }

            let source_mode = (channel.chcr >> 12) & 3;
            let destination_mode = (channel.chcr >> 14) & 3;
            let dma = &mut self.board.sh_dmac[self.side].channels[index];
            match source_mode {
                1 => dma.sar = dma.sar.wrapping_add(size),
                2 => dma.sar = dma.sar.wrapping_sub(size),
                _ => {}
            }
            match destination_mode {
                1 => dma.dar = dma.dar.wrapping_add(size),
                2 => dma.dar = dma.dar.wrapping_sub(size),
                _ => {}
            }
            let remaining = if dma.tcr == 0 { 0x0100_0000 } else { dma.tcr };
            if remaining == 1 {
                self.board.sh_dmac[self.side].transfer_complete(index);
            } else {
                dma.tcr = (remaining - 1) & 0x00ff_ffff;
            }
            return match size {
                1 | 2 => 2,
                4 => 3,
                16 => 8,
                _ => unreachable!(),
            };
        }
        0
    }
}

impl Sh2Bus for SaturnShBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        if let Some(value) = self.board.sh_sci[self.side].read8(address) {
            return value;
        }
        if let Some(value) = self.board.sh_intc[self.side].read8(address) {
            return value;
        }
        if let Some(value) = self.board.sh_wdt[self.side].read8(address) {
            return value;
        }
        if let Some(value) = self.board.sh_frt[self.side].read8(address) {
            return value;
        }
        if let Some(value) = self.board.sh_dmac[self.side].read_drcr(address) {
            return value;
        }
        if let Some(word) = self.peripheral_read32(address & !3) {
            let shift = (3 - (address & 3)) * 8;
            return (word >> shift) as u8;
        }
        self.board.read8(address)
    }

    fn write8(&mut self, address: u32, value: u8) {
        if self.board.sh_sci[self.side].write8(address, value)
            || self.board.sh_intc[self.side].write8(address, value)
            || self.board.sh_wdt[self.side].write8(address, value)
            || self.board.sh_frt[self.side].write8(address, value)
            || self.board.sh_dmac[self.side].write_drcr(address, value)
        {
            return;
        }
        let aligned = address & !3;
        if let Some(old) = self.peripheral_read32(aligned) {
            let shift = (3 - (address & 3)) * 8;
            let mask = 0xffu32 << shift;
            let merged = (old & !mask) | (u32::from(value) << shift);
            let _ = self.peripheral_write32(aligned, merged);
            return;
        }
        self.board.write8(address, value);
    }

    fn read16(&mut self, address: u32) -> u16 {
        if (0xffff_fe00..=0xffff_fe04).contains(&address) {
            let high = self.board.sh_sci[self.side].read8(address).unwrap_or(0xff);
            let low = self.board.sh_sci[self.side]
                .read8(address.wrapping_add(1))
                .unwrap_or(0xff);
            return u16::from_be_bytes([high, low]);
        }
        if let Some(value) = self.board.sh_intc[self.side].read16(address) {
            return value;
        }
        if let (Some(high), Some(low)) = (
            self.board.sh_wdt[self.side].read8(address),
            self.board.sh_wdt[self.side].read8(address.wrapping_add(1)),
        ) {
            return u16::from_be_bytes([high, low]);
        }
        if let (Some(high), Some(low)) = (
            self.board.sh_frt[self.side].read8(address),
            self.board.sh_frt[self.side].read8(address.wrapping_add(1)),
        ) {
            return u16::from_be_bytes([high, low]);
        }
        if address & 3 != 3 {
            let aligned = address & !3;
            if let Some(word) = self.peripheral_read32(aligned) {
                let shift = (2 - (address & 2)) * 8;
                return (word >> shift) as u16;
            }
        }
        self.board.read16(address)
    }

    fn write16(&mut self, address: u32, value: u16) {
        match SaturnBoard::normalize(address) {
            0x0100_0000 => {
                self.board.sh_frt[1].capture();
                return;
            }
            0x0180_0000 => {
                self.board.sh_frt[0].capture();
                return;
            }
            _ => {}
        }
        if (0xffff_fe00..=0xffff_fe04).contains(&address) {
            let [high, low] = value.to_be_bytes();
            let _ = self.board.sh_sci[self.side].write8(address, high);
            let _ = self.board.sh_sci[self.side].write8(address.wrapping_add(1), low);
            return;
        }
        if self.board.sh_wdt[self.side].write16(address, value) {
            return;
        }
        if self.board.sh_intc[self.side].write16(address, value) {
            return;
        }
        if self.board.sh_frt[self.side].read8(address).is_some()
            && self.board.sh_frt[self.side]
                .read8(address.wrapping_add(1))
                .is_some()
        {
            let [high, low] = value.to_be_bytes();
            let _ = self.board.sh_frt[self.side].write8(address, high);
            let _ = self.board.sh_frt[self.side].write8(address.wrapping_add(1), low);
            return;
        }
        if address & 3 != 3 {
            let aligned = address & !3;
            if let Some(old) = self.peripheral_read32(aligned) {
                let shift = (2 - (address & 2)) * 8;
                let mask = 0xffffu32 << shift;
                let merged = (old & !mask) | (u32::from(value) << shift);
                let _ = self.peripheral_write32(aligned, merged);
                return;
            }
        }
        self.board.write16(address, value);
    }

    fn read32(&mut self, address: u32) -> u32 {
        if let Some(value) = self.peripheral_read32(address) {
            value
        } else {
            (u32::from(self.board.read16(address)) << 16)
                | u32::from(self.board.read16(address.wrapping_add(2)))
        }
    }

    fn write32(&mut self, address: u32, value: u32) {
        if !self.peripheral_write32(address, value) {
            self.board.write16(address, (value >> 16) as u16);
            self.board.write16(address.wrapping_add(2), value as u16);
        }
    }
}

struct SaturnSoundBus<'a> {
    board: &'a mut SaturnBoard,
}

impl Bus68000 for SaturnSoundBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        match address & 0x00ff_ffff {
            0x000000..=0x0fffff => self.board.sound_ram[address as usize & (SOUND_RAM_SIZE - 1)],
            0x100000..=0x100fff => self.board.scsp.read8(address as usize - 0x100000),
            _ => 0xff,
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        match address & 0x00ff_ffff {
            0x000000..=0x0fffff => {
                self.board.sound_ram[address as usize & (SOUND_RAM_SIZE - 1)] = value
            }
            0x100000..=0x100fff => self.board.scsp.write8(address as usize - 0x100000, value),
            _ => {}
        }
    }
}

pub struct SaturnMachine {
    master: Sh2,
    slave: Sh2,
    sound_cpu: M68000,
    board: SaturnBoard,
    audio: AudioBuffer,
    frame_phase: u64,
    slave_credit: i64,
    sound_phase: u64,
    sound_credit: i64,
    slave_running: bool,
    sound_running: bool,
    slave_faulted: bool,
    sound_faulted: bool,
    powered: bool,
}

impl SaturnMachine {
    pub fn from_bios_and_disc(bios: &[u8], disc: Option<ResourceBlob>) -> Result<Self, String> {
        let board = SaturnBoard::new(bios, disc)?;
        let mut machine = Self {
            master: Sh2::default(),
            slave: Sh2::default(),
            sound_cpu: M68000::default(),
            board,
            audio: AudioBuffer::new(AUDIO_RATE as u32, 2),
            frame_phase: 0,
            slave_credit: 0,
            sound_phase: 0,
            sound_credit: 0,
            slave_running: false,
            sound_running: false,
            slave_faulted: false,
            sound_faulted: false,
            powered: true,
        };
        machine.reset_processors();
        machine.board.vdp2.render(&machine.board.vdp1);
        Ok(machine)
    }

    fn reset_vectors(&self) -> (u32, u32) {
        (
            be32(self.board.bios.as_ref(), 0),
            be32(self.board.bios.as_ref(), 4),
        )
    }

    fn reset_processors(&mut self) {
        let (pc, stack) = self.reset_vectors();
        self.master.reset(pc, 0, stack);
        self.slave.reset(pc, 0, stack.saturating_sub(0x1000));
        self.sound_cpu = M68000::default();
        self.frame_phase = 0;
        self.slave_credit = 0;
        self.sound_phase = 0;
        self.sound_credit = 0;
        self.slave_running = false;
        self.sound_running = false;
        self.slave_faulted = false;
        self.sound_faulted = false;
        self.powered = true;
    }

    fn frame_budget(&mut self) -> u64 {
        self.frame_phase = self
            .frame_phase
            .saturating_add(SH2_HZ.saturating_mul(FRAME_RATE_DEN));
        let cycles = self.frame_phase / FRAME_RATE_NUM;
        self.frame_phase %= FRAME_RATE_NUM;
        cycles
    }

    fn sync_control_transitions(&mut self) {
        if self.board.smpc.slave_on && !self.slave_running {
            let (pc, stack) = self.reset_vectors();
            self.slave.reset(pc, 0, stack.saturating_sub(0x1000));
            self.board.sh_divu[1] = Sh2Divu::default();
            self.board.sh_dmac[1] = Sh2Dmac::default();
            self.board.sh_frt[1] = Sh2Frt::default();
            self.board.sh_intc[1] = Sh2Intc::default();
            self.board.sh_sci[1] = Sh2Sci::default();
            self.board.sh_wdt[1] = Sh2Wdt::default();
            self.slave_credit = 0;
            self.slave_faulted = false;
            self.slave_running = true;
        } else if !self.board.smpc.slave_on && self.slave_running {
            self.slave_running = false;
            self.slave_credit = 0;
        }

        if self.board.smpc.sound_on && !self.sound_running {
            let mut bus = SaturnSoundBus {
                board: &mut self.board,
            };
            self.sound_cpu.reset(&mut bus);
            self.sound_credit = 0;
            self.sound_faulted = false;
            self.sound_running = true;
        } else if !self.board.smpc.sound_on && self.sound_running {
            self.sound_running = false;
            self.sound_credit = 0;
        }
    }

    fn run_slave(&mut self) {
        if !self.slave_running || self.slave_faulted {
            self.slave_credit = 0;
            return;
        }
        while self.slave_credit > 0 {
            let dma_used = {
                let mut bus = SaturnShBus {
                    board: &mut self.board,
                    side: 1,
                };
                bus.service_auto_dma()
            };
            if dma_used != 0 {
                self.slave_credit -= i64::from(dma_used);
                continue;
            }
            if let Some((level, vector)) = self.board.sh_internal_interrupt(1) {
                let mut bus = SaturnShBus {
                    board: &mut self.board,
                    side: 1,
                };
                let used = self.slave.interrupt(&mut bus, level, vector);
                if used != 0 {
                    self.slave_credit -= i64::from(used);
                    continue;
                }
            }
            let mut bus = SaturnShBus {
                board: &mut self.board,
                side: 1,
            };
            let used = self.slave.step(&mut bus);
            if used == 0 {
                self.slave_faulted = true;
                self.slave_credit = 0;
                break;
            }
            self.slave_credit -= i64::from(used);
            if self.slave_credit < -32 {
                self.slave_credit = -32;
            }
        }
    }

    fn run_sound_cpu(&mut self) {
        if !self.sound_running || self.sound_faulted {
            self.sound_credit = 0;
            return;
        }
        while self.sound_credit > 0 {
            self.board.service_scsp_dma();
            if let Some((source, level, vector)) = self.board.scsp.interrupt_source() {
                let mut bus = SaturnSoundBus {
                    board: &mut self.board,
                };
                let used = self.sound_cpu.interrupt(&mut bus, level, vector);
                if used != 0 {
                    self.board.scsp.acknowledge_interrupt(source);
                    self.sound_credit -= i64::from(used);
                    continue;
                }
            }
            let mut bus = SaturnSoundBus {
                board: &mut self.board,
            };
            let used = self.sound_cpu.step(&mut bus);
            if used == 0 {
                self.sound_faulted = true;
                self.sound_credit = 0;
                break;
            }
            self.sound_credit -= i64::from(used);
            if self.sound_credit < -64 {
                self.sound_credit = -64;
            }
        }
    }

    fn advance_devices(&mut self, sh_cycles: u32) {
        self.board.sh_frt[0].tick(sh_cycles);
        let _ = self.board.sh_sci[0].tick(sh_cycles);
        self.board.sh_wdt[0].tick(sh_cycles);
        if self.slave_running {
            self.board.sh_frt[1].tick(sh_cycles);
            let _ = self.board.sh_sci[1].tick(sh_cycles);
            self.board.sh_wdt[1].tick(sh_cycles);
        }
        self.board.tick(sh_cycles);
        self.board.service_dma();
        self.sync_control_transitions();
        if self.slave_running {
            self.slave_credit = self.slave_credit.saturating_add(i64::from(sh_cycles));
            self.run_slave();
        }
        self.sound_phase = self
            .sound_phase
            .saturating_add(u64::from(sh_cycles).saturating_mul(SOUND_68K_HZ));
        let sound_cycles = self.sound_phase / SH2_HZ;
        self.sound_phase %= SH2_HZ;
        if self.sound_running {
            self.sound_credit = self.sound_credit.saturating_add(sound_cycles as i64);
            self.run_sound_cpu();
        }
    }

    fn service_master_interrupt(&mut self) -> u32 {
        let mut selected = self.board.scu.interrupt();
        if let Some(internal) = self.board.sh_internal_interrupt(0) {
            if selected.is_none_or(|current| internal.0 > current.0) {
                selected = Some(internal);
            }
        }
        let Some((level, vector)) = selected else {
            return 0;
        };
        let mut bus = SaturnShBus {
            board: &mut self.board,
            side: 0,
        };
        self.master.interrupt(&mut bus, level, vector)
    }

    fn clock_master(&mut self) -> u32 {
        let dma_cycles = {
            let mut bus = SaturnShBus {
                board: &mut self.board,
                side: 0,
            };
            bus.service_auto_dma()
        };
        if dma_cycles != 0 {
            self.advance_devices(dma_cycles);
            return dma_cycles;
        }
        let interrupt_cycles = self.service_master_interrupt();
        if interrupt_cycles != 0 {
            self.advance_devices(interrupt_cycles);
            return interrupt_cycles;
        }
        let used = {
            let mut bus = SaturnShBus {
                board: &mut self.board,
                side: 0,
            };
            self.master.step(&mut bus)
        };
        if used == 0 {
            self.powered = false;
            return 0;
        }
        self.advance_devices(used);
        used
    }

    fn begin_audio_frame(&mut self) {
        self.board.scsp.begin_frame();
        self.audio.begin_frame();
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        for &(left, right) in &self.board.scsp.samples {
            self.audio.push_stereo(left, right);
        }
    }

    fn save_runtime(&self, out: &mut StateWriter) {
        out.u64(self.frame_phase);
        out.u64(self.slave_credit as u64);
        out.u64(self.sound_phase);
        out.u64(self.sound_credit as u64);
        out.u8(u8::from(self.slave_running));
        out.u8(u8::from(self.sound_running));
        out.u8(u8::from(self.slave_faulted));
        out.u8(u8::from(self.sound_faulted));
        out.u8(u8::from(self.powered));
    }

    fn load_runtime(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.frame_phase = input.u64()? % FRAME_RATE_NUM;
        self.slave_credit = input.u64()? as i64;
        self.sound_phase = input.u64()? % SH2_HZ;
        self.sound_credit = input.u64()? as i64;
        self.slave_running = input.u8()? != 0;
        self.sound_running = input.u8()? != 0;
        self.slave_faulted = input.u8()? != 0;
        self.sound_faulted = input.u8()? != 0;
        self.powered = input.u8()? != 0;
        Ok(())
    }
}

impl Machine for SaturnMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Saturn
    }

    fn reset(&mut self) {
        self.board.reset_devices();
        self.reset_processors();
        self.audio.begin_frame();
        self.board.vdp2.render(&self.board.vdp1);
    }

    fn run_frame(&mut self, input: &InputState) {
        if !self.powered {
            return;
        }
        self.board.smpc.set_input(input);
        self.board.scu.vblank_out();
        self.begin_audio_frame();
        let budget = self.frame_budget();
        let target = self.master.cycles.saturating_add(budget);
        let deadline = target.saturating_add(32);
        while self.master.cycles < target && self.master.cycles < deadline {
            if self.clock_master() == 0 {
                break;
            }
        }
        if self.master.cycles < target {
            self.powered = false;
        }
        self.board.end_frame();
        self.flush_audio();
    }

    fn frame_rate(&self) -> f64 {
        FRAME_RATE_NUM as f64 / FRAME_RATE_DEN as f64
    }

    fn video(&self) -> &VideoBuffer {
        &self.board.vdp2.video
    }
    fn audio(&self) -> &AudioBuffer {
        &self.audio
    }

    fn save_state(&self) -> Result<Vec<u8>, String> {
        let mut out = StateWriter::new(PlatformId::Saturn, STATE_VERSION);
        self.master.save(&mut out);
        self.slave.save(&mut out);
        self.sound_cpu.save(&mut out);
        self.board.save(&mut out);
        self.save_runtime(&mut out);
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Saturn, STATE_VERSION)?;
        self.master.load(&mut input)?;
        self.slave.load(&mut input)?;
        self.sound_cpu.load(&mut input)?;
        self.board.load(&mut input)?;
        self.load_runtime(&mut input)?;
        input.finish()?;
        self.board.vdp2.compose(&self.board.vdp1);
        self.audio.begin_frame();
        Ok(())
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            self.board.backup_ram.len()
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("Saturn persistent resource is Storage slot 0".into());
        }
        if out.len() != self.board.backup_ram.len() {
            return Err("Saturn persistent output length mismatch".into());
        }
        out.copy_from_slice(self.board.backup_ram.as_ref());
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("Saturn persistent resource is Storage slot 0".into());
        }
        if data.len() != self.board.backup_ram.len() {
            return Err("Saturn persistent input length mismatch".into());
        }
        self.board.backup_ram.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write16(bytes: &mut [u8], offset: usize, value: u16) {
        bytes[offset..offset + 2].copy_from_slice(&value.to_be_bytes());
    }

    fn write32(bytes: &mut [u8], offset: usize, value: u32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
    }

    fn write_scsp_word(scsp: &mut SaturnScsp, offset: usize, value: u16) {
        let [high, low] = value.to_be_bytes();
        scsp.write8(offset + 1, low);
        scsp.write8(offset, high);
    }

    fn write_identity_rotation_parameter(vdp2: &mut SaturnVdp2, parameter: usize) {
        let base = 0x60000usize + parameter * 0x80;
        write16(vdp2.regs.as_mut_slice(), 0x00bc, 0x0003);
        write16(vdp2.regs.as_mut_slice(), 0x00be, 0x0000);
        write32(vdp2.vram.as_mut_slice(), base + 4 * 4, 0x0001_0000);
        write32(vdp2.vram.as_mut_slice(), base + 5 * 4, 0x0001_0000);
        write32(vdp2.vram.as_mut_slice(), base + 7 * 4, 0x0001_0000);
        write32(vdp2.vram.as_mut_slice(), base + 11 * 4, 0x0001_0000);
        write32(vdp2.vram.as_mut_slice(), base + 19 * 4, 0x0001_0000);
        write32(vdp2.vram.as_mut_slice(), base + 20 * 4, 0x0001_0000);
    }

    fn synthetic_bios() -> Vec<u8> {
        let mut bios = vec![0u8; BIOS_SIZE];
        write32(&mut bios, 0, 0x0000_0100);
        write32(&mut bios, 4, 0x060f_fff0);
        write16(&mut bios, 0x100, 0x0009);
        write16(&mut bios, 0x102, 0xaffe);
        write16(&mut bios, 0x104, 0x0009);
        bios
    }

    fn cue_container(cue: &str, files: &[(&str, Vec<u8>)]) -> ResourceBlob {
        let cue_bytes = cue.as_bytes();
        let directory_len: usize = files.iter().map(|(name, _)| 2 + name.len() + 16).sum();
        let data_start = 16 + cue_bytes.len() + directory_len;
        let total_len = data_start + files.iter().map(|(_, data)| data.len()).sum::<usize>();
        let mut bytes = vec![0u8; total_len];
        bytes[..8].copy_from_slice(&crate::cd_image::CUE_CONTAINER_MAGIC);
        bytes[8..12].copy_from_slice(&(cue_bytes.len() as u32).to_le_bytes());
        bytes[12..16].copy_from_slice(&(files.len() as u32).to_le_bytes());
        bytes[16..16 + cue_bytes.len()].copy_from_slice(cue_bytes);
        let mut directory_cursor = 16 + cue_bytes.len();
        let mut data_cursor = data_start;
        for (name, data) in files {
            let name_bytes = name.as_bytes();
            bytes[directory_cursor..directory_cursor + 2]
                .copy_from_slice(&(name_bytes.len() as u16).to_le_bytes());
            directory_cursor += 2;
            bytes[directory_cursor..directory_cursor + name_bytes.len()]
                .copy_from_slice(name_bytes);
            directory_cursor += name_bytes.len();
            bytes[directory_cursor..directory_cursor + 8]
                .copy_from_slice(&(data.len() as u64).to_le_bytes());
            bytes[directory_cursor + 8..directory_cursor + 16]
                .copy_from_slice(&(data_cursor as u64).to_le_bytes());
            directory_cursor += 16;
            bytes[data_cursor..data_cursor + data.len()].copy_from_slice(data);
            data_cursor += data.len();
        }
        ResourceBlob::from_bytes(&bytes)
    }

    #[test]
    fn memory_map_exposes_bios_workram_and_backup_ram() {
        let bios = synthetic_bios();
        let mut board = SaturnBoard::new(&bios, None).unwrap();
        assert_eq!(board.read8(0), 0);
        assert_eq!(board.read8(3), 0x00);
        board.write8(0x0020_1234, 0x5a);
        board.write8(0x0600_5678, 0xa5);
        board.write8(0x0018_0010, 0x3c);
        assert_eq!(board.read8(0x0020_1234), 0x5a);
        assert_eq!(board.read8(0x0600_5678), 0xa5);
        assert_eq!(board.read8(0x0018_0010), 0x3c);
    }

    #[test]
    fn dual_sh2_frt_input_capture_links_master_and_slave() {
        let bios = synthetic_bios();
        let mut board = SaturnBoard::new(&bios, None).unwrap();
        board.sh_frt[1].write8(0xffff_fe12, 0x12);
        board.sh_frt[1].write8(0xffff_fe13, 0x34);
        {
            let mut master_bus = SaturnShBus {
                board: &mut board,
                side: 0,
            };
            master_bus.write16(0x2100_0000, 0);
        }
        assert_eq!(board.sh_frt[1].read8(0xffff_fe18), Some(0x12));
        assert_eq!(board.sh_frt[1].read8(0xffff_fe19), Some(0x34));
        assert_eq!(board.sh_frt[1].read8(0xffff_fe11).unwrap() & 0x80, 0x80);

        board.sh_frt[0].write8(0xffff_fe12, 0xab);
        board.sh_frt[0].write8(0xffff_fe13, 0xcd);
        {
            let mut slave_bus = SaturnShBus {
                board: &mut board,
                side: 1,
            };
            slave_bus.write16(0x2180_0000, 0);
        }
        assert_eq!(board.sh_frt[0].read8(0xffff_fe18), Some(0xab));
        assert_eq!(board.sh_frt[0].read8(0xffff_fe19), Some(0xcd));
        assert_eq!(board.sh_frt[0].read8(0xffff_fe11).unwrap() & 0x80, 0x80);
    }

    #[test]
    fn sh2_dmac_auto_request_moves_work_ram_and_raises_completion_interrupt() {
        let bios = synthetic_bios();
        let mut board = SaturnBoard::new(&bios, None).unwrap();
        board.work_ram_l[..8].copy_from_slice(&[0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88]);
        {
            let mut bus = SaturnShBus {
                board: &mut board,
                side: 0,
            };
            bus.write32(0xffff_ff80, 0x0020_0000);
            bus.write32(0xffff_ff84, 0x0600_0000);
            bus.write32(0xffff_ff88, 4);
            bus.write32(0xffff_ff8c, 0x0000_56e5);
            bus.write32(0xffff_ffa0, 0x62);
            bus.write16(0xffff_fee2, 0x0600);
            assert_eq!(bus.service_auto_dma(), 0);
            bus.write32(0xffff_ffb0, 1);
            for _ in 0..4 {
                assert_eq!(bus.service_auto_dma(), 2);
            }
            assert_eq!(bus.service_auto_dma(), 0);
            assert_eq!(bus.read32(0xffff_ff80), 0x0020_0008);
            assert_eq!(bus.read32(0xffff_ff84), 0x0600_0008);
            assert_eq!(bus.read32(0xffff_ff88), 0);
            assert_eq!(bus.read32(0xffff_ff8c) & 7, 7);
        }
        assert_eq!(&board.work_ram_h[..8], &board.work_ram_l[..8]);
        assert_eq!(board.sh_internal_interrupt(0), Some((6, 0x62)));
        assert_eq!(board.sh_internal_interrupt(1), None);
    }

    #[test]
    fn dual_sh2_divu_executes_and_routes_overflow_interrupt() {
        let bios = synthetic_bios();
        let mut machine = SaturnMachine::from_bios_and_disc(&bios, None).unwrap();
        {
            let mut bus = SaturnShBus {
                board: &mut machine.board,
                side: 0,
            };
            bus.write32(0xffff_ff00, 7);
            bus.write32(0xffff_ff04, 100);
            assert_eq!(bus.read32(0xffff_ff04), 14);
            assert_eq!(bus.read32(0xffff_ff10), 2);
            bus.write16(0xffff_fee2, 0x0006);
            bus.write32(0xffff_ff0c, 0x65);
            bus.write32(0xffff_ff08, 0x0000_0002);
            bus.write32(0xffff_ff00, 0);
            bus.write32(0xffff_ff04, 1);
        }
        assert_eq!(machine.board.sh_internal_interrupt(0), Some((6, 0x65)));
        assert_eq!(machine.board.sh_internal_interrupt(1), None);
        assert_eq!(machine.service_master_interrupt(), 5);
    }

    #[test]
    fn sh2_sci_transmit_timing_and_interrupts_are_exposed_on_saturn() {
        let bios = synthetic_bios();
        let mut machine = SaturnMachine::from_bios_and_disc(&bios, None).unwrap();
        {
            let mut bus = SaturnShBus {
                board: &mut machine.board,
                side: 0,
            };
            bus.write16(0xffff_fe60, 0x9000);
            bus.write16(0xffff_fe64, 0x6263);
            bus.write8(0xffff_fe01, 0x00);
            bus.write8(0xffff_fe02, 0x24);
            assert_eq!(bus.read8(0xffff_fe04) & 0x84, 0x84);
            bus.write8(0xffff_fe03, 0xa5);
            bus.write8(0xffff_fe04, 0x04);
            assert_eq!(bus.read8(0xffff_fe04) & 0x04, 0);
        }

        machine.advance_devices(320);
        {
            let mut bus = SaturnShBus {
                board: &mut machine.board,
                side: 0,
            };
            assert_eq!(bus.read8(0xffff_fe04) & 0x84, 0x84);
        }
        assert_eq!(machine.board.sh_internal_interrupt(0), Some((9, 0x63)));
    }

    #[test]
    fn sh2_wdt_interval_timer_ticks_and_interrupts_through_saturn_machine() {
        let bios = synthetic_bios();
        let mut machine = SaturnMachine::from_bios_and_disc(&bios, None).unwrap();
        {
            let mut bus = SaturnShBus {
                board: &mut machine.board,
                side: 0,
            };
            bus.write16(0xffff_fee2, 0x0050);
            bus.write16(0xffff_fee4, 0x6300);
            bus.write16(0xffff_fe80, 0x5aff);
            bus.write16(0xffff_fe80, 0xa53e);
        }

        machine.advance_devices(4096);
        assert_eq!(
            machine.board.sh_wdt[0].interrupt(&machine.board.sh_intc[0]),
            Some((5, 0x63))
        );
        assert_eq!(machine.service_master_interrupt(), 5);
    }

    #[test]
    fn smpc_intback_reports_active_low_controller_and_releases_slave() {
        let mut smpc = SaturnSmpc::default();
        let mut input = InputState::default();
        input.buttons[0] = UP | START | FACE_SOUTH | R1;
        smpc.set_input(&input);
        smpc.write8(0x1f, 0x10);
        let pad = u16::from_be_bytes([smpc.read8(0x23), smpc.read8(0x25)]);
        assert_eq!(pad & (1 << 0), 0);
        assert_eq!(pad & (1 << 4), 0);
        assert_eq!(pad & (1 << 5), 0);
        assert_eq!(pad & (1 << 10), 0);
        assert!(smpc.interrupt_pending);
        smpc.write8(0x1f, 0x02);
        assert!(smpc.slave_on);
    }

    #[test]
    fn scu_interrupt_levels_vectors_and_version_match_source_map() {
        let mut scu = SaturnScu {
            irq_mask: 0,
            ..Default::default()
        };
        for (bit, level) in [(0u8, 15u8), (3, 12), (4, 11), (6, 9), (11, 5), (13, 2)] {
            scu.irq_status = 1u32 << bit;
            assert_eq!(scu.interrupt(), Some((level, 0x40 + bit)));
        }
        assert_eq!(
            [
                scu.read8(0xc8),
                scu.read8(0xc9),
                scu.read8(0xca),
                scu.read8(0xcb),
            ],
            4u32.to_be_bytes()
        );
    }

    #[test]
    fn scu_hblank_timers_follow_compare_delay_and_mode() {
        let mut scu = SaturnScu::default();
        put_be32(&mut scu.regs, 0x90, 2);
        put_be32(&mut scu.regs, 0x94, 3);
        put_be16(&mut scu.regs, 0x9a, 0x0001);

        scu.vblank_out();
        assert_eq!(scu.timer0_counter, 0);
        assert_ne!(scu.irq_status & (1 << 1), 0);

        scu.irq_status = 0;
        scu.hblank_in();
        assert_eq!(scu.timer0_counter, 1);
        assert_eq!(scu.irq_status & (1 << 3), 0);
        assert!(scu.timer1_armed);
        assert_eq!(scu.timer1_countdown, 12);

        scu.tick_timing(11);
        assert_eq!(scu.irq_status & (1 << 4), 0);
        scu.tick_timing(1);
        assert_ne!(scu.irq_status & (1 << 4), 0);

        scu.irq_status = 0;
        scu.hblank_in();
        assert_eq!(scu.timer0_counter, 2);
        scu.hblank_in();
        assert_ne!(scu.irq_status & (1 << 3), 0);

        let mut gated = SaturnScu::default();
        put_be32(&mut gated.regs, 0x90, 1);
        put_be32(&mut gated.regs, 0x94, 2);
        put_be16(&mut gated.regs, 0x9a, 0x0101);
        gated.hblank_in();
        assert!(!gated.timer1_armed);
        gated.hblank_in();
        assert!(gated.timer1_armed);
        assert_eq!(gated.timer1_countdown, 8);

        put_be16(&mut gated.regs, 0x9a, 0x0000);
        gated.write8(0x9b, 0);
        assert_eq!(gated.timer0_counter, 0);
        assert!(!gated.timer1_armed);
    }

    #[test]
    fn saturn_frame_events_raise_vblank_and_vdp1_draw_end() {
        let bios = synthetic_bios();
        let mut board = SaturnBoard::new(&bios, None).unwrap();
        board.scu.irq_status = 0;
        board.scu.vblank_out();
        assert_ne!(board.scu.irq_status & (1 << 1), 0);
        board.scu.irq_status = 0;
        board.end_frame();
        assert_ne!(board.scu.irq_status & (1 << 0), 0);
        assert_ne!(board.scu.irq_status & (1 << 13), 0);
    }

    #[test]
    fn scu_dma_start_factors_require_enable_and_matching_event() {
        let mut scu = SaturnScu::default();

        put_be32(&mut scu.regs, 0x10, 0x0000_0100);
        put_be32(&mut scu.regs, 0x14, 0);
        scu.vblank_in();
        assert!(scu.dma_pending[0]);
        assert_eq!(scu.dma_status(), 0x20);
        assert_eq!(
            [
                scu.read8(0x5c),
                scu.read8(0x5d),
                scu.read8(0x5e),
                scu.read8(0x5f),
            ],
            0x20u32.to_be_bytes()
        );
        assert_eq!(
            [
                scu.read8(0x7c),
                scu.read8(0x7d),
                scu.read8(0x7e),
                scu.read8(0x7f),
            ],
            0x20u32.to_be_bytes()
        );
        scu.dma_pending[0] = false;

        put_be32(&mut scu.regs, 0x14, 2);
        scu.vblank_in();
        assert!(!scu.dma_pending[0]);
        scu.hblank_in();
        assert!(scu.dma_pending[0]);
        scu.dma_pending[0] = false;

        put_be32(&mut scu.regs, 0x14, 3);
        put_be32(&mut scu.regs, 0x90, u32::from(scu.timer0_counter));
        put_be16(&mut scu.regs, 0x9a, 1);
        scu.hblank_in();
        assert!(scu.dma_pending[0]);
        scu.dma_pending[0] = false;

        put_be32(&mut scu.regs, 0x14, 6);
        scu.dma_event(6);
        assert!(scu.dma_pending[0]);
        scu.dma_pending[0] = false;

        put_be32(&mut scu.regs, 0x14, 7);
        put_be32(&mut scu.regs, 0x10, 0);
        scu.write8(0x13, 1);
        assert!(!scu.dma_pending[0]);
        scu.write8(0x12, 1);
        assert!(!scu.dma_pending[0]);
        scu.write8(0x13, 1);
        assert!(scu.dma_pending[0]);
    }

    #[test]
    fn scu_dma_copies_between_work_ram_regions_and_raises_interrupt() {
        let bios = synthetic_bios();
        let mut board = SaturnBoard::new(&bios, None).unwrap();
        board.work_ram_l[..4].copy_from_slice(&[1, 2, 3, 4]);
        {
            let mut bus = SaturnShBus {
                board: &mut board,
                side: 0,
            };
            bus.write32(0x05fe_0000, 0x0020_0000);
            bus.write32(0x05fe_0004, 0x0600_0000);
            bus.write32(0x05fe_0008, 4);
            bus.write32(0x05fe_000c, 0x0000_0101);
            bus.write32(0x05fe_0014, 0x0001_0107);
            bus.write32(0x05fe_0010, 0x0000_0101);
        }
        board.service_dma();
        assert_eq!(&board.work_ram_h[..4], &[1, 2, 3, 4]);
        assert_eq!(board.scu.reg32(0x00), 0x0020_0004);
        assert_eq!(board.scu.reg32(0x04), 0x0600_0004);
        assert_ne!(board.scu.irq_status & (1 << 11), 0);
    }

    #[test]
    fn scu_dma_source_hold_repeats_words() {
        let bios = synthetic_bios();
        let mut board = SaturnBoard::new(&bios, None).unwrap();
        board.work_ram_l[..4].copy_from_slice(&[0x12, 0x34, 0x56, 0x78]);
        {
            let mut bus = SaturnShBus {
                board: &mut board,
                side: 0,
            };
            bus.write32(0x05fe_0000, 0x0020_0000);
            bus.write32(0x05fe_0004, 0x0600_0000);
            bus.write32(0x05fe_0008, 4);
            bus.write32(0x05fe_000c, 0x0000_0001);
            bus.write32(0x05fe_0014, 7);
            bus.write32(0x05fe_0010, 0x0000_0101);
        }

        board.service_dma();
        assert_eq!(&board.work_ram_h[..4], &[0x12, 0x34, 0x12, 0x34]);
    }

    #[test]
    fn scu_indirect_dma_follows_descriptor_chain_and_end_flag() {
        let bios = synthetic_bios();
        let mut board = SaturnBoard::new(&bios, None).unwrap();
        board.work_ram_l[..4].copy_from_slice(&[9, 8, 7, 6]);
        write32(&mut board.work_ram_l[..], 0x100, 4);
        write32(&mut board.work_ram_l[..], 0x104, 0x0600_0000);
        write32(&mut board.work_ram_l[..], 0x108, 0x8020_0000);

        {
            let mut bus = SaturnShBus {
                board: &mut board,
                side: 0,
            };
            bus.write32(0x05fe_0004, 0x0020_0100);
            bus.write32(0x05fe_000c, 0x0000_0101);
            bus.write32(0x05fe_0014, 0x0100_0107);
            bus.write32(0x05fe_0010, 0x0000_0101);
        }

        board.service_dma();
        assert_eq!(&board.work_ram_h[..4], &[9, 8, 7, 6]);
        assert_eq!(board.scu.reg32(0x04), 0x0020_010c);
        assert_ne!(board.scu.irq_status & (1 << 11), 0);
        assert_eq!(board.scu.irq_status & (1 << 12), 0);
    }

    #[test]
    fn vdp1_zero_ctrl_normal_sprite_reads_rgb_texture() {
        let mut vdp1 = SaturnVdp1::default();
        let source = 0x1000usize;
        write16(vdp1.vram.as_mut_slice(), 0x00, 0x0000);
        write16(vdp1.vram.as_mut_slice(), 0x04, 0x00a8);
        write16(vdp1.vram.as_mut_slice(), 0x08, (source / 8) as u16);
        write16(vdp1.vram.as_mut_slice(), 0x0a, 0x0101);
        write16(vdp1.vram.as_mut_slice(), 0x0c, 4);
        write16(vdp1.vram.as_mut_slice(), 0x0e, 5);
        write16(vdp1.vram.as_mut_slice(), 0x20, 0x8000);
        for pixel in 0..8 {
            write16(vdp1.vram.as_mut_slice(), source + pixel * 2, 0x801f);
        }

        vdp1.end_frame();
        let offset = (5 * 512 + 4) * 2;
        assert_eq!(be16(vdp1.display_buffer(), offset), 0x801f);
    }

    #[test]
    fn vdp1_local_and_user_clipping_limit_draw_commands() {
        let mut vdp1 = SaturnVdp1::default();

        write16(vdp1.vram.as_mut_slice(), 0x00, 0x000a);
        write16(vdp1.vram.as_mut_slice(), 0x0c, 10);
        write16(vdp1.vram.as_mut_slice(), 0x0e, 20);

        write16(vdp1.vram.as_mut_slice(), 0x20, 0x0008);
        write16(vdp1.vram.as_mut_slice(), 0x2c, 12);
        write16(vdp1.vram.as_mut_slice(), 0x2e, 22);
        write16(vdp1.vram.as_mut_slice(), 0x34, 13);
        write16(vdp1.vram.as_mut_slice(), 0x36, 23);

        write16(vdp1.vram.as_mut_slice(), 0x40, 0x0009);
        write16(vdp1.vram.as_mut_slice(), 0x54, 15);
        write16(vdp1.vram.as_mut_slice(), 0x56, 25);

        write16(vdp1.vram.as_mut_slice(), 0x60, 0x0006);
        write16(vdp1.vram.as_mut_slice(), 0x64, 0x0400);
        write16(vdp1.vram.as_mut_slice(), 0x66, 0x801f);
        write16(vdp1.vram.as_mut_slice(), 0x6c, 0);
        write16(vdp1.vram.as_mut_slice(), 0x6e, 0);
        write16(vdp1.vram.as_mut_slice(), 0x70, 10);
        write16(vdp1.vram.as_mut_slice(), 0x72, 10);
        write16(vdp1.vram.as_mut_slice(), 0x80, 0x8000);

        vdp1.end_frame();
        assert_eq!(be16(vdp1.display_buffer(), (22 * 512 + 12) * 2), 0x801f);
        assert_eq!(be16(vdp1.display_buffer(), (23 * 512 + 13) * 2), 0x801f);
        assert_eq!(be16(vdp1.display_buffer(), (21 * 512 + 11) * 2), 0);
        assert_eq!(be16(vdp1.display_buffer(), (24 * 512 + 14) * 2), 0);
    }

    #[test]
    fn vdp1_call_return_and_skip_traverse_command_lists() {
        let mut vdp1 = SaturnVdp1::default();

        write16(vdp1.vram.as_mut_slice(), 0x00, 0x200a);
        write16(vdp1.vram.as_mut_slice(), 0x02, 0x000c);
        write16(vdp1.vram.as_mut_slice(), 0x0c, 1);
        write16(vdp1.vram.as_mut_slice(), 0x0e, 1);

        write16(vdp1.vram.as_mut_slice(), 0x20, 0x0006);
        write16(vdp1.vram.as_mut_slice(), 0x26, 0x83e0);
        write16(vdp1.vram.as_mut_slice(), 0x2c, 1);
        write16(vdp1.vram.as_mut_slice(), 0x2e, 0);
        write16(vdp1.vram.as_mut_slice(), 0x30, 1);
        write16(vdp1.vram.as_mut_slice(), 0x32, 0);
        write16(vdp1.vram.as_mut_slice(), 0x40, 0x8000);

        write16(vdp1.vram.as_mut_slice(), 0x60, 0x0006);
        write16(vdp1.vram.as_mut_slice(), 0x66, 0x801f);
        write16(vdp1.vram.as_mut_slice(), 0x6c, 0);
        write16(vdp1.vram.as_mut_slice(), 0x6e, 0);
        write16(vdp1.vram.as_mut_slice(), 0x70, 0);
        write16(vdp1.vram.as_mut_slice(), 0x72, 0);
        write16(vdp1.vram.as_mut_slice(), 0x80, 0x7000);

        vdp1.end_frame();
        assert_eq!(be16(vdp1.display_buffer(), 1026), 0x801f);
        assert_eq!(be16(vdp1.display_buffer(), 1028), 0x83e0);
    }

    #[test]
    fn vdp2_rbg0_cell_mode_uses_parameter_specific_rotation_maps() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0010);
        write16(vdp2.regs.as_mut_slice(), 0x002a, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0038, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x003a, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x003e, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0050, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0060, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00fc, 0x0001);
        write_identity_rotation_parameter(&mut vdp2, 0);

        write16(vdp2.vram.as_mut_slice(), 0x0000, 0x0001);
        vdp2.vram[0x0020] = 0x10;
        write16(vdp2.cram.as_mut_slice(), 0x0002, 0x001f);

        write16(vdp2.vram.as_mut_slice(), 0x2000, 0x0002);
        vdp2.vram[0x0040] = 0x20;
        write16(vdp2.cram.as_mut_slice(), 0x0004, 0x03e0);

        write16(vdp2.regs.as_mut_slice(), 0x00b0, 0x0000);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x00b0, 0x0001);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 255, 0, 255]);
    }

    #[test]
    fn vdp2_rectangular_windows_gate_scroll_layers_with_area_and_logic() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0012);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00ac, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00ae, 0x8000);
        write16(vdp2.vram.as_mut_slice(), 0x30000, 0x7c00);
        vdp2.vram[0] = 1;
        vdp2.vram[1] = 1;
        vdp2.vram[2] = 1;
        write16(vdp2.cram.as_mut_slice(), 2, 0x001f);

        write16(vdp2.regs.as_mut_slice(), 0x00c0, 1);
        write16(vdp2.regs.as_mut_slice(), 0x00c2, 0);
        write16(vdp2.regs.as_mut_slice(), 0x00c4, 1);
        write16(vdp2.regs.as_mut_slice(), 0x00c6, 0);
        write16(vdp2.regs.as_mut_slice(), 0x00d0, 0x0002);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 0, 255, 255]);
        assert_eq!(&vdp2.video.pixels()[4..8], &[255, 0, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x00d0, 0x0003);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);
        assert_eq!(&vdp2.video.pixels()[4..8], &[0, 0, 255, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x00c0, 0);
        write16(vdp2.regs.as_mut_slice(), 0x00c4, 0);
        write16(vdp2.regs.as_mut_slice(), 0x00c8, 1);
        write16(vdp2.regs.as_mut_slice(), 0x00ca, 0);
        write16(vdp2.regs.as_mut_slice(), 0x00cc, 1);
        write16(vdp2.regs.as_mut_slice(), 0x00ce, 0);
        write16(vdp2.regs.as_mut_slice(), 0x00d0, 0x000a);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);
        assert_eq!(&vdp2.video.pixels()[4..8], &[255, 0, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x00d0, 0x008a);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 0, 255, 255]);
        assert_eq!(&vdp2.video.pixels()[4..8], &[0, 0, 255, 255]);
    }

    #[test]
    fn vdp2_line_window_reads_per_scanline_horizontal_ranges() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0012);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00ac, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00ae, 0x8000);
        write16(vdp2.vram.as_mut_slice(), 0x30000, 0x7c00);
        for offset in [0usize, 1, 512, 513] {
            vdp2.vram[offset] = 1;
        }
        write16(vdp2.cram.as_mut_slice(), 2, 0x001f);

        write16(vdp2.regs.as_mut_slice(), 0x00c0, 0);
        write16(vdp2.regs.as_mut_slice(), 0x00c2, 0);
        write16(vdp2.regs.as_mut_slice(), 0x00c4, 1);
        write16(vdp2.regs.as_mut_slice(), 0x00c6, 1);
        write16(vdp2.regs.as_mut_slice(), 0x00d0, 0x0002);
        write16(vdp2.regs.as_mut_slice(), 0x00d8, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x00da, 0x2000);
        write16(vdp2.vram.as_mut_slice(), 0x4000, 0);
        write16(vdp2.vram.as_mut_slice(), 0x4002, 0);
        write16(vdp2.vram.as_mut_slice(), 0x4004, 1);
        write16(vdp2.vram.as_mut_slice(), 0x4006, 1);

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);
        assert_eq!(&vdp2.video.pixels()[4..8], &[0, 0, 255, 255]);
        let line1 = WIDTH * 4;
        assert_eq!(&vdp2.video.pixels()[line1..line1 + 4], &[0, 0, 255, 255]);
        assert_eq!(
            &vdp2.video.pixels()[line1 + 4..line1 + 8],
            &[255, 0, 0, 255]
        );
    }

    #[test]
    fn vdp2_rectangular_window_gates_sprite_layer() {
        let mut vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        put_be16(vdp1.framebuffers[vdp1.display].as_mut_slice(), 0, 0x801f);
        put_be16(vdp1.framebuffers[vdp1.display].as_mut_slice(), 2, 0x801f);
        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x00e0, 0x0020);
        write16(vdp2.regs.as_mut_slice(), 0x00f0, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00ac, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00ae, 0x8000);
        write16(vdp2.vram.as_mut_slice(), 0x30000, 0x7c00);
        write16(vdp2.regs.as_mut_slice(), 0x00c0, 1);
        write16(vdp2.regs.as_mut_slice(), 0x00c2, 0);
        write16(vdp2.regs.as_mut_slice(), 0x00c4, 1);
        write16(vdp2.regs.as_mut_slice(), 0x00c6, 0);
        write16(vdp2.regs.as_mut_slice(), 0x00d4, 0x0200);

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 0, 255, 255]);
        assert_eq!(&vdp2.video.pixels()[4..8], &[255, 0, 0, 255]);
    }

    #[test]
    fn vdp2_nbg0_line_scroll_applies_horizontal_then_vertical_entries() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();
        let table = 0x4000usize;

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0012);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x009a, 0x0006);
        write16(vdp2.regs.as_mut_slice(), 0x00a0, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x00a2, 0x2000);
        for (index, color) in [(1u8, 0x001fu16), (2, 0x03e0), (3, 0x7c00), (4, 0x03ff)] {
            write16(vdp2.cram.as_mut_slice(), usize::from(index) * 2, color);
        }
        vdp2.vram[0] = 1;
        vdp2.vram[1] = 2;
        vdp2.vram[512] = 3;
        vdp2.vram[513] = 4;

        put_be32(vdp2.vram.as_mut_slice(), table, 0x0001_0000);
        put_be32(vdp2.vram.as_mut_slice(), table + 4, 0x0001_0000);
        put_be32(vdp2.vram.as_mut_slice(), table + 8, 0x0000_0000);
        put_be32(vdp2.vram.as_mut_slice(), table + 12, 0x0000_0000);

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 255, 0, 255]);
        let line1 = WIDTH * 4;
        assert_eq!(&vdp2.video.pixels()[line1..line1 + 4], &[255, 0, 0, 255]);
    }

    #[test]
    fn vdp2_nbg0_line_zoom_overrides_horizontal_source_increment() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();
        let table = 0x4000usize;

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0012);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x009a, 0x0008);
        write16(vdp2.regs.as_mut_slice(), 0x00a0, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x00a2, 0x2000);
        write16(vdp2.cram.as_mut_slice(), 2, 0x001f);
        write16(vdp2.cram.as_mut_slice(), 0x0002 * 2, 0x03e0);
        vdp2.vram[0] = 1;
        vdp2.vram[1] = 2;
        vdp2.vram[512] = 1;
        vdp2.vram[513] = 2;
        put_be32(vdp2.vram.as_mut_slice(), table, 0x0000_8000);
        put_be32(vdp2.vram.as_mut_slice(), table + 4, 0x0001_0000);

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[4..8], &[255, 0, 0, 255]);
        assert_eq!(&vdp2.video.pixels()[8..12], &[0, 255, 0, 255]);
        let line1_x1 = (WIDTH + 1) * 4;
        assert_eq!(
            &vdp2.video.pixels()[line1_x1..line1_x1 + 4],
            &[0, 255, 0, 255]
        );
    }

    #[test]
    fn vdp2_nbg0_zoom_uses_fixed_point_source_increment() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0012);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.cram.as_mut_slice(), 2, 0x001f);
        write16(vdp2.cram.as_mut_slice(), 0x0002 * 2, 0x03e0);
        vdp2.vram[0] = 1;
        vdp2.vram[1] = 2;

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);
        assert_eq!(&vdp2.video.pixels()[4..8], &[0, 255, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x0078, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x007a, 0x8000);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);
        assert_eq!(&vdp2.video.pixels()[4..8], &[255, 0, 0, 255]);
        assert_eq!(&vdp2.video.pixels()[8..12], &[0, 255, 0, 255]);
    }

    #[test]
    fn vdp2_mosaic_reuses_block_origin_for_normal_background_pixels() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0012);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.cram.as_mut_slice(), 2, 0x001f);
        write16(vdp2.cram.as_mut_slice(), 0x0002 * 2, 0x03e0);
        vdp2.vram[0] = 1;
        vdp2.vram[1] = 2;
        vdp2.vram[512] = 2;
        vdp2.vram[513] = 2;

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[4..8], &[0, 255, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x0022, 0x1101);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[4..8], &[255, 0, 0, 255]);
        let line1_x1 = (WIDTH + 1) * 4;
        assert_eq!(
            &vdp2.video.pixels()[line1_x1..line1_x1 + 4],
            &[255, 0, 0, 255]
        );
    }

    #[test]
    fn vdp2_color_offset_selects_signed_a_and_b_adjustments() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0012);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.cram.as_mut_slice(), 2, 0x001f);
        vdp2.vram[0] = 1;

        write16(vdp2.regs.as_mut_slice(), 0x0110, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0112, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0114, 0x01c0);
        write16(vdp2.regs.as_mut_slice(), 0x0116, 0x0040);
        write16(vdp2.regs.as_mut_slice(), 0x0118, 0x0000);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[191, 64, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x0112, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x011a, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x011c, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x011e, 0x0020);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 32, 255]);
    }

    #[test]
    fn vdp2_back_screen_reads_single_and_per_line_vram_colors() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();
        let base = 0x2000usize;

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x00ac, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x00ae, 0x1000);
        write16(vdp2.vram.as_mut_slice(), base, 0x001f);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);
        let line1 = WIDTH * 4;
        assert_eq!(&vdp2.video.pixels()[line1..line1 + 4], &[255, 0, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x00ac, 0x8000);
        write16(vdp2.vram.as_mut_slice(), base + 2, 0x03e0);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);
        assert_eq!(&vdp2.video.pixels()[line1..line1 + 4], &[0, 255, 0, 255]);
    }

    #[test]
    fn vdp2_nbg0_bitmap_palette_scroll_and_transparency() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0012);
        write16(vdp2.regs.as_mut_slice(), 0x002c, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0070, 2);
        write16(vdp2.regs.as_mut_slice(), 0x0074, 3);
        write16(vdp2.regs.as_mut_slice(), 0x00ac, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00ae, 0xf000);
        write16(vdp2.vram.as_mut_slice(), 0x3e000, 0x7c00);
        write16(vdp2.cram.as_mut_slice(), 0x0105 * 2, 0x03e0);
        write16(vdp2.cram.as_mut_slice(), 0x0100 * 2, 0x001f);
        vdp2.vram[3 * 512 + 2] = 5;

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 255, 0, 255]);
        assert_eq!(&vdp2.video.pixels()[4..8], &[0, 0, 255, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0101);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[4..8], &[255, 0, 0, 255]);
    }

    #[test]
    fn vdp2_nbg0_bitmap_map_offset_and_rgb_formats() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();
        let base = 0x20000usize;

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x003c, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0032);
        write16(vdp2.vram.as_mut_slice(), base, 0x801f);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0042);
        vdp2.vram[base..base + 4].copy_from_slice(&[0x80, 0x12, 0x34, 0x56]);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0x56, 0x34, 0x12, 255]);
    }

    #[test]
    fn vdp2_nbg2_cell_uses_chctlb_map_and_cram() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0004);
        write16(vdp2.regs.as_mut_slice(), 0x00fa, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x002a, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0034, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x003a, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x003c, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0048, 0x0000);
        write16(vdp2.vram.as_mut_slice(), 0x0000, 0x0001);
        write16(vdp2.vram.as_mut_slice(), 0x0002, 0x0100);
        vdp2.vram[0x2000] = 0x20;
        write16(vdp2.cram.as_mut_slice(), 0x0012 * 2, 0x001f);

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);
    }

    #[test]
    fn vdp2_nbg3_cell_uses_256_color_mode_and_plane_a_field() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0008);
        write16(vdp2.regs.as_mut_slice(), 0x00fa, 0x0100);
        write16(vdp2.regs.as_mut_slice(), 0x002a, 0x0020);
        write16(vdp2.regs.as_mut_slice(), 0x0036, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x003a, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x003c, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x004c, 0x0001);
        write16(vdp2.vram.as_mut_slice(), 0x4000, 0x0001);
        write16(vdp2.vram.as_mut_slice(), 0x4002, 0x0180);
        vdp2.vram[0x3000] = 0x03;
        write16(vdp2.cram.as_mut_slice(), 0x0103 * 2, 0x03e0);

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 255, 0, 255]);
    }

    #[test]
    fn vdp2_normal_bg_priority_and_zero_transparency_follow_prin() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0003);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x1212);
        write16(vdp2.regs.as_mut_slice(), 0x003c, 0x0010);
        vdp2.vram[0x00000] = 1;
        vdp2.vram[0x20000] = 2;
        write16(vdp2.cram.as_mut_slice(), 2, 0x001f);
        write16(vdp2.cram.as_mut_slice(), 0x0002 * 2, 0x03e0);

        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0201);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 255, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0102);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0101);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0100);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 255, 0, 255]);
    }

    #[test]
    fn vdp2_nbg1_bitmap_uses_n1_registers_palette_and_scroll() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0002);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0100);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x1200);
        write16(vdp2.regs.as_mut_slice(), 0x002c, 0x0100);
        write16(vdp2.regs.as_mut_slice(), 0x0080, 2);
        write16(vdp2.regs.as_mut_slice(), 0x0084, 3);
        vdp2.vram[3 * 512 + 2] = 5;
        write16(vdp2.cram.as_mut_slice(), 0x0105 * 2, 0x001f);

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);
    }

    #[test]
    fn vdp2_nbg1_cell_scrolls_between_plane_a_and_b() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0002);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0100);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0032, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x003a, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x003c, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0044, 0x0100);

        write16(vdp2.vram.as_mut_slice(), 0x0000, 0x0001);
        write16(vdp2.vram.as_mut_slice(), 0x0002, 0x0100);
        vdp2.vram[0x2000] = 0x20;
        write16(vdp2.cram.as_mut_slice(), 0x0012 * 2, 0x001f);

        write16(vdp2.vram.as_mut_slice(), 0x4000, 0x0002);
        write16(vdp2.vram.as_mut_slice(), 0x4002, 0x0180);
        vdp2.vram[0x3000] = 0x30;
        write16(vdp2.cram.as_mut_slice(), 0x0023 * 2, 0x03e0);

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x0080, 512);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 255, 0, 255]);
    }

    #[test]
    fn vdp2_nbg0_cell_two_word_map_scrolls_between_planes() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0030, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x003a, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x003c, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0040, 0x0100);

        write16(vdp2.vram.as_mut_slice(), 0x0000, 0x0001);
        write16(vdp2.vram.as_mut_slice(), 0x0002, 0x0100);
        vdp2.vram[0x2000] = 0x20;
        write16(vdp2.cram.as_mut_slice(), 0x0012 * 2, 0x001f);

        write16(vdp2.vram.as_mut_slice(), 0x4000, 0x0002);
        write16(vdp2.vram.as_mut_slice(), 0x4002, 0x0180);
        vdp2.vram[0x3000] = 0x30;
        write16(vdp2.cram.as_mut_slice(), 0x0023 * 2, 0x03e0);

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x0070, 512);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 255, 0, 255]);
    }

    #[test]
    fn vdp2_nbg0_cell_one_word_auxiliary_flip_decodes_character_and_palette() {
        let vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0030, 0x8020);
        write16(vdp2.regs.as_mut_slice(), 0x003a, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x003c, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x0040, 0x0000);

        write16(vdp2.vram.as_mut_slice(), 0x0000, 0x2410);
        vdp2.vram[0x0203] = 0x03;
        write16(vdp2.cram.as_mut_slice(), 0x0123 * 2, 0x7c00);

        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 0, 255, 255]);
    }

    #[test]
    fn vdp2_sprite_priority_selectors_compete_with_normal_bg_priority() {
        let mut vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();

        put_be16(vdp1.framebuffers[vdp1.display].as_mut_slice(), 0, 0x4002);
        write16(vdp2.regs.as_mut_slice(), 0x0000, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x0020, 0x0001);
        write16(vdp2.regs.as_mut_slice(), 0x0028, 0x0012);
        write16(vdp2.regs.as_mut_slice(), 0x00e0, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x00f8, 0x0002);
        vdp2.vram[0] = 1;
        write16(vdp2.cram.as_mut_slice(), 2, 0x001f);
        write16(vdp2.cram.as_mut_slice(), 0x0002 * 2, 0x03e0);

        write16(vdp2.regs.as_mut_slice(), 0x00f0, 0x0300);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 255, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x00f0, 0x0100);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[255, 0, 0, 255]);

        write16(vdp2.regs.as_mut_slice(), 0x00f0, 0x0200);
        vdp2.render(&vdp1);
        assert_eq!(&vdp2.video.pixels()[0..4], &[0, 255, 0, 255]);
    }

    #[test]
    fn vdp1_color_bank_sprite_resolves_through_vdp2_cram() {
        let mut vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();
        let source = 0x1000usize;

        write16(vdp1.vram.as_mut_slice(), 0x00, 0x0000);
        write16(vdp1.vram.as_mut_slice(), 0x04, 0x0080);
        write16(vdp1.vram.as_mut_slice(), 0x06, 0x0020);
        write16(vdp1.vram.as_mut_slice(), 0x08, (source / 8) as u16);
        write16(vdp1.vram.as_mut_slice(), 0x0a, 0x0101);
        write16(vdp1.vram.as_mut_slice(), 0x0c, 7);
        write16(vdp1.vram.as_mut_slice(), 0x0e, 9);
        write16(vdp1.vram.as_mut_slice(), 0x20, 0x8000);
        vdp1.vram[source] = 0x10;

        write16(vdp2.regs.as_mut_slice(), 0, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x00e0, 0x0000);
        write16(vdp2.regs.as_mut_slice(), 0x00f0, 0x0001);
        write16(vdp2.cram.as_mut_slice(), 0x21 * 2, 0x03e0);

        vdp1.end_frame();
        vdp2.render(&vdp1);
        let pixel = (9 * WIDTH + 7) * 4;
        assert!(vdp2.video.pixels()[pixel] < 20);
        assert!(vdp2.video.pixels()[pixel + 1] > 200);
        assert!(vdp2.video.pixels()[pixel + 2] < 20);
    }

    #[test]
    fn vdp1_commands_compose_through_vdp2() {
        let mut vdp1 = SaturnVdp1::default();
        let mut vdp2 = SaturnVdp2::default();
        write16(vdp1.vram.as_mut_slice(), 0, 0x0006);
        write16(vdp1.vram.as_mut_slice(), 0x20, 0x8000);
        write16(vdp1.vram.as_mut_slice(), 6, 0x801f);
        write16(vdp1.vram.as_mut_slice(), 0x0c, 10);
        write16(vdp1.vram.as_mut_slice(), 0x0e, 10);
        write16(vdp1.vram.as_mut_slice(), 0x10, 20);
        write16(vdp1.vram.as_mut_slice(), 0x12, 10);
        write16(vdp2.regs.as_mut_slice(), 0, 0x8000);
        write16(vdp2.regs.as_mut_slice(), 0x00e0, 0x0020);
        write16(vdp2.regs.as_mut_slice(), 0x00f0, 0x0001);
        vdp1.end_frame();
        vdp2.render(&vdp1);
        let pixel = (10 * WIDTH + 10) * 4;
        assert!(vdp2.video.pixels()[pixel] > 200);
        assert!(vdp2.video.pixels()[pixel + 1] < 20);
    }

    #[test]
    fn scsp_pcm_slot_generates_stereo_audio() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        ram[..8].copy_from_slice(&[0x7f, 0x40, 0x20, 0x10, 0x80, 0xc0, 0xe0, 0xf0]);
        put_be16(scsp.regs.as_mut_slice(), 4, 0);
        put_be16(scsp.regs.as_mut_slice(), 6, 8);
        put_be16(scsp.regs.as_mut_slice(), 0x10, 0);
        scsp.regs[0x0c] = 0;
        scsp.regs[0x16] = 0xe0;
        put_be16(scsp.regs.as_mut_slice(), 0x400, 0x000f);
        scsp.write8(1, 0x30);
        scsp.write8(0, 0x18);
        scsp.tick((SH2_HZ / AUDIO_RATE * 16) as u32, &mut ram);
        assert!(scsp.keyed[0]);
        assert!(scsp
            .samples
            .iter()
            .any(|&(left, right)| left.abs() > 0.01 && right.abs() > 0.01));
    }

    #[test]
    fn scsp_fractional_pcm_position_linearly_interpolates_samples() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        ram[0] = 0;
        ram[1] = 127;
        put_be16(scsp.regs.as_mut_slice(), 0, 0x0830);
        put_be16(scsp.regs.as_mut_slice(), 4, 0);
        put_be16(scsp.regs.as_mut_slice(), 6, 2);
        put_be16(scsp.regs.as_mut_slice(), 0x0c, 0x0100);
        put_be16(scsp.regs.as_mut_slice(), 0x10, 0);
        scsp.keyed[0] = true;
        scsp.phase[0] = 0x0000_8000;

        let sample = scsp.sample_slot(0, &ram);
        assert!((sample - 0.496).abs() < 0.01);
    }

    #[test]
    fn scsp_key_off_enters_release_and_stops_after_rr_decay() {
        let mut scsp = SaturnScsp::default();
        put_be16(scsp.regs.as_mut_slice(), 4, 0);
        put_be16(scsp.regs.as_mut_slice(), 6, 8);
        put_be16(scsp.regs.as_mut_slice(), 0x08, 0x001f);
        put_be16(scsp.regs.as_mut_slice(), 0x0a, 0x001f);
        put_be16(scsp.regs.as_mut_slice(), 0x10, 0);

        scsp.write8(1, 0x30);
        scsp.write8(0, 0x18);
        assert!(scsp.keyed[0]);
        assert_eq!(scsp.envelope_state[0], 0);
        assert_eq!(scsp.update_envelope(0), 1.0);
        assert_eq!(scsp.envelope_state[0], 1);

        scsp.write8(1, 0x30);
        scsp.write8(0, 0x10);
        assert!(scsp.keyed[0]);
        assert_eq!(scsp.envelope_state[0], 3);

        for _ in 0..200 {
            if !scsp.keyed[0] {
                break;
            }
            let _ = scsp.update_envelope(0);
        }
        assert!(!scsp.keyed[0]);
        assert_eq!(scsp.envelope_level[0], 0.0);
    }

    #[test]
    fn scsp_loop_modes_follow_lpctl_cursor_rules() {
        let mut scsp = SaturnScsp::default();
        put_be16(scsp.regs.as_mut_slice(), 0x10, 0);
        put_be16(scsp.regs.as_mut_slice(), 0, 0x0800);

        scsp.keyed[0] = true;
        scsp.phase[0] = 3 << 16;
        scsp.advance_slot_phase(0, 1, 4, 0);
        assert!(!scsp.keyed[0]);

        put_be16(scsp.regs.as_mut_slice(), 0, 0x0800);
        scsp.keyed[0] = true;
        scsp.phase[0] = 3 << 16;
        scsp.backwards[0] = false;
        scsp.advance_slot_phase(0, 1, 4, 1);
        assert!(scsp.keyed[0]);
        assert_eq!(scsp.phase[0] >> 16, 1);

        scsp.phase[0] = 0;
        scsp.backwards[0] = false;
        scsp.advance_slot_phase(0, 1, 4, 2);
        assert!(scsp.backwards[0]);
        assert_eq!(scsp.phase[0] >> 16, 4);

        scsp.phase[0] = 3 << 16;
        scsp.backwards[0] = false;
        scsp.advance_slot_phase(0, 1, 4, 3);
        assert!(scsp.backwards[0]);
        assert_eq!(scsp.phase[0] >> 16, 4);
        scsp.advance_slot_phase(0, 1, 4, 3);
        assert_eq!(scsp.phase[0] >> 16, 3);
        scsp.phase[0] = 1 << 16;
        scsp.backwards[0] = true;
        scsp.advance_slot_phase(0, 1, 4, 3);
        assert!(!scsp.backwards[0]);
        assert_eq!(scsp.phase[0] >> 16, 2);
    }

    #[test]
    fn scsp_source_and_bit_control_follow_slot_registers() {
        let mut scsp = SaturnScsp::default();
        let ram = vec![0u8; SOUND_RAM_SIZE];
        put_be16(scsp.regs.as_mut_slice(), 4, 0);
        put_be16(scsp.regs.as_mut_slice(), 6, 4);
        put_be16(scsp.regs.as_mut_slice(), 0x0c, 0x0100);
        put_be16(scsp.regs.as_mut_slice(), 0x10, 0);

        put_be16(scsp.regs.as_mut_slice(), 0, 0x0210);
        scsp.keyed[0] = true;
        let inverted_zero = scsp.sample_slot(0, &ram);
        assert!(inverted_zero > 0.99);

        put_be16(scsp.regs.as_mut_slice(), 0, 0x0110);
        scsp.phase[0] = 0;
        scsp.keyed[0] = true;
        assert_eq!(scsp.sample_slot(0, &ram), 0.0);

        put_be16(scsp.regs.as_mut_slice(), 0, 0x0090);
        scsp.phase[0] = 0;
        scsp.keyed[0] = true;
        assert_ne!(scsp.sample_slot(0, &ram), 0.0);
    }

    #[test]
    fn scsp_sound_direct_bypasses_envelope_and_total_level() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        ram[0] = 0x7f;
        put_be16(scsp.regs.as_mut_slice(), 0, 0x0010);
        put_be16(scsp.regs.as_mut_slice(), 4, 0);
        put_be16(scsp.regs.as_mut_slice(), 6, 4);
        put_be16(scsp.regs.as_mut_slice(), 0x0c, 0x01ff);
        put_be16(scsp.regs.as_mut_slice(), 0x10, 0);
        put_be16(scsp.regs.as_mut_slice(), 0x16, 0xe000);
        put_be16(scsp.regs.as_mut_slice(), 0x400, 0x000f);
        scsp.keyed[0] = true;

        let (left, right) = scsp.mix_sample(&mut ram);
        assert!(left > 0.9);
        assert!(right > 0.9);
    }

    #[test]
    fn scsp_ring_modulation_offsets_pcm_address() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        ram[512] = 0x7f;
        put_be16(scsp.regs.as_mut_slice(), 0, 0x0010);
        put_be16(scsp.regs.as_mut_slice(), 4, 0);
        put_be16(scsp.regs.as_mut_slice(), 6, 4);
        put_be16(scsp.regs.as_mut_slice(), 0x0c, 0x0100);
        put_be16(scsp.regs.as_mut_slice(), 0x0e, 0xf000);
        put_be16(scsp.regs.as_mut_slice(), 0x10, 0);
        scsp.ring_buffer[0] = 1024;
        scsp.keyed[0] = true;

        assert!(scsp.sample_slot(0, &ram) > 0.9);
    }

    #[test]
    fn scsp_lfo_modulates_pitch_and_amplitude() {
        let mut scsp = SaturnScsp::default();
        put_be16(scsp.regs.as_mut_slice(), 0x10, 0);
        let pitch_lfo = (31 << 10) | (1 << 8) | (7 << 5);
        put_be16(scsp.regs.as_mut_slice(), 0x12, pitch_lfo);
        scsp.pitch_lfo_phase[0] = 63 << 16;
        assert!(scsp.pitch_step(0) > 65_536);

        let amplitude_lfo = (31 << 10) | (1 << 3) | 7;
        scsp.amplitude_lfo_phase[0] = 0;
        assert!(scsp.amplitude_lfo_gain(0, amplitude_lfo) < 0.1);
    }

    #[test]
    fn scsp_loop_link_moves_attack_to_decay_at_loop_start() {
        let mut scsp = SaturnScsp::default();
        put_be16(scsp.regs.as_mut_slice(), 0x0a, 0x4000);
        put_be16(scsp.regs.as_mut_slice(), 0x10, 0);
        scsp.keyed[0] = true;
        scsp.envelope_state[0] = 0;
        scsp.phase[0] = 1 << 16;
        scsp.advance_slot_phase(0, 2, 8, 1);
        assert_eq!(scsp.envelope_state[0], 1);
    }

    #[test]
    fn scsp_master_volume_scales_output() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        ram[0] = 0x7f;
        put_be16(scsp.regs.as_mut_slice(), 0, 0x0010);
        put_be16(scsp.regs.as_mut_slice(), 4, 0);
        put_be16(scsp.regs.as_mut_slice(), 6, 4);
        put_be16(scsp.regs.as_mut_slice(), 0x0c, 0x0100);
        put_be16(scsp.regs.as_mut_slice(), 0x16, 0xe000);
        put_be16(scsp.regs.as_mut_slice(), 0x400, 0x000f);
        scsp.keyed[0] = true;
        let full = scsp.mix_sample(&mut ram).0;

        scsp.phase[0] = 0;
        scsp.keyed[0] = true;
        put_be16(scsp.regs.as_mut_slice(), 0x400, 0x0007);
        let reduced = scsp.mix_sample(&mut ram).0;
        assert!(full > 0.9);
        assert!((reduced / full - 7.0 / 15.0).abs() < 0.02);
    }

    #[test]
    fn scsp_ring_buffer_mmio_round_trips() {
        let mut scsp = SaturnScsp::default();
        write_scsp_word(&mut scsp, 0x600, 0x7abc);
        assert_eq!(scsp.ring_buffer[0] as u16, 0x7abc);
        let high = scsp.read8(0x600);
        let low = scsp.read8(0x601);
        assert_eq!(u16::from_be_bytes([high, low]), 0x7abc);

        write_scsp_word(&mut scsp, 0x680, 0x9234);
        assert_eq!(scsp.ring_buffer[0] as u16, 0x9234);
    }

    #[test]
    fn scsp_slot_monitor_reports_selected_voice_state() {
        let mut scsp = SaturnScsp::default();
        write_scsp_word(&mut scsp, 0x408, 3 << 11);
        scsp.phase[3] = 0xa000_0000;
        scsp.envelope_state[3] = 2;
        scsp.envelope_level[3] = 0.5;

        let high = scsp.read8(0x408);
        let low = scsp.read8(0x409);
        let monitor = u16::from_be_bytes([high, low]);
        assert_eq!((monitor >> 7) & 0x0f, 0x0a);
        assert_eq!((monitor >> 5) & 0x03, 2);
        assert_eq!(monitor & 0x1f, 15);
    }

    #[test]
    fn scsp_external_inputs_follow_slot_16_17_routing() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        put_be16(scsp.regs.as_mut_slice(), 0x400, 0x000f);
        scsp.set_external_inputs(0x4000, 0);

        put_be16(scsp.regs.as_mut_slice(), 16 * 0x20 + 0x16, 0xe000);
        let direct = scsp.mix_sample(&mut ram);
        assert!((direct.0 - 0.5).abs() < 0.01);
        assert!((direct.1 - 0.5).abs() < 0.01);

        put_be16(scsp.regs.as_mut_slice(), 16 * 0x20 + 0x16, 0x00e0);
        let effect_route = scsp.mix_sample(&mut ram);
        assert!((effect_route.0 - 0.5).abs() < 0.01);
        assert!((effect_route.1 - 0.5).abs() < 0.01);

        put_be16(scsp.regs.as_mut_slice(), 16 * 0x20 + 0x16, 0);
        assert_eq!(scsp.mix_sample(&mut ram), (0.0, 0.0));
    }

    #[test]
    fn scsp_dsp_microprogram_executes_effect_write() {
        let mut dsp = SaturnScspDsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        dsp.coef[0] = 0x4000;
        dsp.mixs[0] = 0x1000;
        dsp.mpro[1] = 0xa800;
        dsp.mpro[6] = 0x1000;
        dsp.start();

        dsp.step(&mut ram);

        assert_eq!(dsp.last_step, 2);
        assert_eq!(dsp.efreg[0], 0x0080);
        assert_eq!(dsp.mixs[0], 0);
        assert_eq!(dsp.dec, u32::MAX);
    }

    #[test]
    fn scsp_dsp_mmio_and_ring_configuration_follow_register_map() {
        let mut scsp = SaturnScsp::default();
        write_scsp_word(&mut scsp, 0x402, 0x0115);
        write_scsp_word(&mut scsp, 0x700, 0xc321);
        write_scsp_word(&mut scsp, 0x780, 0x4321);
        write_scsp_word(&mut scsp, 0x7c0, 0x2345);
        write_scsp_word(&mut scsp, 0x800, 0xabcd);
        write_scsp_word(&mut scsp, 0xbf0, 0x0000);

        assert_eq!(scsp.dsp.rbl, 32 * 1024);
        assert_eq!(scsp.dsp.rbp, 0x15);
        assert_eq!(scsp.dsp.coef[0] as u16, 0xc321);
        assert_eq!(scsp.dsp.madrs[0], 0x2345);
        assert_eq!(scsp.dsp.mpro[0], 0xabcd);
        assert!(!scsp.dsp.stopped);
        assert_eq!(scsp.dsp.last_step, 1);
        assert_eq!(scsp.read_word(0x700), 0xc321);
        assert_eq!(scsp.read_word(0x780), 0x2345);
        assert_eq!(scsp.read_word(0x800), 0xabcd);
    }

    #[test]
    fn scsp_dsp_effect_path_feeds_and_returns_slot_audio() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        ram[0] = 0x7f;
        put_be16(scsp.regs.as_mut_slice(), 0, 0x0010);
        put_be16(scsp.regs.as_mut_slice(), 4, 0);
        put_be16(scsp.regs.as_mut_slice(), 6, 4);
        put_be16(scsp.regs.as_mut_slice(), 0x0c, 0x0100);
        put_be16(scsp.regs.as_mut_slice(), 0x10, 0);
        put_be16(scsp.regs.as_mut_slice(), 0x14, 0x0007);
        put_be16(scsp.regs.as_mut_slice(), 0x16, 0x00e0);
        put_be16(scsp.regs.as_mut_slice(), 0x400, 0x000f);
        scsp.keyed[0] = true;

        write_scsp_word(&mut scsp, 0x700, 0x4000);
        write_scsp_word(&mut scsp, 0x802, 0xa800);
        write_scsp_word(&mut scsp, 0x80c, 0x1000);
        write_scsp_word(&mut scsp, 0xbf0, 0x0000);

        let (left, right) = scsp.mix_sample(&mut ram);

        assert!(left > 0.02);
        assert!(right > 0.02);
        assert!(scsp.dsp.efreg[0] > 0);
    }

    #[test]
    fn scsp_dsp_reads_delay_ram_on_odd_microsteps() {
        let mut dsp = SaturnScspDsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        ram[2] = 0x12;
        ram[3] = 0x34;
        dsp.madrs[0] = 1;
        dsp.mpro[6] = 0x2000;
        dsp.mpro[7] = 0x8000;
        dsp.mpro[9] = 0x0020;
        dsp.start();

        dsp.step(&mut ram);

        assert_eq!(dsp.mems[0], 0x123400);
    }

    #[test]
    fn scsp_dma_copies_sound_ram_to_registers_and_preserves_parameters() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        ram[0x100] = 0x7a;
        ram[0x101] = 0xbc;
        write_scsp_word(&mut scsp, 0x412, 0x0100);
        write_scsp_word(&mut scsp, 0x414, 0x0600);
        write_scsp_word(&mut scsp, 0x416, 0x1002);
        assert!(scsp.dma_pending);

        scsp.execute_dma(&mut ram);

        assert_eq!(scsp.ring_buffer[0] as u16, 0x7abc);
        assert_eq!(be16(scsp.regs.as_slice(), 0x412), 0x0100);
        assert_eq!(be16(scsp.regs.as_slice(), 0x414), 0x0600);
        assert_eq!(be16(scsp.regs.as_slice(), 0x416), 0x0002);
        assert!(!scsp.dma_pending);
    }

    #[test]
    fn scsp_dma_direction_and_gate_write_sound_ram() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        write_scsp_word(&mut scsp, 0x600, 0x9234);
        write_scsp_word(&mut scsp, 0x412, 0x0200);
        write_scsp_word(&mut scsp, 0x414, 0x0600);
        write_scsp_word(&mut scsp, 0x416, 0x3002);
        scsp.execute_dma(&mut ram);
        assert_eq!(&ram[0x200..0x202], &[0x92, 0x34]);

        ram[0x202] = 0xff;
        ram[0x203] = 0xff;
        write_scsp_word(&mut scsp, 0x412, 0x0202);
        write_scsp_word(&mut scsp, 0x414, 0x0600);
        write_scsp_word(&mut scsp, 0x416, 0x7002);
        scsp.execute_dma(&mut ram);
        assert_eq!(&ram[0x202..0x204], &[0, 0]);
    }

    #[test]
    fn scsp_dma_completion_routes_and_acknowledges_sci_interrupt() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        write_scsp_word(&mut scsp, 0x41e, 0x0010);
        write_scsp_word(&mut scsp, 0x424, 0x0010);
        write_scsp_word(&mut scsp, 0x426, 0x0000);
        write_scsp_word(&mut scsp, 0x428, 0x0010);
        write_scsp_word(&mut scsp, 0x412, 0x0100);
        write_scsp_word(&mut scsp, 0x414, 0x0600);
        write_scsp_word(&mut scsp, 0x416, 0x7002);

        scsp.execute_dma(&mut ram);
        assert!(scsp.dma_irq_pending);
        assert_eq!(scsp.interrupt_source(), Some((4, 5, 29)));
        assert_eq!(scsp.interrupt(), Some((5, 29)));
        scsp.acknowledge_interrupt(4);
        assert!(!scsp.dma_irq_pending);
        assert_eq!(scsp.interrupt(), None);
    }

    #[test]
    fn scsp_midi_input_fifo_updates_status_and_sci_irq() {
        let mut scsp = SaturnScsp::default();
        write_scsp_word(&mut scsp, 0x41e, 0x0008);
        write_scsp_word(&mut scsp, 0x424, 0x0008);
        write_scsp_word(&mut scsp, 0x426, 0x0008);
        write_scsp_word(&mut scsp, 0x428, 0x0000);

        scsp.push_midi_input(0x5a);

        assert_eq!(scsp.midi_status_word() & 0x0100, 0);
        assert_eq!(scsp.interrupt(), Some((3, 27)));
        assert_eq!(scsp.read8(0x404) & 0x01, 0);
        assert_eq!(scsp.read8(0x405), 0x5a);
        assert_eq!(scsp.midi_status_word() & 0x0100, 0x0100);
        assert_eq!(scsp.interrupt(), None);

        for value in 0..32 {
            scsp.push_midi_input(value);
        }
        scsp.push_midi_input(0xff);
        assert_eq!(scsp.midi_status_word() & 0x0600, 0x0600);
    }

    #[test]
    fn scsp_midi_output_fifo_drains_at_serial_rate() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];
        assert_eq!(scsp.midi_status_word() & 0x0800, 0x0800);

        scsp.write8(0x407, 0xa5);
        assert_eq!(scsp.midi_out_count, 1);
        assert_eq!(scsp.midi_status_word() & 0x0800, 0);

        let byte_cycles = (SH2_HZ / 3_125 + 1) as u32;
        scsp.tick(byte_cycles, &mut ram);

        assert_eq!(scsp.midi_out_count, 0);
        assert_eq!(scsp.midi_status_word() & 0x0800, 0x0800);
    }

    #[test]
    fn scsp_timer_a_routes_sci_and_main_interrupts() {
        let mut scsp = SaturnScsp::default();
        let mut ram = vec![0u8; SOUND_RAM_SIZE];

        write_scsp_word(&mut scsp, 0x418, 0x00fe);
        write_scsp_word(&mut scsp, 0x41e, 0x0040);
        write_scsp_word(&mut scsp, 0x424, 0x0040);
        write_scsp_word(&mut scsp, 0x426, 0x0000);
        write_scsp_word(&mut scsp, 0x428, 0x0040);
        write_scsp_word(&mut scsp, 0x42a, 0x0040);

        scsp.tick((SH2_HZ / AUDIO_RATE + 1) as u32, &mut ram);
        assert_eq!(be16(scsp.regs.as_slice(), 0x418) & 0x00ff, 0x00ff);
        assert_eq!(be16(scsp.regs.as_slice(), 0x420) & 0x0040, 0x0040);
        assert_eq!(be16(scsp.regs.as_slice(), 0x42c) & 0x0040, 0x0040);
        assert_eq!(scsp.interrupt(), Some((5, 29)));
        assert!(scsp.main_interrupt_pending());

        write_scsp_word(&mut scsp, 0x422, 0x0040);
        assert_eq!(be16(scsp.regs.as_slice(), 0x420) & 0x0040, 0x0040);

        write_scsp_word(&mut scsp, 0x418, 0x00fe);
        write_scsp_word(&mut scsp, 0x422, 0x0040);
        write_scsp_word(&mut scsp, 0x42e, 0x0040);
        assert_eq!(scsp.interrupt(), None);
        assert!(!scsp.main_interrupt_pending());
    }

    #[test]
    fn scsp_timer_a_raises_scu_sound_request() {
        let bios = synthetic_bios();
        let mut board = SaturnBoard::new(&bios, None).unwrap();
        write_scsp_word(&mut board.scsp, 0x418, 0x00fe);
        write_scsp_word(&mut board.scsp, 0x42a, 0x0040);

        board.tick((SH2_HZ / AUDIO_RATE + 1) as u32);
        assert_ne!(board.scu.irq_status & (1 << 6), 0);
    }

    #[test]
    fn cd_information_commands_return_hardware_toc_and_session_data() {
        let producer = ResourceBlob::streaming(4 * 2048, 2).unwrap();
        let mut cd = SaturnCdBlock::new(Some(producer)).unwrap();

        cd.cr[0] = 0x0100;
        cd.issue_command();
        assert_eq!(cd.cr, [0x0100, 0x0201, 0x0000, 0x0400]);
        assert_ne!(cd.hirq & 0x0001, 0);

        cd.hirq = 0;
        cd.cr[0] = 0x0200;
        cd.issue_command();
        assert_eq!(cd.cr, [0x4100, 204, 0, 0]);
        assert_eq!(cd.data.len(), 408);
        assert_eq!(cd.hirq & 0x0003, 0x0003);
        assert_eq!(
            cd.data.iter().take(4).copied().collect::<Vec<_>>(),
            vec![0x41, 0x00, 0x00, 150]
        );
        let toc = cd.data.iter().copied().collect::<Vec<_>>();
        assert_eq!(&toc[404..408], &[0x41, 0x00, 0x00, 154]);

        for _ in 0..204 {
            let _ = cd.read16(0x0581_8000);
        }
        assert!(cd.data.is_empty());
        assert_eq!(cd.hirq & 0x0002, 0);
        assert_eq!(cd.cr[0] & 0x4000, 0);

        cd.cr[0] = 0x0300;
        cd.issue_command();
        assert_eq!(cd.cr[2], 0x0100);
        assert_eq!(cd.cr[3], 154);

        cd.cr[0] = 0x0301;
        cd.issue_command();
        assert_eq!(cd.cr[2], 0x0100);
        assert_eq!(cd.cr[3], 0);
    }

    #[test]
    fn cd_multitrack_cue_populates_toc_and_track_addressing() {
        let cue = r#"
FILE "data.bin" BINARY
  TRACK 01 MODE1/2048
    INDEX 01 00:00:00
FILE "audio.bin" BINARY
  TRACK 02 AUDIO
    INDEX 01 00:00:00
"#;
        let data = vec![0u8; 2 * 2048];
        let audio = vec![0u8; 2352];
        let mut cd = SaturnCdBlock::new(Some(cue_container(
            cue,
            &[("data.bin", data), ("audio.bin", audio)],
        )))
        .unwrap();

        assert_eq!(cd.disc.as_ref().unwrap().tracks.len(), 2);
        assert_eq!(cd.disc.as_ref().unwrap().tracks[1].kind, TrackKind::Audio);

        cd.cr[0] = 0x0200;
        cd.issue_command();
        let toc = cd.data.iter().copied().collect::<Vec<_>>();
        assert_eq!(&toc[0..4], &[0x41, 0x00, 0x00, 150]);
        assert_eq!(&toc[4..8], &[0x01, 0x00, 0x00, 152]);
        assert_eq!(toc[397], 1);
        assert_eq!(toc[401], 2);
        assert_eq!(&toc[404..408], &[0x41, 0x00, 0x00, 153]);

        cd.cr = [0x1000, 0x0200, 0, 0];
        cd.issue_command();
        assert_eq!(cd.current_lba, 2);
        assert!(cd.reading);

        cd.cr = [0x1100, 0x0200, 0, 0];
        cd.issue_command();
        assert_eq!(cd.current_lba, 2);
        assert!(!cd.reading);
    }

    #[test]
    fn cd_multitrack_cue_streams_cdda_at_scsp_rate() {
        let cue = r#"
FILE "data.bin" BINARY
  TRACK 01 MODE1/2048
    INDEX 01 00:00:00
FILE "audio.bin" BINARY
  TRACK 02 AUDIO
    INDEX 01 00:00:00
"#;
        let data = vec![0u8; 2048];
        let mut audio = vec![0u8; 2352];
        for frame in audio.as_chunks_mut::<4>().0 {
            frame[..2].copy_from_slice(&0x2345i16.to_le_bytes());
            frame[2..].copy_from_slice(&(-0x1234i16).to_le_bytes());
        }
        let mut cd = SaturnCdBlock::new(Some(cue_container(
            cue,
            &[("data.bin", data), ("audio.bin", audio)],
        )))
        .unwrap();

        cd.cr = [0x1000, 0x0200, 0, 0];
        cd.issue_command();
        assert_eq!(cd.current_lba, 1);
        assert!(cd.audio_path_active());

        cd.tick((SH2_HZ / AUDIO_RATE + 1) as u32);

        assert_eq!(cd.cdda_samples.front(), Some(&[0x2345, -0x1234]));
        assert_eq!(cd.current_lba, 2);
        assert_eq!(cd.cdda_sample_index, 1);
    }

    #[test]
    fn cd_cdda_state_round_trip_preserves_playback_position_and_queue() {
        let cue = r#"
FILE "data.bin" BINARY
  TRACK 01 MODE1/2048
    INDEX 01 00:00:00
FILE "audio.bin" BINARY
  TRACK 02 AUDIO
    INDEX 01 00:00:00
"#;
        let data = vec![0u8; 2048];
        let mut audio = vec![0u8; 2 * 2352];
        for (index, frame) in audio.as_chunks_mut::<4>().0.iter_mut().enumerate() {
            let left = (index as i16).wrapping_mul(3);
            let right = left.wrapping_neg();
            frame[..2].copy_from_slice(&left.to_le_bytes());
            frame[2..].copy_from_slice(&right.to_le_bytes());
        }
        let disc = cue_container(cue, &[("data.bin", data), ("audio.bin", audio)]);
        let mut cd = SaturnCdBlock::new(Some(disc.clone())).unwrap();
        cd.cr = [0x1000, 0x0200, 0, 0];
        cd.issue_command();
        cd.tick((SH2_HZ / AUDIO_RATE * 37 + 1) as u32);

        let expected_lba = cd.current_lba;
        let expected_index = cd.cdda_sample_index;
        let expected_phase = cd.cdda_phase;
        let expected_sector = cd.cdda_sector.clone();
        let expected_samples = cd.cdda_samples.clone();

        let mut writer = StateWriter::new(PlatformId::Saturn, 99);
        cd.save(&mut writer);
        let state = writer.finish();

        let mut restored = SaturnCdBlock::new(Some(disc)).unwrap();
        let mut reader = StateReader::new(&state, PlatformId::Saturn, 99).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.current_lba, expected_lba);
        assert_eq!(restored.cdda_sample_index, expected_index);
        assert_eq!(restored.cdda_phase, expected_phase);
        assert_eq!(restored.cdda_sector.as_slice(), expected_sector.as_slice());
        assert_eq!(restored.cdda_samples, expected_samples);

        cd.tick((SH2_HZ / AUDIO_RATE * 11 + 1) as u32);
        restored.tick((SH2_HZ / AUDIO_RATE * 11 + 1) as u32);
        assert_eq!(restored.current_lba, cd.current_lba);
        assert_eq!(restored.cdda_sample_index, cd.cdda_sample_index);
        assert_eq!(restored.cdda_samples, cd.cdda_samples);
    }

    #[test]
    fn cdda_reaches_scsp_external_input_mixer() {
        let cue = r#"
FILE "data.bin" BINARY
  TRACK 01 MODE1/2048
    INDEX 01 00:00:00
FILE "audio.bin" BINARY
  TRACK 02 AUDIO
    INDEX 01 00:00:00
"#;
        let data = vec![0u8; 2048];
        let mut audio = vec![0u8; 2352];
        for frame in audio.as_chunks_mut::<4>().0 {
            frame[..2].copy_from_slice(&0x2000i16.to_le_bytes());
        }
        let bios = synthetic_bios();
        let mut board = SaturnBoard::new(
            &bios,
            Some(cue_container(
                cue,
                &[("data.bin", data), ("audio.bin", audio)],
            )),
        )
        .unwrap();
        put_be16(board.scsp.regs.as_mut_slice(), 0x400, 0x000f);
        put_be16(board.scsp.regs.as_mut_slice(), 16 * 0x20 + 0x16, 0xe000);
        board.cd.cr = [0x1000, 0x0200, 0, 0];
        board.cd.issue_command();

        board.tick((SH2_HZ / AUDIO_RATE + 1) as u32);

        assert!(board.cd.cdda_samples.is_empty());
        let (left, right) = board.scsp.samples.last().copied().unwrap();
        assert!(left > 0.2);
        assert!(right > 0.2);
    }

    #[test]
    fn cd_play_and_seek_use_fad_addresses_with_150_sector_origin() {
        let producer = ResourceBlob::streaming(8 * 2048, 2).unwrap();
        let mut cd = SaturnCdBlock::new(Some(producer)).unwrap();

        cd.cr = [0x1080, 151, 0x0080, 2];
        cd.issue_command();
        assert_eq!(cd.current_lba, 1);
        assert_eq!(cd.target_lba, 3);
        assert!(cd.reading);
        assert_eq!(cd.cr[0] & 0x0f00, 0x0300);

        cd.cr = [0x1180, 152, 0, 0];
        cd.issue_command();
        assert_eq!(cd.current_lba, 2);
        assert!(!cd.reading);
        assert_eq!(cd.cr[0] & 0x0f00, 0x0100);

        cd.cr = [0x11ff, 0xffff, 0, 0];
        cd.issue_command();
        assert_eq!(cd.current_lba, 2);
        assert_eq!(cd.cr[0] & 0x0f00, 0x0100);
    }

    #[test]
    fn cd_device_connection_command_preserves_drive_position() {
        let producer = ResourceBlob::streaming(8 * 2048, 2).unwrap();
        let mut cd = SaturnCdBlock::new(Some(producer)).unwrap();
        cd.current_lba = 3;
        cd.target_lba = 3;
        cd.cr = [0x3000, 0, 0x0200, 0];
        cd.issue_command();
        assert_eq!(cd.current_lba, 3);
        assert_eq!(cd.target_lba, 3);
        assert_eq!(cd.cd_device_connection, 2);
        assert_eq!(cd.hirq & 0x0041, 0x0041);

        cd.cr = [0x3100, 0, 0, 0];
        cd.issue_command();
        assert_eq!(cd.cr[2], 0x0200);
    }

    #[test]
    fn cd_buffered_read_protocol_matches_standard_host_flow() {
        let mut image = vec![0u8; 4 * 2048];
        image[..2048].fill(0x11);
        image[2048..4096].fill(0x22);
        image[4096..6144].fill(0x33);
        image[6144..].fill(0x44);
        let mut cd = SaturnCdBlock::new(Some(ResourceBlob::from_bytes(&image))).unwrap();

        cd.cr = [0x4800, 0, 0, 0];
        cd.issue_command();
        cd.cr = [0x3000, 0, 0, 0];
        cd.issue_command();
        cd.cr = [0x1080, 150, 0x0080, 2];
        cd.issue_command();
        let two_sector_cycles = (SH2_HZ * 2).div_ceil(SECTORS_PER_SECOND) as u32;
        cd.tick(two_sector_cycles);
        cd.tick((SH2_HZ / SECTORS_PER_SECOND + 1) as u32);

        assert_eq!(cd.partitions[0].len(), 2);
        assert!(!cd.reading);
        assert_eq!(cd.current_lba, 2);
        assert_eq!(cd.target_lba, 2);
        assert_eq!(cd.data.len(), 0);
        assert_eq!(cd.hirq & 0x0004, 0x0004);

        cd.cr = [0x5000, 0, 0, 0];
        cd.issue_command();
        assert_eq!(cd.cr[1], 198);
        assert_eq!(cd.cr[2], 0x1800);
        assert_eq!(cd.cr[3], 200);

        cd.cr = [0x5100, 0, 0, 0];
        cd.issue_command();
        assert_eq!(cd.cr[3], 2);

        cd.cr = [0x5200, 0, 0, 1];
        cd.issue_command();
        assert_eq!(cd.actual_size_words, 1024);
        cd.cr = [0x5300, 0, 0, 0];
        cd.issue_command();
        assert_eq!(cd.cr[0] & 0x00ff, 0);
        assert_eq!(cd.cr[1], 1024);

        cd.cr = [0x5400, 0, 0, 0];
        cd.issue_command();
        assert_eq!(cd.cr[0] & 0x00ff, 0);
        assert_eq!(cd.cr[1], 150);

        cd.cr = [0x6300, 0, 0, 1];
        cd.issue_command();
        assert_eq!(cd.partitions[0].len(), 1);
        assert_eq!(cd.data.len(), 2048);
        assert_eq!(cd.hirq & 0x0002, 0x0002);
        assert_eq!(cd.read16(0x0581_8000), 0x1111);
        assert_eq!(cd.transfer_bytes_read, 2);

        cd.cr = [0x0600, 0, 0, 0];
        cd.issue_command();
        assert!(cd.data.is_empty());
        assert_eq!(cd.cr[1], 1);
        assert_eq!(cd.hirq & 0x0080, 0x0080);
        assert_eq!(cd.hirq & 0x0002, 0);
    }

    #[test]
    fn cd_filter_range_and_false_chain_route_sectors() {
        let mut image = vec![0u8; 2 * 2048];
        image[..2048].fill(0x11);
        image[2048..].fill(0x22);
        let mut cd = SaturnCdBlock::new(Some(ResourceBlob::from_bytes(&image))).unwrap();

        cd.cr = [0x48fc, 0, 0, 0];
        cd.issue_command();
        cd.cr = [0x4603, 0x0201, 0x0000, 0];
        cd.issue_command();
        cd.cr = [0x4601, 0x0300, 0x0100, 0];
        cd.issue_command();
        cd.cr = [0x4000, 151, 0x0000, 1];
        cd.issue_command();
        cd.cr = [0x4440, 0, 0x0000, 0];
        cd.issue_command();

        cd.cr = [0x4100, 0, 0x0000, 0];
        cd.issue_command();
        assert_eq!(cd.cr[1], 151);
        assert_eq!(cd.cr[3], 1);
        cd.cr = [0x4700, 0, 0x0000, 0];
        cd.issue_command();
        assert_eq!(cd.cr[1], 0x0201);

        cd.cr = [0x3000, 0, 0x0000, 0];
        cd.issue_command();
        cd.cr = [0x1080, 150, 0x0080, 2];
        cd.issue_command();
        cd.tick((SH2_HZ * 2).div_ceil(SECTORS_PER_SECOND) as u32);

        assert_eq!(cd.partitions[2].len(), 1);
        assert_eq!(cd.partitions[2][0].fad, 151);
        assert_eq!(cd.partitions[3].len(), 1);
        assert_eq!(cd.partitions[3][0].fad, 150);
        assert_eq!(cd.last_buffer_destination, 0);

        cd.cr = [0x3200, 0, 0, 0];
        cd.issue_command();
        assert_eq!(cd.cr[2], 0x0000);
    }

    #[test]
    fn cd_filter_subheader_conditions_and_reverse_match() {
        let mut raw = Box::new([0u8; 2352]);
        raw[15] = 2;
        raw[16] = 3;
        raw[17] = 7;
        raw[18] = 0x24;
        raw[19] = 0x11;
        let sector = SaturnCdBufferedSector { fad: 321, raw };
        let mut filter = SaturnCdFilter::new(0);
        filter.mode = 0x0f;
        filter.file = 3;
        filter.channel = 7;
        filter.submode_mask = 0x3f;
        filter.submode = 0x24;
        filter.coding_info_mask = 0x1f;
        filter.coding_info = 0x11;
        assert!(filter.matches(&sector));

        filter.channel = 8;
        assert!(!filter.matches(&sector));
        filter.mode |= 0x10;
        assert!(filter.matches(&sector));

        filter.mode = 0x40;
        filter.fad = 320;
        filter.range = 2;
        assert!(filter.matches(&sector));
        filter.range = 1;
        assert!(!filter.matches(&sector));
    }

    #[test]
    fn cd_fad_search_and_copy_move_follow_partition_protocol() {
        let mut image = vec![0u8; 3 * 2048];
        image[..2048].fill(0x11);
        image[2048..4096].fill(0x22);
        image[4096..].fill(0x33);
        let mut cd = SaturnCdBlock::new(Some(ResourceBlob::from_bytes(&image))).unwrap();

        cd.cr = [0x3000, 0, 0, 0];
        cd.issue_command();
        cd.cr = [0x1080, 150, 0x0080, 3];
        cd.issue_command();
        cd.tick((SH2_HZ * 3).div_ceil(SECTORS_PER_SECOND) as u32);
        assert_eq!(cd.partitions[0].len(), 3);

        cd.cr = [0x5500, 0, 0x0000, 151];
        cd.issue_command();
        assert_eq!(cd.fad_search_pos, 1);
        assert_eq!(cd.fad_search_fad, 151);
        assert_eq!(cd.fad_search_partition, 0);

        cd.cr = [0x5600, 0, 0, 0];
        cd.issue_command();
        assert_eq!(cd.cr[1], 1);
        assert_eq!(cd.cr[2], 0);
        assert_eq!(cd.cr[3], 151);

        cd.cr = [0x6501, 0, 0x0000, 1];
        cd.issue_command();
        assert_eq!(cd.partitions[0].len(), 3);
        assert_eq!(cd.partitions[1].len(), 1);
        assert_eq!(cd.partitions[1][0].fad, 150);
        assert_eq!(cd.hirq & 0x0100, 0x0100);

        cd.cr = [0x6601, 1, 0x0000, 1];
        cd.issue_command();
        assert_eq!(cd.partitions[0].len(), 2);
        assert_eq!(cd.partitions[0][0].fad, 150);
        assert_eq!(cd.partitions[0][1].fad, 152);
        assert_eq!(cd.partitions[1].len(), 2);
        assert_eq!(cd.partitions[1][1].fad, 151);

        cd.cr = [0x6700, 0, 0, 0];
        cd.issue_command();
        assert_eq!(cd.cr[1..], [0, 0, 0]);
    }

    #[test]
    fn cd_sector_length_controls_host_transfer_view() {
        let cue = r#"
FILE "track.bin" BINARY
  TRACK 01 MODE1/2352
    INDEX 01 00:00:00
"#;
        let mut raw = vec![0u8; 2352];
        raw[0] = 0;
        raw[1..11].fill(0xff);
        raw[11] = 0;
        raw[15] = 1;
        for (index, byte) in raw.iter_mut().enumerate().skip(16) {
            *byte = index as u8;
        }
        let mut cd =
            SaturnCdBlock::new(Some(cue_container(cue, &[("track.bin", raw.clone())]))).unwrap();
        cd.cr = [0x3000, 0, 0, 0];
        cd.issue_command();
        cd.cr = [0x1080, 150, 0x0080, 1];
        cd.issue_command();
        cd.tick(SH2_HZ.div_ceil(SECTORS_PER_SECOND) as u32);
        assert_eq!(cd.partitions[0].len(), 1);

        for (format, expected_len, expected_first) in [
            (0u8, 2048usize, raw[16]),
            (1, 2336, raw[16]),
            (2, 2340, raw[12]),
            (3, 2352, raw[0]),
        ] {
            cd.cr = [0x6000 | u16::from(format), 0xff00, 0, 0];
            cd.issue_command();
            cd.cr = [0x6100, 0, 0, 1];
            cd.issue_command();
            assert_eq!(cd.data.len(), expected_len);
            assert_eq!(cd.data.front(), Some(&expected_first));
            assert_eq!(cd.partitions[0].len(), 1);
            cd.cr = [0x0600, 0, 0, 0];
            cd.issue_command();
        }
    }

    #[test]
    fn cd_buffer_state_round_trip_preserves_partitions_and_transfer() {
        let image = vec![0x6du8; 2 * 2048];
        let disc = ResourceBlob::from_bytes(&image);
        let mut cd = SaturnCdBlock::new(Some(disc.clone())).unwrap();
        cd.cr = [0x3000, 0, 0, 0];
        cd.issue_command();
        cd.cr = [0x1080, 150, 0x0080, 2];
        cd.issue_command();
        cd.tick((SH2_HZ * 2).div_ceil(SECTORS_PER_SECOND) as u32);
        cd.cr = [0x6100, 0, 0, 1];
        cd.issue_command();
        let _ = cd.read16(0x0581_8000);
        cd.filters[3].mode = 0x47;
        cd.filters[3].true_conn = 5;
        cd.filters[3].false_conn = 7;
        cd.filters[3].fad = 150;
        cd.filters[3].range = 2;
        cd.filters[3].channel = 4;
        cd.filters[3].file = 6;
        cd.filters[3].submode = 0x20;
        cd.filters[3].submode_mask = 0x3f;
        cd.filters[3].coding_info = 0x11;
        cd.filters[3].coding_info_mask = 0x1f;
        cd.cr = [0x5500, 0, 0x0000, 151];
        cd.issue_command();

        let mut writer = StateWriter::new(PlatformId::Saturn, 123);
        cd.save(&mut writer);
        let state = writer.finish();
        let mut restored = SaturnCdBlock::new(Some(disc)).unwrap();
        let mut reader = StateReader::new(&state, PlatformId::Saturn, 123).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();

        assert_eq!(restored.cd_device_connection, cd.cd_device_connection);
        assert_eq!(restored.partitions[0].len(), cd.partitions[0].len());
        assert_eq!(restored.partitions[0][0].fad, cd.partitions[0][0].fad);
        assert_eq!(
            restored.partitions[0][0].raw.as_slice(),
            cd.partitions[0][0].raw.as_slice()
        );
        assert_eq!(restored.data, cd.data);
        assert_eq!(restored.transfer_bytes_read, cd.transfer_bytes_read);
        assert_eq!(restored.fad_search_fad, cd.fad_search_fad);
        assert_eq!(restored.fad_search_pos, cd.fad_search_pos);
        assert_eq!(restored.fad_search_partition, cd.fad_search_partition);
        assert_eq!(restored.filters[3].mode, cd.filters[3].mode);
        assert_eq!(restored.filters[3].true_conn, cd.filters[3].true_conn);
        assert_eq!(restored.filters[3].false_conn, cd.filters[3].false_conn);
        assert_eq!(restored.filters[3].fad, cd.filters[3].fad);
        assert_eq!(restored.filters[3].range, cd.filters[3].range);
        assert_eq!(restored.filters[3].channel, cd.filters[3].channel);
        assert_eq!(restored.filters[3].file, cd.filters[3].file);
        assert_eq!(restored.filters[3].submode, cd.filters[3].submode);
        assert_eq!(restored.filters[3].submode_mask, cd.filters[3].submode_mask);
        assert_eq!(restored.filters[3].coding_info, cd.filters[3].coding_info);
        assert_eq!(
            restored.filters[3].coding_info_mask,
            cd.filters[3].coding_info_mask
        );
    }

    #[test]
    fn streaming_cd_requests_missing_sector_and_hydrates() {
        let mut producer = ResourceBlob::streaming(4 * 2048, 2).unwrap();
        let shared = producer.clone();
        let mut cd = SaturnCdBlock::new(Some(shared.clone())).unwrap();
        cd.cr = [0x3000, 0, 0, 0];
        cd.issue_command();
        cd.cr = [0x1080, 151, 0x0080, 1];
        cd.issue_command();
        cd.tick((SH2_HZ / SECTORS_PER_SECOND + 1) as u32);
        assert_eq!(shared.pending_range(), Some((2048, 4096)));
        let sector = vec![0x5a; 2048];
        producer.write(2048, &sector).unwrap();
        cd.tick((SH2_HZ / SECTORS_PER_SECOND + 1) as u32);
        assert_eq!(cd.partitions[0].len(), 1);
        assert!(cd.data.is_empty());

        cd.cr = [0x6300, 0, 0, 1];
        cd.issue_command();
        assert_eq!(cd.partitions[0].len(), 0);
        assert_eq!(cd.data.len(), 2048);
        assert_eq!(cd.data.front(), Some(&0x5a));
        assert_eq!(cd.read8(0x0581_8000), 0x5a);
        assert_eq!(cd.data.len(), 2047);
        assert_eq!(cd.read16(0x0581_8000), 0x5a5a);
        assert_eq!(cd.data.len(), 2045);
    }

    #[test]
    fn dual_sh2_machine_executes_and_state_round_trips() {
        let bios = synthetic_bios();
        let mut machine = SaturnMachine::from_bios_and_disc(&bios, None).unwrap();
        machine.board.smpc.write8(0x1f, 0x02);
        machine.run_frame(&InputState::default());
        assert!(machine.powered);
        assert!(machine.master.cycles > 1000);
        assert!(machine.slave.cycles > 0);
        assert_eq!(machine.video().width(), WIDTH as u32);
        assert_eq!(machine.video().height(), HEIGHT as u32);
        machine.board.sh_frt[0].write8(0xffff_fe12, 0x12);
        machine.board.sh_frt[0].write8(0xffff_fe13, 0x34);
        machine.board.sh_frt[1].write8(0xffff_fe12, 0x56);
        machine.board.sh_frt[1].write8(0xffff_fe13, 0x78);
        machine.board.sh_intc[0].write16(0xffff_fee2, 0x0050);
        machine.board.sh_intc[1].write16(0xffff_fe60, 0x0a00);
        machine.board.sh_divu[0].dvsr = 7;
        machine.board.sh_divu[0].divide32(100);
        machine.board.sh_divu[1].vcrdiv = 0x5a;
        machine.board.sh_dmac[0].write32(0xffff_ff80, 0x0020_1234);
        machine.board.sh_dmac[0].write32(0xffff_ff84, 0x0600_5678);
        machine.board.sh_dmac[0].write32(0xffff_ff88, 0x1234);
        machine.board.sh_dmac[1].write32(0xffff_ffa8, 0x66);
        machine.board.sh_wdt[0].write16(0xffff_fe80, 0x5a9a);
        machine.board.sh_wdt[1].write16(0xffff_fe80, 0x5abc);
        machine.board.sh_sci[0].write8(0xffff_fe01, 0x12);
        machine.board.sh_sci[1].write8(0xffff_fe01, 0x34);
        let saved = machine.save_state().unwrap();
        let master_pc = machine.master.pc;
        let slave_pc = machine.slave.pc;
        let frame = machine.board.vdp2.frame;
        machine.run_frame(&InputState::default());
        machine.board.sh_divu = [Sh2Divu::default(); 2];
        machine.board.sh_dmac = [Sh2Dmac::default(); 2];
        machine.board.sh_frt = [Sh2Frt::default(); 2];
        machine.board.sh_intc = [Sh2Intc::default(); 2];
        machine.board.sh_sci = [Sh2Sci::default(); 2];
        machine.board.sh_wdt = [Sh2Wdt::default(); 2];
        machine.load_state(&saved).unwrap();
        assert_eq!(machine.master.pc, master_pc);
        assert_eq!(machine.slave.pc, slave_pc);
        assert_eq!(machine.board.vdp2.frame, frame);
        assert_eq!(machine.board.sh_frt[0].read8(0xffff_fe12), Some(0x12));
        assert_eq!(machine.board.sh_frt[0].read8(0xffff_fe13), Some(0x34));
        assert_eq!(machine.board.sh_frt[1].read8(0xffff_fe12), Some(0x56));
        assert_eq!(machine.board.sh_frt[1].read8(0xffff_fe13), Some(0x78));
        assert_eq!(machine.board.sh_intc[0].read16(0xffff_fee2), Some(0x0050));
        assert_eq!(machine.board.sh_intc[1].read16(0xffff_fe60), Some(0x0a00));
        assert_eq!(machine.board.sh_divu[0].dvsr, 7);
        assert_eq!(machine.board.sh_divu[0].dvdntl, 14);
        assert_eq!(machine.board.sh_divu[0].dvdnth, 2);
        assert_eq!(machine.board.sh_divu[1].vcrdiv, 0x5a);
        assert_eq!(
            machine.board.sh_dmac[0].read32(0xffff_ff80),
            Some(0x0020_1234)
        );
        assert_eq!(
            machine.board.sh_dmac[0].read32(0xffff_ff84),
            Some(0x0600_5678)
        );
        assert_eq!(machine.board.sh_dmac[0].read32(0xffff_ff88), Some(0x1234));
        assert_eq!(machine.board.sh_dmac[1].read32(0xffff_ffa8), Some(0x66));
        assert_eq!(machine.board.sh_wdt[0].read8(0xffff_fe81), Some(0x9a));
        assert_eq!(machine.board.sh_wdt[1].read8(0xffff_fe81), Some(0xbc));
        assert_eq!(machine.board.sh_sci[0].read8(0xffff_fe01), Some(0x12));
        assert_eq!(machine.board.sh_sci[1].read8(0xffff_fe01), Some(0x34));
        let persistent = vec![0x77; BACKUP_RAM_SIZE];
        machine
            .write_persistent(ResourceKind::Storage, 0, &persistent)
            .unwrap();
        let mut output = vec![0; BACKUP_RAM_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut output)
            .unwrap();
        assert_eq!(output, persistent);
    }
}
