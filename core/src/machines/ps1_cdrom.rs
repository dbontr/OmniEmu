use std::collections::VecDeque;

#[cfg(test)]
use crate::cd_image::CUE_CONTAINER_MAGIC;
use crate::cd_image::{DiscImage, TrackKind};
use crate::resources::{ResourceBlob, RESOURCE_PENDING};
use crate::state::{StateReader, StateWriter};

const CPU_HZ: u64 = 33_868_800;
const SECTORS_PER_SECOND: u64 = 75;
const CYCLES_PER_SECTOR: u64 = CPU_HZ / SECTORS_PER_SECOND;
const SHORT_COMMAND_CYCLES: u64 = 0x1e00;
const GET_ID_CYCLES: u64 = 0x4a00;
const ACTIVE_PAUSE_CYCLES: u64 = CYCLES_PER_SECTOR * 5;
const ACTIVE_STOP_CYCLES: u64 = CYCLES_PER_SECTOR * 30;
const SEEK_CYCLES: u64 = CYCLES_PER_SECTOR;
const MAX_CD_AUDIO_FRAMES: usize = 9_408;
const XA_POS_FILTER: [i32; 4] = [0, 60, 115, 98];
const XA_NEG_FILTER: [i32; 4] = [0, 0, -52, -55];
const XA_ZIGZAG: [[i16; 7]; 29] = [
    [0, 0, 0, 0, -0x001, 0x002, -0x005],
    [0, 0, 0, -0x001, 0x003, -0x008, 0x011],
    [0, 0, -0x001, 0x003, -0x008, 0x010, -0x023],
    [0, -0x002, 0x003, -0x008, 0x011, -0x023, 0x046],
    [0, 0, -0x002, 0x006, -0x010, 0x02b, -0x017],
    [-0x002, 0x003, -0x005, 0x005, 0x00a, 0x01a, -0x044],
    [0x00a, -0x013, 0x01f, -0x01b, 0x06b, -0x0eb, 0x15b],
    [-0x022, 0x03c, -0x04a, 0x0a6, -0x16d, 0x27b, -0x347],
    [0x041, -0x04b, 0x0b3, -0x1a8, 0x350, -0x548, 0x80e],
    [-0x054, 0x0a2, -0x192, 0x372, -0x623, 0xafa, -0x1249],
    [0x034, -0x0e3, 0x2b1, -0x5bf, 0xbcd, -0x16fa, 0x3c07],
    [0x009, 0x132, -0x39e, 0x9b8, -0x1780, 0x53e0, 0x53e0],
    [-0x10a, -0x043, 0x4f8, -0x11b4, 0x6794, 0x3c07, -0x16fa],
    [0x400, -0x267, -0x5a6, 0x74bb, 0x234c, -0x1249, 0xafa],
    [-0xa78, 0xc9d, 0x7939, 0xc9d, -0xa78, 0x80e, -0x548],
    [0x234c, 0x74bb, -0x5a6, -0x267, 0x400, -0x347, 0x27b],
    [0x6794, -0x11b4, 0x4f8, -0x043, -0x10a, 0x15b, -0x0eb],
    [-0x1780, 0x9b8, -0x39e, 0x132, 0x009, -0x044, 0x01a],
    [0xbcd, -0x5bf, 0x2b1, -0x0e3, 0x034, -0x017, 0x02b],
    [-0x623, 0x372, -0x192, 0x0a2, -0x054, 0x046, -0x023],
    [0x350, -0x1a8, 0x0b3, -0x04b, 0x041, -0x023, 0x010],
    [-0x16d, 0x0a6, -0x04a, 0x03c, -0x022, 0x011, -0x008],
    [0x06b, -0x01b, 0x01f, -0x013, 0x00a, -0x005, 0x002],
    [0x00a, 0x005, -0x005, 0x003, -0x001, 0, 0],
    [-0x010, 0x006, -0x002, 0, 0, 0, 0],
    [0x011, -0x008, 0x003, -0x002, 0x001, 0, 0],
    [-0x008, 0x003, -0x001, 0, 0, 0, 0],
    [0x003, -0x001, 0, 0, 0, 0, 0],
    [-0x001, 0, 0, 0, 0, 0, 0],
];

#[derive(Clone)]
struct ScheduledResponse {
    cycles_remaining: u64,
    interrupt: u8,
    status_prefix: u8,
    status_set: u8,
    status_clear: u8,
    payload: Vec<u8>,
}

pub struct Ps1CdRom {
    disc: Option<DiscImage>,
    index: u8,
    parameters: VecDeque<u8>,
    responses: VecDeque<u8>,
    data: VecDeque<u8>,
    interrupt_enable: u8,
    interrupt_flags: u8,
    mode: u8,
    filter_file: u8,
    filter_channel: u8,
    status_byte: u8,
    target_lba: u32,
    current_lba: u32,
    subq_lba: u32,
    reading: bool,
    playing_audio: bool,
    muted: bool,
    xa_muted: bool,
    xa_active: bool,
    pending_volume: [u8; 4],
    active_volume: [u8; 4],
    audio: VecDeque<[i16; 2]>,
    xa_history: [[i32; 2]; 2],
    xa_ring: [[i32; 32]; 2],
    xa_ring_pos: u8,
    xa_sixstep: u8,
    request_data: bool,
    cycle_phase: u64,
    scheduled_response: Option<ScheduledResponse>,
    pending_command: Option<u8>,
}

impl Ps1CdRom {
    pub fn new(disc: Option<ResourceBlob>) -> Result<Self, String> {
        let disc = disc.map(DiscImage::new).transpose()?;
        Ok(Self {
            disc,
            index: 0,
            parameters: VecDeque::with_capacity(16),
            responses: VecDeque::with_capacity(16),
            data: VecDeque::with_capacity(4096),
            interrupt_enable: 0,
            interrupt_flags: 0,
            mode: 0,
            filter_file: 0,
            filter_channel: 0,
            status_byte: 0x02,
            target_lba: 0,
            current_lba: 0,
            subq_lba: 0,
            reading: false,
            playing_audio: false,
            muted: false,
            xa_muted: false,
            xa_active: false,
            pending_volume: [0x80, 0, 0x80, 0],
            active_volume: [0x80, 0, 0x80, 0],
            audio: VecDeque::with_capacity(MAX_CD_AUDIO_FRAMES),
            xa_history: [[0; 2]; 2],
            xa_ring: [[0; 32]; 2],
            xa_ring_pos: 0,
            xa_sixstep: 6,
            request_data: false,
            cycle_phase: 0,
            scheduled_response: None,
            pending_command: None,
        })
    }
    pub fn reset(&mut self) {
        let disc = self.disc.take();
        *self = Self {
            disc,
            index: 0,
            parameters: VecDeque::with_capacity(16),
            responses: VecDeque::with_capacity(16),
            data: VecDeque::with_capacity(4096),
            interrupt_enable: 0,
            interrupt_flags: 0,
            mode: 0,
            filter_file: 0,
            filter_channel: 0,
            status_byte: 0x02,
            target_lba: 0,
            current_lba: 0,
            subq_lba: 0,
            reading: false,
            playing_audio: false,
            muted: false,
            xa_muted: false,
            xa_active: false,
            pending_volume: [0x80, 0, 0x80, 0],
            active_volume: [0x80, 0, 0x80, 0],
            audio: VecDeque::with_capacity(MAX_CD_AUDIO_FRAMES),
            xa_history: [[0; 2]; 2],
            xa_ring: [[0; 32]; 2],
            xa_ring_pos: 0,
            xa_sixstep: 6,
            request_data: false,
            cycle_phase: 0,
            scheduled_response: None,
            pending_command: None,
        };
    }

    pub fn has_disc(&self) -> bool {
        self.disc.is_some()
    }

    pub fn irq_pending(&self) -> bool {
        self.interrupt_enable & self.interrupt_flags & 0x1f != 0
    }

    fn fifo_status(&self) -> u8 {
        let mut value = self.index & 3;
        if self.xa_active {
            value |= 1 << 2;
        }
        if self.parameters.is_empty() {
            value |= 1 << 3;
        }
        if self.parameters.len() < 16 {
            value |= 1 << 4;
        }
        if !self.responses.is_empty() {
            value |= 1 << 5;
        }
        if self.request_data && !self.data.is_empty() {
            value |= 1 << 6;
        }
        value
    }
    pub fn read_register(&mut self, offset: u8) -> u8 {
        match offset & 3 {
            0 => self.fifo_status(),
            1 => self.responses.pop_front().unwrap_or(0),
            2 => {
                if self.request_data {
                    self.data.pop_front().unwrap_or(0)
                } else {
                    0
                }
            }
            3 => match self.index {
                0 | 2 => self.interrupt_enable | 0xe0,
                _ => self.interrupt_flags | 0xe0,
            },
            _ => 0,
        }
    }

    pub fn write_register(&mut self, offset: u8, value: u8) {
        match offset & 3 {
            0 => self.index = value & 3,
            1 => match self.index {
                0 => self.submit_command(value),
                3 => self.pending_volume[2] = value,
                _ => {}
            },
            2 => match self.index {
                0 => {
                    if self.parameters.len() < 16 {
                        self.parameters.push_back(value);
                    }
                }
                1 => self.interrupt_enable = value & 0x1f,
                2 => self.pending_volume[0] = value,
                3 => self.pending_volume[3] = value,
                _ => {}
            },
            3 => match self.index {
                0 => {
                    self.request_data = value & 0x80 != 0;
                    if value & 0x40 != 0 {
                        self.data.clear();
                    }
                }
                1 => {
                    self.interrupt_flags &= !(value & 0x1f);
                    if value & 0x40 != 0 {
                        self.parameters.clear();
                    }
                    self.service_scheduled_response();
                    self.service_pending_command();
                }
                2 => self.pending_volume[1] = value,
                3 => {
                    self.xa_muted = value & 1 != 0;
                    if value & (1 << 5) != 0 {
                        self.active_volume = self.pending_volume;
                    }
                }
                _ => {}
            },
            _ => {}
        }
    }
    fn raise_interrupt(&mut self, code: u8) {
        self.interrupt_flags = code & 0x1f;
    }

    fn push_status(&mut self, interrupt: u8) {
        self.responses.clear();
        self.responses.push_back(self.status_byte);
        self.raise_interrupt(interrupt);
    }

    fn schedule_response(
        &mut self,
        cycles: u64,
        interrupt: u8,
        status_prefix: u8,
        status_set: u8,
        status_clear: u8,
        payload: Vec<u8>,
    ) {
        self.scheduled_response = Some(ScheduledResponse {
            cycles_remaining: cycles,
            interrupt,
            status_prefix,
            status_set,
            status_clear,
            payload,
        });
    }

    fn service_scheduled_response(&mut self) {
        if self.interrupt_flags != 0 {
            return;
        }
        let ready = self
            .scheduled_response
            .as_ref()
            .is_some_and(|response| response.cycles_remaining == 0);
        if !ready {
            return;
        }
        let response = self
            .scheduled_response
            .take()
            .expect("scheduled response readiness checked above");
        self.status_byte = (self.status_byte | response.status_set) & !response.status_clear;
        self.responses.clear();
        match response.status_prefix {
            1 => self.responses.push_back(self.status_byte),
            2 => self.responses.push_back(self.status_byte | 0x01),
            _ => {}
        }
        self.responses.extend(response.payload);
        self.raise_interrupt(response.interrupt);
    }

    fn advance_scheduled_response(&mut self, cycles: u32) {
        if let Some(response) = self.scheduled_response.as_mut() {
            response.cycles_remaining = response.cycles_remaining.saturating_sub(u64::from(cycles));
        }
        self.service_scheduled_response();
        self.service_pending_command();
    }

    fn update_subq(&mut self, lba: u32) {
        if self
            .disc
            .as_ref()
            .is_some_and(|disc| !disc.subq_crc_bad(lba))
        {
            self.subq_lba = lba;
        }
    }

    fn submit_command(&mut self, command: u8) {
        if self.interrupt_flags != 0 || self.scheduled_response.is_some() {
            if self.pending_command.is_none() {
                self.pending_command = Some(command);
            }
            return;
        }
        self.command(command);
    }

    fn service_pending_command(&mut self) {
        if self.interrupt_flags != 0 || self.scheduled_response.is_some() {
            return;
        }
        if let Some(command) = self.pending_command.take() {
            self.command(command);
        }
    }

    fn command(&mut self, command: u8) {
        match command {
            0x01 => self.push_status(3),
            0x02 => self.setloc(),
            0x03 => self.play_audio(),
            0x04 => self.fast_scan(1),
            0x05 => self.fast_scan(-1),
            0x06 | 0x1b => self.read_n(),
            0x07 => self.motor_on(),
            0x08 => self.stop(),
            0x09 => self.pause(),
            0x0a => self.init_command(),
            0x0b => {
                self.muted = true;
                self.push_status(3);
            }
            0x0c => {
                self.muted = false;
                self.push_status(3);
            }
            0x0d => self.setfilter(),
            0x0e => self.setmode(),
            0x0f => self.getparam(),
            0x10 => self.getloc_l(),
            0x11 => self.getloc_p(),
            0x12 => self.set_session(),
            0x13 => self.get_tn(),
            0x14 => self.get_td(),
            0x15 | 0x16 => self.seek(),
            0x19 => self.test(),
            0x1a => self.get_id(),
            0x1c => self.reset_command(),
            0x1d => self.get_q(),
            0x1e => self.read_toc(),
            _ => self.command_error(),
        }
    }
    fn command_error(&mut self) {
        self.command_error_code(0x20);
    }

    fn command_error_code(&mut self, code: u8) {
        self.responses.clear();
        self.responses.push_back(self.status_byte | 0x01);
        self.responses.push_back(code);
        self.raise_interrupt(5);
    }

    fn fast_scan(&mut self, direction: i32) {
        let Some(disc) = self.disc.as_ref() else {
            self.command_error_code(0x80);
            return;
        };
        if !self.playing_audio {
            self.command_error_code(0x80);
            return;
        }
        let delta = 75u32;
        self.current_lba = if direction >= 0 {
            self.current_lba
                .saturating_add(delta)
                .min(disc.sectors.saturating_sub(1))
        } else {
            self.current_lba.saturating_sub(delta)
        };
        self.update_subq(self.current_lba);
        self.push_status(3);
    }

    fn motor_on(&mut self) {
        if self.disc.is_none() {
            self.command_error_code(0x80);
            return;
        }
        self.status_byte |= 0x02;
        self.push_status(3);
        self.schedule_response(SHORT_COMMAND_CYCLES, 2, 1, 0x02, 0, Vec::new());
    }

    fn stop(&mut self) {
        let delay = if self.reading || self.playing_audio {
            ACTIVE_STOP_CYCLES
        } else {
            SHORT_COMMAND_CYCLES
        };
        self.reading = false;
        self.playing_audio = false;
        self.xa_active = false;
        self.status_byte &= !(0x20 | 0x40 | 0x80);
        self.data.clear();
        self.audio.clear();
        self.push_status(3);
        self.schedule_response(delay, 2, 1, 0, 0x02, Vec::new());
    }

    fn pause(&mut self) {
        let delay = if self.reading || self.playing_audio {
            ACTIVE_PAUSE_CYCLES
        } else {
            SHORT_COMMAND_CYCLES
        };
        self.reading = false;
        self.playing_audio = false;
        self.xa_active = false;
        self.status_byte &= !(0x20 | 0x80);
        self.push_status(3);
        self.schedule_response(delay, 2, 1, 0, 0, Vec::new());
    }

    fn init_command(&mut self) {
        self.mode = 0;
        self.filter_file = 0;
        self.filter_channel = 0;
        self.reading = false;
        self.playing_audio = false;
        self.muted = false;
        self.xa_muted = false;
        self.status_byte &= !(0x20 | 0x40 | 0x80);
        if self.disc.is_some() {
            self.status_byte |= 0x02;
        }
        self.data.clear();
        self.audio.clear();
        self.reset_xa_decoder();
        self.push_status(3);
        self.schedule_response(SHORT_COMMAND_CYCLES, 2, 1, 0, 0, Vec::new());
    }

    fn set_session(&mut self) {
        if self.disc.is_none() {
            self.command_error_code(0x80);
            return;
        }
        let session = self.parameters.pop_front().unwrap_or(0);
        if session != 1 {
            self.command_error_code(0x10);
            return;
        }
        self.status_byte |= 0x40;
        self.push_status(3);
        self.schedule_response(SEEK_CYCLES, 2, 1, 0, 0x40, Vec::new());
    }

    fn reset_command(&mut self) {
        self.mode = 0;
        self.filter_file = 0;
        self.filter_channel = 0;
        self.reading = false;
        self.playing_audio = false;
        self.muted = false;
        self.xa_muted = false;
        self.data.clear();
        self.audio.clear();
        self.reset_xa_decoder();
        self.scheduled_response = None;
        self.status_byte = if self.disc.is_some() { 0x02 } else { 0 };
        self.push_status(3);
    }

    fn bcd(value: u8) -> Option<u8> {
        let high = value >> 4;
        let low = value & 0x0f;
        if high > 9 || low > 9 {
            None
        } else {
            Some(high * 10 + low)
        }
    }

    fn to_bcd(value: u32) -> u8 {
        let value = value.min(99) as u8;
        ((value / 10) << 4) | (value % 10)
    }

    fn setloc(&mut self) {
        let Some(minutes) = self.parameters.pop_front().and_then(Self::bcd) else {
            self.command_error();
            return;
        };
        let Some(seconds) = self.parameters.pop_front().and_then(Self::bcd) else {
            self.command_error();
            return;
        };
        let Some(frames) = self.parameters.pop_front().and_then(Self::bcd) else {
            self.command_error();
            return;
        };
        let absolute = u32::from(minutes) * 60 * 75 + u32::from(seconds) * 75 + u32::from(frames);
        self.target_lba = absolute.saturating_sub(150);
        self.push_status(3);
    }

    fn play_audio(&mut self) {
        let Some(disc) = self.disc.as_ref() else {
            self.command_error();
            return;
        };
        let track_parameter = self.parameters.pop_front().unwrap_or(0);
        if track_parameter == 0 {
            self.current_lba = self.target_lba;
        } else {
            let Some(track) = Self::bcd(track_parameter) else {
                self.command_error();
                return;
            };
            let Some(start_lba) = disc.track_start(track) else {
                self.command_error();
                return;
            };
            self.current_lba = start_lba;
        }
        self.update_subq(self.current_lba);
        self.reading = false;
        self.playing_audio = true;
        self.xa_active = false;
        self.data.clear();
        self.audio.clear();
        self.cycle_phase = CYCLES_PER_SECTOR;
        self.status_byte = (self.status_byte & !0x20) | 0x80;
        self.push_status(3);
    }

    fn read_n(&mut self) {
        if self.disc.is_none() {
            self.command_error();
            return;
        }
        self.current_lba = self.target_lba;
        self.update_subq(self.current_lba);
        self.reading = true;
        self.playing_audio = false;
        self.audio.clear();
        self.reset_xa_decoder();
        self.cycle_phase = CYCLES_PER_SECTOR;
        self.status_byte = (self.status_byte & !0x80) | 0x20;
        self.push_status(3);
    }
    fn setfilter(&mut self) {
        let Some(file) = self.parameters.pop_front() else {
            self.command_error();
            return;
        };
        let Some(channel) = self.parameters.pop_front() else {
            self.command_error();
            return;
        };
        if self.filter_file != file || self.filter_channel != channel {
            self.reset_xa_decoder();
        }
        self.filter_file = file;
        self.filter_channel = channel;
        self.push_status(3);
    }

    fn setmode(&mut self) {
        let Some(mode) = self.parameters.pop_front() else {
            self.command_error();
            return;
        };
        if mode & 0x40 == 0 {
            self.xa_active = false;
        }
        self.mode = mode;
        self.push_status(3);
    }

    fn getparam(&mut self) {
        self.responses.clear();
        self.responses.push_back(self.status_byte);
        self.responses.push_back(self.mode);
        self.responses.push_back(0);
        self.responses.push_back(self.filter_file);
        self.responses.push_back(self.filter_channel);
        self.raise_interrupt(3);
    }

    fn getloc_l(&mut self) {
        if self
            .disc
            .as_ref()
            .and_then(|disc| disc.track_for_lba(self.current_lba))
            .is_some_and(|track| track.kind == TrackKind::Audio)
        {
            self.command_error();
            return;
        }
        self.responses.clear();
        let absolute = self.current_lba.saturating_add(150);
        let minute = absolute / (60 * 75);
        let second = (absolute / 75) % 60;
        let frame = absolute % 75;
        self.responses.extend([
            Self::to_bcd(minute),
            Self::to_bcd(second),
            Self::to_bcd(frame),
            0x02,
            0x00,
            0x00,
            0x00,
            0x00,
        ]);
        self.raise_interrupt(3);
    }
    fn getloc_p(&mut self) {
        self.responses.clear();
        let position_lba = self.subq_lba;
        let absolute = position_lba.saturating_add(150);
        let (track_number, index, relative) = self
            .disc
            .as_ref()
            .and_then(|disc| disc.track_for_lba(position_lba))
            .map_or((0xaau8, 1u8, 0u32), |track| {
                if position_lba >= track.start_lba {
                    (track.number, 1, position_lba - track.start_lba)
                } else {
                    (track.number, 0, track.start_lba - position_lba)
                }
            });
        self.responses.extend([
            if track_number == 0xaa {
                0xaa
            } else {
                Self::to_bcd(u32::from(track_number))
            },
            Self::to_bcd(u32::from(index)),
            Self::to_bcd(relative / (60 * 75)),
            Self::to_bcd((relative / 75) % 60),
            Self::to_bcd(relative % 75),
            Self::to_bcd(absolute / (60 * 75)),
            Self::to_bcd((absolute / 75) % 60),
            Self::to_bcd(absolute % 75),
        ]);
        self.raise_interrupt(3);
    }

    fn get_tn(&mut self) {
        let Some(disc) = self.disc.as_ref() else {
            self.command_error();
            return;
        };
        self.responses.clear();
        self.responses.extend([
            self.status_byte,
            Self::to_bcd(u32::from(disc.first_track())),
            Self::to_bcd(u32::from(disc.last_track())),
        ]);
        self.raise_interrupt(3);
    }

    fn get_td(&mut self) {
        let track_parameter = self.parameters.pop_front().unwrap_or(0);
        let Some(disc) = self.disc.as_ref() else {
            self.command_error();
            return;
        };
        let lba = if track_parameter == 0 {
            disc.sectors
        } else {
            let Some(track) = Self::bcd(track_parameter) else {
                self.command_error();
                return;
            };
            let Some(lba) = disc.track_start(track) else {
                self.command_error();
                return;
            };
            lba
        };
        let absolute = lba.saturating_add(150);
        self.responses.clear();
        self.responses.extend([
            self.status_byte,
            Self::to_bcd(absolute / (60 * 75)),
            Self::to_bcd((absolute / 75) % 60),
        ]);
        self.raise_interrupt(3);
    }
    fn seek(&mut self) {
        if self.disc.is_none() {
            self.command_error_code(0x80);
            return;
        }
        if self
            .disc
            .as_ref()
            .is_some_and(|disc| self.target_lba >= disc.sectors)
        {
            self.command_error_code(0x10);
            return;
        }
        self.current_lba = self.target_lba;
        self.update_subq(self.current_lba);
        self.reading = false;
        self.playing_audio = false;
        self.status_byte &= !(0x20 | 0x80);
        self.status_byte |= 0x40;
        self.push_status(3);
        self.schedule_response(SEEK_CYCLES, 2, 1, 0, 0x40, Vec::new());
    }

    fn test(&mut self) {
        let sub = self.parameters.pop_front().unwrap_or(0);
        self.responses.clear();
        match sub {
            0x20 => self.responses.extend([0x98, 0x06, 0x10, 0xc3]),
            _ => self.responses.push_back(self.status_byte),
        }
        self.raise_interrupt(3);
    }

    fn get_id(&mut self) {
        self.push_status(3);
        if self.disc.is_some() {
            self.schedule_response(
                GET_ID_CYCLES,
                2,
                1,
                0,
                0,
                vec![0x00, 0x20, 0x00, b'S', b'C', b'E', b'A'],
            );
        } else {
            self.schedule_response(
                GET_ID_CYCLES,
                5,
                2,
                0,
                0,
                vec![0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
            );
        }
    }

    fn get_q(&mut self) {
        let adr = self.parameters.pop_front().unwrap_or(0);
        let point = self.parameters.pop_front().unwrap_or(0);
        let Some(disc) = self.disc.as_ref() else {
            self.command_error_code(0x80);
            return;
        };
        if adr != 1 {
            self.command_error_code(0x10);
            return;
        }

        let first_track = disc.first_track();
        let last_track = disc.last_track();
        let (control_adr, pmin, psec, pframe) = match point {
            0xa0 => (0x41, Self::to_bcd(u32::from(first_track)), 0x20, 0),
            0xa1 => (0x41, Self::to_bcd(u32::from(last_track)), 0, 0),
            0xa2 => {
                let absolute = disc.sectors.saturating_add(150);
                (
                    0x41,
                    Self::to_bcd(absolute / (60 * 75)),
                    Self::to_bcd((absolute / 75) % 60),
                    Self::to_bcd(absolute % 75),
                )
            }
            _ => {
                let Some(track_number) = Self::bcd(point) else {
                    self.command_error_code(0x10);
                    return;
                };
                let Some(track) = disc
                    .tracks
                    .iter()
                    .find(|track| track.number == track_number)
                else {
                    self.command_error_code(0x10);
                    return;
                };
                let absolute = track.start_lba.saturating_add(150);
                (
                    if track.kind == TrackKind::Data {
                        0x41
                    } else {
                        0x01
                    },
                    Self::to_bcd(absolute / (60 * 75)),
                    Self::to_bcd((absolute / 75) % 60),
                    Self::to_bcd(absolute % 75),
                )
            }
        };
        let payload = vec![
            control_adr,
            0x00,
            point,
            0x00,
            0x00,
            0x00,
            0x00,
            pmin,
            psec,
            pframe,
            0x00,
        ];
        self.push_status(3);
        self.schedule_response(SEEK_CYCLES, 2, 0, 0, 0, payload);
    }

    fn read_toc(&mut self) {
        if self.disc.is_none() {
            self.command_error_code(0x80);
            return;
        }
        self.push_status(3);
        self.schedule_response(CPU_HZ, 2, 1, 0, 0, Vec::new());
    }

    fn reset_xa_decoder(&mut self) {
        self.xa_active = false;
        self.xa_history = [[0; 2]; 2];
        self.xa_ring = [[0; 32]; 2];
        self.xa_ring_pos = 0;
        self.xa_sixstep = 6;
    }

    fn push_cd_audio_frame(&mut self, left: i32, right: i32, xa: bool) {
        let (out_left, out_right) = if self.muted || (xa && self.xa_muted) {
            (0, 0)
        } else {
            (
                (left * i32::from(self.active_volume[0])
                    + right * i32::from(self.active_volume[3]))
                    / 0x80,
                (right * i32::from(self.active_volume[2])
                    + left * i32::from(self.active_volume[1]))
                    / 0x80,
            )
        };
        self.audio.push_back([
            out_left.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16,
            out_right.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16,
        ]);
    }

    fn decode_xa_unit(
        &mut self,
        group: &[u8; 128],
        unit: usize,
        channel: usize,
        eight_bit: bool,
    ) -> [i16; 28] {
        let parameter_index = if eight_bit || unit < 4 {
            unit
        } else {
            unit + 4
        };
        let parameter = group[parameter_index];
        let filter = usize::from((parameter >> 4) & 3);
        let range = u32::from(parameter & 0x0f).min(if eight_bit { 8 } else { 12 });
        let mut decoded = [0i16; 28];
        for (sample_index, sample) in decoded.iter_mut().enumerate() {
            let source = if eight_bit {
                i32::from(group[16 + sample_index * 4 + unit] as i8) << 8
            } else {
                let byte = group[16 + sample_index * 4 + unit / 2];
                let nibble = if unit & 1 == 0 {
                    byte & 0x0f
                } else {
                    byte >> 4
                };
                i32::from((nibble << 4) as i8) << 8
            };
            let source = source >> range;
            let old = self.xa_history[channel][0];
            let older = self.xa_history[channel][1];
            let predictor = (old * XA_POS_FILTER[filter] + older * XA_NEG_FILTER[filter] + 32) / 64;
            let value = (source + predictor).clamp(i32::from(i16::MIN), i32::from(i16::MAX));
            self.xa_history[channel] = [value, old];
            *sample = value as i16;
        }
        decoded
    }

    fn zigzag_sample(&self, channel: usize, table: usize) -> i32 {
        let position = usize::from(self.xa_ring_pos);
        let mut sum = 0i64;
        for (offset, coefficients) in XA_ZIGZAG.iter().enumerate() {
            let ring_index = position.wrapping_sub(offset + 1) & 31;
            sum += (i64::from(self.xa_ring[channel][ring_index]) * i64::from(coefficients[table]))
                / 0x8000;
        }
        sum.clamp(i64::from(i16::MIN), i64::from(i16::MAX)) as i32
    }

    fn push_xa_37800_frame(&mut self, left: i16, right: i16) {
        let index = usize::from(self.xa_ring_pos);
        self.xa_ring[0][index] = i32::from(left);
        self.xa_ring[1][index] = i32::from(right);
        self.xa_ring_pos = self.xa_ring_pos.wrapping_add(1) & 31;
        self.xa_sixstep = self.xa_sixstep.saturating_sub(1);
        if self.xa_sixstep != 0 {
            return;
        }
        self.xa_sixstep = 6;
        for table in 0..7 {
            let left = self.zigzag_sample(0, table);
            let right = self.zigzag_sample(1, table);
            self.push_cd_audio_frame(left, right, true);
        }
    }

    fn push_xa_native_frame(&mut self, left: i16, right: i16, half_rate: bool) {
        self.push_xa_37800_frame(left, right);
        if half_rate {
            self.push_xa_37800_frame(left, right);
        }
    }

    fn decode_xa_sector(&mut self, payload: &[u8], coding_info: u8) -> Result<(), String> {
        if payload.len() < 2304 {
            return Err("PlayStation XA-ADPCM sector payload is truncated".into());
        }
        let stereo = coding_info & 1 != 0;
        let half_rate = coding_info & 4 != 0;
        let eight_bit = coding_info & 0x10 != 0;
        let units = if eight_bit { 4 } else { 8 };
        for group_bytes in payload[..2304].as_chunks::<128>().0 {
            if stereo {
                for pair in 0..(units / 2) {
                    let left = self.decode_xa_unit(group_bytes, pair * 2, 0, eight_bit);
                    let right = self.decode_xa_unit(group_bytes, pair * 2 + 1, 1, eight_bit);
                    for sample in 0..28 {
                        self.push_xa_native_frame(left[sample], right[sample], half_rate);
                    }
                }
            } else {
                for unit in 0..units {
                    let mono = self.decode_xa_unit(group_bytes, unit, 0, eight_bit);
                    for sample in mono {
                        self.push_xa_native_frame(sample, sample, half_rate);
                    }
                }
            }
        }
        Ok(())
    }

    pub fn tick_cpu_cycles(&mut self, cycles: u32) {
        self.advance_scheduled_response(cycles);
        if !self.reading && !self.playing_audio {
            return;
        }
        self.cycle_phase = self.cycle_phase.saturating_add(u64::from(cycles));
        let sector_cycles = if self.reading && self.mode & 0x80 != 0 {
            CYCLES_PER_SECTOR / 2
        } else {
            CYCLES_PER_SECTOR
        };
        while self.cycle_phase >= sector_cycles {
            if (self.reading && self.data.len() > 4096) || self.audio.len() >= MAX_CD_AUDIO_FRAMES {
                break;
            }
            let result = if self.playing_audio {
                self.load_audio_sector()
            } else {
                self.load_sector()
            };
            match result {
                Ok(()) => self.cycle_phase -= sector_cycles,
                Err(error) if error == RESOURCE_PENDING => break,
                Err(_) => {
                    self.reading = false;
                    self.playing_audio = false;
                    self.status_byte &= !(0x20 | 0x80);
                    self.command_error();
                    break;
                }
            }
            if !self.reading && !self.playing_audio {
                break;
            }
        }
    }

    fn load_sector(&mut self) -> Result<(), String> {
        let raw_sector = self
            .disc
            .as_ref()
            .ok_or_else(|| "no PlayStation disc is loaded".to_string())?
            .is_raw_sector(self.current_lba);
        if raw_sector {
            let mut raw = [0u8; 2352];
            self.disc
                .as_ref()
                .expect("disc presence checked above")
                .read_raw_sector(self.current_lba, &mut raw)?;
            let file = raw[16];
            let channel = raw[17];
            let submode = raw[18];
            let coding_info = raw[19];
            let xa_audio = raw[15] == 2 && submode & 0x44 == 0x44 && self.mode & 0x40 != 0;
            if xa_audio {
                self.update_subq(self.current_lba);
                self.current_lba = self.current_lba.saturating_add(1);
                let filter_matches = self.mode & 0x08 == 0
                    || (file == self.filter_file && channel == self.filter_channel);
                if filter_matches {
                    self.xa_active = true;
                    self.decode_xa_sector(&raw[24..24 + 2304], coding_info)?;
                }
                return Ok(());
            }
        }

        let mut sector = [0u8; 2048];
        self.disc
            .as_ref()
            .expect("disc presence checked above")
            .read_user_sector(self.current_lba, &mut sector)?;
        self.update_subq(self.current_lba);
        self.data.extend(sector);
        self.current_lba = self.current_lba.saturating_add(1);
        self.responses.clear();
        self.responses.push_back(self.status_byte);
        self.raise_interrupt(1);
        Ok(())
    }

    fn load_audio_sector(&mut self) -> Result<(), String> {
        let sectors = self
            .disc
            .as_ref()
            .ok_or_else(|| "no PlayStation disc is loaded".to_string())?
            .sectors;
        if self.current_lba >= sectors {
            self.playing_audio = false;
            self.status_byte &= !0x80;
            self.push_status(4);
            return Ok(());
        }

        let current_track = self
            .disc
            .as_ref()
            .and_then(|disc| disc.track_number(self.current_lba));
        let mut sector = [0u8; 2352];
        self.disc
            .as_ref()
            .expect("disc presence checked above")
            .read_raw_sector(self.current_lba, &mut sector)?;
        self.update_subq(self.current_lba);
        for frame in sector.as_chunks::<4>().0 {
            let left = i32::from(i16::from_le_bytes([frame[0], frame[1]]));
            let right = i32::from(i16::from_le_bytes([frame[2], frame[3]]));
            self.push_cd_audio_frame(left, right, false);
        }
        self.current_lba = self.current_lba.saturating_add(1);
        let next_track = self
            .disc
            .as_ref()
            .and_then(|disc| disc.track_number(self.current_lba));
        if self.mode & 0x02 != 0 && current_track.is_some() && next_track != current_track {
            self.playing_audio = false;
            self.status_byte &= !0x80;
            self.push_status(4);
        }
        Ok(())
    }

    pub fn drain_audio(&mut self) -> Vec<[i16; 2]> {
        self.audio.drain(..).collect()
    }

    pub fn dma_words_available(&self) -> usize {
        if self.request_data {
            self.data.len() / 4
        } else {
            0
        }
    }

    pub fn dma_read_word(&mut self) -> u32 {
        let mut bytes = [0u8; 4];
        for byte in &mut bytes {
            *byte = self.data.pop_front().unwrap_or(0);
        }
        u32::from_le_bytes(bytes)
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.u8(self.index);
        Self::save_fifo(out, &self.parameters);
        Self::save_fifo(out, &self.responses);
        Self::save_fifo(out, &self.data);
        out.u8(self.interrupt_enable);
        out.u8(self.interrupt_flags);
        out.u8(self.mode);
        out.u8(self.filter_file);
        out.u8(self.filter_channel);
        out.u8(self.status_byte);
        out.u32(self.target_lba);
        out.u32(self.current_lba);
        out.u32(self.subq_lba);
        out.u8(self.reading as u8);
        out.u8(self.playing_audio as u8);
        out.u8(self.muted as u8);
        out.u8(self.xa_muted as u8);
        out.u8(self.xa_active as u8);
        for volume in self.pending_volume {
            out.u8(volume);
        }
        for volume in self.active_volume {
            out.u8(volume);
        }
        out.u32(self.audio.len() as u32);
        for frame in &self.audio {
            out.u16(frame[0] as u16);
            out.u16(frame[1] as u16);
        }
        for history in self.xa_history {
            out.u32(history[0] as u32);
            out.u32(history[1] as u32);
        }
        for channel in self.xa_ring {
            for sample in channel {
                out.u32(sample as u32);
            }
        }
        out.u8(self.xa_ring_pos);
        out.u8(self.xa_sixstep);
        out.u8(self.request_data as u8);
        out.u64(self.cycle_phase);
        if let Some(response) = &self.scheduled_response {
            out.u8(1);
            out.u64(response.cycles_remaining);
            out.u8(response.interrupt);
            out.u8(response.status_prefix);
            out.u8(response.status_set);
            out.u8(response.status_clear);
            out.u32(response.payload.len() as u32);
            for byte in &response.payload {
                out.u8(*byte);
            }
        } else {
            out.u8(0);
        }
        match self.pending_command {
            Some(command) => {
                out.u8(1);
                out.u8(command);
            }
            None => out.u8(0),
        }
    }

    fn save_fifo(out: &mut StateWriter, fifo: &VecDeque<u8>) {
        out.u32(fifo.len() as u32);
        for value in fifo {
            out.u8(*value);
        }
    }
    fn load_fifo(input: &mut StateReader<'_>, max: usize) -> Result<VecDeque<u8>, String> {
        let len = input.u32()? as usize;
        if len > max {
            return Err("invalid PlayStation CD-ROM FIFO length".into());
        }
        let mut fifo = VecDeque::with_capacity(len);
        for _ in 0..len {
            fifo.push_back(input.u8()?);
        }
        Ok(fifo)
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.index = input.u8()? & 3;
        self.parameters = Self::load_fifo(input, 16)?;
        self.responses = Self::load_fifo(input, 64)?;
        self.data = Self::load_fifo(input, 8192)?;
        self.interrupt_enable = input.u8()? & 0x1f;
        self.interrupt_flags = input.u8()? & 0x1f;
        self.mode = input.u8()?;
        self.filter_file = input.u8()?;
        self.filter_channel = input.u8()?;
        self.status_byte = input.u8()?;
        self.target_lba = input.u32()?;
        self.current_lba = input.u32()?;
        self.subq_lba = input.u32()?;
        self.reading = input.u8()? != 0;
        self.playing_audio = input.u8()? != 0;
        self.muted = input.u8()? != 0;
        self.xa_muted = input.u8()? != 0;
        self.xa_active = input.u8()? != 0;
        for volume in &mut self.pending_volume {
            *volume = input.u8()?;
        }
        for volume in &mut self.active_volume {
            *volume = input.u8()?;
        }
        let audio_len = input.u32()? as usize;
        if audio_len > MAX_CD_AUDIO_FRAMES {
            return Err("invalid PlayStation CD audio queue length".into());
        }
        self.audio.clear();
        for _ in 0..audio_len {
            self.audio
                .push_back([input.u16()? as i16, input.u16()? as i16]);
        }
        for history in &mut self.xa_history {
            history[0] = input.u32()? as i32;
            history[1] = input.u32()? as i32;
        }
        for channel in &mut self.xa_ring {
            for sample in channel {
                *sample = input.u32()? as i32;
            }
        }
        self.xa_ring_pos = input.u8()? & 31;
        self.xa_sixstep = input.u8()?;
        if !(1..=6).contains(&self.xa_sixstep) {
            return Err("invalid PlayStation XA resampler phase".into());
        }
        self.request_data = input.u8()? != 0;
        let sector_cycles = if self.reading && self.mode & 0x80 != 0 {
            CYCLES_PER_SECTOR / 2
        } else {
            CYCLES_PER_SECTOR
        };
        self.cycle_phase = input.u64()? % sector_cycles;
        self.scheduled_response = if input.u8()? != 0 {
            let cycles_remaining = input.u64()?;
            let interrupt = input.u8()? & 0x1f;
            let status_prefix = input.u8()?;
            if status_prefix > 2 {
                return Err("invalid PlayStation CD-ROM scheduled status prefix".into());
            }
            let status_set = input.u8()?;
            let status_clear = input.u8()?;
            let payload_len = input.u32()? as usize;
            if payload_len > 32 {
                return Err("invalid PlayStation CD-ROM scheduled response length".into());
            }
            let mut payload = Vec::with_capacity(payload_len);
            for _ in 0..payload_len {
                payload.push(input.u8()?);
            }
            Some(ScheduledResponse {
                cycles_remaining,
                interrupt,
                status_prefix,
                status_set,
                status_clear,
                payload,
            })
        } else {
            None
        };
        self.pending_command = if input.u8()? != 0 {
            Some(input.u8()?)
        } else {
            None
        };
        if (self.reading || self.playing_audio) && self.disc.is_none() {
            return Err("PlayStation state expects a disc but no disc is loaded".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn iso(sectors: usize) -> ResourceBlob {
        let mut bytes = vec![0u8; sectors * 2048];
        for sector in 0..sectors {
            bytes[sector * 2048] = sector as u8;
            bytes[sector * 2048 + 1] = 0xa5;
        }
        ResourceBlob::from_bytes(&bytes)
    }

    fn cdda(sectors: usize, left: i16, right: i16) -> ResourceBlob {
        let mut bytes = vec![0u8; sectors * 2352];
        for frame in bytes.as_chunks_mut::<4>().0 {
            frame[..2].copy_from_slice(&left.to_le_bytes());
            frame[2..].copy_from_slice(&right.to_le_bytes());
        }
        ResourceBlob::from_bytes(&bytes)
    }

    fn xa_sector(file: u8, channel: u8, coding_info: u8, sample_byte: u8) -> ResourceBlob {
        let mut bytes = vec![0u8; 2352];
        bytes[0] = 0;
        bytes[1..11].fill(0xff);
        bytes[11] = 0;
        bytes[15] = 2;
        let subheader = [file, channel, 0x64, coding_info];
        bytes[16..20].copy_from_slice(&subheader);
        bytes[20..24].copy_from_slice(&subheader);
        for group in 0..18 {
            let start = 24 + group * 128;
            bytes[start + 16..start + 128].fill(sample_byte);
        }
        ResourceBlob::from_bytes(&bytes)
    }

    fn cue_container_bytes(cue: &str, files: &[(&str, Vec<u8>)]) -> Vec<u8> {
        let cue_bytes = cue.as_bytes();
        let directory_len: usize = files.iter().map(|(name, _)| 2 + name.len() + 16).sum();
        let data_start = 16 + cue_bytes.len() + directory_len;
        let total_len = data_start + files.iter().map(|(_, data)| data.len()).sum::<usize>();
        let mut bytes = vec![0u8; total_len];
        bytes[..8].copy_from_slice(&CUE_CONTAINER_MAGIC);
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
        bytes
    }

    fn cue_container(cue: &str, files: &[(&str, Vec<u8>)]) -> ResourceBlob {
        ResourceBlob::from_bytes(&cue_container_bytes(cue, files))
    }

    fn acknowledge_interrupt(cd: &mut Ps1CdRom) {
        cd.write_register(0, 1);
        cd.write_register(3, 0x1f);
        cd.write_register(0, 0);
    }

    #[test]
    fn setloc_readn_streams_iso_sector_and_interrupts() {
        let mut cd = Ps1CdRom::new(Some(iso(4))).unwrap();
        cd.write_register(0, 1);
        cd.write_register(2, 0x1f);
        cd.write_register(0, 0);
        for value in [0x00, 0x02, 0x01] {
            cd.write_register(2, value);
        }
        cd.write_register(1, 0x02);
        assert_eq!(cd.target_lba, 1);
        cd.write_register(0, 1);
        cd.write_register(3, 0x1f);
        cd.write_register(0, 0);
        cd.write_register(1, 0x06);
        cd.tick_cpu_cycles(1);
        assert!(cd.irq_pending());
        cd.write_register(3, 0x80);
        assert_eq!(cd.dma_read_word(), 0x0000_a501);
    }

    #[test]
    fn raw_sector_sync_is_detected_and_user_payload_read() {
        let mut bytes = vec![0u8; 2352];
        bytes[0] = 0;
        bytes[1..11].fill(0xff);
        bytes[11] = 0;
        bytes[24..28].copy_from_slice(&[1, 2, 3, 4]);
        let mut cd = Ps1CdRom::new(Some(ResourceBlob::from_bytes(&bytes))).unwrap();
        cd.reading = true;
        cd.load_sector().unwrap();
        assert_eq!(cd.dma_read_word(), 0x0403_0201);
    }
    #[test]
    fn get_id_delivers_identity_after_first_interrupt_is_acknowledged() {
        let mut cd = Ps1CdRom::new(Some(iso(1))).unwrap();
        cd.write_register(1, 0x1a);
        assert_eq!(cd.responses.iter().copied().collect::<Vec<_>>(), [0x02]);
        assert_eq!(cd.interrupt_flags, 3);
        assert!(cd.scheduled_response.is_some());

        cd.tick_cpu_cycles(GET_ID_CYCLES as u32);
        assert_eq!(cd.interrupt_flags, 3);
        assert_eq!(cd.responses.iter().copied().collect::<Vec<_>>(), [0x02]);

        acknowledge_interrupt(&mut cd);
        assert_eq!(cd.interrupt_flags, 2);
        assert_eq!(cd.responses.len(), 8);
        assert!(cd
            .responses
            .iter()
            .copied()
            .collect::<Vec<_>>()
            .ends_with(b"SCEA"));
    }

    #[test]
    fn mechanical_commands_deliver_completion_after_interrupt_acknowledge() {
        let mut cd = Ps1CdRom::new(Some(iso(10))).unwrap();

        cd.write_register(1, 0x07);
        assert_eq!(cd.interrupt_flags, 3);
        cd.tick_cpu_cycles(SHORT_COMMAND_CYCLES as u32);
        assert_eq!(cd.interrupt_flags, 3);
        acknowledge_interrupt(&mut cd);
        assert_eq!(cd.interrupt_flags, 2);
        acknowledge_interrupt(&mut cd);

        cd.target_lba = 2;
        cd.write_register(1, 0x15);
        assert_ne!(cd.status_byte & 0x40, 0);
        cd.tick_cpu_cycles(SEEK_CYCLES as u32);
        acknowledge_interrupt(&mut cd);
        assert_eq!(cd.interrupt_flags, 2);
        assert_eq!(cd.status_byte & 0x40, 0);
        assert_eq!(cd.current_lba, 2);
        acknowledge_interrupt(&mut cd);

        cd.reading = true;
        cd.status_byte |= 0x20;
        cd.write_register(1, 0x09);
        assert_eq!(cd.interrupt_flags, 3);
        assert!(!cd.reading);
        cd.tick_cpu_cycles(ACTIVE_PAUSE_CYCLES as u32);
        acknowledge_interrupt(&mut cd);
        assert_eq!(cd.interrupt_flags, 2);
        assert_eq!(cd.status_byte & 0x20, 0);
    }

    #[test]
    fn get_q_and_read_toc_use_two_stage_command_protocol() {
        let mut cd = Ps1CdRom::new(Some(iso(4))).unwrap();
        cd.write_register(2, 0x01);
        cd.write_register(2, 0x01);
        cd.write_register(1, 0x1d);
        assert_eq!(cd.interrupt_flags, 3);
        cd.tick_cpu_cycles(SEEK_CYCLES as u32);
        acknowledge_interrupt(&mut cd);
        assert_eq!(cd.interrupt_flags, 2);
        assert_eq!(
            cd.responses.iter().copied().collect::<Vec<_>>(),
            [0x41, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00]
        );
        acknowledge_interrupt(&mut cd);

        cd.write_register(1, 0x1e);
        assert_eq!(cd.interrupt_flags, 3);
        cd.tick_cpu_cycles(CPU_HZ as u32);
        assert_eq!(cd.interrupt_flags, 3);
        acknowledge_interrupt(&mut cd);
        assert_eq!(cd.interrupt_flags, 2);
        assert_eq!(cd.responses.iter().copied().collect::<Vec<_>>(), [0x02]);
    }

    #[test]
    fn scheduled_response_and_pending_command_round_trip_preserve_order() {
        let disc = iso(1);
        let mut first = Ps1CdRom::new(Some(disc.clone())).unwrap();
        first.write_register(1, 0x1a);
        first.write_register(1, 0x01);
        assert_eq!(first.pending_command, Some(0x01));
        first.tick_cpu_cycles((GET_ID_CYCLES / 2) as u32);

        let mut out = StateWriter::new(crate::platform::PlatformId::PlayStation, 21);
        first.save(&mut out);
        let bytes = out.finish();
        let mut second = Ps1CdRom::new(Some(disc)).unwrap();
        let mut input =
            StateReader::new(&bytes, crate::platform::PlatformId::PlayStation, 21).unwrap();
        second.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(second.pending_command, Some(0x01));

        second.tick_cpu_cycles((GET_ID_CYCLES / 2) as u32);
        assert_eq!(second.interrupt_flags, 3);
        acknowledge_interrupt(&mut second);
        assert_eq!(second.interrupt_flags, 2);
        assert!(second
            .responses
            .iter()
            .copied()
            .collect::<Vec<_>>()
            .ends_with(b"SCEA"));
        assert_eq!(second.pending_command, Some(0x01));

        acknowledge_interrupt(&mut second);
        assert_eq!(second.interrupt_flags, 3);
        assert_eq!(second.pending_command, None);
        assert_eq!(second.responses.iter().copied().collect::<Vec<_>>(), [0x02]);
    }

    #[test]
    fn streaming_disc_requests_missing_sector_and_resumes_after_hydration() {
        let mut staged = ResourceBlob::streaming(4 * 2048, 2).unwrap();
        staged.write(0, &[0; 2048]).unwrap();
        let mut cd = Ps1CdRom::new(Some(staged.clone())).unwrap();
        cd.current_lba = 2;
        cd.reading = true;
        cd.cycle_phase = CYCLES_PER_SECTOR;
        cd.tick_cpu_cycles(0);
        assert!(cd.reading);
        assert_eq!(staged.pending_range(), Some((4096, 6144)));
        let mut sector = vec![0u8; 2048];
        sector[..4].copy_from_slice(&[0x11, 0x22, 0x33, 0x44]);
        staged.write(4096, &sector).unwrap();
        assert_eq!(staged.pending_range(), None);
        cd.tick_cpu_cycles(0);
        assert_eq!(cd.current_lba, 3);
        assert_eq!(cd.dma_read_word(), 0x4433_2211);
    }

    #[test]
    fn streamed_cue_container_requests_track_bytes_and_resumes() {
        let mut data = vec![0u8; 2 * 2352];
        for sector in 0..2 {
            let base = sector * 2352;
            data[base] = 0;
            data[base + 1..base + 11].fill(0xff);
            data[base + 11] = 0;
            data[base + 15] = 2;
            data[base + 24..base + 28].copy_from_slice(&[0x11 + sector as u8, 0x22, 0x33, 0x44]);
        }
        let cue = "FILE \"disc.bin\" BINARY\n  TRACK 01 MODE2/2352\n    INDEX 01 00:00:00\n";
        let bytes = cue_container_bytes(cue, &[("disc.bin", data.clone())]);
        let header_len = bytes.len() - data.len();
        let mut staged = ResourceBlob::streaming(bytes.len() as u64, 2).unwrap();
        staged.write(0, &bytes[..header_len]).unwrap();
        let mut cd = Ps1CdRom::new(Some(staged.clone())).unwrap();
        cd.reading = true;
        cd.cycle_phase = CYCLES_PER_SECTOR;
        cd.tick_cpu_cycles(0);
        assert_eq!(cd.current_lba, 0);
        assert_eq!(
            staged.pending_range(),
            Some((header_len as u64, (header_len + 2352) as u64))
        );
        staged
            .write(header_len as u64, &bytes[header_len..header_len + 2352])
            .unwrap();
        cd.tick_cpu_cycles(0);
        assert_eq!(cd.current_lba, 1);
        assert_eq!(cd.dma_read_word(), 0x4433_2211);
    }

    #[test]
    fn sbi_bad_crc_sector_retains_previous_valid_subq_position() {
        let mut data = vec![0u8; 3 * 2352];
        for sector in 0..3 {
            let base = sector * 2352;
            data[base] = 0;
            data[base + 1..base + 11].fill(0xff);
            data[base + 11] = 0;
            data[base + 15] = 2;
        }
        let mut sbi = b"SBI\0".to_vec();
        sbi.extend([0x00, 0x02, 0x01, 0x01]);
        sbi.extend([0u8; 10]);
        let cue = "FILE \"disc.bin\" BINARY\n  TRACK 01 MODE2/2352\n    INDEX 01 00:00:00\n";
        let image = cue_container(cue, &[("disc.bin", data), ("game.sbi", sbi)]);
        let mut cd = Ps1CdRom::new(Some(image)).unwrap();

        cd.current_lba = 0;
        cd.load_sector().unwrap();
        assert_eq!(cd.subq_lba, 0);
        cd.load_sector().unwrap();
        assert_eq!(cd.current_lba, 2);
        assert_eq!(cd.subq_lba, 0);
        acknowledge_interrupt(&mut cd);
        cd.write_register(1, 0x11);
        assert_eq!(
            cd.responses.iter().copied().collect::<Vec<_>>(),
            [0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00]
        );
        acknowledge_interrupt(&mut cd);

        cd.load_sector().unwrap();
        assert_eq!(cd.subq_lba, 2);
        acknowledge_interrupt(&mut cd);
        cd.write_register(1, 0x11);
        assert_eq!(
            cd.responses.iter().copied().collect::<Vec<_>>(),
            [0x01, 0x01, 0x00, 0x00, 0x02, 0x00, 0x02, 0x02]
        );
    }

    #[test]
    fn multitrack_cue_maps_data_audio_toc_subchannel_and_autopause() {
        let mut data = vec![0u8; 2 * 2352];
        for sector in 0..2 {
            let base = sector * 2352;
            data[base] = 0;
            data[base + 1..base + 11].fill(0xff);
            data[base + 11] = 0;
            data[base + 15] = 2;
            data[base + 24..base + 28].copy_from_slice(&[sector as u8 + 1, 0xa5, 0x5a, 0xc3]);
        }
        let mut music = vec![0u8; 2 * 2352];
        for frame in music.as_chunks_mut::<4>().0 {
            frame[..2].copy_from_slice(&0x1800i16.to_le_bytes());
            frame[2..].copy_from_slice(&(-0x1000i16).to_le_bytes());
        }
        let cue = "FILE \"data.bin\" BINARY\n  TRACK 01 MODE2/2352\n    INDEX 01 00:00:00\nFILE \"music.bin\" BINARY\n  TRACK 02 AUDIO\n    PREGAP 00:02:00\n    INDEX 01 00:00:00\n";
        let image = cue_container(cue, &[("data.bin", data), ("music.bin", music)]);
        let mut cd = Ps1CdRom::new(Some(image)).unwrap();

        cd.write_register(1, 0x13);
        assert_eq!(
            cd.responses.iter().copied().collect::<Vec<_>>(),
            [0x02, 0x01, 0x02]
        );
        acknowledge_interrupt(&mut cd);

        cd.write_register(2, 0x02);
        cd.write_register(1, 0x14);
        assert_eq!(
            cd.responses.iter().copied().collect::<Vec<_>>(),
            [0x02, 0x00, 0x04]
        );
        acknowledge_interrupt(&mut cd);

        cd.current_lba = 0;
        cd.load_sector().unwrap();
        assert_eq!(cd.dma_read_word(), 0xc35a_a501);
        acknowledge_interrupt(&mut cd);

        cd.mode = 0x02;
        cd.write_register(2, 0x02);
        cd.write_register(1, 0x03);
        assert_eq!(cd.current_lba, 152);
        acknowledge_interrupt(&mut cd);
        cd.write_register(1, 0x11);
        assert_eq!(
            cd.responses.iter().copied().collect::<Vec<_>>(),
            [0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02]
        );
        acknowledge_interrupt(&mut cd);

        cd.load_audio_sector().unwrap();
        assert_eq!(cd.audio.front().copied(), Some([0x1800, -0x1000]));
        assert!(cd.playing_audio);
        cd.load_audio_sector().unwrap();
        assert!(!cd.playing_audio);
        assert_eq!(cd.interrupt_flags, 4);
        assert_eq!(cd.current_lba, 154);
    }

    #[test]
    fn cdda_playback_applies_banked_volume_matrix_and_mute() {
        let mut cd = Ps1CdRom::new(Some(cdda(2, 0x1000, -0x2000))).unwrap();
        cd.write_register(1, 0x03);
        cd.tick_cpu_cycles(1);
        let stereo = cd.drain_audio();
        assert_eq!(stereo.len(), 588);
        assert_eq!(stereo[0], [0x1000, -0x2000]);
        assert_ne!(cd.status_byte & 0x80, 0);
        acknowledge_interrupt(&mut cd);

        cd.write_register(0, 2);
        cd.write_register(2, 0x40);
        cd.write_register(3, 0x40);
        cd.write_register(0, 3);
        cd.write_register(1, 0x40);
        cd.write_register(2, 0x40);
        cd.write_register(3, 1 << 5);
        cd.current_lba = 0;
        cd.write_register(0, 0);
        cd.write_register(1, 0x03);
        cd.tick_cpu_cycles(1);
        let mono = cd.drain_audio();
        assert_eq!(mono[0][0], mono[0][1]);
        assert_eq!(mono[0][0], -0x0800);
        acknowledge_interrupt(&mut cd);

        cd.write_register(1, 0x0b);
        acknowledge_interrupt(&mut cd);
        cd.current_lba = 0;
        cd.write_register(1, 0x03);
        cd.tick_cpu_cycles(1);
        assert!(cd.drain_audio().iter().all(|frame| *frame == [0, 0]));
    }

    #[test]
    fn host_status_tracks_data_request_and_xa_decoder_activity() {
        let mut cd = Ps1CdRom::new(Some(iso(1))).unwrap();
        assert_eq!(cd.read_register(0) & 0x80, 0);
        cd.data.push_back(0x55);
        assert_eq!(cd.read_register(0) & 0x40, 0);
        cd.write_register(0, 0);
        cd.write_register(3, 0x80);
        assert_ne!(cd.read_register(0) & 0x40, 0);

        let mut xa = Ps1CdRom::new(Some(xa_sector(1, 2, 0x01, 0x11))).unwrap();
        xa.mode = 0x40;
        xa.reading = true;
        xa.load_sector().unwrap();
        assert_ne!(xa.read_register(0) & 0x04, 0);
    }

    #[test]
    fn xa_four_bit_stereo_is_filtered_from_data_and_resampled() {
        let mut cd = Ps1CdRom::new(Some(xa_sector(1, 2, 0x01, 0x11))).unwrap();
        cd.mode = 0x40;
        cd.reading = true;
        cd.load_sector().unwrap();
        assert!(cd.data.is_empty());
        assert_eq!(cd.interrupt_flags, 0);
        assert_eq!(cd.audio.len(), 2352);
        assert!(cd.audio.iter().any(|frame| *frame != [0, 0]));
    }

    #[test]
    fn xa_filter_and_adpmute_apply_without_muting_cdda() {
        let mut filtered = Ps1CdRom::new(Some(xa_sector(1, 2, 0x01, 0x11))).unwrap();
        filtered.mode = 0x48;
        filtered.filter_file = 1;
        filtered.filter_channel = 3;
        filtered.reading = true;
        filtered.load_sector().unwrap();
        assert!(filtered.audio.is_empty());

        let mut muted = Ps1CdRom::new(Some(xa_sector(1, 2, 0x01, 0x11))).unwrap();
        muted.mode = 0x48;
        muted.filter_file = 1;
        muted.filter_channel = 2;
        muted.write_register(0, 3);
        muted.write_register(3, 1);
        muted.reading = true;
        muted.load_sector().unwrap();
        assert_eq!(muted.audio.len(), 2352);
        assert!(muted.audio.iter().all(|frame| *frame == [0, 0]));

        let mut cdda = Ps1CdRom::new(Some(cdda(1, 0x1000, -0x1000))).unwrap();
        cdda.write_register(0, 3);
        cdda.write_register(3, 1);
        cdda.write_register(0, 0);
        cdda.write_register(1, 0x03);
        cdda.tick_cpu_cycles(1);
        assert_eq!(cdda.drain_audio()[0], [0x1000, -0x1000]);
    }

    #[test]
    fn xa_low_rate_mono_fills_one_complete_sector_queue() {
        let mut cd = Ps1CdRom::new(Some(xa_sector(0, 0, 0x04, 0x11))).unwrap();
        cd.mode = 0x40;
        cd.reading = true;
        cd.load_sector().unwrap();
        assert_eq!(cd.audio.len(), MAX_CD_AUDIO_FRAMES);
        assert!(cd.audio.iter().any(|frame| frame[0] != 0));
        assert!(cd.audio.iter().all(|frame| frame[0] == frame[1]));
    }

    #[test]
    fn xa_decoder_state_round_trip_preserves_audio_and_resampler_history() {
        let disc = xa_sector(1, 2, 0x01, 0x11);
        let mut first = Ps1CdRom::new(Some(disc.clone())).unwrap();
        first.mode = 0x40;
        first.reading = true;
        first.load_sector().unwrap();
        let mut out = StateWriter::new(crate::platform::PlatformId::PlayStation, 45);
        first.save(&mut out);
        let bytes = out.finish();

        let mut second = Ps1CdRom::new(Some(disc)).unwrap();
        let mut input =
            StateReader::new(&bytes, crate::platform::PlatformId::PlayStation, 45).unwrap();
        second.load(&mut input).unwrap();
        input.finish().unwrap();
        let mut first_again = StateWriter::new(crate::platform::PlatformId::PlayStation, 45);
        first.save(&mut first_again);
        let mut second_again = StateWriter::new(crate::platform::PlatformId::PlayStation, 45);
        second.save(&mut second_again);
        assert_eq!(second_again.finish(), first_again.finish());
    }

    #[test]
    fn state_round_trip_preserves_stream_position_and_fifo() {
        let mut cd = Ps1CdRom::new(Some(iso(3))).unwrap();
        cd.current_lba = 1;
        cd.target_lba = 2;
        cd.reading = true;
        cd.request_data = true;
        cd.data.extend([1, 2, 3, 4]);
        let mut out = StateWriter::new(crate::platform::PlatformId::PlayStation, 44);
        cd.save(&mut out);
        let bytes = out.finish();
        let mut restored = Ps1CdRom::new(Some(iso(3))).unwrap();
        let mut input =
            StateReader::new(&bytes, crate::platform::PlatformId::PlayStation, 44).unwrap();
        restored.load(&mut input).unwrap();
        input.finish().unwrap();
        assert_eq!(restored.current_lba, 1);
        assert_eq!(restored.target_lba, 2);
        assert_eq!(restored.dma_read_word(), 0x0403_0201);
    }
}
