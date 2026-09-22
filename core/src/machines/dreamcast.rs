use std::collections::VecDeque;

use crate::cd_image::{DiscImage, DiscLayout, TrackKind};
use crate::cpu_arm7::{Arm7, Arm7Bus};
use crate::cpu_sh4::{Sh4, Sh4Bus};
use crate::input::{
    AXIS_LEFT_TRIGGER, AXIS_LEFT_X, AXIS_LEFT_Y, AXIS_RIGHT_TRIGGER, DOWN, FACE_EAST, FACE_NORTH,
    FACE_SOUTH, FACE_WEST, LEFT, RIGHT, START, UP,
};
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machine::Machine;
use crate::platform::PlatformId;
use crate::resources::{ResourceBlob, ResourceKind};
use crate::state::{StateReader, StateWriter};

const SH4_HZ: u64 = 200_000_000;
const ARM7_HZ: u64 = 45_158_400;
const AUDIO_RATE: u64 = 44_100;
const FRAME_RATE: u64 = 60;
const GD_SECTORS_PER_SECOND: u64 = 150;
const BIOS_SIZE: usize = 2 * 1024 * 1024;
const FLASH_SIZE: usize = 128 * 1024;
const MAIN_RAM_SIZE: usize = 16 * 1024 * 1024;
const VRAM_SIZE: usize = 8 * 1024 * 1024;
const SOUND_RAM_SIZE: usize = 2 * 1024 * 1024;
const WIDTH: usize = 640;
const HEIGHT: usize = 480;
const STATE_VERSION: u32 = 1;

const IRQ_RENDER_DONE: u32 = 1 << 2;
const IRQ_VBLANK: u32 = 1 << 5;
const IRQ_MAPLE_DMA: u32 = 1 << 12;
const EXT_GDROM: u32 = 1 << 0;

fn le16(bytes: &[u8], offset: usize) -> u16 {
    u16::from_le_bytes([bytes[offset], bytes[offset + 1]])
}

fn put_le16(bytes: &mut [u8], offset: usize, value: u16) {
    bytes[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn physical(address: u32) -> u32 {
    if address >= 0x8000_0000 {
        address & 0x1fff_ffff
    } else {
        address
    }
}
struct DreamcastGdrom {
    image: DiscImage,
    lba: u32,
    remaining: u32,
    phase: u64,
    fifo: VecDeque<u8>,
    packet: [u8; 12],
    packet_pos: usize,
    expecting_packet: bool,
    status: u8,
    error: u8,
    interrupt_reason: u8,
    byte_count: u16,
    irq: bool,
}

impl DreamcastGdrom {
    fn new(disc: ResourceBlob) -> Result<Self, String> {
        let image = DiscImage::new(disc)?;
        Ok(Self {
            image,
            lba: 0,
            remaining: 0,
            phase: 0,
            fifo: VecDeque::with_capacity(4096),
            packet: [0; 12],
            packet_pos: 0,
            expecting_packet: false,
            status: 0x50,
            error: 0,
            interrupt_reason: 0,
            byte_count: 2048,
            irq: false,
        })
    }

    fn reset(&mut self) {
        self.lba = 0;
        self.remaining = 0;
        self.phase = 0;
        self.fifo.clear();
        self.packet.fill(0);
        self.packet_pos = 0;
        self.expecting_packet = false;
        self.status = 0x50;
        self.error = 0;
        self.interrupt_reason = 0;
        self.byte_count = 2048;
        self.irq = false;
    }

    fn issue_ata_command(&mut self, command: u8) {
        match command {
            0x08 => self.reset(),
            0x90 => {
                self.error = 1;
                self.status = 0x50;
                self.irq = true;
            }
            0xa0 => {
                self.packet_pos = 0;
                self.expecting_packet = true;
                self.status = 0x58;
                self.interrupt_reason = 1;
                self.irq = true;
            }
            0xa1 => {
                self.fifo.clear();
                let mut identify = [0u8; 512];
                identify[..16].copy_from_slice(b"SEGA GD-ROM      ");
                self.fifo.extend(identify);
                self.status = 0x58;
                self.interrupt_reason = 2;
                self.byte_count = 512;
                self.irq = true;
            }
            _ => {
                self.error = 4;
                self.status = 0x51;
                self.irq = true;
            }
        }
    }

    fn write_data16(&mut self, value: u16) {
        if !self.expecting_packet || self.packet_pos >= self.packet.len() {
            return;
        }
        let bytes = value.to_le_bytes();
        for byte in bytes {
            if self.packet_pos < self.packet.len() {
                self.packet[self.packet_pos] = byte;
                self.packet_pos += 1;
            }
        }
        if self.packet_pos == self.packet.len() {
            self.expecting_packet = false;
            self.execute_packet();
        }
    }

    fn drive_status(&self) -> u8 {
        if self.remaining != 0 {
            0x00
        } else if self.lba > 150 {
            0x01
        } else {
            0x02
        }
    }

    fn current_fad(&self) -> u32 {
        self.lba.max(150)
    }

    fn lead_out_fad(&self) -> u32 {
        150u32.saturating_add(self.image.sectors)
    }

    fn track_control_adr(kind: TrackKind) -> u8 {
        match kind {
            TrackKind::Data => 0x41,
            TrackKind::Audio => 0x01,
        }
    }

    fn current_track_info(&self) -> (u8, u8, u8) {
        let image_lba = self.current_fad().saturating_sub(150);
        let Some(track) = self.image.track_for_lba(image_lba) else {
            return (0x41, self.image.first_track(), 1);
        };
        let index = u8::from(image_lba >= track.start_lba);
        (Self::track_control_adr(track.kind), track.number, index)
    }

    fn formatted_subcode_q(&self) -> [u8; 14] {
        let fad = self.current_fad();
        let image_lba = fad.saturating_sub(150);
        let (control_adr, track_number, index) = self.current_track_info();
        let track_start_fad = self
            .image
            .track_for_lba(image_lba)
            .map_or(150, |track| 150u32.saturating_add(track.start_lba));
        let relative_fad = fad.saturating_sub(track_start_fad);
        let audio_status = match self.drive_status() {
            1 => 0x12,
            2 => 0x13,
            _ => 0x15,
        };
        [
            0,
            audio_status,
            0,
            14,
            control_adr,
            track_number,
            index,
            (relative_fad >> 16) as u8,
            (relative_fad >> 8) as u8,
            relative_fad as u8,
            0,
            (fad >> 16) as u8,
            (fad >> 8) as u8,
            fad as u8,
        ]
    }

    fn read_address(bytes: &[u8], msf: bool) -> Option<u32> {
        if bytes.len() < 3 {
            return None;
        }
        if msf {
            if bytes[1] >= 60 || bytes[2] >= 75 {
                return None;
            }
            Some(u32::from(bytes[0]) * 60 * 75 + u32::from(bytes[1]) * 75 + u32::from(bytes[2]))
        } else {
            Some((u32::from(bytes[0]) << 16) | (u32::from(bytes[1]) << 8) | u32::from(bytes[2]))
        }
    }

    fn read_address_valid(&self, fad: u32) -> bool {
        (150..self.lead_out_fad()).contains(&fad)
    }

    fn expected_sector_type_matches(layout: DiscLayout, expected: u8) -> bool {
        if expected == 0 {
            return true;
        }
        match layout {
            DiscLayout::Raw2352Audio => expected == 1,
            DiscLayout::Iso2048 | DiscLayout::Raw2352Mode1 => expected == 2,
            DiscLayout::Raw2352Mode2 => (3..=6).contains(&expected),
        }
    }

    fn packet_sector_size(&self, image_lba: u32) -> Result<usize, String> {
        let track = self
            .image
            .track_for_lba(image_lba)
            .ok_or_else(|| "Dreamcast GD-ROM read address is outside a track".to_string())?;
        let expected = (self.packet[1] >> 1) & 0x07;
        if !Self::expected_sector_type_matches(track.layout, expected) {
            return Err("Dreamcast GD-ROM sector does not match expected data type".into());
        }
        if track.kind == TrackKind::Audio || self.packet[1] & 0x10 != 0 {
            return Ok(2352);
        }
        match self.packet[1] & 0xf0 {
            0x20 => Ok(2048),
            0xe0 if expected == 3 => Ok(2340),
            _ => Err("Dreamcast GD-ROM data-select combination is not supported".into()),
        }
    }

    fn read_packet_sector(&self, image_lba: u32, out: &mut [u8; 2352]) -> Result<usize, String> {
        let size = self.packet_sector_size(image_lba)?;
        match size {
            2048 => {
                let mut user = [0u8; 2048];
                self.image.read_user_sector(image_lba, &mut user)?;
                out[..user.len()].copy_from_slice(&user);
            }
            2340 => {
                self.image.read_raw_sector(image_lba, out)?;
                out.copy_within(12..2352, 0);
            }
            2352 => self.image.read_raw_sector(image_lba, out)?,
            _ => unreachable!(),
        }
        Ok(size)
    }

    fn fail_packet(&mut self) {
        self.fifo.clear();
        self.remaining = 0;
        self.error = 4;
        self.status = 0x51;
        self.interrupt_reason = 3;
        self.byte_count = 0;
        self.irq = true;
    }

    fn begin_sector_read(&mut self, start: Option<u32>, count: u32, pre_read: Option<u32>) {
        let Some(start) = start else {
            self.fail_packet();
            return;
        };
        let end = start.checked_add(count);
        let addresses_valid = self.read_address_valid(start)
            && pre_read.is_none_or(|fad| self.read_address_valid(fad))
            && end.is_some_and(|fad| fad <= self.lead_out_fad());
        let format_valid = count == 0 || self.packet_sector_size(start - 150).is_ok();
        if !addresses_valid || !format_valid {
            self.fail_packet();
            return;
        }
        self.lba = start;
        self.remaining = count;
        if count == 0 {
            self.finish_packet();
        } else {
            self.phase = SH4_HZ;
            self.status = 0x50;
            self.interrupt_reason = 3;
        }
    }

    fn finish_packet(&mut self) {
        self.fifo.clear();
        self.status = 0x50;
        self.interrupt_reason = 3;
        self.byte_count = 0;
        self.irq = true;
    }

    fn send_packet_data(&mut self, data: &[u8]) {
        self.fifo.clear();
        self.fifo.extend(data.iter().copied());
        if data.is_empty() {
            self.status = 0x50;
            self.interrupt_reason = 3;
            self.byte_count = 0;
        } else {
            self.status = 0x58;
            self.interrupt_reason = 2;
            self.byte_count = data.len().min(usize::from(u16::MAX)) as u16;
        }
        self.irq = true;
    }

    fn send_packet_window(&mut self, data: &[u8], start: usize, length: usize) {
        if length == 0 || start >= data.len() {
            self.send_packet_data(&[]);
            return;
        }
        let end = start.saturating_add(length).min(data.len());
        self.send_packet_data(&data[start..end]);
    }

    fn execute_packet(&mut self) {
        const MODE_INFO: [u8; 32] = [
            0x00, 0x00, 0x00, 0x00, 0x00, 0xb4, 0x19, 0x00, 0x00, 0x08, b'S', b'E', b' ', b' ',
            b' ', b' ', b' ', b' ', b'R', b'e', b'v', b' ', b'6', b'.', b'4', b'3', b'9', b'9',
            b'0', b'4', b'0', b'8',
        ];

        let opcode = self.packet[0];
        if opcode != 0x13 {
            self.error = 0;
        }
        match opcode {
            0x00 => self.finish_packet(),
            0x10 => {
                let fad = self.current_fad();
                let (control_adr, track, index) = self.current_track_info();
                let status = [
                    self.drive_status(),
                    0x80,
                    control_adr,
                    track,
                    index,
                    (fad >> 16) as u8,
                    (fad >> 8) as u8,
                    fad as u8,
                    0,
                    0,
                ];
                self.send_packet_window(
                    &status,
                    usize::from(self.packet[2]),
                    usize::from(self.packet[4]),
                );
            }
            0x11 => self.send_packet_window(
                &MODE_INFO,
                usize::from(self.packet[2]),
                usize::from(self.packet[4]),
            ),
            0x13 => {
                let had_error = self.error != 0;
                let response = [
                    0xf0,
                    0,
                    if had_error { 0x05 } else { 0 },
                    0,
                    0,
                    0,
                    0,
                    0,
                    if had_error { 0x20 } else { 0 },
                    0,
                ];
                self.error = 0;
                self.send_packet_window(&response, 0, usize::from(self.packet[4]));
            }
            0x14 => {
                let mut toc = [0u8; 408];
                for entry in toc[..396].as_chunks_mut::<4>().0 {
                    entry[1..4].fill(0xff);
                }
                for track in &self.image.tracks {
                    if !(1..=99).contains(&track.number) {
                        continue;
                    }
                    let offset = usize::from(track.number - 1) * 4;
                    let fad = 150u32.saturating_add(track.start_lba);
                    toc[offset..offset + 4].copy_from_slice(&[
                        Self::track_control_adr(track.kind),
                        (fad >> 16) as u8,
                        (fad >> 8) as u8,
                        fad as u8,
                    ]);
                }
                let first = self.image.tracks.first();
                let last = self.image.tracks.last();
                let first_control = first.map_or(0x41, |track| Self::track_control_adr(track.kind));
                let last_control = last.map_or(0x41, |track| Self::track_control_adr(track.kind));
                toc[396..400].copy_from_slice(&[first_control, self.image.first_track(), 0, 0]);
                toc[400..404].copy_from_slice(&[last_control, self.image.last_track(), 0, 0]);
                let lead_out = self.lead_out_fad();
                toc[404..408].copy_from_slice(&[
                    last_control,
                    (lead_out >> 16) as u8,
                    (lead_out >> 8) as u8,
                    lead_out as u8,
                ]);
                let allocation = usize::from(u16::from_be_bytes([self.packet[3], self.packet[4]]));
                self.send_packet_window(&toc, 0, allocation);
            }
            0x15 => {
                let session = self.packet[2];
                let fad = if session == 0 {
                    self.lead_out_fad()
                } else {
                    150u32.saturating_add(
                        self.image
                            .track_start(self.image.first_track())
                            .unwrap_or_default(),
                    )
                };
                let response = [
                    self.drive_status(),
                    0,
                    1,
                    (fad >> 16) as u8,
                    (fad >> 8) as u8,
                    fad as u8,
                ];
                self.send_packet_window(&response, 0, usize::from(self.packet[4]));
            }
            0x21 => {
                let parameter_type = self.packet[1] & 0x07;
                let target = match parameter_type {
                    1 => Some(
                        (u32::from(self.packet[2]) << 16)
                            | (u32::from(self.packet[3]) << 8)
                            | u32::from(self.packet[4]),
                    ),
                    2 if self.packet[3] < 60 && self.packet[4] < 75 => Some(
                        u32::from(self.packet[2]) * 60 * 75
                            + u32::from(self.packet[3]) * 75
                            + u32::from(self.packet[4]),
                    ),
                    3 => Some(150),
                    4 => Some(self.current_fad()),
                    _ => None,
                };
                let valid = target.is_some_and(|fad| {
                    parameter_type >= 3 || (150..self.lead_out_fad()).contains(&fad)
                });
                if valid {
                    self.lba = target.unwrap_or(150);
                    self.remaining = 0;
                    self.phase = 0;
                    self.finish_packet();
                } else {
                    self.fifo.clear();
                    self.error = 4;
                    self.status = 0x51;
                    self.interrupt_reason = 3;
                    self.byte_count = 0;
                    self.irq = true;
                }
            }
            0x30 => {
                let msf = self.packet[1] & 1 != 0;
                let start = Self::read_address(&self.packet[2..5], msf);
                let count = (u32::from(self.packet[8]) << 16)
                    | (u32::from(self.packet[9]) << 8)
                    | u32::from(self.packet[10]);
                self.begin_sector_read(start, count, None);
            }
            0x31 => {
                let msf = self.packet[1] & 1 != 0;
                let start = Self::read_address(&self.packet[2..5], msf);
                let count = u32::from(u16::from_be_bytes([self.packet[6], self.packet[7]]));
                let pre_read = Self::read_address(&self.packet[8..11], msf);
                self.begin_sector_read(start, count, pre_read);
            }
            0x40 => {
                let allocation = usize::from(u16::from_be_bytes([self.packet[3], self.packet[4]]));
                if self.packet[1] & 0x0f == 1 {
                    let response = self.formatted_subcode_q();
                    self.send_packet_window(&response, 0, allocation);
                } else {
                    self.fifo.clear();
                    self.error = 4;
                    self.status = 0x51;
                    self.interrupt_reason = 3;
                    self.byte_count = 0;
                    self.irq = true;
                }
            }
            _ => {
                self.fifo.clear();
                self.error = 4;
                self.status = 0x51;
                self.interrupt_reason = 3;
                self.byte_count = 0;
                self.irq = true;
            }
        }
    }

    fn tick(&mut self, sh4_cycles: u32) {
        if self.remaining == 0 || self.fifo.len() > 4096 {
            return;
        }
        self.phase = self
            .phase
            .saturating_add(u64::from(sh4_cycles) * GD_SECTORS_PER_SECOND);
        while self.phase >= SH4_HZ && self.remaining != 0 {
            self.phase -= SH4_HZ;
            let image_lba = self.lba.saturating_sub(150);
            let sector_size = match self.packet_sector_size(image_lba) {
                Ok(size) => size,
                Err(_) => {
                    self.fail_packet();
                    break;
                }
            };
            let mut sector = [0u8; 2352];
            if self.read_packet_sector(image_lba, &mut sector).is_err() {
                self.phase = SH4_HZ;
                break;
            }
            self.fifo.extend(&sector[..sector_size]);
            self.lba = self.lba.wrapping_add(1);
            self.remaining -= 1;
            if self.remaining == 0 && self.packet[0] == 0x31 {
                let msf = self.packet[1] & 1 != 0;
                if let Some(pre_read) = Self::read_address(&self.packet[8..11], msf) {
                    self.lba = pre_read;
                }
            }
            self.status = 0x58;
            self.interrupt_reason = 2;
            self.byte_count = self.fifo.len().min(usize::from(u16::MAX)) as u16;
            self.irq = true;
            if self.fifo.len() > 4096 {
                break;
            }
        }
    }

    fn read_data16(&mut self) -> u16 {
        let low = self.fifo.pop_front().unwrap_or(0);
        let high = self.fifo.pop_front().unwrap_or(0);
        if self.fifo.is_empty() {
            self.status = 0x50;
            self.interrupt_reason = 3;
            self.byte_count = 0;
            self.irq = true;
        } else {
            self.byte_count = self.byte_count.saturating_sub(2);
        }
        u16::from_le_bytes([low, high])
    }

    fn save(&self, out: &mut StateWriter) {
        let media_format = if self.image.tracks.len() > 1 {
            2
        } else if self.image.is_raw_sector(0) {
            1
        } else {
            0
        };
        out.u8(media_format);
        out.u32(self.lba);
        out.u32(self.remaining);
        out.u64(self.phase);
        out.blob(&self.packet);
        out.u32(self.packet_pos as u32);
        out.u8(u8::from(self.expecting_packet));
        out.u8(self.status);
        out.u8(self.error);
        out.u8(self.interrupt_reason);
        out.u16(self.byte_count);
        out.u8(u8::from(self.irq));
        out.u32(self.fifo.len() as u32);
        for byte in &self.fifo {
            out.u8(*byte);
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        match input.u8()? {
            0..=2 => {}
            _ => return Err("Dreamcast GD-ROM state has invalid media format".into()),
        }
        self.lba = input.u32()?;
        self.remaining = input.u32()?;
        self.phase = input.u64()? % SH4_HZ;
        let packet = input.blob()?;
        if packet.len() != self.packet.len() {
            return Err("Dreamcast GD-ROM packet state has wrong size".into());
        }
        self.packet.copy_from_slice(packet);
        self.packet_pos = input.u32()?.min(12) as usize;
        self.expecting_packet = input.u8()? != 0;
        self.status = input.u8()?;
        self.error = input.u8()?;
        self.interrupt_reason = input.u8()?;
        self.byte_count = input.u16()?;
        self.irq = input.u8()? != 0;
        self.fifo.clear();
        let len = input.u32()?.min(8192);
        for _ in 0..len {
            self.fifo.push_back(input.u8()?);
        }
        Ok(())
    }
}

#[derive(Clone, Default)]
struct SystemAsic {
    normal: u32,
    external: u32,
    error: u32,
    mask2: [u32; 3],
    mask4: [u32; 3],
    mask6: [u32; 3],
}

impl SystemAsic {
    fn reset(&mut self) {
        *self = Self::default();
    }
    fn raise_normal(&mut self, bit: u32) {
        self.normal |= bit;
    }

    fn raise_external(&mut self, bit: u32) {
        self.external |= bit;
    }

    fn interrupt(&self) -> Option<(u8, u16)> {
        for (level, masks, event) in [
            (6, &self.mask6, 0x320),
            (4, &self.mask4, 0x360),
            (2, &self.mask2, 0x3a0),
        ] {
            if self.normal & masks[0] != 0
                || self.external & masks[1] != 0
                || self.error & masks[2] != 0
            {
                return Some((level, event));
            }
        }
        None
    }

    fn read32(&self, address: u32) -> u32 {
        match address {
            0x005f_689c => 0x0b,
            0x005f_6900 => self.normal,
            0x005f_6904 => self.external,
            0x005f_6908 => self.error,
            0x005f_6910 => self.mask2[0],
            0x005f_6914 => self.mask2[1],
            0x005f_6918 => self.mask2[2],
            0x005f_6920 => self.mask4[0],
            0x005f_6924 => self.mask4[1],
            0x005f_6928 => self.mask4[2],
            0x005f_6930 => self.mask6[0],
            0x005f_6934 => self.mask6[1],
            0x005f_6938 => self.mask6[2],
            _ => 0,
        }
    }

    fn write32(&mut self, address: u32, value: u32) {
        match address {
            0x005f_6900 => self.normal &= !value,
            0x005f_6908 => self.error &= !value,
            0x005f_6910 => self.mask2[0] = value,
            0x005f_6914 => self.mask2[1] = value,
            0x005f_6918 => self.mask2[2] = value,
            0x005f_6920 => self.mask4[0] = value,
            0x005f_6924 => self.mask4[1] = value,
            0x005f_6928 => self.mask4[2] = value,
            0x005f_6930 => self.mask6[0] = value,
            0x005f_6934 => self.mask6[1] = value,
            0x005f_6938 => self.mask6[2] = value,
            _ => {}
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.u32(self.normal);
        out.u32(self.external);
        out.u32(self.error);
        for masks in [self.mask2, self.mask4, self.mask6] {
            for mask in masks {
                out.u32(mask);
            }
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.normal = input.u32()?;
        self.external = input.u32()?;
        self.error = input.u32()?;
        for mask in &mut self.mask2 {
            *mask = input.u32()?;
        }
        for mask in &mut self.mask4 {
            *mask = input.u32()?;
        }
        for mask in &mut self.mask6 {
            *mask = input.u32()?;
        }
        Ok(())
    }
}

#[derive(Clone, Default)]
struct Maple {
    dma_addr: u32,
    trigger: u32,
    enable: u32,
    state: u32,
    speed: u32,
    protect: u32,
    buttons: [u64; 4],
    axes: [[i16; 4]; 4],
}
impl Maple {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn set_input(&mut self, input: &InputState) {
        self.buttons.copy_from_slice(&input.buttons);
        for port in 0..4 {
            self.axes[port] = [
                input.axes[port][AXIS_LEFT_X],
                input.axes[port][AXIS_LEFT_Y],
                input.axes[port][AXIS_LEFT_TRIGGER],
                input.axes[port][AXIS_RIGHT_TRIGGER],
            ];
        }
    }

    fn button_word(&self, port: usize) -> u16 {
        let state = self.buttons[port];
        let mut pressed = 0u16;
        let mapping = [
            (FACE_NORTH, 0),
            (FACE_EAST, 1),
            (FACE_SOUTH, 2),
            (START, 3),
            (UP, 4),
            (DOWN, 5),
            (LEFT, 6),
            (RIGHT, 7),
            (FACE_WEST, 9),
        ];
        for (button, bit) in mapping {
            if state & button != 0 {
                pressed |= 1 << bit;
            }
        }
        !pressed
    }

    fn analog(value: i16) -> u8 {
        (128 + i32::from(value) * 127 / 32767).clamp(0, 255) as u8
    }

    fn trigger(value: i16) -> u8 {
        (i32::from(value).max(0) * 255 / 32767).clamp(0, 255) as u8
    }

    fn ram_offset(address: u32, len: usize) -> Option<usize> {
        let address = physical(address);
        if !(0x0c00_0000..0x1000_0000).contains(&address) {
            return None;
        }
        let offset = address as usize & (MAIN_RAM_SIZE - 1);
        (offset + len <= MAIN_RAM_SIZE).then_some(offset)
    }

    fn read_ram32(ram: &[u8], address: u32) -> Option<u32> {
        let offset = Self::ram_offset(address, 4)?;
        Some(u32::from_le_bytes(ram[offset..offset + 4].try_into().ok()?))
    }
    fn write_ram32(ram: &mut [u8], address: u32, value: u32) -> bool {
        let Some(offset) = Self::ram_offset(address, 4) else {
            return false;
        };
        ram[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
        true
    }

    fn write_condition(&self, ram: &mut [u8], address: u32, port: usize) {
        let _ = Self::write_ram32(ram, address, 0x0800_2003);
        let _ = Self::write_ram32(ram, address.wrapping_add(4), 0x0000_0001);
        let buttons = self.button_word(port).to_le_bytes();
        let payload = [
            buttons[0],
            buttons[1],
            Self::trigger(self.axes[port][2]),
            Self::trigger(self.axes[port][3]),
            Self::analog(self.axes[port][0]),
            Self::analog(self.axes[port][1]),
            128,
            128,
        ];
        let first = u32::from_le_bytes(payload[..4].try_into().unwrap());
        let second = u32::from_le_bytes(payload[4..].try_into().unwrap());
        let _ = Self::write_ram32(ram, address.wrapping_add(8), first);
        let _ = Self::write_ram32(ram, address.wrapping_add(12), second);
    }

    fn write_device_info(ram: &mut [u8], address: u32) {
        let _ = Self::write_ram32(ram, address, 0x0500_201c);
        let _ = Self::write_ram32(ram, address.wrapping_add(4), 0x0000_0001);
        let mut info = [0u8; 108];
        info[..4].copy_from_slice(&0xfe06_ffffu32.to_le_bytes());
        let name = b"Dreamcast Controller             ";
        info[16..16 + name.len()].copy_from_slice(name);
        for (index, chunk) in info.as_chunks::<4>().0.iter().enumerate() {
            let value = u32::from_le_bytes(*chunk);
            let _ = Self::write_ram32(ram, address.wrapping_add(8 + (index as u32 * 4)), value);
        }
    }

    fn run_dma(&mut self, ram: &mut [u8]) -> bool {
        if self.enable & 1 == 0 {
            return false;
        }
        self.state = 1;
        let mut cursor = self.dma_addr;
        for _ in 0..64 {
            let Some(descriptor) = Self::read_ram32(ram, cursor) else {
                break;
            };
            let Some(receive) = Self::read_ram32(ram, cursor.wrapping_add(4)) else {
                break;
            };
            let words = (descriptor & 0xff).wrapping_add(1);
            let port = ((descriptor >> 16) & 3) as usize;
            let Some(frame) = Self::read_ram32(ram, cursor.wrapping_add(8)) else {
                break;
            };
            match (frame >> 24) as u8 {
                0x01 => Self::write_device_info(ram, receive),
                0x09 => self.write_condition(ram, receive, port),
                _ => {
                    let _ = Self::write_ram32(ram, receive, 0xffff_ffff);
                }
            }
            cursor = cursor.wrapping_add(8 + words * 4);
            if descriptor & 0x8000_0000 != 0 {
                break;
            }
        }
        self.state = 0;
        true
    }

    fn read32(&self, address: u32) -> u32 {
        match address {
            0x005f_6c04 => self.dma_addr,
            0x005f_6c10 => self.trigger,
            0x005f_6c14 => self.enable,
            0x005f_6c18 => self.state,
            0x005f_6c80 => self.speed,
            0x005f_6c8c => self.protect,
            _ => 0,
        }
    }

    fn write32(&mut self, address: u32, value: u32) -> bool {
        match address {
            0x005f_6c04 => self.dma_addr = value & 0x1fff_ffe0,
            0x005f_6c10 => self.trigger = value & 1,
            0x005f_6c14 => self.enable = value & 1,
            0x005f_6c18 => return value & 1 != 0,
            0x005f_6c80 => self.speed = value,
            0x005f_6c8c => self.protect = value,
            _ => {}
        }
        false
    }

    fn save(&self, out: &mut StateWriter) {
        for value in [
            self.dma_addr,
            self.trigger,
            self.enable,
            self.state,
            self.speed,
            self.protect,
        ] {
            out.u32(value);
        }
        for buttons in self.buttons {
            out.u64(buttons);
        }
        for axes in self.axes {
            for axis in axes {
                out.u16(axis as u16);
            }
        }
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        self.dma_addr = input.u32()?;
        self.trigger = input.u32()?;
        self.enable = input.u32()?;
        self.state = input.u32()?;
        self.speed = input.u32()?;
        self.protect = input.u32()?;
        for buttons in &mut self.buttons {
            *buttons = input.u64()?;
        }
        for axes in &mut self.axes {
            for axis in axes {
                *axis = input.u16()? as i16;
            }
        }
        Ok(())
    }
}

struct Pvr {
    regs: Box<[u32; 0x800]>,
    frame: u64,
}

impl Default for Pvr {
    fn default() -> Self {
        let mut regs = Box::new([0; 0x800]);
        regs[0] = 0x17fd_11db;
        regs[1] = 0x0000_0011;
        Self { regs, frame: 0 }
    }
}
impl Pvr {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn read32(&self, address: u32) -> u32 {
        let offset = (address - 0x005f_8000) as usize;
        self.regs.get(offset >> 2).copied().unwrap_or(0)
    }

    fn write32(&mut self, address: u32, value: u32) -> bool {
        let offset = (address - 0x005f_8000) as usize;
        if let Some(register) = self.regs.get_mut(offset >> 2) {
            if offset != 0 && offset != 4 {
                *register = value;
            }
        }
        offset == 0x14
    }

    fn color16(value: u16, mode: u32) -> [u8; 4] {
        if mode == 0 {
            let r = u32::from((value >> 10) & 0x1f) * 255 / 31;
            let g = u32::from((value >> 5) & 0x1f) * 255 / 31;
            let b = u32::from(value & 0x1f) * 255 / 31;
            [r as u8, g as u8, b as u8, 255]
        } else {
            let r = u32::from((value >> 11) & 0x1f) * 255 / 31;
            let g = u32::from((value >> 5) & 0x3f) * 255 / 63;
            let b = u32::from(value & 0x1f) * 255 / 31;
            [r as u8, g as u8, b as u8, 255]
        }
    }

    fn pixel(&self, vram: &[u8], base: usize, index: usize, mode: u32) -> [u8; 4] {
        match mode {
            0 | 1 => {
                let address = (base + index * 2) & (VRAM_SIZE - 1);
                let next = (address + 1) & (VRAM_SIZE - 1);
                Self::color16(u16::from_le_bytes([vram[address], vram[next]]), mode)
            }
            2 => {
                let address = (base + index * 3) & (VRAM_SIZE - 1);
                [
                    vram[(address + 2) & (VRAM_SIZE - 1)],
                    vram[(address + 1) & (VRAM_SIZE - 1)],
                    vram[address],
                    255,
                ]
            }
            _ => {
                let address = (base + index * 4) & (VRAM_SIZE - 1);
                [
                    vram[(address + 2) & (VRAM_SIZE - 1)],
                    vram[(address + 1) & (VRAM_SIZE - 1)],
                    vram[address],
                    vram[(address + 3) & (VRAM_SIZE - 1)],
                ]
            }
        }
    }

    fn render(&mut self, vram: &[u8], video: &mut VideoBuffer) {
        video.clear([0, 0, 0, 255]);
        let control = self.regs[0x44 / 4];
        let mode = control & 3;
        let base = self.regs[0x50 / 4] as usize & (VRAM_SIZE - 1);
        let size = self.regs[0x5c / 4];
        let source_width = ((size & 0x3ff) + 1).clamp(1, WIDTH as u32) as usize;
        let source_height = (((size >> 10) & 0x3ff) + 1).clamp(1, HEIGHT as u32) as usize;
        for y in 0..HEIGHT {
            let sy = y * source_height / HEIGHT;
            for x in 0..WIDTH {
                let sx = x * source_width / WIDTH;
                let rgba = self.pixel(vram, base, sy * source_width + sx, mode);
                let target = (y * WIDTH + x) * 4;
                video.pixels_mut()[target..target + 4].copy_from_slice(&rgba);
            }
        }
    }

    fn save(&self, out: &mut StateWriter) {
        for value in self.regs.iter().copied() {
            out.u32(value);
        }
        out.u64(self.frame);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for value in self.regs.iter_mut() {
            *value = input.u32()?;
        }
        self.frame = input.u64()?;
        self.regs[0] = 0x17fd_11db;
        self.regs[1] = 0x0000_0011;
        Ok(())
    }
}
struct Aica {
    regs: Box<[u8; 0x8000]>,
    phase: [u32; 64],
    active: [bool; 64],
    sample_phase: u64,
    samples: Vec<(f32, f32)>,
}

impl Default for Aica {
    fn default() -> Self {
        let mut regs = Box::new([0; 0x8000]);
        regs[0x2c00] = 1;
        Self {
            regs,
            phase: [0; 64],
            active: [false; 64],
            sample_phase: 0,
            samples: Vec::with_capacity(1024),
        }
    }
}

impl Aica {
    fn reset(&mut self) {
        *self = Self::default();
    }

    fn begin_frame(&mut self) {
        self.samples.clear();
    }

    fn arm_reset(&self) -> bool {
        self.regs[0x2c00] & 1 != 0
    }

    fn read8(&self, offset: usize) -> u8 {
        self.regs[offset & 0x7fff]
    }

    fn write8(&mut self, offset: usize, value: u8) {
        let offset = offset & 0x7fff;
        self.regs[offset] = value;
        if offset < 0x2000 && offset % 0x80 < 2 {
            let base = offset & !0x7f;
            if le16(self.regs.as_ref(), base) & 0x8000 != 0 {
                self.execute_key_on();
                let control = le16(self.regs.as_ref(), base) & !0x8000;
                put_le16(self.regs.as_mut(), base, control);
            }
        }
    }
    fn execute_key_on(&mut self) {
        for slot in 0..64 {
            let base = slot * 0x80;
            let enabled = le16(self.regs.as_ref(), base) & 0x4000 != 0;
            if enabled && !self.active[slot] {
                self.phase[slot] = 0;
            }
            self.active[slot] = enabled;
        }
    }

    fn pitch_step(&self, slot: usize) -> u32 {
        let pitch = le16(self.regs.as_ref(), slot * 0x80 + 0x18);
        let fns = u32::from(pitch & 0x03ff);
        let raw_octave = ((pitch >> 11) & 0x0f) as i8;
        let octave = if raw_octave & 8 != 0 {
            raw_octave - 16
        } else {
            raw_octave
        };
        let mut step = 65_536u32.saturating_mul(1024 + fns) / 1024;
        if octave > 0 {
            step = step.checked_shl(octave as u32).unwrap_or(u32::MAX);
        } else if octave < 0 {
            step >>= (-octave) as u32;
        }
        step.max(1)
    }

    fn sample_slot(&mut self, slot: usize, ram: &[u8]) -> (f32, f32) {
        if !self.active[slot] {
            return (0.0, 0.0);
        }
        let base = slot * 0x80;
        let control = le16(self.regs.as_ref(), base);
        let start = ((usize::from(control & 0x007f)) << 16)
            | usize::from(le16(self.regs.as_ref(), base + 4));
        let loop_start = usize::from(le16(self.regs.as_ref(), base + 8));
        let loop_end = usize::from(le16(self.regs.as_ref(), base + 0x0c)).max(loop_start + 1);
        let mut position = (self.phase[slot] >> 16) as usize;
        if position >= loop_end {
            if control & 0x0200 != 0 {
                position = loop_start;
                self.phase[slot] = (position as u32) << 16;
            } else {
                self.active[slot] = false;
                return (0.0, 0.0);
            }
        }
        let pcms = (control >> 7) & 3;
        let sample = match pcms {
            0 => {
                let address = (start + position * 2) & (SOUND_RAM_SIZE - 1);
                let next = (address + 1) & (SOUND_RAM_SIZE - 1);
                f32::from(i16::from_le_bytes([ram[address], ram[next]])) / 32768.0
            }
            1 => {
                let address = (start + position) & (SOUND_RAM_SIZE - 1);
                f32::from(ram[address] as i8) / 128.0
            }
            _ => 0.0,
        };
        self.phase[slot] = self.phase[slot].wrapping_add(self.pitch_step(slot));
        let total_level = f32::from(self.regs[base + 0x29]) / 255.0;
        let route = le16(self.regs.as_ref(), base + 0x24);
        let send = f32::from((route >> 8) & 0x0f) / 15.0;
        let pan = (route & 0x1f) as u8;
        let gain = sample * (1.0 - total_level) * send;
        let attenuation = f32::from(pan & 0x0f) / 15.0;
        if pan & 0x10 == 0 {
            (gain * (1.0 - attenuation), gain)
        } else {
            (gain, gain * (1.0 - attenuation))
        }
    }
    fn tick(&mut self, sh4_cycles: u32, ram: &[u8]) {
        self.sample_phase = self
            .sample_phase
            .saturating_add(u64::from(sh4_cycles) * AUDIO_RATE);
        while self.sample_phase >= SH4_HZ {
            self.sample_phase -= SH4_HZ;
            let mut left = 0.0f32;
            let mut right = 0.0f32;
            for slot in 0..64 {
                let (l, r) = self.sample_slot(slot, ram);
                left += l;
                right += r;
            }
            self.samples
                .push((left.clamp(-1.0, 1.0), right.clamp(-1.0, 1.0)));
        }
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.regs.as_ref());
        for phase in self.phase {
            out.u32(phase);
        }
        for active in self.active {
            out.u8(u8::from(active));
        }
        out.u64(self.sample_phase);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let regs = input.blob()?;
        if regs.len() != self.regs.len() {
            return Err("Dreamcast AICA register state size mismatch".into());
        }
        self.regs.copy_from_slice(regs);
        for phase in &mut self.phase {
            *phase = input.u32()?;
        }
        for active in &mut self.active {
            *active = input.u8()? != 0;
        }
        self.sample_phase = input.u64()? % SH4_HZ;
        self.samples.clear();
        Ok(())
    }
}

struct DreamcastBoard {
    bios: Box<[u8]>,
    flash: Box<[u8]>,
    ram: Box<[u8]>,
    vram: Box<[u8]>,
    sound_ram: Box<[u8]>,
    asic: SystemAsic,
    maple: Maple,
    pvr: Pvr,
    aica: Aica,
    gdrom: DreamcastGdrom,
    video: VideoBuffer,
    frame: u64,
}

impl DreamcastBoard {
    fn new(bios: &[u8], disc: ResourceBlob) -> Result<Self, String> {
        if bios.len() != BIOS_SIZE {
            return Err("Dreamcast requires a 2 MiB boot ROM".into());
        }
        let mut bios_mem = vec![0; BIOS_SIZE].into_boxed_slice();
        bios_mem.copy_from_slice(bios);
        Ok(Self {
            bios: bios_mem,
            flash: vec![0xff; FLASH_SIZE].into_boxed_slice(),
            ram: vec![0; MAIN_RAM_SIZE].into_boxed_slice(),
            vram: vec![0; VRAM_SIZE].into_boxed_slice(),
            sound_ram: vec![0; SOUND_RAM_SIZE].into_boxed_slice(),
            asic: SystemAsic::default(),
            maple: Maple::default(),
            pvr: Pvr::default(),
            aica: Aica::default(),
            gdrom: DreamcastGdrom::new(disc)?,
            video: VideoBuffer::new(WIDTH as u32, HEIGHT as u32),
            frame: 0,
        })
    }

    fn reset(&mut self) {
        self.ram.fill(0);
        self.vram.fill(0);
        self.sound_ram.fill(0);
        self.asic.reset();
        self.maple.reset();
        self.pvr.reset();
        self.aica.reset();
        self.gdrom.reset();
        self.video.clear([0, 0, 0, 255]);
        self.frame = 0;
    }

    fn memory_read8(&self, address: u32) -> u8 {
        let address = physical(address);
        match address {
            0x0000_0000..=0x001f_ffff => self.bios[address as usize],
            0x0020_0000..=0x0021_ffff => {
                self.flash[(address as usize - 0x0020_0000) & (FLASH_SIZE - 1)]
            }
            0x0070_0000..=0x0070_7fff => self.aica.read8((address - 0x0070_0000) as usize),
            0x0080_0000..=0x009f_ffff => {
                self.sound_ram[(address as usize - 0x0080_0000) & (SOUND_RAM_SIZE - 1)]
            }
            0x0400_0000..=0x07ff_ffff => {
                self.vram[(address as usize - 0x0400_0000) & (VRAM_SIZE - 1)]
            }
            0x0c00_0000..=0x0fff_ffff => self.ram[address as usize & (MAIN_RAM_SIZE - 1)],
            _ => 0,
        }
    }

    fn memory_write8(&mut self, address: u32, value: u8) {
        let address = physical(address);
        match address {
            0x0020_0000..=0x0021_ffff => {
                self.flash[(address as usize - 0x0020_0000) & (FLASH_SIZE - 1)] = value
            }
            0x0070_0000..=0x0070_7fff => self.aica.write8((address - 0x0070_0000) as usize, value),
            0x0080_0000..=0x009f_ffff => {
                self.sound_ram[(address as usize - 0x0080_0000) & (SOUND_RAM_SIZE - 1)] = value
            }
            0x0400_0000..=0x07ff_ffff => {
                self.vram[(address as usize - 0x0400_0000) & (VRAM_SIZE - 1)] = value
            }
            0x0c00_0000..=0x0fff_ffff => self.ram[address as usize & (MAIN_RAM_SIZE - 1)] = value,
            0x1100_0000..=0x117f_ffff => {
                self.vram[(address as usize - 0x1100_0000) & (VRAM_SIZE - 1)] = value
            }
            _ => {}
        }
    }

    fn read_bytes(&self, address: u32, count: usize) -> u32 {
        let mut bytes = [0u8; 4];
        for (offset, byte) in bytes.iter_mut().take(count).enumerate() {
            *byte = self.memory_read8(address.wrapping_add(offset as u32));
        }
        u32::from_le_bytes(bytes)
    }

    fn write_bytes(&mut self, address: u32, value: u32, count: usize) {
        for (offset, byte) in value.to_le_bytes().into_iter().take(count).enumerate() {
            self.memory_write8(address.wrapping_add(offset as u32), byte);
        }
    }

    fn read32(&mut self, address: u32) -> u32 {
        let address = physical(address) & !3;
        match address {
            0x005f_6800..=0x005f_69ff => self.asic.read32(address),
            0x005f_6c00..=0x005f_6cff => self.maple.read32(address),
            0x005f_7018 => u32::from(self.gdrom.status),
            0x005f_7080 => {
                let low = u32::from(self.gdrom.read_data16());
                let high = u32::from(self.gdrom.read_data16());
                low | (high << 16)
            }
            0x005f_7084 => u32::from(self.gdrom.error),
            0x005f_7088 => u32::from(self.gdrom.interrupt_reason),
            0x005f_7090 => u32::from(self.gdrom.byte_count as u8),
            0x005f_7094 => u32::from(self.gdrom.byte_count >> 8),
            0x005f_7098 => 0xa0,
            0x005f_709c => u32::from(self.gdrom.status),
            0x005f_8000..=0x005f_9fff => self.pvr.read32(address),
            _ => self.read_bytes(address, 4),
        }
    }

    fn write32(&mut self, address: u32, value: u32) {
        let address = physical(address) & !3;
        match address {
            0x005f_6800..=0x005f_69ff => self.asic.write32(address, value),
            0x005f_6c00..=0x005f_6cff => {
                if self.maple.write32(address, value) && self.maple.run_dma(self.ram.as_mut()) {
                    self.asic.raise_normal(IRQ_MAPLE_DMA);
                }
            }
            0x005f_7018 => {
                if value & 4 != 0 {
                    self.gdrom.irq = false;
                    self.asic.external &= !EXT_GDROM;
                }
            }
            0x005f_7080 => {
                self.gdrom.write_data16(value as u16);
                self.gdrom.write_data16((value >> 16) as u16);
            }
            0x005f_7090 => {
                self.gdrom.byte_count = (self.gdrom.byte_count & 0xff00) | u16::from(value as u8);
            }
            0x005f_7094 => {
                self.gdrom.byte_count =
                    (self.gdrom.byte_count & 0x00ff) | (u16::from(value as u8) << 8);
            }
            0x005f_709c => {
                self.gdrom.issue_ata_command(value as u8);
                if self.gdrom.irq {
                    self.asic.raise_external(EXT_GDROM);
                }
            }
            0x005f_8000..=0x005f_9fff => {
                if self.pvr.write32(address, value) {
                    self.asic.raise_normal(IRQ_RENDER_DONE);
                }
            }
            _ => self.write_bytes(address, value, 4),
        }
    }

    fn read8(&mut self, address: u32) -> u8 {
        let physical = physical(address);
        if matches!(physical, 0x005f_6800..=0x005f_9fff) {
            let value = self.read32(physical & !3).to_le_bytes();
            value[(physical & 3) as usize]
        } else {
            self.memory_read8(physical)
        }
    }

    fn read16(&mut self, address: u32) -> u16 {
        let address = physical(address);
        if address & !3 == 0x005f_7080 {
            self.gdrom.read_data16()
        } else {
            u16::from_le_bytes([self.read8(address), self.read8(address.wrapping_add(1))])
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        let address = physical(address);
        if matches!(address, 0x005f_6800..=0x005f_9fff) {
            let aligned = address & !3;
            let mut bytes = self.read32(aligned).to_le_bytes();
            bytes[(address & 3) as usize] = value;
            self.write32(aligned, u32::from_le_bytes(bytes));
        } else {
            self.memory_write8(address, value);
        }
    }

    fn write16(&mut self, address: u32, value: u16) {
        let address = physical(address);
        if address & !3 == 0x005f_7080 {
            self.gdrom.write_data16(value);
            return;
        }
        if matches!(address, 0x005f_6800..=0x005f_9fff) {
            let aligned = address & !3;
            let mut bytes = self.read32(aligned).to_le_bytes();
            let raw = value.to_le_bytes();
            let lane = (address & 2) as usize;
            bytes[lane..lane + 2].copy_from_slice(&raw);
            self.write32(aligned, u32::from_le_bytes(bytes));
        } else {
            for (offset, byte) in value.to_le_bytes().into_iter().enumerate() {
                self.memory_write8(address.wrapping_add(offset as u32), byte);
            }
        }
    }

    fn tick(&mut self, sh4_cycles: u32) {
        self.gdrom.tick(sh4_cycles);
        if self.gdrom.irq {
            self.asic.raise_external(EXT_GDROM);
        }
        self.aica.tick(sh4_cycles, self.sound_ram.as_ref());
    }

    fn end_frame(&mut self) {
        self.pvr.render(self.vram.as_ref(), &mut self.video);
        self.pvr.frame = self.pvr.frame.wrapping_add(1);
        self.asic.raise_normal(IRQ_VBLANK);
        self.frame = self.frame.wrapping_add(1);
    }

    fn save(&self, out: &mut StateWriter) {
        out.blob(self.flash.as_ref());
        out.blob(self.ram.as_ref());
        out.blob(self.vram.as_ref());
        out.blob(self.sound_ram.as_ref());
        self.asic.save(out);
        self.maple.save(out);
        self.pvr.save(out);
        self.aica.save(out);
        self.gdrom.save(out);
        out.u64(self.frame);
    }

    fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        for (name, target) in [
            ("flash", self.flash.as_mut()),
            ("main RAM", self.ram.as_mut()),
            ("VRAM", self.vram.as_mut()),
            ("sound RAM", self.sound_ram.as_mut()),
        ] {
            let data = input.blob()?;
            if data.len() != target.len() {
                return Err(format!("Dreamcast {name} state size mismatch"));
            }
            target.copy_from_slice(data);
        }
        self.asic.load(input)?;
        self.maple.load(input)?;
        self.pvr.load(input)?;
        self.aica.load(input)?;
        self.gdrom.load(input)?;
        self.frame = input.u64()?;
        self.pvr.render(self.vram.as_ref(), &mut self.video);
        Ok(())
    }
}

struct MainBus<'a> {
    board: &'a mut DreamcastBoard,
}

impl Sh4Bus for MainBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        self.board.read8(address)
    }
    fn write8(&mut self, address: u32, value: u8) {
        self.board.write8(address, value);
    }
    fn read16(&mut self, address: u32) -> u16 {
        self.board.read16(address)
    }
    fn write16(&mut self, address: u32, value: u16) {
        self.board.write16(address, value);
    }
    fn read32(&mut self, address: u32) -> u32 {
        self.board.read32(address)
    }
    fn write32(&mut self, address: u32, value: u32) {
        self.board.write32(address, value);
    }
}

struct AicaBus<'a> {
    ram: &'a mut [u8],
    aica: &'a mut Aica,
}

impl Arm7Bus for AicaBus<'_> {
    fn read8(&mut self, address: u32) -> u8 {
        match address {
            0x0000_0000..=0x007f_ffff => self.ram[address as usize & (SOUND_RAM_SIZE - 1)],
            0x0080_0000..=0x0080_7fff => self.aica.read8((address - 0x0080_0000) as usize),
            _ => 0,
        }
    }

    fn write8(&mut self, address: u32, value: u8) {
        match address {
            0x0000_0000..=0x007f_ffff => self.ram[address as usize & (SOUND_RAM_SIZE - 1)] = value,
            0x0080_0000..=0x0080_7fff => self.aica.write8((address - 0x0080_0000) as usize, value),
            _ => {}
        }
    }
}

pub struct DreamcastMachine {
    sh4: Sh4,
    arm7: Arm7,
    board: DreamcastBoard,
    audio: AudioBuffer,
    frame_phase: u64,
    arm_phase: u64,
    arm_held_reset: bool,
    powered: bool,
}

impl DreamcastMachine {
    pub fn from_bios_and_disc(bios: &[u8], disc: ResourceBlob) -> Result<Self, String> {
        let board = DreamcastBoard::new(bios, disc)?;
        let mut machine = Self {
            sh4: Sh4::default(),
            arm7: Arm7::default(),
            board,
            audio: AudioBuffer::new(AUDIO_RATE as u32, 2),
            frame_phase: 0,
            arm_phase: 0,
            arm_held_reset: true,
            powered: true,
        };
        machine.boot_cpus();
        Ok(machine)
    }

    fn boot_cpus(&mut self) {
        self.sh4.reset(0xa000_0000, 0, 0x8d00_0000);
        self.arm7.reset(0, 0x001f_f000);
        self.arm_held_reset = true;
        self.arm_phase = 0;
    }

    fn run_arm_credit(&mut self, sh4_cycles: u32) {
        if self.board.aica.arm_reset() {
            if !self.arm_held_reset {
                self.arm7.reset(0, 0x001f_f000);
            }
            self.arm_held_reset = true;
            self.arm_phase = 0;
            return;
        }
        if self.arm_held_reset {
            self.arm7.reset(0, 0x001f_f000);
            self.arm_held_reset = false;
        }
        self.arm_phase = self
            .arm_phase
            .saturating_add(u64::from(sh4_cycles) * ARM7_HZ);
        let budget = self.arm_phase / SH4_HZ;
        self.arm_phase %= SH4_HZ;
        let target = self.arm7.cycles.saturating_add(budget);
        while self.arm7.cycles < target {
            let used = {
                let (ram, aica) = (&mut self.board.sound_ram, &mut self.board.aica);
                let mut bus = AicaBus {
                    ram: ram.as_mut(),
                    aica,
                };
                self.arm7.step(&mut bus)
            };
            if used == 0 {
                break;
            }
        }
    }

    fn clock_sh4(&mut self) -> u32 {
        if let Some((level, event)) = self.board.asic.interrupt() {
            let used = {
                let mut bus = MainBus {
                    board: &mut self.board,
                };
                self.sh4.interrupt(&mut bus, level, event)
            };
            if used != 0 {
                self.board.tick(used);
                self.run_arm_credit(used);
                return used;
            }
        }
        let used = {
            let mut bus = MainBus {
                board: &mut self.board,
            };
            self.sh4.step(&mut bus)
        };
        if used != 0 {
            self.board.tick(used);
            self.run_arm_credit(used);
        }
        used
    }

    fn begin_frame(&mut self) {
        self.board.aica.begin_frame();
        self.audio.begin_frame();
    }

    fn flush_audio(&mut self) {
        self.audio.begin_frame();
        if self.board.aica.samples.is_empty() {
            for _ in 0..AUDIO_RATE as usize / FRAME_RATE as usize {
                self.audio.push_stereo(0.0, 0.0);
            }
        } else {
            for &(left, right) in &self.board.aica.samples {
                self.audio.push_stereo(left, right);
            }
        }
    }

    fn run_budget(&mut self, target: u64) {
        while self.sh4.cycles < target {
            if self.clock_sh4() == 0 {
                self.powered = false;
                break;
            }
        }
    }
}

impl Machine for DreamcastMachine {
    fn platform(&self) -> PlatformId {
        PlatformId::Dreamcast
    }

    fn reset(&mut self) {
        self.board.reset();
        self.boot_cpus();
        self.frame_phase = 0;
        self.arm_phase = 0;
        self.powered = true;
        self.begin_frame();
    }

    fn run_frame(&mut self, input: &InputState) {
        if !self.powered {
            return;
        }
        self.board.maple.set_input(input);
        self.begin_frame();
        self.frame_phase = self.frame_phase.saturating_add(SH4_HZ);
        let budget = self.frame_phase / FRAME_RATE;
        self.frame_phase %= FRAME_RATE;
        let target = self.sh4.cycles.saturating_add(budget);
        self.run_budget(target);
        if self.powered {
            self.board.end_frame();
        }
        self.flush_audio();
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
        let mut out = StateWriter::new(PlatformId::Dreamcast, STATE_VERSION);
        self.sh4.save(&mut out);
        self.arm7.save(&mut out);
        self.board.save(&mut out);
        out.u64(self.frame_phase);
        out.u64(self.arm_phase);
        out.u8(u8::from(self.arm_held_reset));
        out.u8(u8::from(self.powered));
        Ok(out.finish())
    }

    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String> {
        let mut input = StateReader::new(bytes, PlatformId::Dreamcast, STATE_VERSION)?;
        self.sh4.load(&mut input)?;
        self.arm7.load(&mut input)?;
        self.board.load(&mut input)?;
        self.frame_phase = input.u64()? % FRAME_RATE;
        self.arm_phase = input.u64()? % SH4_HZ;
        self.arm_held_reset = input.u8()? != 0;
        self.powered = input.u8()? != 0;
        input.finish()?;
        self.flush_audio();
        Ok(())
    }

    fn persistent_len(&self, kind: ResourceKind, slot: u32) -> usize {
        if kind == ResourceKind::Storage && slot == 0 {
            FLASH_SIZE
        } else {
            0
        }
    }

    fn read_persistent(&self, kind: ResourceKind, slot: u32, out: &mut [u8]) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("Dreamcast persistent resource is Storage slot 0".into());
        }
        if out.len() != FLASH_SIZE {
            return Err("Dreamcast flash buffer has the wrong size".into());
        }
        out.copy_from_slice(self.board.flash.as_ref());
        Ok(())
    }

    fn write_persistent(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        data: &[u8],
    ) -> Result<(), String> {
        if kind != ResourceKind::Storage || slot != 0 {
            return Err("Dreamcast persistent resource is Storage slot 0".into());
        }
        if data.len() != FLASH_SIZE {
            return Err("Dreamcast flash image has the wrong size".into());
        }
        self.board.flash.copy_from_slice(data);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bios_with_loop() -> Vec<u8> {
        let mut bios = vec![0; BIOS_SIZE];
        bios[0..2].copy_from_slice(&0xaffeu16.to_le_bytes());
        bios[2..4].copy_from_slice(&0x0009u16.to_le_bytes());
        bios
    }

    fn disc_bytes(sectors: usize) -> Vec<u8> {
        let mut bytes = vec![0; sectors * 2048];
        for sector in 0..sectors {
            bytes[sector * 2048..(sector + 1) * 2048].fill((sector + 1) as u8);
        }
        bytes
    }

    fn raw_sector(mode: u8, data: u8, tail: u8) -> Vec<u8> {
        let mut sector = vec![tail; 2352];
        sector[0] = 0;
        sector[1..11].fill(0xff);
        sector[11] = 0;
        sector[12..16].copy_from_slice(&[0, 2, 0, mode]);
        if mode == 1 {
            sector[16..16 + 2048].fill(data);
        } else {
            sector[16..24].copy_from_slice(&[1, 2, 0, 0, 1, 2, 0, 0]);
            sector[24..24 + 2048].fill(data);
        }
        sector
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

    fn board() -> DreamcastBoard {
        DreamcastBoard::new(&bios_with_loop(), ResourceBlob::from_bytes(&disc_bytes(4))).unwrap()
    }

    #[test]
    fn physical_map_aliases_main_vram_flash_and_aica_ram() {
        let mut board = board();
        board.write32(0x8c00_1234, 0x1122_3344);
        assert_eq!(board.read32(0x0c00_1234), 0x1122_3344);
        board.write32(0x0400_0100, 0xa5a5_5a5a);
        assert_eq!(board.read32(0x0500_0100), 0xa5a5_5a5a);
        board.write32(0x0020_0010, 0xdead_beef);
        assert_eq!(board.read32(0x0020_0010), 0xdead_beef);
        board.write32(0x0080_0040, 0x89ab_cdef);
        assert_eq!(board.read32(0x0080_0040), 0x89ab_cdef);
        assert_eq!(board.read32(0xa000_0000), 0x0009_affe);
    }

    #[test]
    fn system_asic_routes_masked_priorities() {
        let mut asic = SystemAsic::default();
        asic.raise_normal(IRQ_VBLANK);
        assert_eq!(asic.interrupt(), None);
        asic.write32(0x005f_6910, IRQ_VBLANK);
        assert_eq!(asic.interrupt(), Some((2, 0x3a0)));
        asic.write32(0x005f_6930, IRQ_VBLANK);
        assert_eq!(asic.interrupt(), Some((6, 0x320)));
        asic.write32(0x005f_6900, IRQ_VBLANK);
        assert_eq!(asic.interrupt(), None);
    }

    #[test]
    fn pvr_vram_scanout_reaches_640x480_surface() {
        let mut board = board();
        board.vram[0..2].copy_from_slice(&0xf800u16.to_le_bytes());
        board.write32(0x005f_8044, 1);
        board.write32(0x005f_8050, 0);
        board.write32(0x005f_805c, 0);
        board.end_frame();
        assert_eq!(board.video.width(), WIDTH as u32);
        assert_eq!(board.video.height(), HEIGHT as u32);
        let pixel = &board.video.pixels()[..4];
        assert!(pixel[0] > 240 && pixel[1] < 16 && pixel[2] < 16);
        assert_ne!(board.asic.normal & IRQ_VBLANK, 0);
    }

    #[test]
    fn aica_pcm_slot_generates_stereo_audio() {
        let mut aica = Aica::default();
        let mut ram = vec![0; SOUND_RAM_SIZE];
        ram[0..2].copy_from_slice(&0x4000i16.to_le_bytes());
        ram[2..4].copy_from_slice(&0xc000u16.to_le_bytes());
        put_le16(aica.regs.as_mut(), 0x04, 0);
        put_le16(aica.regs.as_mut(), 0x08, 0);
        put_le16(aica.regs.as_mut(), 0x0c, 2);
        put_le16(aica.regs.as_mut(), 0x18, 0);
        put_le16(aica.regs.as_mut(), 0x24, 0x0f00);
        aica.write8(0, 0);
        aica.write8(1, 0xc0);
        aica.tick((SH4_HZ / AUDIO_RATE + 1) as u32, &ram);
        assert!(!aica.samples.is_empty());
        assert!(aica.samples[0].0 > 0.4);
        assert!(aica.samples[0].1 > 0.4);
    }

    #[test]
    fn maple_dma_returns_controller_condition_and_completion_irq() {
        let mut board = board();
        board.maple.buttons[0] = FACE_SOUTH | START | UP;
        board.maple.axes[0] = [16384, -16384, 32767, 0];
        let base = 0x0c00_0100;
        let receive: u32 = 0x0c00_0200;
        let offset = (base as usize) & (MAIN_RAM_SIZE - 1);
        board.ram[offset..offset + 4].copy_from_slice(&0x8000_0001u32.to_le_bytes());
        board.ram[offset + 4..offset + 8].copy_from_slice(&receive.to_le_bytes());
        board.ram[offset + 8..offset + 12].copy_from_slice(&0x0920_0001u32.to_le_bytes());
        board.ram[offset + 12..offset + 16].copy_from_slice(&1u32.to_le_bytes());
        board.write32(0x005f_6c04, base);
        board.write32(0x005f_6c14, 1);
        board.write32(0x005f_6c18, 1);
        assert_ne!(board.asic.normal & IRQ_MAPLE_DMA, 0);
        let receive_offset = (receive as usize) & (MAIN_RAM_SIZE - 1);
        let header = u32::from_le_bytes(
            board.ram[receive_offset..receive_offset + 4]
                .try_into()
                .unwrap(),
        );
        assert_eq!(header, 0x0800_2003);
        let buttons =
            u16::from_le_bytes([board.ram[receive_offset + 8], board.ram[receive_offset + 9]]);
        assert_eq!(buttons & (1 << 2), 0);
        assert_eq!(buttons & (1 << 3), 0);
        assert_eq!(buttons & (1 << 4), 0);
        assert!(board.ram[receive_offset + 10] > 200);
        assert!(board.ram[receive_offset + 12] > 128);
        assert!(board.ram[receive_offset + 13] < 128);
    }

    fn issue_gdrom_packet(gdrom: &mut DreamcastGdrom, packet: [u8; 12]) {
        gdrom.issue_ata_command(0xa0);
        for pair in packet.as_chunks::<2>().0 {
            gdrom.write_data16(u16::from_le_bytes([pair[0], pair[1]]));
        }
    }

    fn drain_gdrom_fifo(gdrom: &mut DreamcastGdrom) -> Vec<u8> {
        let length = gdrom.fifo.len();
        let mut data = Vec::with_capacity(length);
        while !gdrom.fifo.is_empty() {
            data.extend_from_slice(&gdrom.read_data16().to_le_bytes());
        }
        data.truncate(length);
        data
    }

    #[test]
    fn gdrom_spi_status_mode_and_completion_follow_drive_protocol() {
        let mut gdrom = DreamcastGdrom::new(ResourceBlob::from_bytes(&disc_bytes(4))).unwrap();

        issue_gdrom_packet(&mut gdrom, [0x10, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(gdrom.status, 0x58);
        assert_eq!(gdrom.interrupt_reason, 2);
        assert_eq!(gdrom.byte_count, 10);
        assert_eq!(
            drain_gdrom_fifo(&mut gdrom),
            [0x02, 0x80, 0x41, 1, 1, 0, 0, 150, 0, 0]
        );
        assert_eq!(gdrom.status, 0x50);
        assert_eq!(gdrom.interrupt_reason, 3);
        assert_eq!(gdrom.byte_count, 0);

        issue_gdrom_packet(&mut gdrom, [0x11, 0, 0, 0, 32, 0, 0, 0, 0, 0, 0, 0]);
        let mode = drain_gdrom_fifo(&mut gdrom);
        assert_eq!(mode.len(), 32);
        assert_eq!(&mode[10..18], b"SE      ");
        assert_eq!(&mode[18..26], b"Rev 6.43");
        assert_eq!(&mode[26..32], b"990408");
    }

    #[test]
    fn gdrom_spi_toc_and_session_report_single_track_disc_geometry() {
        let mut gdrom = DreamcastGdrom::new(ResourceBlob::from_bytes(&disc_bytes(4))).unwrap();

        issue_gdrom_packet(&mut gdrom, [0x14, 0, 0, 0x01, 0x98, 0, 0, 0, 0, 0, 0, 0]);
        let toc = drain_gdrom_fifo(&mut gdrom);
        assert_eq!(toc.len(), 408);
        assert_eq!(&toc[0..4], &[0x41, 0, 0, 150]);
        assert_eq!(&toc[4..8], &[0x00, 0xff, 0xff, 0xff]);
        assert_eq!(&toc[396..400], &[0x41, 1, 0, 0]);
        assert_eq!(&toc[400..404], &[0x41, 1, 0, 0]);
        assert_eq!(&toc[404..408], &[0x41, 0, 0, 154]);

        issue_gdrom_packet(&mut gdrom, [0x15, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(drain_gdrom_fifo(&mut gdrom), [0x02, 0, 1, 0, 0, 154]);
        issue_gdrom_packet(&mut gdrom, [0x15, 0, 1, 0, 6, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(drain_gdrom_fifo(&mut gdrom), [0x02, 0, 1, 0, 0, 150]);
    }

    #[test]
    fn gdrom_multitrack_cue_drives_toc_status_and_data_reads() {
        let cue = concat!(
            "FILE \"track01.bin\" BINARY\n",
            "  TRACK 01 MODE1/2048\n",
            "    INDEX 01 00:00:00\n",
            "FILE \"track02.bin\" BINARY\n",
            "  TRACK 02 AUDIO\n",
            "    INDEX 01 00:00:00\n",
            "FILE \"track03.bin\" BINARY\n",
            "  TRACK 03 MODE1/2048\n",
            "    PREGAP 00:00:02\n",
            "    INDEX 01 00:00:00\n",
        );
        let disc = cue_container(
            cue,
            &[
                ("track01.bin", vec![0x11; 2048]),
                ("track02.bin", vec![0x22; 2352]),
                ("track03.bin", vec![0x33; 2048]),
            ],
        );
        let mut gdrom = DreamcastGdrom::new(disc).unwrap();
        assert_eq!(gdrom.image.tracks.len(), 3);
        assert_eq!(gdrom.lead_out_fad(), 155);

        issue_gdrom_packet(&mut gdrom, [0x14, 0, 0, 0x01, 0x98, 0, 0, 0, 0, 0, 0, 0]);
        let toc = drain_gdrom_fifo(&mut gdrom);
        assert_eq!(&toc[0..4], &[0x41, 0, 0, 150]);
        assert_eq!(&toc[4..8], &[0x01, 0, 0, 151]);
        assert_eq!(&toc[8..12], &[0x41, 0, 0, 154]);
        assert_eq!(&toc[396..400], &[0x41, 1, 0, 0]);
        assert_eq!(&toc[400..404], &[0x41, 3, 0, 0]);
        assert_eq!(&toc[404..408], &[0x41, 0, 0, 155]);

        issue_gdrom_packet(&mut gdrom, [0x21, 1, 0, 0, 151, 0, 0, 0, 0, 0, 0, 0]);
        issue_gdrom_packet(&mut gdrom, [0x10, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0]);
        let status = drain_gdrom_fifo(&mut gdrom);
        assert_eq!(&status[..8], &[0x01, 0x80, 0x01, 2, 1, 0, 0, 151]);

        issue_gdrom_packet(&mut gdrom, [0x40, 1, 0, 0, 14, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            drain_gdrom_fifo(&mut gdrom),
            [0, 0x12, 0, 14, 0x01, 2, 1, 0, 0, 0, 0, 0, 0, 151]
        );

        issue_gdrom_packet(&mut gdrom, [0x30, 0x20, 0, 0, 154, 0, 0, 0, 0, 0, 1, 0]);
        gdrom.tick(1);
        assert_eq!(gdrom.fifo.len(), 2048);
        assert_eq!(gdrom.read_data16(), 0x3333);
    }

    #[test]
    fn gdrom_spi_error_sense_and_test_unit_complete_cleanly() {
        let mut gdrom = DreamcastGdrom::new(ResourceBlob::from_bytes(&disc_bytes(1))).unwrap();

        issue_gdrom_packet(&mut gdrom, [0x7f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(gdrom.status, 0x51);
        assert_eq!(gdrom.error, 4);

        issue_gdrom_packet(&mut gdrom, [0x13, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0]);
        let sense = drain_gdrom_fifo(&mut gdrom);
        assert_eq!(sense, [0xf0, 0, 0x05, 0, 0, 0, 0, 0, 0x20, 0]);
        assert_eq!(gdrom.error, 0);

        issue_gdrom_packet(&mut gdrom, [0x00; 12]);
        assert_eq!(gdrom.status, 0x50);
        assert_eq!(gdrom.interrupt_reason, 3);
        assert_eq!(gdrom.byte_count, 0);
    }

    #[test]
    fn gdrom_spi_seek_supports_fad_msf_stop_and_range_errors() {
        let mut gdrom = DreamcastGdrom::new(ResourceBlob::from_bytes(&disc_bytes(4))).unwrap();

        issue_gdrom_packet(&mut gdrom, [0x21, 1, 0, 0, 151, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(gdrom.lba, 151);
        issue_gdrom_packet(&mut gdrom, [0x10, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0]);
        let status = drain_gdrom_fifo(&mut gdrom);
        assert_eq!(status[0], 0x01);
        assert_eq!(&status[5..8], &[0, 0, 151]);

        issue_gdrom_packet(&mut gdrom, [0x21, 2, 0, 2, 2, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(gdrom.lba, 152);

        issue_gdrom_packet(&mut gdrom, [0x21, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(gdrom.lba, 150);
        issue_gdrom_packet(&mut gdrom, [0x10, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(drain_gdrom_fifo(&mut gdrom)[0], 0x02);

        issue_gdrom_packet(&mut gdrom, [0x21, 1, 0, 0, 149, 0, 0, 0, 0, 0, 0, 0]);
        assert_eq!(gdrom.status, 0x51);
        assert_eq!(gdrom.error, 4);
    }

    #[test]
    fn gdrom_read_data_select_and_cd_read2_cover_raw_and_preread_paths() {
        let mut gdrom =
            DreamcastGdrom::new(ResourceBlob::from_bytes(&raw_sector(1, 0x5a, 0xa5))).unwrap();

        issue_gdrom_packet(&mut gdrom, [0x30, 0x20, 0, 0, 150, 0, 0, 0, 0, 0, 1, 0]);
        gdrom.tick(1);
        let user = drain_gdrom_fifo(&mut gdrom);
        assert_eq!(user.len(), 2048);
        assert!(user.iter().all(|byte| *byte == 0x5a));

        issue_gdrom_packet(&mut gdrom, [0x30, 0x10, 0, 0, 150, 0, 0, 0, 0, 0, 1, 0]);
        gdrom.tick(1);
        let raw = drain_gdrom_fifo(&mut gdrom);
        assert_eq!(raw.len(), 2352);
        assert_eq!(
            &raw[..12],
            &[0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0]
        );
        assert_eq!(&raw[12..16], &[0, 2, 0, 1]);
        assert_eq!(raw[16], 0x5a);
        assert_eq!(raw[2064], 0xa5);

        issue_gdrom_packet(&mut gdrom, [0x30, 0x22, 0, 0, 150, 0, 0, 0, 0, 0, 1, 0]);
        assert_eq!(gdrom.status, 0x51);
        assert_eq!(gdrom.error, 4);

        let mut gdrom =
            DreamcastGdrom::new(ResourceBlob::from_bytes(&raw_sector(2, 0x66, 0xcc))).unwrap();
        issue_gdrom_packet(&mut gdrom, [0x30, 0xe6, 0, 0, 150, 0, 0, 0, 0, 0, 1, 0]);
        gdrom.tick(1);
        let mode2 = drain_gdrom_fifo(&mut gdrom);
        assert_eq!(mode2.len(), 2340);
        assert_eq!(&mode2[..4], &[0, 2, 0, 2]);
        assert_eq!(mode2[12], 0x66);
        assert_eq!(mode2[2339], 0xcc);

        let mut gdrom = DreamcastGdrom::new(ResourceBlob::from_bytes(&disc_bytes(2))).unwrap();
        issue_gdrom_packet(&mut gdrom, [0x31, 0x21, 0, 2, 0, 0, 0, 1, 0, 2, 0, 0]);
        gdrom.tick(1);
        let read2 = drain_gdrom_fifo(&mut gdrom);
        assert_eq!(read2.len(), 2048);
        assert!(read2.iter().all(|byte| *byte == 1));
        assert_eq!(gdrom.lba, 150);
    }

    #[test]
    fn gdrom_packet_read_requests_missing_sector_and_resumes_after_hydration() {
        let disc = ResourceBlob::streaming(4 * 2048, 2).unwrap();
        let mut hydration = disc.clone();
        let mut gdrom = DreamcastGdrom::new(disc.clone()).unwrap();
        gdrom.issue_ata_command(0xa0);
        let packet = [0x30, 0x20, 0x00, 0x00, 150, 0, 0, 0, 0, 0, 1, 0];
        for pair in packet.as_chunks::<2>().0 {
            gdrom.write_data16(u16::from_le_bytes([pair[0], pair[1]]));
        }
        gdrom.tick(1);
        assert_eq!(disc.pending_range(), Some((0, 2048)));
        assert!(gdrom.fifo.is_empty());
        let sector = vec![0x5a; 2048];
        hydration.write(0, &sector).unwrap();
        gdrom.tick(1);
        assert_eq!(disc.pending_range(), None);
        assert_eq!(gdrom.fifo.len(), 2048);
        assert_eq!(gdrom.read_data16(), 0x5a5a);
        assert!(gdrom.irq);
    }

    #[test]
    fn aica_arm7_executes_from_shared_sound_ram_after_release() {
        let mut machine = DreamcastMachine::from_bios_and_disc(
            &bios_with_loop(),
            ResourceBlob::from_bytes(&disc_bytes(2)),
        )
        .unwrap();
        let program: [u32; 5] = [
            0xe3a0_005a,
            0xe59f_1004,
            0xe581_0000,
            0xeaff_fffe,
            0x0000_0100,
        ];
        for (index, word) in program.into_iter().enumerate() {
            let offset = index * 4;
            machine.board.sound_ram[offset..offset + 4].copy_from_slice(&word.to_le_bytes());
        }
        machine.board.aica.write8(0x2c00, 0);
        machine.run_arm_credit(200);
        assert!(!machine.arm_held_reset);
        assert!(machine.arm7.cycles > 0);
        assert_eq!(machine.board.sound_ram[0x100], 0x5a);
    }

    #[test]
    fn machine_state_and_flash_round_trip_are_deterministic() {
        let mut machine = DreamcastMachine::from_bios_and_disc(
            &bios_with_loop(),
            ResourceBlob::from_bytes(&disc_bytes(2)),
        )
        .unwrap();
        let flash = vec![0x5a; FLASH_SIZE];
        machine
            .write_persistent(ResourceKind::Storage, 0, &flash)
            .unwrap();
        machine.board.ram[0x1234] = 0x77;
        machine.board.vram[0x4321] = 0x66;
        machine.sh4.r[3] = 0x1122_3344;
        machine.arm7.r[2] = 0x5566_7788;
        let state = machine.save_state().unwrap();
        machine.board.flash.fill(0);
        machine.board.ram[0x1234] = 0;
        machine.board.vram[0x4321] = 0;
        machine.load_state(&state).unwrap();
        assert_eq!(machine.board.flash[0], 0x5a);
        assert_eq!(machine.board.ram[0x1234], 0x77);
        assert_eq!(machine.board.vram[0x4321], 0x66);
        assert_eq!(machine.sh4.r[3], 0x1122_3344);
        assert_eq!(machine.arm7.r[2], 0x5566_7788);
        assert_eq!(machine.save_state().unwrap(), state);
        let mut restored = vec![0; FLASH_SIZE];
        machine
            .read_persistent(ResourceKind::Storage, 0, &mut restored)
            .unwrap();
        assert_eq!(restored, flash);
    }

    #[test]
    fn synthetic_machine_runs_sh4_frame_and_presents_video_audio() {
        let mut machine = DreamcastMachine::from_bios_and_disc(
            &bios_with_loop(),
            ResourceBlob::from_bytes(&disc_bytes(2)),
        )
        .unwrap();
        machine.board.vram[0..2].copy_from_slice(&0xf800u16.to_le_bytes());
        machine.board.write32(0x005f_8044, 1);
        machine.board.write32(0x005f_8050, 0);
        machine.board.write32(0x005f_805c, 0);
        machine.run_frame(&InputState::default());
        assert!(machine.powered);
        assert!(machine.sh4.cycles >= SH4_HZ / FRAME_RATE);
        assert_eq!(machine.board.pvr.frame, 1);
        assert!(machine.video().pixels()[0] > 240);
        assert_eq!(machine.audio().sample_rate(), AUDIO_RATE as u32);
        assert_eq!(machine.audio().channels(), 2);
        assert!(!machine.audio().samples().is_empty());
    }
}
