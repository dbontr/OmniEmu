use std::collections::BTreeSet;

use crate::resources::ResourceBlob;

pub(crate) const CUE_CONTAINER_MAGIC: [u8; 8] = *b"OMCUE001";
const MAX_CUE_BYTES: usize = 1024 * 1024;
const MAX_CUE_FILES: usize = 99;
const MAX_SBI_BYTES: usize = 1024 * 1024;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum DiscLayout {
    Iso2048,
    Raw2352Mode1,
    Raw2352Mode2,
    Raw2352Audio,
}

impl DiscLayout {
    fn sector_bytes(self) -> u64 {
        match self {
            Self::Iso2048 => 2048,
            Self::Raw2352Mode1 | Self::Raw2352Mode2 | Self::Raw2352Audio => 2352,
        }
    }

    fn user_offset(self) -> Option<u64> {
        match self {
            Self::Iso2048 => Some(0),
            Self::Raw2352Mode1 => Some(16),
            Self::Raw2352Mode2 => Some(24),
            Self::Raw2352Audio => None,
        }
    }

    fn is_raw(self) -> bool {
        self != Self::Iso2048
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum TrackKind {
    Data,
    Audio,
}

#[derive(Clone)]
pub(crate) struct DiscTrack {
    pub(crate) number: u8,
    pub(crate) kind: TrackKind,
    pub(crate) layout: DiscLayout,
    pub(crate) pregap_start_lba: u32,
    pub(crate) start_lba: u32,
    pub(crate) end_lba: u32,
    pub(crate) explicit_pregap: u32,
    pub(crate) stored_pregap: u32,
    pub(crate) pregap_source_offset: u64,
    pub(crate) source_offset: u64,
}

#[derive(Clone)]
struct CueDirectoryEntry {
    name: String,
    offset: u64,
    len: u64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CueFileType {
    Binary,
    Wave,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CueTrackMode {
    Audio,
    Mode1_2048,
    Mode1_2352,
    Mode2_2352,
}

impl CueTrackMode {
    fn layout(self) -> DiscLayout {
        match self {
            Self::Audio => DiscLayout::Raw2352Audio,
            Self::Mode1_2048 => DiscLayout::Iso2048,
            Self::Mode1_2352 => DiscLayout::Raw2352Mode1,
            Self::Mode2_2352 => DiscLayout::Raw2352Mode2,
        }
    }

    fn kind(self) -> TrackKind {
        if self == Self::Audio {
            TrackKind::Audio
        } else {
            TrackKind::Data
        }
    }
}

#[derive(Clone)]
struct CueTrackSpec {
    number: u8,
    file_name: String,
    file_type: CueFileType,
    mode: CueTrackMode,
    index0: Option<u32>,
    index1: Option<u32>,
    pregap: u32,
    postgap: u32,
}

#[derive(Clone, Copy)]
struct CueSource {
    offset: u64,
    len: u64,
    sector_bytes: u64,
}

#[derive(Clone)]
pub(crate) struct DiscImage {
    blob: ResourceBlob,
    pub(crate) sectors: u32,
    pub(crate) tracks: Vec<DiscTrack>,
    bad_subq_lbas: BTreeSet<u32>,
}

impl DiscImage {
    pub(crate) fn new(blob: ResourceBlob) -> Result<Self, String> {
        if blob.is_empty() {
            return Err("CD disc resource is empty".into());
        }
        if blob.len() >= CUE_CONTAINER_MAGIC.len() as u64
            && blob.range_present(0, CUE_CONTAINER_MAGIC.len() as u64)
            && Self::read_array::<8>(&blob, 0)? == CUE_CONTAINER_MAGIC
        {
            Self::from_cue_container(blob)
        } else {
            Self::from_single_image(blob)
        }
    }

    fn from_single_image(blob: ResourceBlob) -> Result<Self, String> {
        let mut header = [0u8; 16];
        let header_len = usize::try_from(blob.len().min(header.len() as u64)).unwrap_or(0);
        let header_present = header_len == 0 || blob.range_present(0, header_len as u64);
        if header_len != 0 && header_present {
            blob.read(0, &mut header[..header_len])?;
        }
        let raw_sync = header_present
            && header_len >= 16
            && header[0] == 0
            && header[11] == 0
            && header[1..11].iter().all(|byte| *byte == 0xff);
        let layout = if raw_sync && blob.len().is_multiple_of(2352) {
            if header[15] == 1 {
                DiscLayout::Raw2352Mode1
            } else {
                DiscLayout::Raw2352Mode2
            }
        } else if blob.len().is_multiple_of(2048) {
            DiscLayout::Iso2048
        } else if !header_present && blob.len().is_multiple_of(2352) {
            DiscLayout::Raw2352Mode1
        } else if blob.len().is_multiple_of(2352) {
            DiscLayout::Raw2352Audio
        } else {
            return Err(format!(
                "CD disc length {} is not a supported 2048-byte ISO or 2352-byte raw image",
                blob.len()
            ));
        };
        let sectors = u32::try_from(blob.len() / layout.sector_bytes())
            .map_err(|_| "CD disc has too many sectors".to_string())?;
        let kind = if layout == DiscLayout::Raw2352Audio {
            TrackKind::Audio
        } else {
            TrackKind::Data
        };
        Ok(Self {
            blob,
            sectors,
            tracks: vec![DiscTrack {
                number: 1,
                kind,
                layout,
                pregap_start_lba: 0,
                start_lba: 0,
                end_lba: sectors,
                explicit_pregap: 0,
                stored_pregap: 0,
                pregap_source_offset: 0,
                source_offset: 0,
            }],
            bad_subq_lbas: BTreeSet::new(),
        })
    }

    fn from_cue_container(blob: ResourceBlob) -> Result<Self, String> {
        let cue_len = Self::read_u32(&blob, 8)? as usize;
        let file_count = Self::read_u32(&blob, 12)? as usize;
        if cue_len == 0 || cue_len > MAX_CUE_BYTES {
            return Err("CUE metadata length is invalid".into());
        }
        if file_count == 0 || file_count > MAX_CUE_FILES {
            return Err("CUE file count is invalid".into());
        }
        let cue_bytes = Self::read_vec(&blob, 16, cue_len)?;
        let cue_text = std::str::from_utf8(&cue_bytes)
            .map_err(|_| "CUE sheet is not valid UTF-8".to_string())?;
        let mut cursor = 16u64 + cue_len as u64;
        let mut entries = Vec::with_capacity(file_count);
        for _ in 0..file_count {
            let name_len = usize::from(Self::read_u16(&blob, cursor)?);
            cursor += 2;
            if name_len == 0 || name_len > 4096 {
                return Err("CUE track filename length is invalid".into());
            }
            let name = String::from_utf8(Self::read_vec(&blob, cursor, name_len)?)
                .map_err(|_| "CUE track filename is not valid UTF-8".to_string())?;
            cursor += name_len as u64;
            let len = Self::read_u64(&blob, cursor)?;
            let offset = Self::read_u64(&blob, cursor + 8)?;
            cursor += 16;
            let end = offset
                .checked_add(len)
                .ok_or_else(|| "CUE track file range overflow".to_string())?;
            if len == 0 || offset < cursor || end > blob.len() {
                return Err("CUE track file range is invalid".into());
            }
            entries.push(CueDirectoryEntry { name, offset, len });
        }
        let specs = Self::parse_cue(cue_text)?;
        let mut tracks = Vec::with_capacity(specs.len());
        let mut disc_cursor = 0u32;
        for (index, spec) in specs.iter().enumerate() {
            let entry = Self::cue_entry(&entries, &spec.file_name)?;
            let source = Self::cue_source(&blob, entry, spec.file_type, spec.mode)?;
            let index1 = spec
                .index1
                .ok_or_else(|| format!("CUE track {:02} has no INDEX 01", spec.number))?;
            let index0 = spec.index0.unwrap_or(index1);
            if index0 > index1 {
                return Err(format!(
                    "CUE track {:02} has INDEX 00 after INDEX 01",
                    spec.number
                ));
            }
            let total_source_sectors = u32::try_from(source.len / source.sector_bytes)
                .map_err(|_| "CUE track source is too large".to_string())?;
            if source.len % source.sector_bytes != 0 || index1 > total_source_sectors {
                return Err(format!(
                    "CUE track {:02} exceeds its source file",
                    spec.number
                ));
            }
            let source_end = if let Some(next) = specs.get(index + 1).filter(|next| {
                Self::normalized_file_name(&next.file_name)
                    == Self::normalized_file_name(&spec.file_name)
            }) {
                if next.mode.layout().sector_bytes() != source.sector_bytes {
                    return Err("CUE changes sector size within one source file".into());
                }
                next.index0
                    .or(next.index1)
                    .ok_or_else(|| format!("CUE track {:02} has no index", next.number))?
            } else {
                total_source_sectors
            };
            if source_end < index1 || source_end > total_source_sectors {
                return Err(format!(
                    "CUE track {:02} has an invalid source extent",
                    spec.number
                ));
            }
            let stored_pregap = index1 - index0;
            let pregap_start_lba = disc_cursor;
            let start_lba = pregap_start_lba
                .checked_add(spec.pregap)
                .and_then(|value| value.checked_add(stored_pregap))
                .ok_or_else(|| "CUE pregap LBA overflow".to_string())?;
            let data_sectors = source_end - index1;
            let end_lba = start_lba
                .checked_add(data_sectors)
                .ok_or_else(|| "CUE track LBA overflow".to_string())?;
            let source_offset = source
                .offset
                .checked_add(u64::from(index1) * source.sector_bytes)
                .ok_or_else(|| "CUE source offset overflow".to_string())?;
            let pregap_source_offset = source
                .offset
                .checked_add(u64::from(index0) * source.sector_bytes)
                .ok_or_else(|| "CUE pregap offset overflow".to_string())?;
            tracks.push(DiscTrack {
                number: spec.number,
                kind: spec.mode.kind(),
                layout: spec.mode.layout(),
                pregap_start_lba,
                start_lba,
                end_lba,
                explicit_pregap: spec.pregap,
                stored_pregap,
                pregap_source_offset,
                source_offset,
            });
            disc_cursor = end_lba
                .checked_add(spec.postgap)
                .ok_or_else(|| "CUE postgap LBA overflow".to_string())?;
        }
        if tracks.is_empty() {
            return Err("CUE sheet contains no tracks".into());
        }
        let bad_subq_lbas = Self::load_sbi(&blob, &entries)?;
        Ok(Self {
            blob,
            sectors: disc_cursor,
            tracks,
            bad_subq_lbas,
        })
    }

    fn read_array<const N: usize>(blob: &ResourceBlob, offset: u64) -> Result<[u8; N], String> {
        let mut bytes = [0u8; N];
        blob.read(offset, &mut bytes)?;
        Ok(bytes)
    }

    fn read_vec(blob: &ResourceBlob, offset: u64, len: usize) -> Result<Vec<u8>, String> {
        let mut bytes = vec![0u8; len];
        blob.read(offset, &mut bytes)?;
        Ok(bytes)
    }

    fn read_u16(blob: &ResourceBlob, offset: u64) -> Result<u16, String> {
        Ok(u16::from_le_bytes(Self::read_array(blob, offset)?))
    }

    fn read_u32(blob: &ResourceBlob, offset: u64) -> Result<u32, String> {
        Ok(u32::from_le_bytes(Self::read_array(blob, offset)?))
    }

    fn read_u64(blob: &ResourceBlob, offset: u64) -> Result<u64, String> {
        Ok(u64::from_le_bytes(Self::read_array(blob, offset)?))
    }

    fn load_sbi(
        blob: &ResourceBlob,
        entries: &[CueDirectoryEntry],
    ) -> Result<BTreeSet<u32>, String> {
        let sbi_entries: Vec<_> = entries
            .iter()
            .filter(|entry| entry.name.to_ascii_lowercase().ends_with(".sbi"))
            .collect();
        if sbi_entries.len() > 1 {
            return Err("CUE container contains more than one SBI file".into());
        }
        let Some(entry) = sbi_entries.first() else {
            return Ok(BTreeSet::new());
        };
        let len = usize::try_from(entry.len).map_err(|_| "SBI file is too large".to_string())?;
        if !(4..=MAX_SBI_BYTES).contains(&len) {
            return Err("SBI file length is invalid".into());
        }
        let bytes = Self::read_vec(blob, entry.offset, len)?;
        if &bytes[..4] != b"SBI\0" {
            return Err("SBI file has an invalid header".into());
        }
        let mut bad = BTreeSet::new();
        let mut cursor = 4usize;
        while cursor < bytes.len() {
            if cursor + 4 > bytes.len() {
                return Err("SBI entry is truncated".into());
            }
            let minute = Self::bcd_byte(bytes[cursor])
                .ok_or_else(|| "SBI minute is invalid BCD".to_string())?;
            let second = Self::bcd_byte(bytes[cursor + 1])
                .filter(|value| *value < 60)
                .ok_or_else(|| "SBI second is invalid".to_string())?;
            let frame = Self::bcd_byte(bytes[cursor + 2])
                .filter(|value| *value < 75)
                .ok_or_else(|| "SBI frame is invalid".to_string())?;
            let payload = match bytes[cursor + 3] {
                1 => 10usize,
                2 | 3 => 3usize,
                _ => return Err("SBI entry format is invalid".into()),
            };
            cursor += 4;
            if cursor + payload > bytes.len() {
                return Err("SBI entry payload is truncated".into());
            }
            cursor += payload;
            let absolute = u32::from(minute) * 60 * 75 + u32::from(second) * 75 + u32::from(frame);
            bad.insert(absolute.saturating_sub(150));
        }
        Ok(bad)
    }

    fn bcd_byte(value: u8) -> Option<u8> {
        let high = value >> 4;
        let low = value & 0x0f;
        (high <= 9 && low <= 9).then_some(high * 10 + low)
    }

    fn parse_cue(cue: &str) -> Result<Vec<CueTrackSpec>, String> {
        let mut current_file: Option<(String, CueFileType)> = None;
        let mut tracks: Vec<CueTrackSpec> = Vec::new();
        for raw_line in cue.lines() {
            let line = raw_line.trim().trim_start_matches('\u{feff}');
            if line.is_empty() {
                continue;
            }
            let mut fields = line.split_whitespace();
            let keyword = fields.next().unwrap_or_default().to_ascii_uppercase();
            match keyword.as_str() {
                "FILE" => {
                    current_file = Some(Self::parse_cue_file(&line[4..])?);
                }
                "TRACK" => {
                    let number = fields
                        .next()
                        .ok_or_else(|| "CUE TRACK is missing a number".to_string())?
                        .parse::<u8>()
                        .map_err(|_| "CUE TRACK number is invalid".to_string())?;
                    let mode = match fields
                        .next()
                        .unwrap_or_default()
                        .to_ascii_uppercase()
                        .as_str()
                    {
                        "AUDIO" => CueTrackMode::Audio,
                        "MODE1/2048" => CueTrackMode::Mode1_2048,
                        "MODE1/2352" => CueTrackMode::Mode1_2352,
                        "MODE2/2352" => CueTrackMode::Mode2_2352,
                        other => {
                            return Err(format!("CUE track mode {other} is unsupported"));
                        }
                    };
                    let (file_name, file_type) = current_file
                        .clone()
                        .ok_or_else(|| "CUE TRACK appears before a FILE directive".to_string())?;
                    if number == 0
                        || number > 99
                        || tracks.last().is_some_and(|track| track.number >= number)
                    {
                        return Err("CUE track numbers must increase from 01 to 99".into());
                    }
                    tracks.push(CueTrackSpec {
                        number,
                        file_name,
                        file_type,
                        mode,
                        index0: None,
                        index1: None,
                        pregap: 0,
                        postgap: 0,
                    });
                }
                "INDEX" => {
                    let track = tracks
                        .last_mut()
                        .ok_or_else(|| "CUE INDEX appears before a TRACK directive".to_string())?;
                    let index = fields
                        .next()
                        .ok_or_else(|| "CUE INDEX is missing a number".to_string())?
                        .parse::<u8>()
                        .map_err(|_| "CUE INDEX number is invalid".to_string())?;
                    let position = Self::parse_msf(fields.next().unwrap_or_default())?;
                    match index {
                        0 => track.index0 = Some(position),
                        1 => track.index1 = Some(position),
                        _ => {}
                    }
                }
                "PREGAP" => {
                    let track = tracks
                        .last_mut()
                        .ok_or_else(|| "CUE PREGAP appears before a TRACK directive".to_string())?;
                    track.pregap = Self::parse_msf(fields.next().unwrap_or_default())?;
                }
                "POSTGAP" => {
                    let track = tracks.last_mut().ok_or_else(|| {
                        "CUE POSTGAP appears before a TRACK directive".to_string()
                    })?;
                    track.postgap = Self::parse_msf(fields.next().unwrap_or_default())?;
                }
                _ => {}
            }
        }
        Ok(tracks)
    }

    fn parse_cue_file(rest: &str) -> Result<(String, CueFileType), String> {
        let rest = rest.trim();
        let (name, file_type) = if let Some(quoted) = rest.strip_prefix('"') {
            let end = quoted
                .find('"')
                .ok_or_else(|| "CUE FILE quote is unterminated".to_string())?;
            (&quoted[..end], quoted[end + 1..].trim())
        } else {
            let mut parts = rest.split_whitespace();
            (
                parts.next().unwrap_or_default(),
                parts.next().unwrap_or_default(),
            )
        };
        if name.is_empty() {
            return Err("CUE FILE is missing a filename".into());
        }
        let file_type = match file_type.to_ascii_uppercase().as_str() {
            "BINARY" => CueFileType::Binary,
            "WAVE" => CueFileType::Wave,
            other => return Err(format!("CUE FILE type {other} is unsupported")),
        };
        Ok((name.to_string(), file_type))
    }

    fn parse_msf(value: &str) -> Result<u32, String> {
        let mut fields = value.split(':');
        let minute = fields
            .next()
            .and_then(|field| field.parse::<u32>().ok())
            .ok_or_else(|| "CUE time has an invalid minute".to_string())?;
        let second = fields
            .next()
            .and_then(|field| field.parse::<u32>().ok())
            .filter(|value| *value < 60)
            .ok_or_else(|| "CUE time has an invalid second".to_string())?;
        let frame = fields
            .next()
            .and_then(|field| field.parse::<u32>().ok())
            .filter(|value| *value < 75)
            .ok_or_else(|| "CUE time has an invalid frame".to_string())?;
        if fields.next().is_some() {
            return Err("CUE time has too many fields".into());
        }
        minute
            .checked_mul(60 * 75)
            .and_then(|value| value.checked_add(second * 75))
            .and_then(|value| value.checked_add(frame))
            .ok_or_else(|| "CUE time overflow".to_string())
    }

    fn normalized_file_name(name: &str) -> String {
        name.replace('\\', "/")
            .rsplit('/')
            .next()
            .unwrap_or(name)
            .to_ascii_lowercase()
    }

    fn cue_entry<'a>(
        entries: &'a [CueDirectoryEntry],
        cue_name: &str,
    ) -> Result<&'a CueDirectoryEntry, String> {
        let wanted = Self::normalized_file_name(cue_name);
        let mut matches = entries
            .iter()
            .filter(|entry| Self::normalized_file_name(&entry.name) == wanted);
        let entry = matches
            .next()
            .ok_or_else(|| format!("CUE references missing track file {cue_name}"))?;
        if matches.next().is_some() {
            return Err(format!("CUE track filename {cue_name} is ambiguous"));
        }
        Ok(entry)
    }

    fn cue_source(
        blob: &ResourceBlob,
        entry: &CueDirectoryEntry,
        file_type: CueFileType,
        mode: CueTrackMode,
    ) -> Result<CueSource, String> {
        let sector_bytes = mode.layout().sector_bytes();
        match file_type {
            CueFileType::Binary => Ok(CueSource {
                offset: entry.offset,
                len: entry.len,
                sector_bytes,
            }),
            CueFileType::Wave => {
                if mode != CueTrackMode::Audio {
                    return Err("CUE WAVE files can only back AUDIO tracks".into());
                }
                Self::wave_source(blob, entry)
            }
        }
    }

    fn wave_source(blob: &ResourceBlob, entry: &CueDirectoryEntry) -> Result<CueSource, String> {
        if entry.len < 44 {
            return Err("CUE WAVE track is truncated".into());
        }
        let riff = Self::read_array::<12>(blob, entry.offset)?;
        if &riff[..4] != b"RIFF" || &riff[8..12] != b"WAVE" {
            return Err("CUE WAVE track has an invalid RIFF header".into());
        }
        let mut cursor = 12u64;
        let mut pcm_ok = false;
        let mut data = None;
        while cursor + 8 <= entry.len {
            let header = Self::read_array::<8>(blob, entry.offset + cursor)?;
            let size = u64::from(u32::from_le_bytes(header[4..8].try_into().unwrap()));
            let payload = cursor + 8;
            if payload + size > entry.len {
                return Err("CUE WAVE chunk exceeds the source file".into());
            }
            if &header[..4] == b"fmt " {
                if size < 16 {
                    return Err("CUE WAVE fmt chunk is truncated".into());
                }
                let format = Self::read_array::<16>(blob, entry.offset + payload)?;
                pcm_ok = u16::from_le_bytes([format[0], format[1]]) == 1
                    && u16::from_le_bytes([format[2], format[3]]) == 2
                    && u32::from_le_bytes(format[4..8].try_into().unwrap()) == 44_100
                    && u16::from_le_bytes([format[14], format[15]]) == 16;
            } else if &header[..4] == b"data" {
                data = Some((entry.offset + payload, size));
            }
            cursor = payload
                .checked_add((size + 1) & !1)
                .ok_or_else(|| "CUE WAVE chunk offset overflow".to_string())?;
        }
        if !pcm_ok {
            return Err("CUE WAVE audio must be 44.1 kHz stereo 16-bit PCM".into());
        }
        let (offset, len) = data.ok_or_else(|| "CUE WAVE has no data chunk".to_string())?;
        Ok(CueSource {
            offset,
            len,
            sector_bytes: 2352,
        })
    }

    pub(crate) fn track_for_lba(&self, lba: u32) -> Option<&DiscTrack> {
        self.tracks
            .iter()
            .find(|track| lba >= track.pregap_start_lba && lba < track.end_lba)
    }

    fn source_offset_for_lba(&self, track: &DiscTrack, lba: u32) -> Option<u64> {
        let sector_bytes = track.layout.sector_bytes();
        if lba < track.start_lba {
            let pregap_sector = lba - track.pregap_start_lba;
            if pregap_sector < track.explicit_pregap {
                return None;
            }
            let stored_sector = pregap_sector - track.explicit_pregap;
            if stored_sector >= track.stored_pregap {
                return None;
            }
            track
                .pregap_source_offset
                .checked_add(u64::from(stored_sector) * sector_bytes)
        } else if lba < track.end_lba {
            track
                .source_offset
                .checked_add(u64::from(lba - track.start_lba) * sector_bytes)
        } else {
            None
        }
    }

    pub(crate) fn track_number(&self, lba: u32) -> Option<u8> {
        self.track_for_lba(lba).map(|track| track.number)
    }

    pub(crate) fn track_start(&self, number: u8) -> Option<u32> {
        self.tracks
            .iter()
            .find(|track| track.number == number)
            .map(|track| track.start_lba)
    }

    pub(crate) fn first_track(&self) -> u8 {
        self.tracks.first().map_or(1, |track| track.number)
    }

    pub(crate) fn last_track(&self) -> u8 {
        self.tracks.last().map_or(1, |track| track.number)
    }

    pub(crate) fn subq_crc_bad(&self, lba: u32) -> bool {
        self.bad_subq_lbas.contains(&lba)
    }

    pub(crate) fn is_raw_sector(&self, lba: u32) -> bool {
        self.track_for_lba(lba)
            .is_some_and(|track| track.layout.is_raw())
    }

    pub(crate) fn read_user_sector(&self, lba: u32, out: &mut [u8; 2048]) -> Result<(), String> {
        if lba >= self.sectors {
            return Err("CD-ROM read exceeded disc image".into());
        }
        let Some(track) = self.track_for_lba(lba) else {
            out.fill(0);
            return Ok(());
        };
        let Some(source_offset) = self.source_offset_for_lba(track, lba) else {
            out.fill(0);
            return Ok(());
        };
        let Some(user_offset) = track.layout.user_offset() else {
            out.fill(0);
            return Ok(());
        };
        self.blob.read(source_offset + user_offset, out)
    }

    pub(crate) fn read_raw_sector(&self, lba: u32, out: &mut [u8; 2352]) -> Result<(), String> {
        if lba >= self.sectors {
            return Err("CD audio read exceeded disc image".into());
        }
        let Some(track) = self.track_for_lba(lba) else {
            out.fill(0);
            return Ok(());
        };
        let Some(source_offset) = self.source_offset_for_lba(track, lba) else {
            out.fill(0);
            return Ok(());
        };
        if track.layout == DiscLayout::Iso2048 {
            out.fill(0);
            out[0] = 0;
            out[1..11].fill(0xff);
            out[11] = 0;
            out[15] = 1;
            self.blob.read(source_offset, &mut out[16..16 + 2048])?;
            return Ok(());
        }
        self.blob.read(source_offset, out)
    }
}
