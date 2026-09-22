use std::collections::{BTreeMap, VecDeque};
use std::sync::{Arc, Mutex};

const CHUNK_SIZE: usize = 64 * 1024;

#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ResourceKind {
    Game = 0,
    Bios = 1,
    Firmware = 2,
    Keys = 3,
    Disc = 4,
    Storage = 5,
    MemoryCard = 6,
    Nand = 7,
}

impl ResourceKind {
    pub const fn from_u32(value: u32) -> Option<Self> {
        Some(match value {
            0 => Self::Game,
            1 => Self::Bios,
            2 => Self::Firmware,
            3 => Self::Keys,
            4 => Self::Disc,
            5 => Self::Storage,
            6 => Self::MemoryCard,
            7 => Self::Nand,
            _ => return None,
        })
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct ResourceKey {
    pub kind: ResourceKind,
    pub slot: u32,
}

pub const RESOURCE_PENDING: &str = "resource range has not been staged yet";

#[derive(Debug, Default)]
struct ResourceState {
    chunks: BTreeMap<u64, Arc<Vec<u8>>>,
    present: Vec<(u64, u64)>,
    pending: Option<(u64, u64)>,
    order: VecDeque<u64>,
    max_chunks: Option<usize>,
}

#[derive(Debug, Clone)]
pub struct ResourceBlob {
    len: u64,
    state: Arc<Mutex<ResourceState>>,
}

impl ResourceBlob {
    pub fn empty(len: u64) -> Self {
        Self {
            len,
            state: Arc::new(Mutex::new(ResourceState::default())),
        }
    }

    pub fn streaming(len: u64, max_chunks: usize) -> Result<Self, String> {
        if max_chunks == 0 {
            return Err("streaming resource cache must hold at least one chunk".into());
        }
        let state = ResourceState {
            max_chunks: Some(max_chunks),
            ..ResourceState::default()
        };
        Ok(Self {
            len,
            state: Arc::new(Mutex::new(state)),
        })
    }

    pub fn from_bytes(bytes: &[u8]) -> Self {
        let mut blob = Self::empty(bytes.len() as u64);
        blob.write(0, bytes).expect("full resource write must fit");
        blob
    }

    pub fn len(&self) -> u64 {
        self.len
    }
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn write(&mut self, offset: u64, bytes: &[u8]) -> Result<(), String> {
        let end = offset
            .checked_add(bytes.len() as u64)
            .ok_or_else(|| "resource write overflow".to_string())?;
        if end > self.len {
            return Err("resource write exceeds declared length".into());
        }
        let mut state = self.state.lock().unwrap();
        let mut cursor = 0usize;
        while cursor < bytes.len() {
            let absolute = offset + cursor as u64;
            let chunk_index = absolute / CHUNK_SIZE as u64;
            let chunk_offset = (absolute % CHUNK_SIZE as u64) as usize;
            let count = (CHUNK_SIZE - chunk_offset).min(bytes.len() - cursor);
            {
                let chunk = state
                    .chunks
                    .entry(chunk_index)
                    .or_insert_with(|| Arc::new(vec![0; CHUNK_SIZE]));
                Arc::make_mut(chunk)[chunk_offset..chunk_offset + count]
                    .copy_from_slice(&bytes[cursor..cursor + count]);
            }
            Self::touch_chunk(&mut state, chunk_index);
            cursor += count;
        }
        Self::add_present_range(&mut state, offset, end);
        Self::evict_to_limit(&mut state, self.len);
        if state
            .pending
            .is_some_and(|(start, end)| Self::contains_range(&state, start, end))
        {
            state.pending = None;
        }
        Ok(())
    }

    pub fn read(&self, offset: u64, output: &mut [u8]) -> Result<(), String> {
        let end = offset
            .checked_add(output.len() as u64)
            .ok_or_else(|| "resource read overflow".to_string())?;
        if end > self.len {
            return Err("resource read exceeds declared length".into());
        }
        let mut state = self.state.lock().unwrap();
        if !Self::contains_range(&state, offset, end) {
            state.pending = Some(match state.pending {
                Some((start, previous_end)) => (start.min(offset), previous_end.max(end)),
                None => (offset, end),
            });
            return Err(RESOURCE_PENDING.into());
        }
        let mut cursor = 0usize;
        while cursor < output.len() {
            let absolute = offset + cursor as u64;
            let chunk_index = absolute / CHUNK_SIZE as u64;
            let chunk_offset = (absolute % CHUNK_SIZE as u64) as usize;
            let count = (CHUNK_SIZE - chunk_offset).min(output.len() - cursor);
            {
                let chunk = state
                    .chunks
                    .get(&chunk_index)
                    .ok_or_else(|| "resource chunk is missing".to_string())?;
                output[cursor..cursor + count]
                    .copy_from_slice(&chunk[chunk_offset..chunk_offset + count]);
            }
            Self::touch_chunk(&mut state, chunk_index);
            cursor += count;
        }
        Ok(())
    }

    pub fn materialize(&self, max_len: usize) -> Result<Vec<u8>, String> {
        let len = usize::try_from(self.len)
            .map_err(|_| "resource is too large to materialize".to_string())?;
        if len > max_len {
            return Err(format!(
                "resource is {len} bytes; materialization limit is {max_len}"
            ));
        }
        if !self.range_present(0, self.len) {
            return Err("resource is not completely staged".into());
        }
        let mut bytes = vec![0; len];
        self.read(0, &mut bytes)?;
        Ok(bytes)
    }

    pub fn is_complete(&self) -> bool {
        self.range_present(0, self.len)
    }

    pub fn range_present(&self, start: u64, end: u64) -> bool {
        Self::contains_range(&self.state.lock().unwrap(), start, end)
    }

    pub fn pending_range(&self) -> Option<(u64, u64)> {
        self.state.lock().unwrap().pending
    }

    pub fn allocated_chunks(&self) -> usize {
        self.state.lock().unwrap().chunks.len()
    }

    fn contains_range(state: &ResourceState, start: u64, end: u64) -> bool {
        start == end || state.present.iter().any(|&(a, b)| a <= start && b >= end)
    }

    fn add_present_range(state: &mut ResourceState, mut start: u64, mut end: u64) {
        let mut merged = Vec::with_capacity(state.present.len() + 1);
        let mut inserted = false;
        for &(a, b) in &state.present {
            if b < start {
                merged.push((a, b));
            } else if end < a {
                if !inserted {
                    merged.push((start, end));
                    inserted = true;
                }
                merged.push((a, b));
            } else {
                start = start.min(a);
                end = end.max(b);
            }
        }
        if !inserted {
            merged.push((start, end));
        }
        state.present = merged;
    }

    fn remove_present_range(state: &mut ResourceState, start: u64, end: u64) {
        let mut remaining = Vec::with_capacity(state.present.len() + 1);
        for &(a, b) in &state.present {
            if b <= start || a >= end {
                remaining.push((a, b));
                continue;
            }
            if a < start {
                remaining.push((a, start));
            }
            if b > end {
                remaining.push((end, b));
            }
        }
        state.present = remaining;
    }

    fn touch_chunk(state: &mut ResourceState, chunk: u64) {
        if let Some(index) = state.order.iter().position(|candidate| *candidate == chunk) {
            state.order.remove(index);
        }
        state.order.push_back(chunk);
    }

    fn evict_to_limit(state: &mut ResourceState, len: u64) {
        let Some(max_chunks) = state.max_chunks else {
            return;
        };
        while state.chunks.len() > max_chunks {
            let Some(chunk) = state.order.pop_front() else {
                break;
            };
            if state.chunks.remove(&chunk).is_none() {
                continue;
            }
            let start = chunk.saturating_mul(CHUNK_SIZE as u64);
            let end = start.saturating_add(CHUNK_SIZE as u64).min(len);
            Self::remove_present_range(state, start, end);
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct ResourceStore {
    blobs: BTreeMap<ResourceKey, ResourceBlob>,
}

impl ResourceStore {
    pub fn clear(&mut self) {
        self.blobs.clear();
    }

    pub fn create(&mut self, kind: ResourceKind, slot: u32, len: u64) {
        self.blobs
            .insert(ResourceKey { kind, slot }, ResourceBlob::empty(len));
    }

    pub fn create_streaming(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        len: u64,
        max_chunks: usize,
    ) -> Result<(), String> {
        self.blobs.insert(
            ResourceKey { kind, slot },
            ResourceBlob::streaming(len, max_chunks)?,
        );
        Ok(())
    }

    pub fn insert_bytes(&mut self, kind: ResourceKind, slot: u32, bytes: &[u8]) {
        self.blobs
            .insert(ResourceKey { kind, slot }, ResourceBlob::from_bytes(bytes));
    }
    pub fn write(
        &mut self,
        kind: ResourceKind,
        slot: u32,
        offset: u64,
        bytes: &[u8],
    ) -> Result<(), String> {
        self.blobs
            .get_mut(&ResourceKey { kind, slot })
            .ok_or_else(|| "resource slot has not been created".to_string())?
            .write(offset, bytes)
    }

    pub fn get(&self, kind: ResourceKind, slot: u32) -> Option<&ResourceBlob> {
        self.blobs.get(&ResourceKey { kind, slot })
    }

    pub fn get_mut(&mut self, kind: ResourceKind, slot: u32) -> Option<&mut ResourceBlob> {
        self.blobs.get_mut(&ResourceKey { kind, slot })
    }

    pub fn primary_game(&self) -> Option<&ResourceBlob> {
        self.get(ResourceKind::Game, 0)
            .or_else(|| self.get(ResourceKind::Disc, 0))
    }

    pub fn contains_complete(&self, kind: ResourceKind, slot: u32) -> bool {
        self.get(kind, slot).is_some_and(ResourceBlob::is_complete)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn chunked_resource_accepts_out_of_order_writes() {
        let mut blob = ResourceBlob::empty((CHUNK_SIZE * 2 + 17) as u64);
        let second = vec![0x22; CHUNK_SIZE + 17];
        let first = vec![0x11; CHUNK_SIZE];
        blob.write(CHUNK_SIZE as u64, &second).unwrap();
        assert!(!blob.is_complete());
        blob.write(0, &first).unwrap();
        assert!(blob.is_complete());
        let all = blob.materialize(CHUNK_SIZE * 3).unwrap();
        assert_eq!(&all[..CHUNK_SIZE], &first);
        assert_eq!(&all[CHUNK_SIZE..], &second);
    }

    #[test]
    fn store_distinguishes_resource_kinds_and_slots() {
        let mut store = ResourceStore::default();
        store.insert_bytes(ResourceKind::Bios, 0, &[1, 2, 3]);
        store.insert_bytes(ResourceKind::Firmware, 2, &[4, 5]);
        assert_eq!(store.get(ResourceKind::Bios, 0).unwrap().len(), 3);
        assert_eq!(store.get(ResourceKind::Firmware, 2).unwrap().len(), 2);
        assert!(store.get(ResourceKind::Firmware, 0).is_none());
    }

    #[test]
    fn unstaged_ranges_fail_closed() {
        let mut blob = ResourceBlob::empty(128);
        blob.write(0, &[1, 2, 3, 4]).unwrap();
        let mut output = [0u8; 4];
        assert_eq!(blob.read(64, &mut output).unwrap_err(), RESOURCE_PENDING);
        assert_eq!(blob.pending_range(), Some((64, 68)));
    }

    #[test]
    fn streaming_clones_share_live_pages_and_evict_lru_chunks() {
        let mut staged = ResourceBlob::streaming((CHUNK_SIZE * 4) as u64, 2).unwrap();
        let machine_view = staged.clone();
        staged.write(0, &[0x11; CHUNK_SIZE]).unwrap();
        staged
            .write(CHUNK_SIZE as u64, &[0x22; CHUNK_SIZE])
            .unwrap();
        let mut byte = [0u8; 1];
        machine_view.read(0, &mut byte).unwrap();
        assert_eq!(byte[0], 0x11);
        staged
            .write((CHUNK_SIZE * 2) as u64, &[0x33; CHUNK_SIZE])
            .unwrap();
        assert_eq!(machine_view.allocated_chunks(), 2);
        assert_eq!(
            machine_view.read(CHUNK_SIZE as u64, &mut byte).unwrap_err(),
            RESOURCE_PENDING
        );
        assert_eq!(
            machine_view.pending_range(),
            Some((CHUNK_SIZE as u64, CHUNK_SIZE as u64 + 1))
        );
        staged
            .write(CHUNK_SIZE as u64, &[0x44; CHUNK_SIZE])
            .unwrap();
        machine_view.read(CHUNK_SIZE as u64, &mut byte).unwrap();
        assert_eq!(byte[0], 0x44);
        assert_eq!(machine_view.pending_range(), None);
        assert_eq!(machine_view.allocated_chunks(), 2);
    }
}
