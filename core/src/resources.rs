use std::collections::BTreeMap;

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

#[derive(Debug, Clone)]
pub struct ResourceBlob {
    len: u64,
    chunks: BTreeMap<u64, Vec<u8>>,
    present: Vec<(u64, u64)>,
}

impl ResourceBlob {
    pub fn empty(len: u64) -> Self {
        Self {
            len,
            chunks: BTreeMap::new(),
            present: Vec::new(),
        }
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
        let mut cursor = 0usize;
        while cursor < bytes.len() {
            let absolute = offset + cursor as u64;
            let chunk_index = absolute / CHUNK_SIZE as u64;
            let chunk_offset = (absolute % CHUNK_SIZE as u64) as usize;
            let count = (CHUNK_SIZE - chunk_offset).min(bytes.len() - cursor);
            let chunk = self
                .chunks
                .entry(chunk_index)
                .or_insert_with(|| vec![0; CHUNK_SIZE]);
            chunk[chunk_offset..chunk_offset + count]
                .copy_from_slice(&bytes[cursor..cursor + count]);
            cursor += count;
        }
        self.add_present_range(offset, end);
        Ok(())
    }

    pub fn read(&self, offset: u64, output: &mut [u8]) -> Result<(), String> {
        let end = offset
            .checked_add(output.len() as u64)
            .ok_or_else(|| "resource read overflow".to_string())?;
        if end > self.len {
            return Err("resource read exceeds declared length".into());
        }
        if !self.range_present(offset, end) {
            return Err("resource range has not been staged yet".into());
        }
        let mut cursor = 0usize;
        while cursor < output.len() {
            let absolute = offset + cursor as u64;
            let chunk_index = absolute / CHUNK_SIZE as u64;
            let chunk_offset = (absolute % CHUNK_SIZE as u64) as usize;
            let count = (CHUNK_SIZE - chunk_offset).min(output.len() - cursor);
            let chunk = self
                .chunks
                .get(&chunk_index)
                .ok_or_else(|| "resource chunk is missing".to_string())?;
            output[cursor..cursor + count]
                .copy_from_slice(&chunk[chunk_offset..chunk_offset + count]);
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

    fn range_present(&self, start: u64, end: u64) -> bool {
        if start == end {
            return true;
        }
        self.present.iter().any(|&(a, b)| a <= start && b >= end)
    }

    fn add_present_range(&mut self, mut start: u64, mut end: u64) {
        let mut merged = Vec::with_capacity(self.present.len() + 1);
        let mut inserted = false;
        for &(a, b) in &self.present {
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
        self.present = merged;
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
        assert!(blob.read(64, &mut output).is_err());
    }
}
