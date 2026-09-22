use std::collections::BTreeMap;

use crate::state::{StateReader, StateWriter};

const PAGE_SHIFT: u32 = 12;
const PAGE_SIZE: usize = 1 << PAGE_SHIFT;
const PAGE_MASK: u64 = PAGE_SIZE as u64 - 1;

#[derive(Clone)]
pub struct SparseMemory {
    len: u64,
    pages: BTreeMap<u64, Box<[u8; PAGE_SIZE]>>,
}

impl SparseMemory {
    pub fn new(len: u64) -> Result<Self, String> {
        if len == 0 {
            return Err("sparse memory length must be non-zero".into());
        }
        Ok(Self {
            len,
            pages: BTreeMap::new(),
        })
    }

    pub fn len(&self) -> u64 {
        self.len
    }
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
    pub fn allocated_pages(&self) -> usize {
        self.pages.len()
    }

    pub fn read8(&self, address: u64) -> u8 {
        if address >= self.len {
            return 0;
        }
        let page = address >> PAGE_SHIFT;
        let offset = (address & PAGE_MASK) as usize;
        self.pages.get(&page).map_or(0, |bytes| bytes[offset])
    }
    pub fn write8(&mut self, address: u64, value: u8) {
        if address >= self.len {
            return;
        }
        let page = address >> PAGE_SHIFT;
        let offset = (address & PAGE_MASK) as usize;
        if value == 0 && !self.pages.contains_key(&page) {
            return;
        }
        self.pages
            .entry(page)
            .or_insert_with(|| Box::new([0; PAGE_SIZE]))[offset] = value;
    }

    pub fn read(&self, address: u64, output: &mut [u8]) -> Result<(), String> {
        let end = address
            .checked_add(output.len() as u64)
            .filter(|end| *end <= self.len)
            .ok_or_else(|| "sparse memory read exceeds address space".to_string())?;
        let _ = end;
        for (offset, byte) in output.iter_mut().enumerate() {
            *byte = self.read8(address + offset as u64);
        }
        Ok(())
    }

    pub fn write(&mut self, address: u64, input: &[u8]) -> Result<(), String> {
        address
            .checked_add(input.len() as u64)
            .filter(|end| *end <= self.len)
            .ok_or_else(|| "sparse memory write exceeds address space".to_string())?;
        for (offset, byte) in input.iter().copied().enumerate() {
            self.write8(address + offset as u64, byte);
        }
        Ok(())
    }
    pub fn clear(&mut self) {
        self.pages.clear();
    }

    pub fn zero(&mut self, address: u64, len: u64) -> Result<(), String> {
        let end = address
            .checked_add(len)
            .filter(|end| *end <= self.len)
            .ok_or_else(|| "sparse memory zero range exceeds address space".to_string())?;
        if len == 0 {
            return Ok(());
        }
        let first_page = address >> PAGE_SHIFT;
        let last_page = (end - 1) >> PAGE_SHIFT;
        for page in first_page..=last_page {
            let page_start = page << PAGE_SHIFT;
            let start = address.saturating_sub(page_start).min(PAGE_SIZE as u64) as usize;
            let stop = end.saturating_sub(page_start).min(PAGE_SIZE as u64) as usize;
            if start == 0 && stop == PAGE_SIZE {
                self.pages.remove(&page);
            } else if let Some(bytes) = self.pages.get_mut(&page) {
                bytes[start..stop].fill(0);
                if bytes.iter().all(|byte| *byte == 0) {
                    self.pages.remove(&page);
                }
            }
        }
        Ok(())
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.u64(self.len);
        out.u32(self.pages.len() as u32);
        for (&page, bytes) in &self.pages {
            out.u64(page);
            out.blob(bytes.as_slice());
        }
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        if input.u64()? != self.len {
            return Err("sparse memory state has the wrong length".into());
        }
        let count = input.u32()? as usize;
        let max_pages = self.len.div_ceil(PAGE_SIZE as u64) as usize;
        if count > max_pages {
            return Err("sparse memory state has too many pages".into());
        }
        self.pages.clear();
        for _ in 0..count {
            let page = input.u64()?;
            if page >= max_pages as u64 || self.pages.contains_key(&page) {
                return Err("sparse memory state has an invalid page index".into());
            }
            let bytes = input.blob()?;
            if bytes.len() != PAGE_SIZE {
                return Err("sparse memory state has an invalid page size".into());
            }
            let mut page_bytes = Box::new([0; PAGE_SIZE]);
            page_bytes.copy_from_slice(bytes);
            self.pages.insert(page, page_bytes);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    #[test]
    fn multi_gib_address_space_allocates_only_touched_pages() {
        let mut memory = SparseMemory::new(4 * 1024 * 1024 * 1024).unwrap();
        memory.write(0x1234_5000, &[1, 2, 3, 4]).unwrap();
        memory.write(0xf000_0000, &[5, 6]).unwrap();
        assert_eq!(memory.allocated_pages(), 2);
        let mut output = [0; 4];
        memory.read(0x1234_5000, &mut output).unwrap();
        assert_eq!(output, [1, 2, 3, 4]);
        assert_eq!(memory.read8(0x7000_0000), 0);
    }

    #[test]
    fn sparse_state_round_trip_preserves_pages() {
        let mut memory = SparseMemory::new(512 * 1024 * 1024).unwrap();
        memory.write(0x1000, &[0xaa, 0xbb]).unwrap();
        let mut writer = StateWriter::new(PlatformId::Xbox360, 9);
        memory.save(&mut writer);
        let state = writer.finish();
        let mut reader = StateReader::new(&state, PlatformId::Xbox360, 9).unwrap();
        let mut restored = SparseMemory::new(512 * 1024 * 1024).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();
        assert_eq!(restored.read8(0x1000), 0xaa);
        assert_eq!(restored.read8(0x1001), 0xbb);
    }
}
