use std::collections::{BTreeMap, VecDeque};
use std::sync::Mutex;

use crate::resources::ResourceBlob;

pub trait RandomAccessMedia: Send + Sync {
    fn len(&self) -> u64;
    fn read_at(&self, offset: u64, output: &mut [u8]) -> Result<(), String>;
    fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

impl RandomAccessMedia for ResourceBlob {
    fn len(&self) -> u64 {
        ResourceBlob::len(self)
    }
    fn read_at(&self, offset: u64, output: &mut [u8]) -> Result<(), String> {
        self.read(offset, output)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlockGeometry {
    pub block_size: u32,
    pub block_count: u64,
}

impl BlockGeometry {
    pub fn for_bytes(bytes: u64, block_size: u32) -> Result<Self, String> {
        if block_size == 0 {
            return Err("block size cannot be zero".into());
        }
        let block_size_u64 = block_size as u64;
        let block_count = bytes.div_ceil(block_size_u64);
        Ok(Self {
            block_size,
            block_count,
        })
    }
}
pub struct BlockReader<'a, T: RandomAccessMedia + ?Sized> {
    source: &'a T,
    geometry: BlockGeometry,
}

impl<'a, T: RandomAccessMedia + ?Sized> BlockReader<'a, T> {
    pub fn new(source: &'a T, block_size: u32) -> Result<Self, String> {
        Ok(Self {
            source,
            geometry: BlockGeometry::for_bytes(source.len(), block_size)?,
        })
    }

    pub fn geometry(&self) -> BlockGeometry {
        self.geometry
    }

    pub fn read_block(&self, block: u64, output: &mut [u8]) -> Result<(), String> {
        if block >= self.geometry.block_count {
            return Err("block index is out of range".into());
        }
        if output.len() != self.geometry.block_size as usize {
            return Err("block output buffer has the wrong size".into());
        }
        let offset = block
            .checked_mul(self.geometry.block_size as u64)
            .ok_or_else(|| "block offset overflow".to_string())?;
        if offset + output.len() as u64 <= self.source.len() {
            return self.source.read_at(offset, output);
        }
        let available = usize::try_from(self.source.len() - offset)
            .map_err(|_| "block tail is too large".to_string())?;
        self.source.read_at(offset, &mut output[..available])?;
        output[available..].fill(0);
        Ok(())
    }
}

#[derive(Debug)]
struct PageCacheState {
    pages: BTreeMap<u64, Vec<u8>>,
    order: VecDeque<u64>,
}

pub struct PagedMedia<T: RandomAccessMedia> {
    source: T,
    page_size: usize,
    max_pages: usize,
    cache: Mutex<PageCacheState>,
}

impl<T: RandomAccessMedia> PagedMedia<T> {
    pub fn new(source: T, page_size: usize, max_pages: usize) -> Result<Self, String> {
        if page_size == 0 || max_pages == 0 {
            return Err("media page size and cache capacity must be non-zero".into());
        }
        Ok(Self {
            source,
            page_size,
            max_pages,
            cache: Mutex::new(PageCacheState {
                pages: BTreeMap::new(),
                order: VecDeque::new(),
            }),
        })
    }

    pub fn cached_pages(&self) -> usize {
        self.cache.lock().unwrap().pages.len()
    }

    fn touch(state: &mut PageCacheState, page: u64) {
        if let Some(index) = state.order.iter().position(|candidate| *candidate == page) {
            state.order.remove(index);
        }
        state.order.push_back(page);
    }

    fn ensure_page(&self, page: u64, state: &mut PageCacheState) -> Result<(), String> {
        if state.pages.contains_key(&page) {
            Self::touch(state, page);
            return Ok(());
        }
        let start = page
            .checked_mul(self.page_size as u64)
            .ok_or_else(|| "media page offset overflow".to_string())?;
        if start >= self.source.len() {
            return Err("media page is out of range".into());
        }
        let available = usize::try_from((self.source.len() - start).min(self.page_size as u64))
            .map_err(|_| "media page length overflow".to_string())?;
        let mut bytes = vec![0; self.page_size];
        self.source.read_at(start, &mut bytes[..available])?;
        while state.pages.len() >= self.max_pages {
            if let Some(evicted) = state.order.pop_front() {
                state.pages.remove(&evicted);
            }
        }
        state.pages.insert(page, bytes);
        Self::touch(state, page);
        Ok(())
    }
}

impl<T: RandomAccessMedia> RandomAccessMedia for PagedMedia<T> {
    fn len(&self) -> u64 {
        self.source.len()
    }

    fn read_at(&self, offset: u64, output: &mut [u8]) -> Result<(), String> {
        let end = offset
            .checked_add(output.len() as u64)
            .ok_or_else(|| "paged media read overflow".to_string())?;
        if end > self.source.len() {
            return Err("paged media read exceeds source length".into());
        }
        let mut state = self.cache.lock().unwrap();
        let mut cursor = 0usize;
        while cursor < output.len() {
            let absolute = offset + cursor as u64;
            let page = absolute / self.page_size as u64;
            let page_offset = (absolute % self.page_size as u64) as usize;
            let count = (self.page_size - page_offset).min(output.len() - cursor);
            self.ensure_page(page, &mut state)?;
            let bytes = state
                .pages
                .get(&page)
                .ok_or_else(|| "media page disappeared from cache".to_string())?;
            output[cursor..cursor + count]
                .copy_from_slice(&bytes[page_offset..page_offset + count]);
            cursor += count;
        }
        Ok(())
    }
}

#[derive(Debug, Clone)]
pub struct SparseBlockStorage {
    geometry: BlockGeometry,
    blocks: BTreeMap<u64, Vec<u8>>,
}
impl SparseBlockStorage {
    pub fn new(block_size: u32, block_count: u64) -> Result<Self, String> {
        if block_size == 0 || block_count == 0 {
            return Err("storage geometry must be non-zero".into());
        }
        Ok(Self {
            geometry: BlockGeometry {
                block_size,
                block_count,
            },
            blocks: BTreeMap::new(),
        })
    }

    pub fn geometry(&self) -> BlockGeometry {
        self.geometry
    }
    pub fn allocated_blocks(&self) -> usize {
        self.blocks.len()
    }

    pub fn read_block(&self, block: u64, output: &mut [u8]) -> Result<(), String> {
        self.validate(block, output.len())?;
        if let Some(bytes) = self.blocks.get(&block) {
            output.copy_from_slice(bytes);
        } else {
            output.fill(0);
        }
        Ok(())
    }

    pub fn write_block(&mut self, block: u64, input: &[u8]) -> Result<(), String> {
        self.validate(block, input.len())?;
        if input.iter().all(|&byte| byte == 0) {
            self.blocks.remove(&block);
        } else {
            self.blocks.insert(block, input.to_vec());
        }
        Ok(())
    }

    fn validate(&self, block: u64, len: usize) -> Result<(), String> {
        if block >= self.geometry.block_count {
            return Err("storage block index is out of range".into());
        }
        if len != self.geometry.block_size as usize {
            return Err("storage block buffer has the wrong size".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn large_media_can_be_sparse_without_multi_gigabyte_allocation() {
        let size = 8u64 * 1024 * 1024 * 1024;
        let mut blob = ResourceBlob::empty(size);
        let offset = size - 2048;
        blob.write(offset, &[0x5a; 2048]).unwrap();
        let reader = BlockReader::new(&blob, 2048).unwrap();
        let mut block = [0u8; 2048];
        reader
            .read_block(reader.geometry().block_count - 1, &mut block)
            .unwrap();
        assert!(block.iter().all(|&value| value == 0x5a));
    }

    #[test]
    fn sparse_storage_allocates_only_written_blocks() {
        let mut storage = SparseBlockStorage::new(512, 10_000_000).unwrap();
        let payload = [0xa5; 512];
        storage.write_block(9_999_999, &payload).unwrap();
        assert_eq!(storage.allocated_blocks(), 1);
        let mut output = [0u8; 512];
        storage.read_block(9_999_999, &mut output).unwrap();
        assert_eq!(output, payload);
        storage.write_block(9_999_999, &[0; 512]).unwrap();
        assert_eq!(storage.allocated_blocks(), 0);
    }

    struct PatternMedia {
        len: u64,
    }

    impl RandomAccessMedia for PatternMedia {
        fn len(&self) -> u64 {
            self.len
        }

        fn read_at(&self, offset: u64, output: &mut [u8]) -> Result<(), String> {
            for (index, byte) in output.iter_mut().enumerate() {
                *byte = offset.wrapping_add(index as u64) as u8;
            }
            Ok(())
        }
    }

    #[test]
    fn paged_media_bounds_cache_while_serving_cross_page_reads() {
        let media = PagedMedia::new(PatternMedia { len: 1 << 40 }, 16, 2).unwrap();
        let mut output = [0u8; 24];
        media.read_at(8, &mut output).unwrap();
        assert_eq!(output[0], 8);
        assert_eq!(output[23], 31);
        assert_eq!(media.cached_pages(), 2);
        media.read_at(40, &mut output).unwrap();
        assert_eq!(output[0], 40);
        assert_eq!(output[23], 63);
        assert_eq!(media.cached_pages(), 2);
    }
}
