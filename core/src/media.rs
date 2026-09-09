use std::collections::BTreeMap;

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
}
