#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Endian {
    Little,
    Big,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BusFault {
    pub address: u64,
    pub width: u8,
    pub reason: &'static str,
}

pub type BusResult<T> = Result<T, BusFault>;

pub trait AddressSpace: Send {
    fn read8(&mut self, address: u64) -> BusResult<u8>;
    fn write8(&mut self, address: u64, value: u8) -> BusResult<()>;

    fn read16(&mut self, address: u64, endian: Endian) -> BusResult<u16> {
        let bytes = [self.read8(address)?, self.read8(address.wrapping_add(1))?];
        Ok(match endian {
            Endian::Little => u16::from_le_bytes(bytes),
            Endian::Big => u16::from_be_bytes(bytes),
        })
    }

    fn read32(&mut self, address: u64, endian: Endian) -> BusResult<u32> {
        let mut bytes = [0u8; 4];
        for (offset, slot) in bytes.iter_mut().enumerate() {
            *slot = self.read8(address.wrapping_add(offset as u64))?;
        }
        Ok(match endian {
            Endian::Little => u32::from_le_bytes(bytes),
            Endian::Big => u32::from_be_bytes(bytes),
        })
    }

    fn read64(&mut self, address: u64, endian: Endian) -> BusResult<u64> {
        let mut bytes = [0u8; 8];
        for (offset, slot) in bytes.iter_mut().enumerate() {
            *slot = self.read8(address.wrapping_add(offset as u64))?;
        }
        Ok(match endian {
            Endian::Little => u64::from_le_bytes(bytes),
            Endian::Big => u64::from_be_bytes(bytes),
        })
    }

    fn write16(&mut self, address: u64, value: u16, endian: Endian) -> BusResult<()> {
        let bytes = match endian {
            Endian::Little => value.to_le_bytes(),
            Endian::Big => value.to_be_bytes(),
        };
        for (offset, byte) in bytes.into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u64), byte)?;
        }
        Ok(())
    }

    fn write32(&mut self, address: u64, value: u32, endian: Endian) -> BusResult<()> {
        let bytes = match endian {
            Endian::Little => value.to_le_bytes(),
            Endian::Big => value.to_be_bytes(),
        };
        for (offset, byte) in bytes.into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u64), byte)?;
        }
        Ok(())
    }

    fn write64(&mut self, address: u64, value: u64, endian: Endian) -> BusResult<()> {
        let bytes = match endian {
            Endian::Little => value.to_le_bytes(),
            Endian::Big => value.to_be_bytes(),
        };
        for (offset, byte) in bytes.into_iter().enumerate() {
            self.write8(address.wrapping_add(offset as u64), byte)?;
        }
        Ok(())
    }
}
pub struct FlatMemory {
    base: u64,
    bytes: Vec<u8>,
    read_only: bool,
}

impl FlatMemory {
    pub fn ram(base: u64, size: usize) -> Self {
        Self {
            base,
            bytes: vec![0; size],
            read_only: false,
        }
    }
    pub fn rom(base: u64, bytes: Vec<u8>) -> Self {
        Self {
            base,
            bytes,
            read_only: true,
        }
    }
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    fn index(&self, address: u64, width: u8) -> BusResult<usize> {
        let Some(offset) = address.checked_sub(self.base) else {
            return Err(BusFault {
                address,
                width,
                reason: "address below mapping",
            });
        };
        let index = offset as usize;
        if index >= self.bytes.len() || offset > usize::MAX as u64 {
            return Err(BusFault {
                address,
                width,
                reason: "address outside mapping",
            });
        }
        Ok(index)
    }
}

impl AddressSpace for FlatMemory {
    fn read8(&mut self, address: u64) -> BusResult<u8> {
        let index = self.index(address, 1)?;
        Ok(self.bytes[index])
    }
    fn write8(&mut self, address: u64, value: u8) -> BusResult<()> {
        if self.read_only {
            return Err(BusFault {
                address,
                width: 1,
                reason: "write to read-only mapping",
            });
        }
        let index = self.index(address, 1)?;
        self.bytes[index] = value;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn address_space_supports_64_bit_addresses_and_both_endiannesses() {
        let mut memory = FlatMemory::ram(0x1_0000_0000, 32);
        memory
            .write32(0x1_0000_0004, 0x1234_5678, Endian::Little)
            .unwrap();
        assert_eq!(
            memory.read32(0x1_0000_0004, Endian::Little).unwrap(),
            0x1234_5678
        );
        assert_eq!(
            memory.read32(0x1_0000_0004, Endian::Big).unwrap(),
            0x7856_3412
        );
        memory
            .write64(0x1_0000_0010, 0x0102_0304_0506_0708, Endian::Big)
            .unwrap();
        assert_eq!(
            memory.read64(0x1_0000_0010, Endian::Big).unwrap(),
            0x0102_0304_0506_0708
        );
    }

    #[test]
    fn rom_mapping_rejects_writes() {
        let mut memory = FlatMemory::rom(0x8000, vec![0xaa; 16]);
        assert_eq!(memory.read8(0x8000).unwrap(), 0xaa);
        assert!(memory.write8(0x8000, 0x55).is_err());
    }
}
