use std::collections::BTreeMap;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AccessType {
    Read,
    Write,
    Execute,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PagePermissions {
    pub read: bool,
    pub write: bool,
    pub execute: bool,
}

impl PagePermissions {
    pub const READ_ONLY: Self = Self {
        read: true,
        write: false,
        execute: false,
    };
    pub const READ_EXECUTE: Self = Self {
        read: true,
        write: false,
        execute: true,
    };
    pub const READ_WRITE: Self = Self {
        read: true,
        write: true,
        execute: false,
    };
    pub const ALL: Self = Self {
        read: true,
        write: true,
        execute: true,
    };

    const fn allows(self, access: AccessType) -> bool {
        match access {
            AccessType::Read => self.read,
            AccessType::Write => self.write,
            AccessType::Execute => self.execute,
        }
    }
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Translation {
    pub physical: u64,
    pub permissions: PagePermissions,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MmuFault {
    pub virtual_address: u64,
    pub access: AccessType,
    pub reason: &'static str,
}

#[derive(Debug, Clone, Copy)]
struct PageEntry {
    physical_page: u64,
    permissions: PagePermissions,
}

#[derive(Debug, Clone)]
pub struct Mmu {
    address_bits: u8,
    page_shift: u8,
    pages: BTreeMap<u64, PageEntry>,
    generation: u64,
}

impl Mmu {
    pub fn new(address_bits: u8, page_shift: u8) -> Result<Self, String> {
        if !(16..=64).contains(&address_bits) {
            return Err("MMU address width must be 16..=64".into());
        }
        if !(8..=24).contains(&page_shift) || page_shift >= address_bits {
            return Err("MMU page shift is incompatible with the address width".into());
        }
        Ok(Self {
            address_bits,
            page_shift,
            pages: BTreeMap::new(),
            generation: 0,
        })
    }

    pub fn generation(&self) -> u64 {
        self.generation
    }
    pub fn page_size(&self) -> u64 {
        1u64 << self.page_shift
    }
    pub fn map_range(
        &mut self,
        virtual_start: u64,
        physical_start: u64,
        length: u64,
        permissions: PagePermissions,
    ) -> Result<(), String> {
        let page_size = self.page_size();
        if length == 0
            || !virtual_start.is_multiple_of(page_size)
            || !physical_start.is_multiple_of(page_size)
            || !length.is_multiple_of(page_size)
        {
            return Err("MMU mappings must be non-empty and page aligned".into());
        }
        self.check_virtual_range(virtual_start, length)?;
        let page_count = length / page_size;
        for page in 0..page_count {
            let virtual_page = (virtual_start / page_size) + page;
            let physical_page = (physical_start / page_size) + page;
            self.pages.insert(
                virtual_page,
                PageEntry {
                    physical_page,
                    permissions,
                },
            );
        }
        self.generation = self.generation.wrapping_add(1);
        Ok(())
    }

    pub fn unmap_range(&mut self, virtual_start: u64, length: u64) -> Result<(), String> {
        let page_size = self.page_size();
        if length == 0
            || !virtual_start.is_multiple_of(page_size)
            || !length.is_multiple_of(page_size)
        {
            return Err("MMU unmaps must be non-empty and page aligned".into());
        }
        self.check_virtual_range(virtual_start, length)?;
        for page in 0..length / page_size {
            self.pages.remove(&(virtual_start / page_size + page));
        }
        self.generation = self.generation.wrapping_add(1);
        Ok(())
    }

    pub fn translate(
        &self,
        virtual_address: u64,
        access: AccessType,
    ) -> Result<Translation, MmuFault> {
        if !self.address_in_range(virtual_address) {
            return Err(MmuFault {
                virtual_address,
                access,
                reason: "virtual address exceeds guest width",
            });
        }
        let page_size = self.page_size();
        let page = virtual_address / page_size;
        let offset = virtual_address & (page_size - 1);
        let Some(entry) = self.pages.get(&page) else {
            return Err(MmuFault {
                virtual_address,
                access,
                reason: "virtual page is unmapped",
            });
        };
        if !entry.permissions.allows(access) {
            return Err(MmuFault {
                virtual_address,
                access,
                reason: "page permission denied",
            });
        }
        Ok(Translation {
            physical: entry.physical_page * page_size + offset,
            permissions: entry.permissions,
        })
    }

    fn address_in_range(&self, address: u64) -> bool {
        self.address_bits == 64 || address < (1u64 << self.address_bits)
    }

    fn check_virtual_range(&self, start: u64, length: u64) -> Result<(), String> {
        let end = start
            .checked_add(length - 1)
            .ok_or_else(|| "MMU mapping overflows".to_string())?;
        if !self.address_in_range(start) || !self.address_in_range(end) {
            return Err("MMU mapping exceeds guest virtual-address width".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn translates_32_bit_pages_and_enforces_permissions() {
        let mut mmu = Mmu::new(32, 12).unwrap();
        mmu.map_range(
            0x8000_0000,
            0x0010_0000,
            0x2000,
            PagePermissions::READ_EXECUTE,
        )
        .unwrap();
        assert_eq!(
            mmu.translate(0x8000_0123, AccessType::Read)
                .unwrap()
                .physical,
            0x0010_0123
        );
        assert!(mmu.translate(0x8000_0123, AccessType::Execute).is_ok());
        assert!(mmu.translate(0x8000_0123, AccessType::Write).is_err());
    }

    #[test]
    fn supports_virtual_64_bit_guests_and_generation_invalidation() {
        let mut mmu = Mmu::new(64, 16).unwrap();
        let before = mmu.generation();
        mmu.map_range(
            0xffff_0000_0000_0000,
            0x1_0000_0000,
            0x1_0000,
            PagePermissions::ALL,
        )
        .unwrap();
        assert_ne!(mmu.generation(), before);
        assert_eq!(
            mmu.translate(0xffff_0000_0000_0042, AccessType::Read)
                .unwrap()
                .physical,
            0x1_0000_0042
        );
        let mapped = mmu.generation();
        mmu.unmap_range(0xffff_0000_0000_0000, 0x1_0000).unwrap();
        assert_ne!(mmu.generation(), mapped);
        assert!(mmu
            .translate(0xffff_0000_0000_0042, AccessType::Read)
            .is_err());
    }

    #[test]
    fn rejects_addresses_beyond_guest_width() {
        let mut mmu = Mmu::new(32, 12).unwrap();
        assert!(mmu
            .map_range(0x1_0000_0000, 0, 0x1000, PagePermissions::READ_WRITE)
            .is_err());
    }
}
