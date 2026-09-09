use crate::interconnect::{AddressSpace, BusFault, BusResult};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DmaStep {
    Fixed,
    Increment,
    Decrement,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DmaTransfer {
    pub source: u64,
    pub destination: u64,
    pub bytes: u64,
    pub source_step: DmaStep,
    pub destination_step: DmaStep,
}

impl DmaTransfer {
    pub const fn linear(source: u64, destination: u64, bytes: u64) -> Self {
        Self {
            source,
            destination,
            bytes,
            source_step: DmaStep::Increment,
            destination_step: DmaStep::Increment,
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct DmaEngine {
    transferred: u64,
}
impl DmaEngine {
    pub fn transferred(&self) -> u64 {
        self.transferred
    }

    pub fn execute(
        &mut self,
        source: &mut dyn AddressSpace,
        destination: &mut dyn AddressSpace,
        transfer: DmaTransfer,
    ) -> BusResult<u64> {
        if transfer.bytes == 0 {
            return Ok(0);
        }
        let mut src = transfer.source;
        let mut dst = transfer.destination;
        for _ in 0..transfer.bytes {
            let value = source.read8(src)?;
            destination.write8(dst, value)?;
            src = step_address(src, transfer.source_step)?;
            dst = step_address(dst, transfer.destination_step)?;
            self.transferred = self.transferred.wrapping_add(1);
        }
        Ok(transfer.bytes)
    }
}

fn step_address(address: u64, step: DmaStep) -> BusResult<u64> {
    match step {
        DmaStep::Fixed => Ok(address),
        DmaStep::Increment => address.checked_add(1).ok_or(BusFault {
            address,
            width: 1,
            reason: "DMA source/destination increment overflow",
        }),
        DmaStep::Decrement => address.checked_sub(1).ok_or(BusFault {
            address,
            width: 1,
            reason: "DMA source/destination decrement underflow",
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::interconnect::FlatMemory;

    #[test]
    fn copies_between_independent_address_spaces() {
        let mut src = FlatMemory::ram(0x1000, 32);
        let mut dst = FlatMemory::ram(0x8000, 32);
        for i in 0..16u64 {
            src.write8(0x1000 + i, i as u8).unwrap();
        }
        let mut dma = DmaEngine::default();
        dma.execute(&mut src, &mut dst, DmaTransfer::linear(0x1000, 0x8000, 16))
            .unwrap();
        for i in 0..16u64 {
            assert_eq!(dst.read8(0x8000 + i).unwrap(), i as u8);
        }
        assert_eq!(dma.transferred(), 16);
    }

    #[test]
    fn fixed_destination_supports_fifo_style_devices() {
        let mut src = FlatMemory::ram(0, 4);
        let mut dst = FlatMemory::ram(0x100, 1);
        src.write8(0, 3).unwrap();
        src.write8(1, 7).unwrap();
        let transfer = DmaTransfer {
            source: 0,
            destination: 0x100,
            bytes: 2,
            source_step: DmaStep::Increment,
            destination_step: DmaStep::Fixed,
        };
        DmaEngine::default()
            .execute(&mut src, &mut dst, transfer)
            .unwrap();
        assert_eq!(dst.read8(0x100).unwrap(), 7);
    }
}
