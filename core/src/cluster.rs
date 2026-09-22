use crate::blueprint::GuestIsa;
use crate::clock::{ClockDomain, ClockRate};
use crate::state::{StateReader, StateWriter};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProcessorId(pub u16);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RunSlice {
    pub processor: ProcessorId,
    pub isa: GuestIsa,
    pub cycles: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RunBatch {
    pub slices: Vec<RunSlice>,
}

#[derive(Debug, Clone)]
struct ProcessorLane {
    id: ProcessorId,
    isa: GuestIsa,
    clock: ClockDomain,
    halted: bool,
}

#[derive(Debug, Clone)]
pub struct ProcessorCluster {
    master_rate: ClockRate,
    quantum: u64,
    lanes: Vec<ProcessorLane>,
    next_id: u16,
}

impl ProcessorCluster {
    pub fn new(master_rate: ClockRate, quantum: u64) -> Result<Self, String> {
        if master_rate.numerator == 0 || master_rate.denominator == 0 {
            return Err("cluster master clock cannot be zero".into());
        }
        if quantum == 0 {
            return Err("cluster quantum cannot be zero".into());
        }
        Ok(Self {
            master_rate,
            quantum,
            lanes: Vec::new(),
            next_id: 0,
        })
    }
    pub fn add_processor(&mut self, isa: GuestIsa, rate: ClockRate) -> ProcessorId {
        let id = ProcessorId(self.next_id);
        self.next_id = self.next_id.wrapping_add(1);
        self.lanes.push(ProcessorLane {
            id,
            isa,
            clock: ClockDomain::new(self.master_rate, rate),
            halted: false,
        });
        id
    }

    pub fn set_halted(&mut self, id: ProcessorId, halted: bool) -> Result<(), String> {
        let lane = self
            .lanes
            .iter_mut()
            .find(|lane| lane.id == id)
            .ok_or_else(|| "processor id is not part of this cluster".to_string())?;
        lane.halted = halted;
        Ok(())
    }

    pub fn processor_count(&self) -> usize {
        self.lanes.len()
    }

    pub fn save(&self, out: &mut StateWriter) {
        out.u16(self.lanes.len() as u16);
        out.u16(self.next_id);
        for lane in &self.lanes {
            out.u16(lane.id.0);
            out.u8(u8::from(lane.halted));
            let (phase, _) = lane.clock.phase();
            out.u128(phase);
            out.u64(lane.clock.cycles());
        }
    }

    pub fn load(&mut self, input: &mut StateReader<'_>) -> Result<(), String> {
        let count = input.u16()? as usize;
        let next_id = input.u16()?;
        if count != self.lanes.len() || next_id != self.next_id {
            return Err("processor-cluster topology does not match the save state".into());
        }
        for lane in &mut self.lanes {
            if input.u16()? != lane.id.0 {
                return Err("processor-cluster lane id does not match the save state".into());
            }
            lane.halted = input.u8()? != 0;
            let phase = input.u128()?;
            let cycles = input.u64()?;
            lane.clock.restore(phase, cycles)?;
        }
        Ok(())
    }

    pub fn advance(&mut self, master_ticks: u64) -> Vec<RunSlice> {
        self.advance_batches(master_ticks)
            .into_iter()
            .flat_map(|batch| batch.slices)
            .collect()
    }

    pub fn advance_batches(&mut self, master_ticks: u64) -> Vec<RunBatch> {
        let mut remaining = Vec::with_capacity(self.lanes.len());
        for lane in &mut self.lanes {
            let cycles = if lane.halted {
                0
            } else {
                lane.clock.advance(master_ticks)
            };
            remaining.push(cycles);
        }
        let mut batches = Vec::new();
        loop {
            let mut slices = Vec::with_capacity(self.lanes.len());
            for (index, lane) in self.lanes.iter().enumerate() {
                let cycles = remaining[index].min(self.quantum);
                if cycles == 0 {
                    continue;
                }
                remaining[index] -= cycles;
                slices.push(RunSlice {
                    processor: lane.id,
                    isa: lane.isa,
                    cycles,
                });
            }
            if slices.is_empty() {
                break;
            }
            batches.push(RunBatch { slices });
        }
        batches
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::PlatformId;

    #[test]
    fn schedules_three_equal_guest_cores_deterministically() {
        let mut cluster = ProcessorCluster::new(ClockRate::hz(100), 4).unwrap();
        cluster.add_processor(GuestIsa::PowerPc64, ClockRate::hz(100));
        cluster.add_processor(GuestIsa::PowerPc64, ClockRate::hz(100));
        cluster.add_processor(GuestIsa::PowerPc64, ClockRate::hz(100));
        let slices = cluster.advance(10);
        let ids = slices
            .iter()
            .map(|slice| slice.processor.0)
            .collect::<Vec<_>>();
        assert_eq!(ids, vec![0, 1, 2, 0, 1, 2, 0, 1, 2]);
        assert_eq!(slices.iter().map(|slice| slice.cycles).sum::<u64>(), 30);
    }

    #[test]
    fn heterogeneous_rates_and_halt_state_are_supported() {
        let mut cluster = ProcessorCluster::new(ClockRate::hz(100), 100).unwrap();
        let main = cluster.add_processor(GuestIsa::MipsR5900, ClockRate::hz(100));
        cluster.add_processor(GuestIsa::MipsR3000, ClockRate::hz(50));
        let slices = cluster.advance(20);
        assert_eq!(slices[0].cycles, 20);
        assert_eq!(slices[1].cycles, 10);
        cluster.set_halted(main, true).unwrap();
        let slices = cluster.advance(20);
        assert_eq!(slices.len(), 1);
        assert_eq!(slices[0].isa, GuestIsa::MipsR3000);
    }

    #[test]
    fn batches_expose_parallel_safe_quantum_boundaries() {
        let mut cluster = ProcessorCluster::new(ClockRate::hz(100), 4).unwrap();
        cluster.add_processor(GuestIsa::PowerPc64, ClockRate::hz(100));
        cluster.add_processor(GuestIsa::PowerPc64, ClockRate::hz(100));
        let batches = cluster.advance_batches(10);
        assert_eq!(batches.len(), 3);
        assert_eq!(batches[0].slices.len(), 2);
        assert_eq!(batches[0].slices[0].cycles, 4);
        assert_eq!(batches[1].slices[1].cycles, 4);
        assert_eq!(batches[2].slices[0].cycles, 2);
        assert_eq!(batches[2].slices[1].cycles, 2);
    }

    #[test]
    fn save_state_preserves_scheduler_phase_and_halt_state() {
        let mut original = ProcessorCluster::new(ClockRate::hz(3), 2).unwrap();
        original.add_processor(GuestIsa::ArmV8, ClockRate::hz(2));
        let secondary = original.add_processor(GuestIsa::PowerPc64, ClockRate::hz(1));
        original.advance_batches(1);
        original.set_halted(secondary, true).unwrap();

        let mut writer = StateWriter::new(PlatformId::Switch, 91);
        original.save(&mut writer);
        let bytes = writer.finish();

        let mut restored = ProcessorCluster::new(ClockRate::hz(3), 2).unwrap();
        restored.add_processor(GuestIsa::ArmV8, ClockRate::hz(2));
        restored.add_processor(GuestIsa::PowerPc64, ClockRate::hz(1));
        let mut reader = StateReader::new(&bytes, PlatformId::Switch, 91).unwrap();
        restored.load(&mut reader).unwrap();
        reader.finish().unwrap();

        for ticks in [1, 2, 3, 5, 8] {
            assert_eq!(
                restored.advance_batches(ticks),
                original.advance_batches(ticks)
            );
        }
    }
}
