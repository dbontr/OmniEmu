use crate::blueprint::GuestIsa;
use crate::clock::{ClockDomain, ClockRate};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProcessorId(pub u16);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RunSlice {
    pub processor: ProcessorId,
    pub isa: GuestIsa,
    pub cycles: u64,
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

    pub fn advance(&mut self, master_ticks: u64) -> Vec<RunSlice> {
        let mut remaining = Vec::with_capacity(self.lanes.len());
        for lane in &mut self.lanes {
            let cycles = if lane.halted {
                0
            } else {
                lane.clock.advance(master_ticks)
            };
            remaining.push(cycles);
        }
        let mut slices = Vec::new();
        loop {
            let mut emitted = false;
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
                emitted = true;
            }
            if !emitted {
                break;
            }
        }
        slices
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
