mod api;
pub mod audio;
pub mod blueprint;
pub mod bus;
pub mod cd_image;
pub mod clock;
pub mod cluster;
pub mod cpu6502;
pub mod cpu65816;
pub mod cpu68000;
pub mod cpu_aarch64;
pub mod cpu_arm60;
pub mod cpu_arm7;
pub mod cpu_cp1610;
pub mod cpu_huc6280;
pub mod cpu_jaguar_risc;
pub mod cpu_mips_r3000;
pub mod cpu_mips_r4300;
pub mod cpu_mips_r5900;
pub mod cpu_powerpc64;
pub mod cpu_powerpc750;
pub mod cpu_rsp;
pub mod cpu_sh2;
pub mod cpu_sh4;
pub mod cpu_spc700;
pub mod cpu_spu;
pub mod cpu_x86;
pub mod cpu_z80;
pub mod dma;
pub mod executable;
pub mod execution;
pub mod graphics;
pub mod input;
pub mod interconnect;
pub mod interrupt;
pub mod kernel;
pub mod machine;
pub mod machines;
pub mod media;
pub mod mmu;
pub mod platform;
pub mod resources;
pub mod sparse_memory;
pub mod state;

#[cfg(test)]
mod tests {
    use crate::kernel::Scheduler;

    #[test]
    fn scheduler_is_deterministic_for_equal_timestamps() {
        let mut scheduler = Scheduler::new();
        scheduler.schedule_at(12, 3, 9);
        scheduler.schedule_at(12, 2, 7);
        scheduler.schedule_at(8, 1, 4);
        let events = scheduler.advance_to(12);
        assert_eq!(
            events.iter().map(|event| event.device).collect::<Vec<_>>(),
            vec![1, 3, 2]
        );
        assert_eq!(scheduler.now(), 12);
    }
}
