use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machines::{NesMachine, PongMachine};
use crate::platform::{PlatformId, SupportLevel};

pub trait Machine: Send {
    fn platform(&self) -> PlatformId;
    fn reset(&mut self);
    fn run_frame(&mut self, input: &InputState);
    fn frame_rate(&self) -> f64;
    fn video(&self) -> &VideoBuffer;
    fn audio(&self) -> &AudioBuffer;
    fn save_state(&self) -> Result<Vec<u8>, String>;
    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String>;
}

pub fn create_machine(
    platform: PlatformId,
    rom: &[u8],
    _bios: &[u8],
) -> Result<Box<dyn Machine>, String> {
    match platform {
        PlatformId::HomePong => Ok(Box::new(PongMachine::new()) as Box<dyn Machine>),
        PlatformId::Nes => Ok(Box::new(NesMachine::from_rom(rom)?) as Box<dyn Machine>),
        other if other.support_level() == SupportLevel::Foundation => Err(format!(
            "platform {} has shared hardware foundation in OmniCore but its complete machine graph is not runnable yet",
            other as u32,
        )),
        other => Err(format!("platform {} is not implemented in OmniCore yet", other as u32)),
    }
}
