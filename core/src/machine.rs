use crate::blueprint::is_targeted;
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machines::{
    Atari2600Machine, Atari5200Machine, Atari7800Machine, ColecoVisionMachine, GenesisMachine,
    MasterSystemMachine, NesMachine, PongMachine,
};
use crate::platform::{PlatformId, SupportLevel};
use crate::resources::{ResourceKind, ResourceStore};

pub trait Machine: Send {
    fn platform(&self) -> PlatformId;
    fn reset(&mut self);
    fn run_frame(&mut self, input: &InputState);
    fn frame_rate(&self) -> f64;
    fn video(&self) -> &VideoBuffer;
    fn audio(&self) -> &AudioBuffer;
    fn save_state(&self) -> Result<Vec<u8>, String>;
    fn load_state(&mut self, bytes: &[u8]) -> Result<(), String>;
    fn persistent_len(&self, _kind: ResourceKind, _slot: u32) -> usize {
        0
    }
    fn read_persistent(
        &self,
        _kind: ResourceKind,
        _slot: u32,
        _out: &mut [u8],
    ) -> Result<(), String> {
        Err("machine has no persistent resource at that slot".into())
    }
    fn write_persistent(
        &mut self,
        _kind: ResourceKind,
        _slot: u32,
        _data: &[u8],
    ) -> Result<(), String> {
        Err("machine has no persistent resource at that slot".into())
    }
}

pub fn create_machine(
    platform: PlatformId,
    resources: &ResourceStore,
) -> Result<Box<dyn Machine>, String> {
    match platform {
        PlatformId::HomePong => Ok(Box::new(PongMachine::new()) as Box<dyn Machine>),
        PlatformId::Atari2600 => {
            let image = resources.primary_game().ok_or_else(|| "Atari 2600 requires a staged cartridge image".to_string())?;
            let rom = image.materialize(128 * 1024)?;
            Ok(Box::new(Atari2600Machine::from_rom(&rom)?) as Box<dyn Machine>)
        }
        PlatformId::Atari5200 => {
            let image = resources.primary_game().ok_or_else(|| "Atari 5200 requires a staged cartridge image".to_string())?;
            let rom = image.materialize(128 * 1024)?;
            let bios = resources.get(ResourceKind::Bios, 0).ok_or_else(|| "Atari 5200 requires a staged BIOS".to_string())?;
            let bios = bios.materialize(0x0800)?;
            Ok(Box::new(Atari5200Machine::new(&rom, &bios)?) as Box<dyn Machine>)
        }
        PlatformId::Atari7800 => {
            let image = resources.primary_game().ok_or_else(|| "Atari 7800 requires a staged cartridge image".to_string())?;
            let rom = image.materialize(2 * 1024 * 1024 + 128)?;
            Ok(Box::new(Atari7800Machine::from_image(&rom)?) as Box<dyn Machine>)
        }
        PlatformId::Genesis => {
            let image = resources.primary_game().ok_or_else(|| "Genesis requires a staged cartridge image".to_string())?;
            let rom = image.materialize(8 * 1024 * 1024 + 512)?;
            Ok(Box::new(GenesisMachine::from_rom(&rom)?) as Box<dyn Machine>)
        }
        PlatformId::Nes => {
            let image = resources.primary_game().ok_or_else(|| "NES requires a staged game image".to_string())?;
            let rom = image.materialize(64 * 1024 * 1024)?;
            Ok(Box::new(NesMachine::from_rom(&rom)?) as Box<dyn Machine>)
        }
        PlatformId::MasterSystem => {
            let image = resources.primary_game().ok_or_else(|| "Master System requires a staged cartridge image".to_string())?;
            let rom = image.materialize(16 * 1024 * 1024)?;
            Ok(Box::new(MasterSystemMachine::from_rom(&rom)?) as Box<dyn Machine>)
        }
        PlatformId::ColecoVision => {
            let image = resources.primary_game().ok_or_else(|| "ColecoVision requires a staged cartridge image".to_string())?;
            let rom = image.materialize(512 * 1024)?;
            let bios = resources.get(ResourceKind::Bios, 0).ok_or_else(|| "ColecoVision requires a staged BIOS".to_string())?;
            let bios = bios.materialize(0x2000)?;
            Ok(Box::new(ColecoVisionMachine::from_images(&bios, &rom)?) as Box<dyn Machine>)
        }

        other if !is_targeted(other) => Err(format!("platform {} is reserved but outside the current OmniEmu target set", other as u32)),
        other if other.support_level() == SupportLevel::Foundation => Err(format!(
            "platform {} has shared hardware foundation in OmniCore but its complete machine graph is not runnable yet",
            other as u32,
        )),
        other => Err(format!("platform {} has an OmniCore hardware blueprint but its machine implementation is not complete yet", other as u32)),
    }
}
