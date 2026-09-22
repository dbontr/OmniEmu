use crate::blueprint::is_targeted;
use crate::kernel::{AudioBuffer, InputState, VideoBuffer};
use crate::machines::{
    Atari2600Machine, Atari5200Machine, Atari7800Machine, ColecoVisionMachine, DreamcastMachine,
    GameCubeMachine, GenesisMachine, IntellivisionMachine, JaguarMachine, MasterSystemMachine,
    NeoGeoMachine, NesMachine, Nintendo64Machine, OdysseyMachine, PcEngineMachine,
    PlayStation2Machine, PlayStation3Machine, PlayStationMachine, PongMachine, SaturnMachine,
    Sega32xMachine, SegaCdMachine, SnesMachine, SwitchMachine, ThreeDoMachine, WiiMachine,
    WiiUMachine, Xbox360Machine, XboxMachine,
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
        PlatformId::Odyssey => Ok(Box::new(OdysseyMachine::new()) as Box<dyn Machine>),
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
        PlatformId::Intellivision => {
            let image = resources.primary_game().ok_or_else(|| "Intellivision requires a staged cartridge image".to_string())?;
            let game = image.materialize(2 * 1024 * 1024)?;
            let exec = resources.get(ResourceKind::Bios, 0).ok_or_else(|| "Intellivision requires a staged EXEC ROM".to_string())?;
            let exec = exec.materialize(0x2000)?;
            let grom = resources.get(ResourceKind::Firmware, 0).ok_or_else(|| "Intellivision requires a staged GROM image".to_string())?;
            let grom = grom.materialize(0x0800)?;
            Ok(Box::new(IntellivisionMachine::from_images(&game, &exec, &grom)?) as Box<dyn Machine>)
        }
        PlatformId::Genesis => {
            let image = resources.primary_game().ok_or_else(|| "Genesis requires a staged cartridge image".to_string())?;
            let rom = image.materialize(8 * 1024 * 1024 + 512)?;
            Ok(Box::new(GenesisMachine::from_rom(&rom)?) as Box<dyn Machine>)
        }
        PlatformId::PcEngine => {
            if let Some(disc) = resources.get(ResourceKind::Disc, 0) {
                let bios = resources
                    .get(ResourceKind::Bios, 0)
                    .ok_or_else(|| "PC Engine CD requires a staged System Card BIOS".to_string())?;
                let bios = bios.materialize(256 * 1024 + 512)?;
                Ok(Box::new(PcEngineMachine::from_bios_and_disc(&bios, disc.clone())?)
                    as Box<dyn Machine>)
            } else {
                let image = resources
                    .primary_game()
                    .ok_or_else(|| "PC Engine requires a staged HuCard image".to_string())?;
                let rom = image.materialize(0x280000 + 512)?;
                Ok(Box::new(PcEngineMachine::from_rom(&rom)?) as Box<dyn Machine>)
            }
        }
        PlatformId::SuperGrafx => {
            let image = resources
                .primary_game()
                .ok_or_else(|| "SuperGrafx requires a staged HuCard image".to_string())?;
            let rom = image.materialize(0x280000 + 512)?;
            Ok(Box::new(PcEngineMachine::from_supergrafx_rom(&rom)?) as Box<dyn Machine>)
        }
        PlatformId::SegaCd => {
            let bios = resources.get(ResourceKind::Bios, 0).ok_or_else(|| "Sega CD requires a staged 128 KiB BIOS".to_string())?;
            let bios = bios.materialize(128 * 1024)?;
            let disc = resources.get(ResourceKind::Disc, 0).or_else(|| resources.primary_game()).ok_or_else(|| "Sega CD requires a staged ISO or raw disc image".to_string())?.clone();
            Ok(Box::new(SegaCdMachine::from_bios_and_disc(&bios, disc)?) as Box<dyn Machine>)
        }
        PlatformId::Sega32x => {
            let image = resources.primary_game().ok_or_else(|| "Sega 32X requires a staged cartridge image".to_string())?;
            let rom = image.materialize(8 * 1024 * 1024 + 512)?;
            Ok(Box::new(Sega32xMachine::from_rom(&rom)?) as Box<dyn Machine>)
        }
        PlatformId::NeoGeo => {
            let image = resources.primary_game().ok_or_else(|| "Neo Geo requires a staged NEO1 cartridge image".to_string())?;
            let game = image.materialize(128 * 1024 * 1024)?;
            let bios = resources.get(ResourceKind::Bios, 0).ok_or_else(|| "Neo Geo requires a staged 128 KiB BIOS".to_string())?;
            let bios = bios.materialize(128 * 1024)?;
            Ok(Box::new(NeoGeoMachine::from_images(&game, &bios)?) as Box<dyn Machine>)
        }
        PlatformId::Snes => {
            let image = resources.primary_game().ok_or_else(|| "SNES requires a staged cartridge image".to_string())?;
            let rom = image.materialize(16 * 1024 * 1024 + 512)?;
            Ok(Box::new(SnesMachine::from_rom(&rom)?) as Box<dyn Machine>)
        }
        PlatformId::PlayStation => {
            let bios = resources.get(ResourceKind::Bios, 0).ok_or_else(|| "PlayStation requires a staged 512 KiB BIOS".to_string())?;
            let bios = bios.materialize(512 * 1024)?;
            if let Some(disc) = resources.get(ResourceKind::Disc, 0) {
                return Ok(Box::new(PlayStationMachine::from_bios_and_disc(
                    &bios,
                    Some(disc.clone()),
                )?) as Box<dyn Machine>);
            }
            if let Some(game) = resources.get(ResourceKind::Game, 0) {
                if game.len() >= 8 && game.range_present(0, 8) {
                    let mut magic = [0u8; 8];
                    game.read(0, &mut magic)?;
                    if &magic == b"PS-X EXE" {
                        let executable = game.materialize(2 * 1024 * 1024 + 0x800)?;
                        return Ok(Box::new(PlayStationMachine::from_bios_and_executable(
                            &bios,
                            &executable,
                        )?) as Box<dyn Machine>);
                    }
                }
                return Ok(Box::new(PlayStationMachine::from_bios_and_disc(
                    &bios,
                    Some(game.clone()),
                )?) as Box<dyn Machine>);
            }
            Ok(Box::new(PlayStationMachine::from_bios(&bios)?) as Box<dyn Machine>)
        }
        PlatformId::Nintendo64 => {
            let image = resources.primary_game().ok_or_else(|| "Nintendo 64 requires a staged cartridge image".to_string())?;
            let rom = image.materialize(64 * 1024 * 1024)?;
            Ok(Box::new(Nintendo64Machine::from_rom(&rom)?) as Box<dyn Machine>)
        }
        PlatformId::Saturn => {
            let bios = resources.get(ResourceKind::Bios, 0).ok_or_else(|| "Saturn requires a staged 512 KiB BIOS".to_string())?;
            let bios = bios.materialize(512 * 1024)?;
            let disc = resources
                .get(ResourceKind::Disc, 0)
                .or_else(|| resources.primary_game())
                .cloned();
            Ok(Box::new(SaturnMachine::from_bios_and_disc(&bios, disc)?) as Box<dyn Machine>)
        }
        PlatformId::Jaguar => {
            let image = resources.primary_game().ok_or_else(|| "Jaguar requires a staged cartridge image".to_string())?;
            let cart = image.materialize(6 * 1024 * 1024)?;
            let bios = resources.get(ResourceKind::Bios, 0).ok_or_else(|| "Jaguar requires a staged 128 KiB BIOS".to_string())?;
            let bios = bios.materialize(128 * 1024)?;
            Ok(Box::new(JaguarMachine::from_images(&cart, &bios)?) as Box<dyn Machine>)
        }
        PlatformId::ThreeDo => {
            let bios = resources.get(ResourceKind::Bios, 0).ok_or_else(|| "3DO requires a staged 1 MiB BIOS".to_string())?;
            let bios = bios.materialize(1024 * 1024)?;
            let disc = resources
                .get(ResourceKind::Disc, 0)
                .or_else(|| resources.primary_game())
                .cloned();
            Ok(Box::new(ThreeDoMachine::from_bios_and_disc(&bios, disc)?) as Box<dyn Machine>)
        }
        PlatformId::Dreamcast => {
            let bios = resources.get(ResourceKind::Bios, 0).ok_or_else(|| "Dreamcast requires a staged 2 MiB boot ROM".to_string())?;
            let bios = bios.materialize(2 * 1024 * 1024)?;
            let disc = resources
                .get(ResourceKind::Disc, 0)
                .or_else(|| resources.primary_game())
                .ok_or_else(|| "Dreamcast requires a staged ISO/raw disc image".to_string())?
                .clone();
            Ok(Box::new(DreamcastMachine::from_bios_and_disc(&bios, disc)?) as Box<dyn Machine>)
        }
        PlatformId::PlayStation2 => {
            let image = resources
                .get(ResourceKind::Disc, 0)
                .or_else(|| resources.primary_game())
                .ok_or_else(|| "PlayStation 2 requires a staged ELF or ISO image".to_string())?
                .clone();
            let bios = resources
                .get(ResourceKind::Bios, 0)
                .map(|blob| blob.materialize(4 * 1024 * 1024))
                .transpose()?;
            Ok(Box::new(PlayStation2Machine::from_images(image, bios.as_deref())?) as Box<dyn Machine>)
        }
        PlatformId::GameCube => {
            let image = resources
                .get(ResourceKind::Disc, 0)
                .or_else(|| resources.primary_game())
                .ok_or_else(|| "GameCube requires a staged DOL or raw disc image".to_string())?
                .clone();
            let ipl = resources
                .get(ResourceKind::Bios, 0)
                .map(|blob| blob.materialize(2 * 1024 * 1024))
                .transpose()?;
            Ok(Box::new(GameCubeMachine::from_image_and_ipl(
                image,
                ipl.as_deref(),
            )?) as Box<dyn Machine>)
        }
        PlatformId::Wii => {
            let image = resources
                .get(ResourceKind::Disc, 0)
                .or_else(|| resources.primary_game())
                .ok_or_else(|| "Wii development mode requires a staged DOL image".to_string())?
                .clone();
            Ok(Box::new(WiiMachine::from_dol(image)?) as Box<dyn Machine>)
        }
        PlatformId::Xbox => {
            let image = resources
                .primary_game()
                .ok_or_else(|| "Xbox development mode requires a staged XBE image".to_string())?
                .clone();
            let disc = resources.get(ResourceKind::Disc, 0).cloned();
            Ok(Box::new(XboxMachine::from_images(image, disc)?) as Box<dyn Machine>)
        }
        PlatformId::Xbox360 => {
            let image = resources
                .primary_game()
                .ok_or_else(|| "Xbox 360 development mode requires a staged PowerPC64 ELF image".to_string())?
                .clone();
            Ok(Box::new(Xbox360Machine::from_elf(image)?) as Box<dyn Machine>)
        }
        PlatformId::PlayStation3 => {
            let image = resources
                .primary_game()
                .ok_or_else(|| "PlayStation 3 development mode requires a staged PowerPC64 ELF image".to_string())?
                .clone();
            Ok(Box::new(PlayStation3Machine::from_elf(image)?) as Box<dyn Machine>)
        }
        PlatformId::WiiU => {
            let image = resources
                .primary_game()
                .ok_or_else(|| "Wii U development mode requires a staged PowerPC ELF image".to_string())?
                .clone();
            Ok(Box::new(WiiUMachine::from_elf(image)?) as Box<dyn Machine>)
        }
        PlatformId::Switch => {
            let image = resources
                .primary_game()
                .ok_or_else(|| "Switch homebrew mode requires a staged NRO image".to_string())?
                .clone();
            Ok(Box::new(SwitchMachine::from_nro(image)?) as Box<dyn Machine>)
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
