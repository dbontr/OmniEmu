use crate::interconnect::Endian;
use crate::platform::PlatformId;
use crate::resources::ResourceKind;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GuestIsa {
    DiscreteLogic,
    Mos6502,
    Cp1610,
    Z80,
    W65c816,
    Spc700,
    HuC6280,
    M68000,
    Sh2,
    Sh4,
    MipsR3000,
    MipsR4300,
    MipsR5900,
    Rsp,
    VectorUnit,
    ArmV3,
    ArmV4,
    ArmV5,
    ArmV7,
    ArmV8,
    PowerPc32,
    PowerPc64,
    X86,
    Spu,
    CustomRisc,
    Dsp,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryModel {
    Physical16,
    Physical24,
    Physical32,
    Virtual32,
    Virtual64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GraphicsClass {
    Discrete,
    Scanline,
    TileSprite,
    FixedFunction3d,
    Programmable3d,
    UnifiedShader,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MediaKind {
    BuiltIn,
    Cartridge,
    OpticalDisc,
    Card,
    Package,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CpuBlueprint {
    pub isa: GuestIsa,
    pub count: u8,
    pub endian: Endian,
    pub address_bits: u8,
}

const fn cpu(isa: GuestIsa, count: u8, endian: Endian, address_bits: u8) -> CpuBlueprint {
    CpuBlueprint {
        isa,
        count,
        endian,
        address_bits,
    }
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SystemBlueprint {
    pub platform: PlatformId,
    pub generation: u8,
    pub cpus: Vec<CpuBlueprint>,
    pub memory: MemoryModel,
    pub graphics: GraphicsClass,
    pub media: MediaKind,
    pub required_resources: &'static [ResourceKind],
    pub persistent_storage: bool,
    pub encrypted_content: bool,
}

const NONE: &[ResourceKind] = &[];
const BIOS: &[ResourceKind] = &[ResourceKind::Bios];
const BIOS_DISC: &[ResourceKind] = &[ResourceKind::Bios, ResourceKind::Disc];
const FIRMWARE_DISC: &[ResourceKind] = &[ResourceKind::Firmware, ResourceKind::Disc];
const FIRMWARE_KEYS: &[ResourceKind] = &[ResourceKind::Firmware, ResourceKind::Keys];

pub const TARGET_PLATFORMS: &[PlatformId] = &[
    PlatformId::Odyssey,
    PlatformId::HomePong,
    PlatformId::Atari2600,
    PlatformId::Atari5200,
    PlatformId::ColecoVision,
    PlatformId::Intellivision,
    PlatformId::Nes,
    PlatformId::MasterSystem,
    PlatformId::Atari7800,
    PlatformId::Snes,
    PlatformId::Genesis,
    PlatformId::SegaCd,
    PlatformId::Sega32x,
    PlatformId::PcEngine,
    PlatformId::NeoGeo,
    PlatformId::PlayStation,
    PlatformId::Nintendo64,
    PlatformId::Saturn,
    PlatformId::Jaguar,
    PlatformId::ThreeDo,
    PlatformId::Dreamcast,
    PlatformId::PlayStation2,
    PlatformId::GameCube,
    PlatformId::Xbox,
    PlatformId::Xbox360,
    PlatformId::PlayStation3,
    PlatformId::Wii,
    PlatformId::WiiU,
    PlatformId::Switch,
];

pub fn blueprint(platform: PlatformId) -> Option<SystemBlueprint> {
    use Endian::{Big, Little};
    use GraphicsClass::*;
    use GuestIsa::*;
    use MediaKind::*;
    use MemoryModel::*;
    Some(match platform {
        PlatformId::Odyssey => SystemBlueprint {
            platform,
            generation: 1,
            cpus: vec![cpu(DiscreteLogic, 1, Little, 16)],
            memory: Physical16,
            graphics: Discrete,
            media: BuiltIn,
            required_resources: NONE,
            persistent_storage: false,
            encrypted_content: false,
        },
        PlatformId::HomePong => SystemBlueprint {
            platform,
            generation: 1,
            cpus: vec![cpu(DiscreteLogic, 1, Little, 16)],
            memory: Physical16,
            graphics: Discrete,
            media: BuiltIn,
            required_resources: NONE,
            persistent_storage: false,
            encrypted_content: false,
        },
        PlatformId::Atari2600 => SystemBlueprint {
            platform,
            generation: 2,
            cpus: vec![cpu(Mos6502, 1, Little, 16)],
            memory: Physical16,
            graphics: Scanline,
            media: Cartridge,
            required_resources: NONE,
            persistent_storage: false,
            encrypted_content: false,
        },
        PlatformId::Atari5200 => SystemBlueprint {
            platform,
            generation: 2,
            cpus: vec![cpu(Mos6502, 1, Little, 16)],
            memory: Physical16,
            graphics: TileSprite,
            media: Cartridge,
            required_resources: BIOS,
            persistent_storage: false,
            encrypted_content: false,
        },
        PlatformId::ColecoVision => SystemBlueprint {
            platform,
            generation: 2,
            cpus: vec![cpu(Z80, 1, Little, 16)],
            memory: Physical16,
            graphics: TileSprite,
            media: Cartridge,
            required_resources: BIOS,
            persistent_storage: false,
            encrypted_content: false,
        },
        PlatformId::Intellivision => SystemBlueprint {
            platform,
            generation: 2,
            cpus: vec![cpu(Cp1610, 1, Little, 16)],
            memory: Physical16,
            graphics: TileSprite,
            media: Cartridge,
            required_resources: BIOS,
            persistent_storage: false,
            encrypted_content: false,
        },
        PlatformId::Nes => SystemBlueprint {
            platform,
            generation: 3,
            cpus: vec![cpu(Mos6502, 1, Little, 16)],
            memory: Physical16,
            graphics: TileSprite,
            media: Cartridge,
            required_resources: NONE,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::MasterSystem => SystemBlueprint {
            platform,
            generation: 3,
            cpus: vec![cpu(Z80, 1, Little, 16)],
            memory: Physical16,
            graphics: TileSprite,
            media: Cartridge,
            required_resources: NONE,
            persistent_storage: false,
            encrypted_content: false,
        },
        PlatformId::Atari7800 => SystemBlueprint {
            platform,
            generation: 3,
            cpus: vec![cpu(Mos6502, 1, Little, 16)],
            memory: Physical16,
            graphics: Scanline,
            media: Cartridge,
            required_resources: NONE,
            persistent_storage: false,
            encrypted_content: false,
        },
        PlatformId::Snes => SystemBlueprint {
            platform,
            generation: 4,
            cpus: vec![cpu(W65c816, 1, Little, 24), cpu(Spc700, 1, Little, 16)],
            memory: Physical24,
            graphics: TileSprite,
            media: Cartridge,
            required_resources: NONE,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::Genesis => SystemBlueprint {
            platform,
            generation: 4,
            cpus: vec![cpu(M68000, 1, Big, 24), cpu(Z80, 1, Little, 16)],
            memory: Physical24,
            graphics: TileSprite,
            media: Cartridge,
            required_resources: NONE,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::SegaCd => SystemBlueprint {
            platform,
            generation: 4,
            cpus: vec![cpu(M68000, 2, Big, 24), cpu(Z80, 1, Little, 16)],
            memory: Physical24,
            graphics: TileSprite,
            media: OpticalDisc,
            required_resources: BIOS_DISC,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::Sega32x => SystemBlueprint {
            platform,
            generation: 4,
            cpus: vec![
                cpu(M68000, 1, Big, 24),
                cpu(Z80, 1, Little, 16),
                cpu(Sh2, 2, Little, 32),
            ],
            memory: Physical32,
            graphics: FixedFunction3d,
            media: Cartridge,
            required_resources: NONE,
            persistent_storage: false,
            encrypted_content: false,
        },
        PlatformId::PcEngine => SystemBlueprint {
            platform,
            generation: 4,
            cpus: vec![cpu(HuC6280, 1, Little, 21)],
            memory: Physical24,
            graphics: TileSprite,
            media: Cartridge,
            required_resources: NONE,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::NeoGeo => SystemBlueprint {
            platform,
            generation: 4,
            cpus: vec![cpu(M68000, 1, Big, 24), cpu(Z80, 1, Little, 16)],
            memory: Physical24,
            graphics: TileSprite,
            media: Cartridge,
            required_resources: BIOS,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::PlayStation => SystemBlueprint {
            platform,
            generation: 5,
            cpus: vec![cpu(MipsR3000, 1, Little, 32)],
            memory: Physical32,
            graphics: FixedFunction3d,
            media: OpticalDisc,
            required_resources: BIOS_DISC,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::Nintendo64 => SystemBlueprint {
            platform,
            generation: 5,
            cpus: vec![cpu(MipsR4300, 1, Big, 64), cpu(Rsp, 1, Big, 32)],
            memory: Virtual64,
            graphics: FixedFunction3d,
            media: Cartridge,
            required_resources: NONE,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::Saturn => SystemBlueprint {
            platform,
            generation: 5,
            cpus: vec![
                cpu(Sh2, 2, Little, 32),
                cpu(M68000, 1, Big, 24),
                cpu(Dsp, 1, Little, 32),
            ],
            memory: Physical32,
            graphics: FixedFunction3d,
            media: OpticalDisc,
            required_resources: BIOS_DISC,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::Jaguar => SystemBlueprint {
            platform,
            generation: 5,
            cpus: vec![cpu(M68000, 1, Big, 24), cpu(CustomRisc, 2, Big, 32)],
            memory: Physical32,
            graphics: FixedFunction3d,
            media: Cartridge,
            required_resources: NONE,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::ThreeDo => SystemBlueprint {
            platform,
            generation: 5,
            cpus: vec![cpu(ArmV3, 1, Little, 32), cpu(Dsp, 1, Little, 32)],
            memory: Physical32,
            graphics: FixedFunction3d,
            media: OpticalDisc,
            required_resources: BIOS_DISC,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::Dreamcast => SystemBlueprint {
            platform,
            generation: 6,
            cpus: vec![cpu(Sh4, 1, Little, 32), cpu(ArmV4, 1, Little, 32)],
            memory: Virtual32,
            graphics: Programmable3d,
            media: OpticalDisc,
            required_resources: BIOS_DISC,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::PlayStation2 => SystemBlueprint {
            platform,
            generation: 6,
            cpus: vec![
                cpu(MipsR5900, 1, Little, 64),
                cpu(MipsR3000, 1, Little, 32),
                cpu(VectorUnit, 2, Little, 32),
            ],
            memory: Virtual64,
            graphics: Programmable3d,
            media: OpticalDisc,
            required_resources: BIOS_DISC,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::GameCube => SystemBlueprint {
            platform,
            generation: 6,
            cpus: vec![cpu(PowerPc32, 1, Big, 32), cpu(Dsp, 1, Big, 32)],
            memory: Virtual32,
            graphics: Programmable3d,
            media: OpticalDisc,
            required_resources: NONE,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::Xbox => SystemBlueprint {
            platform,
            generation: 6,
            cpus: vec![cpu(X86, 1, Little, 32)],
            memory: Virtual32,
            graphics: Programmable3d,
            media: OpticalDisc,
            required_resources: FIRMWARE_DISC,
            persistent_storage: true,
            encrypted_content: false,
        },
        PlatformId::Xbox360 => SystemBlueprint {
            platform,
            generation: 7,
            cpus: vec![cpu(PowerPc64, 3, Big, 64)],
            memory: Virtual64,
            graphics: UnifiedShader,
            media: OpticalDisc,
            required_resources: FIRMWARE_DISC,
            persistent_storage: true,
            encrypted_content: true,
        },
        PlatformId::PlayStation3 => SystemBlueprint {
            platform,
            generation: 7,
            cpus: vec![cpu(PowerPc64, 1, Big, 64), cpu(Spu, 6, Big, 32)],
            memory: Virtual64,
            graphics: UnifiedShader,
            media: OpticalDisc,
            required_resources: FIRMWARE_DISC,
            persistent_storage: true,
            encrypted_content: true,
        },
        PlatformId::Wii => SystemBlueprint {
            platform,
            generation: 7,
            cpus: vec![
                cpu(PowerPc32, 1, Big, 32),
                cpu(Dsp, 1, Big, 32),
                cpu(ArmV5, 1, Little, 32),
            ],
            memory: Virtual32,
            graphics: Programmable3d,
            media: OpticalDisc,
            required_resources: FIRMWARE_KEYS,
            persistent_storage: true,
            encrypted_content: true,
        },
        PlatformId::WiiU => SystemBlueprint {
            platform,
            generation: 8,
            cpus: vec![cpu(PowerPc32, 3, Big, 32), cpu(ArmV5, 1, Little, 32)],
            memory: Virtual32,
            graphics: UnifiedShader,
            media: OpticalDisc,
            required_resources: FIRMWARE_KEYS,
            persistent_storage: true,
            encrypted_content: true,
        },
        PlatformId::Switch => SystemBlueprint {
            platform,
            generation: 8,
            cpus: vec![cpu(ArmV8, 4, Little, 64)],
            memory: Virtual64,
            graphics: UnifiedShader,
            media: Package,
            required_resources: FIRMWARE_KEYS,
            persistent_storage: true,
            encrypted_content: true,
        },
        PlatformId::PlayStation4 | PlatformId::XboxOne => return None,
    })
}

pub fn is_targeted(platform: PlatformId) -> bool {
    blueprint(platform).is_some()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_target_has_a_blueprint_and_high_end_addressing() {
        assert_eq!(TARGET_PLATFORMS.len(), 29);
        for &platform in TARGET_PLATFORMS {
            let system = blueprint(platform).expect("target must have blueprint");
            assert_eq!(system.platform, platform);
            assert!(!system.cpus.is_empty());
        }
        assert_eq!(
            blueprint(PlatformId::Switch).unwrap().memory,
            MemoryModel::Virtual64
        );
    }

    #[test]
    fn ps4_and_xbox_one_are_reserved_but_out_of_scope() {
        assert!(blueprint(PlatformId::PlayStation4).is_none());
        assert!(blueprint(PlatformId::XboxOne).is_none());
        assert!(!TARGET_PLATFORMS.contains(&PlatformId::PlayStation4));
        assert!(!TARGET_PLATFORMS.contains(&PlatformId::XboxOne));
    }

    #[test]
    fn heterogeneous_systems_keep_multiple_guest_isa_domains() {
        let ps2 = blueprint(PlatformId::PlayStation2).unwrap();
        assert_eq!(ps2.cpus.len(), 3);
        assert!(ps2.cpus.iter().any(|cpu| cpu.isa == GuestIsa::MipsR5900));
        assert!(ps2.cpus.iter().any(|cpu| cpu.isa == GuestIsa::VectorUnit));
        let sega32x = blueprint(PlatformId::Sega32x).unwrap();
        assert!(sega32x
            .cpus
            .iter()
            .any(|cpu| cpu.isa == GuestIsa::Sh2 && cpu.count == 2));
    }
}
