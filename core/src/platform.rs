#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlatformId {
    Odyssey = 1,
    HomePong = 2,
    Atari2600 = 10,
    Atari5200 = 11,
    ColecoVision = 12,
    Intellivision = 13,
    Nes = 20,
    MasterSystem = 21,
    Atari7800 = 22,
    Snes = 30,
    Genesis = 31,
    SegaCd = 32,
    Sega32x = 33,
    PcEngine = 34,
    NeoGeo = 35,
    PlayStation = 40,
    Nintendo64 = 41,
    Saturn = 42,
    Jaguar = 43,
    ThreeDo = 44,
    Dreamcast = 50,
    PlayStation2 = 51,
    GameCube = 52,
    Xbox = 53,
    Xbox360 = 60,
    PlayStation3 = 61,
    Wii = 62,
    WiiU = 70,
    Switch = 71,
    PlayStation4 = 72,
    XboxOne = 73,
}

impl PlatformId {
    pub const fn from_u32(value: u32) -> Option<Self> {
        Some(match value {
            1 => Self::Odyssey,
            2 => Self::HomePong,
            10 => Self::Atari2600,
            11 => Self::Atari5200,
            12 => Self::ColecoVision,
            13 => Self::Intellivision,
            20 => Self::Nes,
            21 => Self::MasterSystem,
            22 => Self::Atari7800,
            30 => Self::Snes,
            31 => Self::Genesis,
            32 => Self::SegaCd,
            33 => Self::Sega32x,
            34 => Self::PcEngine,
            35 => Self::NeoGeo,
            40 => Self::PlayStation,
            41 => Self::Nintendo64,
            42 => Self::Saturn,
            43 => Self::Jaguar,
            44 => Self::ThreeDo,
            50 => Self::Dreamcast,
            51 => Self::PlayStation2,
            52 => Self::GameCube,
            53 => Self::Xbox,
            60 => Self::Xbox360,
            61 => Self::PlayStation3,
            62 => Self::Wii,
            70 => Self::WiiU,
            71 => Self::Switch,
            72 => Self::PlayStation4,
            73 => Self::XboxOne,
            _ => return None,
        })
    }

    pub const fn is_launchable(self) -> bool {
        matches!(
            self,
            Self::HomePong
                | Self::Atari2600
                | Self::Atari5200
                | Self::ColecoVision
                | Self::Nes
                | Self::MasterSystem
        )
    }

    pub const fn support_level(self) -> SupportLevel {
        match self {
            Self::HomePong => SupportLevel::Playable,
            Self::Atari2600
            | Self::Atari5200
            | Self::ColecoVision
            | Self::Atari7800
            | Self::Nes
            | Self::MasterSystem => SupportLevel::Foundation,
            _ => SupportLevel::Planned,
        }
    }
}

#[repr(u32)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SupportLevel {
    Planned = 0,
    Foundation = 1,
    Playable = 2,
    Validated = 3,
}
