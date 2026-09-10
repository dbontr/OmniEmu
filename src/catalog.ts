export type SupportTier = 'playable' | 'foundation' | 'planned'

export interface Platform {
  id: string
  omniCode: number
  name: string
  generation: number
  vendor: string
  years: string
  extensions: string[]
  tier: SupportTier
  bios?: 'optional' | 'required'
  romless?: boolean
  launchable?: boolean
  note?: string
}

const p = (platform: Platform) => platform

export const PLATFORMS: Platform[] = [
  p({ id: 'odyssey', omniCode: 1, name: 'Magnavox Odyssey', generation: 1, vendor: 'Magnavox', years: '1972–1975', extensions: [], tier: 'planned', romless: true, note: 'Dedicated analog hardware; it will be represented directly in OmniCore rather than as a ROM-driven machine.' }),
  p({ id: 'home-pong', omniCode: 2, name: 'Home Pong / Telstar class', generation: 1, vendor: 'Atari / Coleco', years: '1975–1978', extensions: [], tier: 'playable', romless: true, launchable: true, note: 'Built directly into the single OmniCore binary; no ROM is required.' }),
  p({ id: 'atari2600', omniCode: 10, name: 'Atari 2600', generation: 2, vendor: 'Atari', years: '1977–1992', extensions: ['a26', 'bin'], tier: 'foundation', launchable: true, note: 'Launchable development graph: 6507-style 13-bit bus using the shared 6502, RIOT RAM/I/O/timer, TIA playfield/players/missiles/ball/collisions/audio, WSYNC timing, controllers and 2K/4K/F8/F6/F4 cartridges. Cycle-level TIA validation and more bankswitch schemes remain.' }),
  p({ id: 'atari5200', omniCode: 11, name: 'Atari 5200', generation: 2, vendor: 'Atari', years: '1982–1984', extensions: ['a52', 'bin'], tier: 'foundation', bios: 'required', launchable: true, note: 'Launchable development graph: shared NMOS 6502, ANTIC display-list foundation, GTIA color/trigger registers, POKEY analog inputs/audio/IRQ, 16 KiB RAM and cartridge/BIOS map. ANTIC/GTIA timing and mode coverage remain incomplete.' }),
  p({ id: 'coleco', omniCode: 12, name: 'ColecoVision', generation: 2, vendor: 'Coleco', years: '1982–1985', extensions: ['col', 'rom', 'bin'], tier: 'foundation', launchable: true, bios: 'required', note: 'Launchable development graph: shared Z80 and SN76489, required user BIOS, TMS9918 Graphics I/II tile and sprite rendering, VDP NMI, controllers, audio and deterministic save states. Keypad completeness, timing and compatibility validation remain.' }),
  p({ id: 'intellivision', omniCode: 13, name: 'Intellivision', generation: 2, vendor: 'Mattel', years: '1979–1990', extensions: ['int', 'bin', 'rom'], tier: 'planned', bios: 'required' }),
  p({ id: 'nes', omniCode: 20, name: 'NES / Famicom', generation: 3, vendor: 'Nintendo', years: '1983–2003', extensions: ['nes', 'fds', 'unf', 'unif'], tier: 'foundation', launchable: true, note: 'Launchable development graph: shared 6502 CPU, CPU/PPU buses, controllers, OAM/DMC DMA, PPU/NMI/IRQ rendering, APU audio, deterministic state/persistence, NES 2.0 sizing, and mapper 0/1/2/3/4/7/11/66. More mapper and timing validation remain.' }),
  p({ id: 'segaMS', omniCode: 21, name: 'Sega Master System', generation: 3, vendor: 'Sega', years: '1985–1996', extensions: ['sms'], tier: 'foundation', launchable: true, bios: 'optional', note: 'Launchable development graph: reusable Z80, Sega mapper/SRAM, Mode 4 VDP background/sprites, frame/line interrupts, scroll locks, controllers, SN76489 audio and save states. Timing, legacy VDP modes, uncommon mappers and compatibility validation remain.' }),
  p({ id: 'atari7800', omniCode: 22, name: 'Atari 7800', generation: 3, vendor: 'Atari', years: '1986–1992', extensions: ['a78', 'bin'], tier: 'foundation', launchable: true, note: 'Launchable development graph: shared NMOS 6502/SALLY execution, console RAM/PIA/TIA audio/input, MARIA DLL/display-list rendering foundation, linear and SuperGame cartridges, optional POKEY mappings, and deterministic save states. MARIA DMA timing/modes and mapper breadth remain incomplete.' }),
  p({ id: 'snes', omniCode: 30, name: 'SNES / Super Famicom', generation: 4, vendor: 'Nintendo', years: '1990–2003', extensions: ['sfc', 'smc', 'fig', 'swc'], tier: 'foundation', launchable: true, bios: 'optional', note: 'Launchable development graph: reusable W65C816, LoROM/HiROM mapping, WRAM and CPU I/O, serial controllers, NMI/vblank timing, VRAM/CGRAM/OAM PPU register foundation, Mode 0/1 BG1 rendering, SRAM persistence and deterministic state. General DMA is implemented; SPC700/S-DSP audio, HDMA/timing, full PPU modes/sprites and enhancement chips remain.' }),
  p({ id: 'segaMD', omniCode: 31, name: 'Genesis / Mega Drive', generation: 4, vendor: 'Sega', years: '1988–1997', extensions: ['md', 'gen', 'smd', 'bin'], tier: 'foundation', launchable: true, note: 'Launchable development graph: shared Motorola 68000 and Z80 execution, Genesis memory/I/O and bus-request/reset paths, VRAM/CRAM/VSRAM VDP foundation with planes/sprites/interrupts, YM2612 register/timer/FM/DAC foundation, SN76489 PSG, controllers, SRAM persistence and SMD decoding. Cycle-exact VDP/DMA, full FM accuracy and broad cartridge compatibility remain.' }),
  p({ id: 'segaCD', omniCode: 32, name: 'Sega CD / Mega-CD', generation: 4, vendor: 'Sega', years: '1991–1996', extensions: ['cue', 'chd', 'iso'], tier: 'planned', bios: 'required' }),
  p({ id: 'sega32x', omniCode: 33, name: 'Sega 32X', generation: 4, vendor: 'Sega', years: '1994–1996', extensions: ['32x', 'bin'], tier: 'planned' }),
  p({ id: 'pce', omniCode: 34, name: 'TurboGrafx-16 / PC Engine', generation: 4, vendor: 'NEC', years: '1987–1994', extensions: ['pce', 'sgx', 'cue', 'chd'], tier: 'planned' }),
  p({ id: 'neo-geo', omniCode: 35, name: 'Neo Geo', generation: 4, vendor: 'SNK', years: '1990–2004', extensions: ['zip'], tier: 'planned', bios: 'required' }),
  p({ id: 'psx', omniCode: 40, name: 'PlayStation', generation: 5, vendor: 'Sony', years: '1994–2006', extensions: ['cue', 'chd', 'pbp', 'iso'], tier: 'planned', bios: 'required' }),
  p({ id: 'n64', omniCode: 41, name: 'Nintendo 64', generation: 5, vendor: 'Nintendo', years: '1996–2002', extensions: ['n64', 'z64', 'v64'], tier: 'planned' }),
  p({ id: 'segaSaturn', omniCode: 42, name: 'Sega Saturn', generation: 5, vendor: 'Sega', years: '1994–2000', extensions: ['cue', 'chd', 'iso'], tier: 'planned', bios: 'required' }),
  p({ id: 'jaguar', omniCode: 43, name: 'Atari Jaguar', generation: 5, vendor: 'Atari', years: '1993–1996', extensions: ['j64', 'jag', 'rom'], tier: 'planned' }),
  p({ id: '3do', omniCode: 44, name: '3DO', generation: 5, vendor: '3DO', years: '1993–1996', extensions: ['cue', 'chd', 'iso'], tier: 'planned', bios: 'required' }),
  p({ id: 'dreamcast', omniCode: 50, name: 'Dreamcast', generation: 6, vendor: 'Sega', years: '1998–2001', extensions: ['gdi', 'cdi', 'chd'], tier: 'planned' }),
  p({ id: 'ps2', omniCode: 51, name: 'PlayStation 2', generation: 6, vendor: 'Sony', years: '2000–2013', extensions: ['iso', 'chd'], tier: 'planned' }),
  p({ id: 'gamecube', omniCode: 52, name: 'Nintendo GameCube', generation: 6, vendor: 'Nintendo', years: '2001–2007', extensions: ['iso', 'gcm', 'rvz'], tier: 'planned' }),
  p({ id: 'xbox', omniCode: 53, name: 'Xbox', generation: 6, vendor: 'Microsoft', years: '2001–2009', extensions: ['iso', 'xiso'], tier: 'planned', bios: 'required' }),
  p({ id: 'xbox360', omniCode: 60, name: 'Xbox 360', generation: 7, vendor: 'Microsoft', years: '2005–2016', extensions: ['iso', 'xex'], tier: 'planned' }),
  p({ id: 'ps3', omniCode: 61, name: 'PlayStation 3', generation: 7, vendor: 'Sony', years: '2006–2017', extensions: ['iso', 'pkg'], tier: 'planned' }),
  p({ id: 'wii', omniCode: 62, name: 'Nintendo Wii', generation: 7, vendor: 'Nintendo', years: '2006–2017', extensions: ['iso', 'wbfs', 'rvz'], tier: 'planned' }),
  p({ id: 'wiiu', omniCode: 70, name: 'Wii U', generation: 8, vendor: 'Nintendo', years: '2012–2017', extensions: ['wux', 'wud', 'rpx'], tier: 'planned' }),
  p({ id: 'switch', omniCode: 71, name: 'Nintendo Switch', generation: 8, vendor: 'Nintendo', years: '2017–', extensions: ['nsp', 'xci', 'nro'], tier: 'planned', note: 'User-owned keys and firmware only; OmniEmu will never distribute console keys or firmware.' }),
]

export const READY_PLATFORMS = PLATFORMS.filter((platform) => platform.tier === 'playable')
export const platformById = (id: string) => PLATFORMS.find((platform) => platform.id === id)
export const corePlayable = (platform: Platform) => platform.tier === 'playable'

export const candidatesForFile = (name: string) => {
  const extension = name.split('.').pop()?.toLowerCase() ?? ''
  return PLATFORMS.filter((platform) => platform.extensions.includes(extension))
}
