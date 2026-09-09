export type SupportTier = 'ready' | 'experimental' | 'planned'

export interface Platform {
  id: string
  name: string
  generation: number
  vendor: string
  years: string
  core?: string
  extensions: string[]
  tier: SupportTier
  bios?: 'optional' | 'required'
  threads?: boolean
  note?: string
}

const p = (platform: Platform) => platform

export const PLATFORMS: Platform[] = [
  p({ id: 'odyssey', name: 'Magnavox Odyssey', generation: 1, vendor: 'Magnavox', years: '1972–1975', extensions: [], tier: 'planned', note: 'Dedicated analog hardware; no conventional ROM format.' }),
  p({ id: 'home-pong', name: 'Home Pong / Telstar class', generation: 1, vendor: 'Atari / Coleco', years: '1975–1978', extensions: [], tier: 'planned', note: 'Dedicated game hardware; implemented as hardware simulations rather than ROM loading.' }),
  p({ id: 'atari2600', name: 'Atari 2600', generation: 2, vendor: 'Atari', years: '1977–1992', core: 'atari2600', extensions: ['a26', 'bin'], tier: 'ready' }),
  p({ id: 'atari5200', name: 'Atari 5200', generation: 2, vendor: 'Atari', years: '1982–1984', core: 'a5200', extensions: ['a52', 'bin'], tier: 'ready' }),
  p({ id: 'coleco', name: 'ColecoVision', generation: 2, vendor: 'Coleco', years: '1982–1985', core: 'coleco', extensions: ['col', 'rom', 'bin'], tier: 'ready', bios: 'required' }),
  p({ id: 'intellivision', name: 'Intellivision', generation: 2, vendor: 'Mattel', years: '1979–1990', extensions: ['int', 'bin', 'rom'], tier: 'planned', bios: 'required' }),
  p({ id: 'nes', name: 'NES / Famicom', generation: 3, vendor: 'Nintendo', years: '1983–2003', core: 'nes', extensions: ['nes', 'fds', 'unf', 'unif'], tier: 'ready', bios: 'optional' }),
  p({ id: 'segaMS', name: 'Sega Master System', generation: 3, vendor: 'Sega', years: '1985–1996', core: 'segaMS', extensions: ['sms'], tier: 'ready', bios: 'optional' }),
  p({ id: 'atari7800', name: 'Atari 7800', generation: 3, vendor: 'Atari', years: '1986–1992', core: 'atari7800', extensions: ['a78', 'bin'], tier: 'ready' }),
  p({ id: 'snes', name: 'SNES / Super Famicom', generation: 4, vendor: 'Nintendo', years: '1990–2003', core: 'snes', extensions: ['sfc', 'smc', 'fig', 'swc'], tier: 'ready', bios: 'optional' }),
  p({ id: 'segaMD', name: 'Genesis / Mega Drive', generation: 4, vendor: 'Sega', years: '1988–1997', core: 'segaMD', extensions: ['md', 'gen', 'smd', 'bin'], tier: 'ready' }),
  p({ id: 'segaCD', name: 'Sega CD / Mega-CD', generation: 4, vendor: 'Sega', years: '1991–1996', core: 'segaCD', extensions: ['cue', 'chd', 'iso'], tier: 'ready', bios: 'required' }),
  p({ id: 'sega32x', name: 'Sega 32X', generation: 4, vendor: 'Sega', years: '1994–1996', core: 'sega32x', extensions: ['32x', 'bin'], tier: 'ready' }),
  p({ id: 'pce', name: 'TurboGrafx-16 / PC Engine', generation: 4, vendor: 'NEC', years: '1987–1994', core: 'pce', extensions: ['pce', 'sgx', 'cue', 'chd'], tier: 'ready' }),
  p({ id: 'neo-geo', name: 'Neo Geo', generation: 4, vendor: 'SNK', years: '1990–2004', core: 'arcade', extensions: ['zip'], tier: 'experimental', bios: 'required' }),
  p({ id: 'psx', name: 'PlayStation', generation: 5, vendor: 'Sony', years: '1994–2006', core: 'psx', extensions: ['cue', 'chd', 'pbp', 'iso'], tier: 'ready', bios: 'required' }),
  p({ id: 'n64', name: 'Nintendo 64', generation: 5, vendor: 'Nintendo', years: '1996–2002', core: 'n64', extensions: ['n64', 'z64', 'v64'], tier: 'ready' }),
  p({ id: 'segaSaturn', name: 'Sega Saturn', generation: 5, vendor: 'Sega', years: '1994–2000', core: 'segaSaturn', extensions: ['cue', 'chd', 'iso'], tier: 'experimental', bios: 'required' }),
  p({ id: 'jaguar', name: 'Atari Jaguar', generation: 5, vendor: 'Atari', years: '1993–1996', core: 'jaguar', extensions: ['j64', 'jag', 'rom'], tier: 'experimental' }),
  p({ id: '3do', name: '3DO', generation: 5, vendor: '3DO', years: '1993–1996', core: '3do', extensions: ['cue', 'chd', 'iso'], tier: 'experimental', bios: 'required' }),
  p({ id: 'dreamcast', name: 'Dreamcast', generation: 6, vendor: 'Sega', years: '1998–2001', extensions: ['gdi', 'cdi', 'chd'], tier: 'planned', note: 'Flycast WebAssembly is the initial dedicated-adapter candidate; its system files will use a multi-file bundle rather than the generic single-BIOS field.' }),
  p({ id: 'ps2', name: 'PlayStation 2', generation: 6, vendor: 'Sony', years: '2000–2013', extensions: ['iso', 'chd'], tier: 'planned', note: 'Play! is the initial browser-adapter candidate and uses high-level system emulation rather than an external BIOS.' }),
  p({ id: 'gamecube', name: 'Nintendo GameCube', generation: 6, vendor: 'Nintendo', years: '2001–2007', extensions: ['iso', 'gcm', 'rvz'], tier: 'planned', note: 'Requires a production browser Dolphin-class adapter.' }),
  p({ id: 'xbox', name: 'Xbox', generation: 6, vendor: 'Microsoft', years: '2001–2009', extensions: ['iso', 'xiso'], tier: 'planned', bios: 'required' }),
  p({ id: 'xbox360', name: 'Xbox 360', generation: 7, vendor: 'Microsoft', years: '2005–2016', extensions: ['iso', 'xex'], tier: 'planned' }),
  p({ id: 'ps3', name: 'PlayStation 3', generation: 7, vendor: 'Sony', years: '2006–2017', extensions: ['iso', 'pkg'], tier: 'planned' }),
  p({ id: 'wii', name: 'Nintendo Wii', generation: 7, vendor: 'Nintendo', years: '2006–2017', extensions: ['iso', 'wbfs', 'rvz'], tier: 'planned' }),
  p({ id: 'wiiu', name: 'Wii U', generation: 8, vendor: 'Nintendo', years: '2012–2017', extensions: ['wux', 'wud', 'rpx'], tier: 'planned' }),
  p({ id: 'switch', name: 'Nintendo Switch', generation: 8, vendor: 'Nintendo', years: '2017–', extensions: ['nsp', 'xci', 'nro'], tier: 'planned', note: 'User-owned keys and firmware only; no keys or firmware are distributed.' }),
  p({ id: 'ps4', name: 'PlayStation 4', generation: 8, vendor: 'Sony', years: '2013–', extensions: ['pkg'], tier: 'planned' }),
  p({ id: 'xboxone', name: 'Xbox One', generation: 8, vendor: 'Microsoft', years: '2013–', extensions: ['xvc', 'iso'], tier: 'planned' }),
]

export const READY_PLATFORMS = PLATFORMS.filter((platform) => platform.tier !== 'planned')

export const platformById = (id: string) => PLATFORMS.find((platform) => platform.id === id)

export const candidatesForFile = (name: string) => {
  const extension = name.split('.').pop()?.toLowerCase() ?? ''
  return PLATFORMS.filter((platform) => platform.extensions.includes(extension))
}
