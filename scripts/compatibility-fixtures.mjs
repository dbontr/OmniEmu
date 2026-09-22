const put16 = (view, offset, value, little = false) => view.setUint16(offset, value, little)
const put32 = (view, offset, value, little = false) => view.setUint32(offset, value, little)
const put64 = (view, offset, value) => view.setBigUint64(offset, BigInt(value), false)

export const ppcDol = (program) => {
  const image = new Uint8Array(0x100 + program.length * 4)
  const view = new DataView(image.buffer)
  put32(view, 0x00, 0x100)
  put32(view, 0x48, 0x80003100)
  put32(view, 0x90, program.length * 4)
  put32(view, 0xd8, 0x80004000)
  put32(view, 0xdc, 0x100)
  put32(view, 0xe0, 0x80003100)
  program.forEach((instruction, index) => put32(view, 0x100 + index * 4, instruction))
  return image
}

export const gameCubeIpl = () => {
  const image = new Uint8Array(2 * 1024 * 1024)
  image.set(new TextEncoder().encode('OmniEmu GameCube IPL compatibility fixture'), 0)
  image[0x100] = 0x12
  image[0x1aff00] = 0x34
  image[0x1fcf00] = 0x56
  return image
}

export const ps2Elf = (program) => {
  const image = new Uint8Array(0x100 + program.length * 4)
  const view = new DataView(image.buffer)
  image.set([0x7f, 0x45, 0x4c, 0x46, 1, 1, 1])
  put16(view, 16, 2, true); put16(view, 18, 8, true); put32(view, 20, 1, true)
  put32(view, 24, 0x00100000, true); put32(view, 28, 52, true)
  put16(view, 40, 52, true); put16(view, 42, 32, true); put16(view, 44, 1, true)
  put32(view, 52, 1, true); put32(view, 56, 0x100, true)
  put32(view, 60, 0x00100000, true); put32(view, 64, 0x00100000, true)
  put32(view, 68, program.length * 4, true); put32(view, 72, 0x100, true)
  put32(view, 76, 5, true); put32(view, 80, 0x1000, true)
  program.forEach((instruction, index) => put32(view, 0x100 + index * 4, instruction, true))
  return image
}

export const ppc64Elf = (program) => {
  const image = new Uint8Array(0x100 + program.length * 4)
  const view = new DataView(image.buffer)
  image.set([0x7f, 0x45, 0x4c, 0x46, 2, 2, 1])
  put16(view, 16, 2); put16(view, 18, 21); put32(view, 20, 1)
  put64(view, 24, 0x1000); put64(view, 32, 64)
  put16(view, 52, 64); put16(view, 54, 56); put16(view, 56, 1)
  put32(view, 64, 1); put32(view, 68, 5); put64(view, 72, 0x100)
  put64(view, 80, 0x1000); put64(view, 88, 0x1000)
  put64(view, 96, program.length * 4); put64(view, 104, program.length * 4); put64(view, 112, 0x1000)
  program.forEach((instruction, index) => put32(view, 0x100 + index * 4, instruction))
  return image
}

export const ppc32Elf = (program) => {
  const image = new Uint8Array(0x100 + program.length * 4)
  const view = new DataView(image.buffer)
  image.set([0x7f, 0x45, 0x4c, 0x46, 1, 2, 1])
  put16(view, 16, 2); put16(view, 18, 20); put32(view, 20, 1)
  put32(view, 24, 0x1000); put32(view, 28, 52)
  put16(view, 40, 52); put16(view, 42, 32); put16(view, 44, 1)
  put32(view, 52, 1); put32(view, 56, 0x100)
  put32(view, 60, 0x1000); put32(view, 64, 0x1000)
  put32(view, 68, program.length * 4); put32(view, 72, program.length * 4)
  put32(view, 76, 5); put32(view, 80, 0x1000)
  program.forEach((instruction, index) => put32(view, 0x100 + index * 4, instruction))
  return image
}

export const xboxXbe = (program) => {
  const base = 0x00010000
  const sectionVa = 0x00011000
  const sectionRaw = 0x400
  const table = 0x178
  const image = new Uint8Array(sectionRaw + program.length)
  const view = new DataView(image.buffer)
  put32(view, 0, 0x48454258, true); put32(view, 0x104, base, true)
  put32(view, 0x108, 0x400, true); put32(view, 0x11c, 1, true)
  put32(view, 0x120, base + table, true); put32(view, 0x128, sectionVa ^ 0xa8fc57ab, true)
  put32(view, table, 1 << 2, true); put32(view, table + 4, sectionVa, true)
  put32(view, table + 8, 0x1000, true); put32(view, table + 12, sectionRaw, true)
  put32(view, table + 16, program.length, true); image.set(program, sectionRaw)
  return image
}

export const switchNro = (program) => {
  const image = new Uint8Array(0x1000)
  const view = new DataView(image.buffer)
  put32(view, 0, 0x14000020, true)
  image.set(new TextEncoder().encode('NRO0'), 0x10)
  put32(view, 0x18, image.length, true)
  put32(view, 0x20, 0, true); put32(view, 0x24, 0x1000, true)
  put32(view, 0x28, 0x1000, true); put32(view, 0x2c, 0, true)
  put32(view, 0x30, 0x1000, true); put32(view, 0x34, 0, true)
  put32(view, 0x38, 0x1000, true)
  program.forEach((instruction, index) => put32(view, 0x80 + index * 4, instruction, true))
  return image
}

export const compatibilityFixture = (name) => {
  switch (name) {
    case 'nes-nrom': return nesNrom()
    case 'master-system-cart': return masterSystemCartridge()
    case 'coleco-cart': return colecoCartridge()
    case 'coleco-bios': return colecoBios()
    case 'intellivision-cart': return intellivisionCartridge()
    case 'intellivision-exec': return intellivisionExec()
    case 'intellivision-grom': return intellivisionGrom()
    case 'sega-cd-bios': return segaCdBios()
    case 'sega-cd-disc': return segaCdDisc()
    case 'pc-engine-cart': return pcEngineCartridge()
    case 'pc-engine-sf2-cart': return pcEngineSf2Cartridge()
    case 'pc-engine-system-card': return pcEngineSystemCard()
    case 'pc-engine-arcade-card-system-card': return pcEngineArcadeCardSystemCard()
    case 'pc-engine-cd-disc': return pcEngineCdDisc()
    case 'pc-engine-cd-cue': return pcEngineCueDisc()
    case 'neo-geo-cart': return neoGeoCartridge()
    case 'neo-geo-bios': return neoGeoBios()
    case 'atari2600-cart': return atari2600Cartridge()
    case 'atari5200-cart': return atari5200Cartridge()
    case 'atari5200-bios': return atari5200Bios()
    case 'atari7800-cart': return atari7800Cartridge()
    case 'genesis-cart': return genesisCartridge()
    case 'genesis-ssf2-cart': return genesisSsf2Cartridge()
    case 'sega32x-cart': return sega32xCartridge()
    case 'snes-cart': return snesCartridge()
    case 'ps1-bios': return ps1Bios()
    case 'n64-cart': return n64Cartridge()
    case 'saturn-bios': return saturnBios()
    case 'jaguar-cart': return jaguarCartridge()
    case 'jaguar-bios': return jaguarBios()
    case '3do-bios': return threeDoBios()
    case 'dreamcast-bios': return dreamcastBios()
    case 'dreamcast-disc': return dreamcastDisc()
    case 'dreamcast-cue': return pcEngineCueDisc()
    case 'ps2-elf-loop': return ps2Elf([0x1000ffff, 0])
    case 'ppc-dol-loop': return ppcDol([0x48000000])
    case 'gamecube-ipl': return gameCubeIpl()
    case 'xbox-xbe-loop': return xboxXbe(Uint8Array.from([0xeb, 0xfe]))
    case 'ppc64-elf-loop': return ppc64Elf([0x48000000])
    case 'ppc32-elf-loop': return ppc32Elf([0x48000000])
    case 'switch-nro-loop': return switchNro([0x14000000])
    default: throw new Error(`unknown compatibility fixture: ${name}`)
  }
}
export const nesNrom = () => {
  const image = new Uint8Array(16 + 16 * 1024 + 8 * 1024)
  image.set([0x4e, 0x45, 0x53, 0x1a, 1, 1], 0)
  image[6] = 0x02
  image.set([
    0x78, 0xd8, 0xa2, 0xff, 0x9a,
    0xa9, 0x80, 0x8d, 0x00, 0x20,
    0xa9, 0x08, 0x8d, 0x01, 0x20,
    0xa9, 0x3f, 0x8d, 0x06, 0x20,
    0xa9, 0x00, 0x8d, 0x06, 0x20,
    0xa9, 0x0f, 0x8d, 0x07, 0x20,
    0xa9, 0x30, 0x8d, 0x07, 0x20,
    0x4c, 0x23, 0x80,
  ], 16)
  image.set([0x00, 0x80, 0x00, 0x80, 0x00, 0x80], 16 + 0x3ffa)
  image.fill(0xff, 16 + 16 * 1024, 16 + 16 * 1024 + 8)
  return image
}

export const masterSystemCartridge = () => {
  const image = new Uint8Array(0x8000)
  image.set([
    0xf3, 0x31, 0xf0, 0xdf,
    0x3e, 0x04, 0xd3, 0xbf, 0x3e, 0x80, 0xd3, 0xbf,
    0x3e, 0xc0, 0xd3, 0xbf, 0x3e, 0x81, 0xd3, 0xbf,
    0x3e, 0xff, 0xd3, 0xbf, 0x3e, 0x82, 0xd3, 0xbf,
    0x3e, 0xff, 0xd3, 0xbf, 0x3e, 0x85, 0xd3, 0xbf,
    0x3e, 0xfb, 0xd3, 0xbf, 0x3e, 0x86, 0xd3, 0xbf,
    0x3e, 0x01, 0xd3, 0xbf, 0x3e, 0xc0, 0xd3, 0xbf,
    0x3e, 0x03, 0xd3, 0xbe,
    0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x40, 0xd3, 0xbf,
    0x21, 0x00, 0x01, 0x06, 0x20,
    0x7e, 0xd3, 0xbe, 0x23, 0x10, 0xfa,
    0xc3, 0x4b, 0x00,
  ])
  for (let row = 0; row < 8; row++) image[0x100 + row * 4] = 0xff
  return image
}

export const colecoBios = () => {
  const image = new Uint8Array(0x2000)
  image.set([0xc3, 0x00, 0x80], 0)
  image.set([0xed, 0x45], 0x66)
  return image
}

export const colecoCartridge = () => {
  const image = new Uint8Array(0x2000)
  const program = [
    0xf3, 0x31, 0xff, 0x7f,
    0x3e, 0x02, 0xd3, 0xbf, 0x3e, 0x80, 0xd3, 0xbf,
    0x3e, 0x40, 0xd3, 0xbf, 0x3e, 0x81, 0xd3, 0xbf,
    0x3e, 0x06, 0xd3, 0xbf, 0x3e, 0x82, 0xd3, 0xbf,
    0x3e, 0x80, 0xd3, 0xbf, 0x3e, 0x83, 0xd3, 0xbf,
    0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x84, 0xd3, 0xbf,
    0x3e, 0x36, 0xd3, 0xbf, 0x3e, 0x85, 0xd3, 0xbf,
    0x3e, 0x07, 0xd3, 0xbf, 0x3e, 0x86, 0xd3, 0xbf,
    0x3e, 0x01, 0xd3, 0xbf, 0x3e, 0x87, 0xd3, 0xbf,
  ]
  image.set(program)
  const tail = [
    0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x40, 0xd3, 0xbf,
    0x21, 0x00, 0x81, 0x06, 0x08, 0x7e, 0xd3, 0xbe, 0x23, 0x10, 0xfa,
    0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x60, 0xd3, 0xbf,
    0x21, 0x10, 0x81, 0x06, 0x08, 0x7e, 0xd3, 0xbe, 0x23, 0x10, 0xfa,
    0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x58, 0xd3, 0xbf, 0xaf, 0xd3, 0xbe,
    0x3e, 0x84, 0xd3, 0xff, 0x3e, 0x10, 0xd3, 0xff, 0x3e, 0x90, 0xd3, 0xff,
  ]
  image.set(tail, program.length)
  const loop = 0x8000 + program.length + tail.length
  image.set([0xc3, loop & 0xff, loop >> 8], program.length + tail.length)
  image.fill(0xff, 0x100, 0x108)
  image.fill(0xf1, 0x110, 0x118)
  return image
}

export const intellivisionExec = () => {
  const image = new Uint8Array(0x2000)
  const view = new DataView(image.buffer)
  ;[0x0004, 0x0050, 0x0000].forEach((word, index) => view.setUint16(index * 2, word, false))
  return image
}

export const intellivisionGrom = () => {
  const image = new Uint8Array(0x0800)
  for (let row = 0; row < 8; row++) image[row] = row === 0 || row === 7 ? 0xff : 0x81
  return image
}

export const intellivisionCartridge = () => {
  const words = [
    0x02b8, 0x0001, 0x0240, 0x0020,
    0x02b8, 0x0007, 0x0240, 0x002c,
    0x02b8, 0x0007, 0x0240, 0x0200,
    0x0000,
  ]
  const image = new Uint8Array(words.length * 2)
  const view = new DataView(image.buffer)
  words.forEach((word, index) => view.setUint16(index * 2, word, false))
  return image
}
export const segaCdBios = () => {
  const image = new Uint8Array(128 * 1024)
  image.fill(0xff)
  const view = new DataView(image.buffer)
  view.setUint32(0, 0x00ffff00, false)
  view.setUint32(4, 0x00000200, false)
  const words = [
    0x13fc, 0x0084, 0x00c0, 0x0011,
    0x13fc, 0x0010, 0x00c0, 0x0011,
    0x13fc, 0x0090, 0x00c0, 0x0011,
    0x60fe,
  ]
  words.forEach((word, index) => view.setUint16(0x200 + index * 2, word, false))
  return image
}

export const segaCdDisc = () => {
  const image = new Uint8Array(2048 * 4)
  for (let sector = 0; sector < 4; sector++) {
    image.fill(sector + 1, sector * 2048, (sector + 1) * 2048)
  }
  return image
}
export const pcEngineCartridge = () => {
  const image = new Uint8Array(0x2000)
  image.fill(0xea)
  const code = [
    0xd4, 0xa9,0xf8,0x53,0x02, 0xa9,0xff,0x53,0x40,
    0xa9,0x01,0x8d,0x02,0xc4, 0xa9,0x00,0x8d,0x03,0xc4,
    0xa9,0x38,0x8d,0x04,0xc4, 0xa9,0x00,0x8d,0x05,0xc4,
    0x03,0x00,0x13,0x00,0x23,0x00, 0x03,0x02,0x13,0x01,0x23,0x00,
    0x03,0x00,0x13,0x10,0x23,0x00, 0x03,0x02,
  ]
  for (let row = 0; row < 8; row++) code.push(0x13,0xff,0x23,0x00)
  for (let row = 0; row < 8; row++) code.push(0x13,0x00,0x23,0x00)
  code.push(
    0x03,0x05,0x13,0x80,0x23,0x00, 0xa9,0x00,0x8d,0x00,0xc8,
    0xa9,0xff,0x8d,0x01,0xc8, 0xa9,0x20,0x8d,0x02,0xc8,
    0xa9,0x00,0x8d,0x03,0xc8, 0xa9,0xff,0x8d,0x05,0xc8,
  )
  for (let index = 0; index < 32; index++) code.push(0xa9, index & 1 ? 31 : 0, 0x8d,0x06,0xc8)
  code.push(0xa9,0x9f,0x8d,0x04,0xc8,0x80,0xfe)
  image.set(code)
  image[0x1ffe] = 0x00
  image[0x1fff] = 0xe0
  return image
}

export const pcEngineSf2Cartridge = () => {
  const base = pcEngineCartridge()
  const image = new Uint8Array(0x280000)
  image.fill(0xea)
  image.set(base, 0)
  for (let bank = 0; bank < 4; bank++) {
    image[0x080000 + bank * 0x080000] = 0x40 + bank
    image[0x080123 + bank * 0x080000] = 0x80 + bank
  }
  return image
}

export const pcEngineSystemCard = () => {
  const image = new Uint8Array(256 * 1024)
  image.fill(0xea)
  image.set(pcEngineCartridge(), 0)
  return image
}

export const pcEngineArcadeCardSystemCard = () => {
  const image = pcEngineSystemCard()
  const probe = [
    0xd4,
    0xa9,0xff,0x53,0x40,
    0xa9,0x40,0x53,0x20,
    0xad,0xfe,0xda,0xc9,0x10,0xf0,0x02,0x80,0xfe,
    0xad,0xff,0xda,0xc9,0x51,0xf0,0x02,0x80,0xfe,
    0xa9,0x00,0x8d,0x02,0xda,0x8d,0x03,0xda,0x8d,0x04,0xda,
    0xa9,0x01,0x8d,0x07,0xda,0xa9,0x00,0x8d,0x08,0xda,
    0xa9,0x11,0x8d,0x09,0xda,
    0xa9,0x5a,0x8d,0x00,0xa0,0xa9,0xa5,0x8d,0x00,0xa0,
    0xa9,0x00,0x8d,0x02,0xda,0x8d,0x09,0xda,
    0xad,0x00,0xa0,0xc9,0x5a,0xf0,0x02,0x80,0xfe,
    0xa9,0x01,0x8d,0x02,0xda,
    0xad,0x00,0xa0,0xc9,0xa5,0xf0,0x02,0x80,0xfe,
    0x4c,0x00,0xe0,
  ]
  image.set(probe, 0x1000)
  image[0x1ffe] = 0x00
  image[0x1fff] = 0xf0
  return image
}

export const pcEngineCdDisc = () => {
  const image = new Uint8Array(2048 * 4)
  for (let sector = 0; sector < 4; sector++) {
    image.fill(0x40 + sector, sector * 2048, (sector + 1) * 2048)
  }
  return image
}

export const pcEngineCueDisc = () => {
  const cueText = 'FILE "data.bin" BINARY\n  TRACK 01 MODE1/2048\n    INDEX 01 00:00:00\nFILE "audio.bin" BINARY\n  TRACK 02 AUDIO\n    INDEX 01 00:00:00\n'
  const cue = new TextEncoder().encode(cueText)
  const data = new Uint8Array(2 * 2048); data.fill(0x5a)
  const audio = new Uint8Array(2 * 2352)
  const audioView = new DataView(audio.buffer)
  for (let offset = 0; offset < audio.length; offset += 4) {
    audioView.setInt16(offset, 0x2000, true)
    audioView.setInt16(offset + 2, -0x2000, true)
  }
  const files = [['data.bin', data], ['audio.bin', audio]]
  const directoryLength = files.reduce((sum, [name]) => sum + 2 + name.length + 16, 0)
  const dataStart = 16 + cue.length + directoryLength
  const image = new Uint8Array(dataStart + data.length + audio.length)
  const view = new DataView(image.buffer)
  image.set(new TextEncoder().encode('OMCUE001'), 0)
  view.setUint32(8, cue.length, true); view.setUint32(12, files.length, true)
  image.set(cue, 16)
  let directoryOffset = 16 + cue.length
  let dataOffset = dataStart
  for (const [name, bytes] of files) {
    const encodedName = new TextEncoder().encode(name)
    view.setUint16(directoryOffset, encodedName.length, true); directoryOffset += 2
    image.set(encodedName, directoryOffset); directoryOffset += encodedName.length
    view.setBigUint64(directoryOffset, BigInt(bytes.length), true)
    view.setBigUint64(directoryOffset + 8, BigInt(dataOffset), true); directoryOffset += 16
    image.set(bytes, dataOffset); dataOffset += bytes.length
  }
  return image
}

export const neoGeoCartridge = () => {
  const p = new Uint8Array(0x10000); p.fill(0xff)
  const s = new Uint8Array(0x10000); s.fill(0xff, 32, 64)
  const m = new Uint8Array(0x10000)
  m.set([
    0x3e,0x00,0xd3,0x04, 0x3e,0x20,0xd3,0x05,
    0x3e,0x01,0xd3,0x04, 0x3e,0x00,0xd3,0x05,
    0x3e,0x07,0xd3,0x04, 0x3e,0x3e,0xd3,0x05,
    0x3e,0x08,0xd3,0x04, 0x3e,0x0f,0xd3,0x05, 0x76,
  ])
  m.set([0xdb,0x00,0xd3,0x0c,0xed,0x45], 0x66)
  const v1 = new Uint8Array(0x10000)
  const c = new Uint8Array(0x40000)
  const sections = [p, s, m, v1, new Uint8Array(0), c]
  const length = 4096 + sections.reduce((sum, section) => sum + section.byteLength, 0)
  const image = new Uint8Array(length)
  image.set([0x4e,0x45,0x4f,0x01])
  const header = new DataView(image.buffer)
  sections.forEach((section, index) => header.setUint32(4 + index * 4, section.byteLength, true))
  header.setUint32(40, 0x0123, true)
  let offset = 4096
  for (const section of sections) { image.set(section, offset); offset += section.byteLength }
  return image
}
export const neoGeoBios = () => {
  const image = new Uint8Array(128 * 1024)
  image.fill(0x4e)
  const view = new DataView(image.buffer)
  view.setUint32(0, 0x0010ff00, false)
  view.setUint32(4, 0x00c00100, false)
  for (let vector = 24; vector <= 27; vector++) view.setUint32(vector * 4, 0x00c00200, false)
  const words = [
    0x33fc,0x0f00,0x0040,0x003e,
    0x33fc,0x7002,0x003c,0x0000,
    0x33fc,0x1001,0x003c,0x0002,
    0x60fe,
  ]
  words.forEach((word, index) => view.setUint16(0x100 + index * 2, word, false))
  view.setUint16(0x200, 0x4e73, false)
  return image
}

export const atari2600Cartridge = () => {
  const image = new Uint8Array(0x1000)
  image.fill(0xea)
  image.set([
    0x78, 0xd8, 0xa2, 0xff, 0x9a, 0xa9, 0x00, 0x85, 0x01,
    0xa9, 0x2e, 0x85, 0x09, 0xa9, 0x4e, 0x85, 0x08,
    0xa9, 0xf0, 0x85, 0x0d, 0xa9, 0xff, 0x85, 0x0e, 0x85, 0x0f,
    0xa9, 0x04, 0x85, 0x15, 0xa9, 0x08, 0x85, 0x17,
    0xa9, 0x0f, 0x85, 0x19, 0xa9, 0x00, 0x85, 0x02,
    0x4c, 0x27, 0xf0,
  ])
  image.set([0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0], 0x0ffa)
  return image
}

export const atari5200Bios = () => {
  const image = new Uint8Array(0x800)
  image.fill(0xea)
  image.set([
    0x78, 0xd8, 0xa9, 0x3a, 0x8d, 0x18, 0xc0,
    0xa9, 0x84, 0x8d, 0x1a, 0xc0,
    0xa9, 0x20, 0x8d, 0x00, 0xd4,
    0xa9, 0x00, 0x8d, 0x02, 0xd4,
    0xa9, 0x20, 0x8d, 0x03, 0xd4,
    0xa9, 0x40, 0x8d, 0x0e, 0xd4,
    0x4c, 0x00, 0x40,
  ])
  image.set([0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8], 0x7fa)
  return image
}
export const atari5200Cartridge = () => {
  const image = new Uint8Array(0x8000)
  image.fill(0xea)
  image.set([
    0xa9, 0x08, 0x8d, 0x00, 0xe8,
    0xa9, 0xaf, 0x8d, 0x01, 0xe8,
    0x4c, 0x0a, 0x40,
  ])
  return image
}

export const atari7800Cartridge = () => {
  const image = new Uint8Array(0xc000)
  image.fill(0xea)
  image.set([
    0x78, 0xd8, 0xa9, 0x22, 0x85, 0x20,
    0xa9, 0x2e, 0x85, 0x21, 0xa9, 0x4e, 0x85, 0x22,
    0xa9, 0x6e, 0x85, 0x23, 0xa9, 0x00, 0x85, 0x30,
    0xa9, 0x18, 0x85, 0x2c, 0xa9, 0x40, 0x85, 0x3c,
    0xa9, 0x04, 0x85, 0x15, 0xa9, 0x08, 0x85, 0x17,
    0xa9, 0x0f, 0x85, 0x19, 0x4c, 0x2a, 0x40,
  ])
  image.set([0x00, 0x40, 0x00, 0x40, 0x00, 0x40], 0xbffa)
  return image
}
export const genesisCartridge = () => {
  const image = new Uint8Array(0x40000)
  image.fill(0xff)
  const view = new DataView(image.buffer)
  view.setUint32(0, 0x00ffff00, false)
  view.setUint32(4, 0x00000200, false)
  const words = [
    0x13fc, 0x0084, 0x00c0, 0x0011,
    0x13fc, 0x0010, 0x00c0, 0x0011,
    0x13fc, 0x0090, 0x00c0, 0x0011,
    0x60fe,
  ]
  words.forEach((word, index) => view.setUint16(0x200 + index * 2, word, false))
  return image
}

export const genesisSsf2Cartridge = () => {
  const image = new Uint8Array(0x500000)
  image.fill(0xff)
  image.set(genesisCartridge(), 0)
  const view = new DataView(image.buffer)
  image.set(new TextEncoder().encode('SUPER STREET FIGHTER2'), 0x120)
  view.setUint32(4, 0x00000300, false)
  const probe = [
    0x13fc,0x0008,0x00a1,0x30fd,
    0x13fc,0x0009,0x00a1,0x30ff,
    0x4ef9,0x0030,0x0000,
  ]
  probe.forEach((word, index) => view.setUint16(0x300 + index * 2, word, false))
  view.setUint16(0x300000, 0x60fe, false)
  view.setUint16(0x380000, 0x60fe, false)
  ;[0x4ef9,0x0038,0x0000].forEach((word, index) => view.setUint16(0x400000 + index * 2, word, false))
  const success = [
    0x33fc,0xc000,0x00c0,0x0004,
    0x33fc,0x0000,0x00c0,0x0004,
    0x33fc,0x0eee,0x00c0,0x0000,
    0x60fe,
  ]
  success.forEach((word, index) => view.setUint16(0x480000 + index * 2, word, false))
  return image
}

export const sega32xCartridge = () => {
  const image = new Uint8Array(0x20000)
  image.fill(0xff)
  const view = new DataView(image.buffer)
  const u16 = (offset, value) => view.setUint16(offset, value, false)
  const u32 = (offset, value) => view.setUint32(offset, value, false)
  u32(0, 0x00ffff00); u32(4, 0x00000400)
  image.set(new TextEncoder().encode('SEGA 32X'), 0x100)
  image.set(new TextEncoder().encode('OMNICORE '), 0x3c0)
  u32(0x3d0, 1); u32(0x3d4, 0x00001000); u32(0x3d8, 0x00000000)
  u32(0x3dc, 0x00000400); u32(0x3e0, 0x06000000)
  u32(0x3e4, 0x06000020); u32(0x3e8, 0x06000200); u32(0x3ec, 0x06000240)
  const main = [
    0x33fc,0x0003,0x00a1,0x5100, 0x33fc,0x0000,0x00a1,0x5120,
    0x33fc,0x0000,0x00a1,0x5122, 0x33fc,0x0000,0x00a1,0x5124,
    0x33fc,0x0000,0x00a1,0x5126, 0x33fc,0x001f,0x00a1,0x5202,
    0x33fc,0x0081,0x00a1,0x5180, 0x33fc,0x0001,0x00a1,0x518a,
    0x33fc,0x0100,0x0084,0x0000, 0x13fc,0x0001,0x0084,0x0200,
    0x60fe,
  ]
  main.forEach((word, index) => u16(0x400 + index * 2, word))
  const master = [0xd103,0xe05a,0x2102,0xaffe,0x0009]
  master.forEach((word, index) => u16(0x1000 + index * 2, word))
  u32(0x1010, 0x06000100)
  const slave = [0xd103,0xe066,0x2102,0xaffe,0x0009]
  slave.forEach((word, index) => u16(0x1020 + index * 2, word))
  u32(0x1030, 0x06000104)
  return image
}

export const snesCartridge = () => {
  const image = new Uint8Array(0x8000)
  image.fill(0xea)
  image.set([
    0x78,0xa9,0x0f,0x8d,0x00,0x21,0xa9,0x00,0x8d,0x21,0x21,
    0xa9,0x1f,0x8d,0x22,0x21,0xa9,0x00,0x8d,0x22,0x21,0x80,0xfe,
  ])
  image[0x7fd5] = 0x20
  image[0x7fd8] = 0x01
  image.set([0xff,0xff,0x00,0x00], 0x7fdc)
  image.set([0x00,0x80,0x00,0x80], 0x7ffc)
  return image
}

export const ps1Bios = () => {
  const image = new Uint8Array(512 * 1024)
  const view = new DataView(image.buffer)
  const words = [
    0x3c081f80,0x35081814,0x3c090800,0x35290001,0xad090000,
    0x3c090300,0xad090000,0x2508fffc,0x3c090200,0x352900ff,
    0xad090000,0x24090000,0xad090000,0x3c0900f0,0x35290140,
    0xad090000,0x0bf00010,0x00000000,
  ]
  words.forEach((word, index) => view.setUint32(index * 4, word, true))
  return image
}

export const n64Cartridge = () => {
  const image = new Uint8Array(0x4000)
  const view = new DataView(image.buffer)
  view.setUint32(0, 0x80371240, false)
  const words = [
    0x3c08a440,0x34090002,0xad090000,
    0x34091000,0xad090004,0x34090004,0xad090008,0x34090008,0xad090028,
    0x3c0a8000,0x354a1000,0x3409f801,0xa5490000,
    0x3c0a8000,0x354a2000,0x3c094000,0x3529c000,0xad490000,
    0x3c092000,0x3529e000,0xad490004,
    0x3c08a450,0x34092000,0xad090000,0x34090008,0xad090004,
    0x1000ffff,0x00000000,
  ]
  words.forEach((word, index) => view.setUint32(0x40 + index * 4, word, false))
  return image
}

export const saturnBios = () => {
  const image = new Uint8Array(512 * 1024)
  const view = new DataView(image.buffer)
  view.setUint32(0, 0x00000100, false)
  view.setUint32(4, 0x060ffff0, false)
  view.setUint16(0x100, 0x0009, false)
  view.setUint16(0x102, 0xaffe, false)
  view.setUint16(0x104, 0x0009, false)
  return image
}

export const jaguarCartridge = () => {
  const image = new Uint8Array(0x20000)
  image.fill(0xff)
  return image
}

export const jaguarBios = () => {
  const image = new Uint8Array(128 * 1024)
  image.fill(0xff)
  const view = new DataView(image.buffer)
  view.setUint32(0, 0x001fff00, false)
  view.setUint32(4, 0x00e00100, false)
  const words = [
    0x33fc,0x0007,0x00f0,0x0028,
    0x33fc,0xf800,0x00f0,0x0058,
    0x33fc,0x2000,0x00f1,0xa14c,
    0x33fc,0xe000,0x00f1,0xa148,
    0x60fe,
  ]
  words.forEach((word, index) => view.setUint16(0x100 + index * 2, word, false))
  return image
}
export const threeDoBios = () => {
  const image = new Uint8Array(1024 * 1024)
  new DataView(image.buffer).setUint32(0, 0xeafffffe, true)
  return image
}

export const dreamcastBios = () => {
  const image = new Uint8Array(2 * 1024 * 1024)
  const view = new DataView(image.buffer)
  view.setUint16(0, 0xaffe, true)
  view.setUint16(2, 0x0009, true)
  return image
}

export const dreamcastDisc = () => {
  const image = new Uint8Array(4 * 2048)
  for (let sector = 0; sector < 4; sector++) {
    image.fill(sector + 1, sector * 2048, (sector + 1) * 2048)
  }
  return image
}
