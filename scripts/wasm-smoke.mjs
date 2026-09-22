import { readFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'
import { ppc32Elf, ppc64Elf, ppcDol, ps2Elf, switchNro, xboxXbe } from './compatibility-fixtures.mjs'

const root = dirname(dirname(fileURLToPath(import.meta.url)))
const wasmPath = join(root, 'public', 'omnicore.wasm')
const bytes = readFileSync(wasmPath)
const { instance } = await WebAssembly.instantiate(bytes, {})
const core = instance.exports

const assert = (condition, message) => {
  if (!condition) throw new Error(`OmniCore WASM smoke failed: ${message}`)
}
const errorText = () => {
  const ptr = core.omni_last_error_ptr()
  const len = core.omni_last_error_len()
  return ptr && len ? new TextDecoder().decode(new Uint8Array(core.memory.buffer, ptr, len)) : 'unknown error'
}
const ok = (value, operation) => assert(value === 0, `${operation}: ${errorText()}`)

assert(core.omni_core_version() === 1, 'unexpected ABI version')
assert(core.omni_support_level(2) >= 2, 'built-in Pong is not playable')
ok(core.omni_load(2, 0, 0, 0, 0), 'load Pong')
ok(core.omni_set_input(0, 1n << 5n), 'set player-one input')
ok(core.omni_set_axis(0, 0, 12345), 'set player-one analog axis')
ok(core.omni_set_axis(0, 5, 32767), 'set player-one analog trigger')
for (let frame = 0; frame < 8; frame++) ok(core.omni_run_frame(), `run frame ${frame}`)

const width = core.omni_video_width()
const height = core.omni_video_height()
const videoPtr = core.omni_video_ptr()
const videoLen = core.omni_video_len()
assert(width === 320 && height === 240, `unexpected Pong surface ${width}x${height}`)
assert(videoPtr > 0 && videoLen === width * height * 4, 'invalid framebuffer')
const video = new Uint8Array(core.memory.buffer, videoPtr, videoLen)
assert(video.some((value) => value !== 0), 'framebuffer is empty')

const audioPtr = core.omni_audio_ptr()
const audioLen = core.omni_audio_len()
assert(audioPtr > 0 && audioLen > 0, 'audio buffer is empty')
assert(core.omni_audio_rate() === 48_000, 'unexpected audio sample rate')
assert(core.omni_audio_channels() === 2, 'unexpected audio channel count')

const saveSize = core.omni_save_state(0, 0)
assert(saveSize > 0, 'save state reports zero bytes')
const savePtr = core.omni_alloc(saveSize)
assert(savePtr > 0, 'save-state allocation failed')
const written = core.omni_save_state(savePtr, saveSize)
assert(written === saveSize, `save state wrote ${written}/${saveSize} bytes`)
const snapshot = new Uint8Array(core.memory.buffer, savePtr, saveSize).slice()
ok(core.omni_reset(), 'reset Pong')
new Uint8Array(core.memory.buffer, savePtr, saveSize).set(snapshot)
ok(core.omni_load_state(savePtr, saveSize), 'restore Pong')
core.omni_free(savePtr, saveSize)
core.omni_unload()

assert(core.omni_can_launch(1) === 1, 'Odyssey should be launchable')
assert(core.omni_can_launch(2) === 1, 'Pong should be launchable')
assert(core.omni_can_launch(13) === 1, 'Intellivision foundation should be launchable for development')
assert(core.omni_can_launch(20) === 1, 'NES foundation should be launchable for development')
assert(core.omni_can_launch(30) === 1, 'SNES foundation should be launchable for development')
assert(core.omni_can_launch(32) === 1, 'Sega CD foundation should be launchable for development')
assert(core.omni_can_launch(33) === 1, 'Sega 32X foundation should be launchable for development')
assert(core.omni_can_launch(34) === 1, 'PC Engine foundation should be launchable for development')
assert(core.omni_can_launch(35) === 1, 'Neo Geo foundation should be launchable for development')
assert(core.omni_can_launch(36) === 1, 'SuperGrafx foundation should be launchable for development')
assert(core.omni_can_launch(41) === 1, 'Nintendo 64 foundation should be launchable for development')
assert(core.omni_can_launch(42) === 1, 'Saturn foundation should be launchable for development')
assert(core.omni_can_launch(43) === 1, 'Jaguar foundation should be launchable for development')
assert(core.omni_can_launch(44) === 1, '3DO foundation should be launchable for development')
assert(core.omni_is_targeted(30) === 1, 'SNES should remain an OmniCore target')
assert(core.omni_is_targeted(71) === 1, 'Switch should remain an OmniCore target')
assert(core.omni_is_targeted(72) === 0, 'PS4 should be outside the current target set')
assert(core.omni_is_targeted(73) === 0, 'Xbox One should be outside the current target set')
assert(core.omni_target_count() === 30, 'unexpected active target count')
for (const platform of [1, 2, 10, 11, 12, 13, 20, 21, 22]) {
  assert(core.omni_can_launch(platform) === 1, `generation 1-3 platform ${platform} is not launchable`)
}
assert(typeof core.omni_resource_create_streaming === 'function', 'streaming resource ABI is missing')
assert(typeof core.omni_resource_pending_start === 'function', 'pending-range ABI is missing')
assert(typeof core.omni_resource_pending_end === 'function', 'pending-range end ABI is missing')
core.omni_resources_clear()
ok(core.omni_resource_create_streaming(4, 0, 8192n, 2), 'create streaming disc resource')
assert(core.omni_resource_pending_start(4, 0) === -1n, 'fresh streaming resource has a pending read')
assert(core.omni_resource_pending_end(4, 0) === 0n, 'fresh streaming resource has an invalid pending end')
const targetIds = Array.from({ length: core.omni_target_count() }, (_, index) => core.omni_target_at(index))
assert(targetIds.includes(71), 'Switch target is missing')
assert(!targetIds.includes(72) && !targetIds.includes(73), 'excluded generation-8 targets leaked into active list')

core.omni_resources_clear()
ok(core.omni_load_staged(1), 'load Odyssey')
ok(core.omni_run_frame(), 'run Odyssey frame')
assert(core.omni_video_width() === 320 && core.omni_video_height() === 240, 'unexpected Odyssey surface')
assert(core.omni_video_len() === 320 * 240 * 4, 'invalid Odyssey framebuffer')
assert(core.omni_audio_len() === 0, 'Odyssey must remain silent')
assert(core.omni_save_state(0, 0) > 0, 'Odyssey save state is empty')
core.omni_unload()

const nes = new Uint8Array(16 + 16 * 1024 + 8 * 1024)
nes.set([0x4e, 0x45, 0x53, 0x1a, 1, 1], 0)
nes[6] = 0x02 // battery-backed PRG RAM
const program = [
  0x78, 0xd8, 0xa2, 0xff, 0x9a,
  0xa9, 0x80, 0x8d, 0x00, 0x20,
  0xa9, 0x08, 0x8d, 0x01, 0x20,
  0xa9, 0x3f, 0x8d, 0x06, 0x20,
  0xa9, 0x00, 0x8d, 0x06, 0x20,
  0xa9, 0x0f, 0x8d, 0x07, 0x20,
  0xa9, 0x30, 0x8d, 0x07, 0x20,
  0x4c, 0x23, 0x80,
]
nes.set(program, 16)
const vectorOffset = 16 + 0x3ffa
nes.set([0x00, 0x80, 0x00, 0x80, 0x00, 0x80], vectorOffset)
nes.fill(0xff, 16 + 16 * 1024, 16 + 16 * 1024 + 8)
core.omni_resources_clear()
ok(core.omni_resource_create(0, 0, BigInt(nes.byteLength)), 'create staged NES resource')
const nesPtr = core.omni_alloc(nes.byteLength)
assert(nesPtr > 0, 'NES staging allocation failed')
new Uint8Array(core.memory.buffer, nesPtr, nes.byteLength).set(nes)
ok(core.omni_resource_write(0, 0, 0n, nesPtr, nes.byteLength), 'stage NES resource')
core.omni_free(nesPtr, nes.byteLength)
ok(core.omni_load_staged(20), 'load staged NES NROM')
const persistentLen = core.omni_persistent_len(5, 0)
assert(persistentLen === 8192, `unexpected NES persistent length ${persistentLen}`)
const persistentPtr = core.omni_alloc(persistentLen)
assert(persistentPtr > 0, 'persistent allocation failed')
const persistentInput = new Uint8Array(core.memory.buffer, persistentPtr, persistentLen)
persistentInput.fill(0x5a)
ok(core.omni_write_persistent(5, 0, persistentPtr, persistentLen), 'write NES persistent RAM')
persistentInput.fill(0)
ok(core.omni_read_persistent(5, 0, persistentPtr, persistentLen), 'read NES persistent RAM')
assert(persistentInput.every((value) => value === 0x5a), 'NES persistent RAM did not round trip')
core.omni_free(persistentPtr, persistentLen)
for (let frame = 0; frame < 2; frame++) ok(core.omni_run_frame(), `run NES frame ${frame}`)
const nesWidth = core.omni_video_width()
const nesHeight = core.omni_video_height()
const nesVideoLen = core.omni_video_len()
assert(nesWidth === 256 && nesHeight === 240, `unexpected NES surface ${nesWidth}x${nesHeight}`)
assert(nesVideoLen === nesWidth * nesHeight * 4, 'invalid NES framebuffer')
const nesVideo = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), nesVideoLen)
assert(nesVideo.some((value) => value !== 0), 'NES framebuffer is empty')
assert(core.omni_audio_rate() === 48_000, 'unexpected NES audio sample rate')
assert(core.omni_audio_channels() === 2, 'unexpected NES audio channel count')
core.omni_unload()

assert(core.omni_can_launch(21) === 1, 'Master System foundation should be launchable for development')
const sms = new Uint8Array(0x8000)
const smsProgram = [
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
]
sms.set(smsProgram)
for (let row = 0; row < 8; row++) sms[0x100 + row * 4] = 0xff
core.omni_resources_clear()
ok(core.omni_resource_create(0, 0, BigInt(sms.byteLength)), 'create staged Master System resource')
const smsPtr = core.omni_alloc(sms.byteLength)
assert(smsPtr > 0, 'Master System staging allocation failed')
new Uint8Array(core.memory.buffer, smsPtr, sms.byteLength).set(sms)
ok(core.omni_resource_write(0, 0, 0n, smsPtr, sms.byteLength), 'stage Master System resource')
core.omni_free(smsPtr, sms.byteLength)
ok(core.omni_load_staged(21), 'load staged Master System cartridge')
for (let frame = 0; frame < 2; frame++) ok(core.omni_run_frame(), `run Master System frame ${frame}`)
const smsWidth = core.omni_video_width()
const smsHeight = core.omni_video_height()
const smsVideoLen = core.omni_video_len()
assert(smsWidth === 256 && smsHeight === 192, `unexpected Master System surface ${smsWidth}x${smsHeight}`)
assert(smsVideoLen === smsWidth * smsHeight * 4, 'invalid Master System framebuffer')
const smsVideo = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), smsVideoLen)
let smsHasRed = false
for (let i = 0; i < smsVideo.length; i += 4) {
  if (smsVideo[i] > smsVideo[i + 1] && smsVideo[i] > smsVideo[i + 2]) { smsHasRed = true; break }
}
assert(smsHasRed, 'Master System Mode 4 framebuffer did not render expected palette output')
assert(core.omni_audio_rate() === 48_000, 'unexpected Master System audio sample rate')
assert(core.omni_audio_channels() === 2, 'unexpected Master System audio channel count')
core.omni_unload()

assert(core.omni_can_launch(12) === 1, 'ColecoVision foundation should be launchable for development')
const colecoBios = new Uint8Array(0x2000)
colecoBios.set([0xc3, 0x00, 0x80], 0)
colecoBios.set([0xed, 0x45], 0x66)
const coleco = new Uint8Array(0x2000)
const colecoProgram = [
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
coleco.set(colecoProgram)
const colecoTail = [
  0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x40, 0xd3, 0xbf,
  0x21, 0x00, 0x81, 0x06, 0x08, 0x7e, 0xd3, 0xbe, 0x23, 0x10, 0xfa,
  0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x60, 0xd3, 0xbf,
  0x21, 0x10, 0x81, 0x06, 0x08, 0x7e, 0xd3, 0xbe, 0x23, 0x10, 0xfa,
  0x3e, 0x00, 0xd3, 0xbf, 0x3e, 0x58, 0xd3, 0xbf, 0xaf, 0xd3, 0xbe,
  0x3e, 0x84, 0xd3, 0xff, 0x3e, 0x10, 0xd3, 0xff, 0x3e, 0x90, 0xd3, 0xff,
]
const colecoTailStart = colecoProgram.length
coleco.set(colecoTail, colecoTailStart)
const colecoLoop = 0x8000 + colecoTailStart + colecoTail.length
coleco.set([0xc3, colecoLoop & 0xff, colecoLoop >> 8], colecoTailStart + colecoTail.length)
coleco.fill(0xff, 0x100, 0x108)
coleco.fill(0xf1, 0x110, 0x118)
core.omni_resources_clear()
const stageBytes = (kind, data, label) => {
  ok(core.omni_resource_create(kind, 0, BigInt(data.byteLength)), `create staged ${label}`)
  const ptr = core.omni_alloc(data.byteLength)
  assert(ptr > 0, `${label} staging allocation failed`)
  new Uint8Array(core.memory.buffer, ptr, data.byteLength).set(data)
  ok(core.omni_resource_write(kind, 0, 0n, ptr, data.byteLength), `stage ${label}`)
  core.omni_free(ptr, data.byteLength)
}
stageBytes(0, coleco, 'ColecoVision cartridge')
stageBytes(1, colecoBios, 'ColecoVision BIOS')
ok(core.omni_load_staged(12), 'load staged ColecoVision cartridge')
for (let frame = 0; frame < 2; frame++) ok(core.omni_run_frame(), `run ColecoVision frame ${frame}`)
const colecoWidth = core.omni_video_width()
const colecoHeight = core.omni_video_height()
assert(colecoWidth === 256 && colecoHeight === 192, `unexpected ColecoVision surface ${colecoWidth}x${colecoHeight}`)
const colecoVideo = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
assert(colecoVideo.some((value) => value > 180), 'ColecoVision framebuffer did not render pattern output')
assert(core.omni_audio_rate() === 48_000, 'unexpected ColecoVision audio sample rate')
assert(core.omni_audio_channels() === 2, 'unexpected ColecoVision audio channel count')
core.omni_unload()

const intvExec = new Uint8Array(0x2000)
const intvExecView = new DataView(intvExec.buffer)
;[0x0004, 0x0050, 0x0000].forEach((word, index) => intvExecView.setUint16(index * 2, word, false))
const intvGrom = new Uint8Array(0x0800)
for (let row = 0; row < 8; row++) intvGrom[row] = row === 0 || row === 7 ? 0xff : 0x81
const intvWords = [
  0x02b8, 0x0001, 0x0240, 0x0020,
  0x02b8, 0x0007, 0x0240, 0x002c,
  0x02b8, 0x0007, 0x0240, 0x0200,
  0x0000,
]
const intv = new Uint8Array(intvWords.length * 2)
const intvView = new DataView(intv.buffer)
intvWords.forEach((word, index) => intvView.setUint16(index * 2, word, false))
core.omni_resources_clear()
stageBytes(0, intv, 'Intellivision cartridge')
stageBytes(1, intvExec, 'Intellivision EXEC')
stageBytes(2, intvGrom, 'Intellivision GROM')
ok(core.omni_load_staged(13), 'load staged Intellivision cartridge')
ok(core.omni_run_frame(), 'run Intellivision frame')
assert(core.omni_video_width() === 320 && core.omni_video_height() === 192, 'unexpected Intellivision surface')
assert(core.omni_video_len() === 320 * 192 * 4, 'invalid Intellivision framebuffer')
assert(core.omni_audio_rate() === 48_000 && core.omni_audio_channels() === 2, 'unexpected Intellivision audio surface')
assert(core.omni_audio_len() > 0, 'Intellivision audio buffer is empty')
assert(core.omni_save_state(0, 0) > 0, 'Intellivision save state is empty')
core.omni_unload()

const scdBios = new Uint8Array(128 * 1024)
scdBios.fill(0xff)
const scdBiosView = new DataView(scdBios.buffer)
scdBiosView.setUint32(0, 0x00ffff00, false)
scdBiosView.setUint32(4, 0x00000200, false)
const scdWords = [
  0x13fc,0x0084,0x00c0,0x0011,
  0x13fc,0x0010,0x00c0,0x0011,
  0x13fc,0x0090,0x00c0,0x0011,
  0x60fe,
]
scdWords.forEach((word,index) => scdBiosView.setUint16(0x200 + index * 2, word, false))
const scdDisc = new Uint8Array(2048 * 4)
for (let sector = 0; sector < 4; sector++) scdDisc.fill(sector + 1, sector * 2048, (sector + 1) * 2048)
core.omni_resources_clear()
stageBytes(1, scdBios, 'Sega CD BIOS')
stageBytes(4, scdDisc, 'Sega CD ISO')
ok(core.omni_load_staged(32), 'load staged Sega CD')
ok(core.omni_run_frame(), 'run Sega CD frame')
assert(core.omni_video_width() === 320 && core.omni_video_height() === 224, 'unexpected Sega CD surface')
const scdAudio = new Float32Array(core.memory.buffer, core.omni_audio_ptr(), core.omni_audio_len())
assert(scdAudio.some((value) => Math.abs(value) > 0.001), 'Sega CD base audio output is silent')
assert(core.omni_persistent_len(5, 0) === 0x2000, 'unexpected Sega CD backup RAM length')
assert(core.omni_save_state(0, 0) > 0, 'Sega CD save state is empty')
core.omni_unload()

const pce = new Uint8Array(0x2000)
pce.fill(0xea)
const pceCode = [
  0xd4,
  0xa9,0xf8,0x53,0x02,
  0xa9,0xff,0x53,0x40,
  0xa9,0x01,0x8d,0x02,0xc4, 0xa9,0x00,0x8d,0x03,0xc4,
  0xa9,0x38,0x8d,0x04,0xc4, 0xa9,0x00,0x8d,0x05,0xc4,
  0x03,0x00,0x13,0x00,0x23,0x00,
  0x03,0x02,0x13,0x01,0x23,0x00,
  0x03,0x00,0x13,0x10,0x23,0x00,
  0x03,0x02,
]
for (let row = 0; row < 8; row++) pceCode.push(0x13,0xff,0x23,0x00)
for (let row = 0; row < 8; row++) pceCode.push(0x13,0x00,0x23,0x00)
pceCode.push(
  0x03,0x05,0x13,0x80,0x23,0x00,
  0xa9,0x00,0x8d,0x00,0xc8,
  0xa9,0xff,0x8d,0x01,0xc8,
  0xa9,0x20,0x8d,0x02,0xc8,
  0xa9,0x00,0x8d,0x03,0xc8,
  0xa9,0xff,0x8d,0x05,0xc8,
)
for (let index = 0; index < 32; index++) pceCode.push(0xa9, index & 1 ? 31 : 0, 0x8d,0x06,0xc8)
pceCode.push(0xa9,0x9f,0x8d,0x04,0xc8,0x80,0xfe)
pce.set(pceCode)
pce[0x1ffe] = 0x00
pce[0x1fff] = 0xe0
core.omni_resources_clear()
stageBytes(0, pce, 'PC Engine HuCard')
ok(core.omni_load_staged(34), 'load staged PC Engine HuCard')
ok(core.omni_run_frame(), 'run PC Engine frame')
assert(core.omni_video_width() === 256 && core.omni_video_height() === 240, 'unexpected PC Engine surface')
const pceVideo = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
let pceHasRed = false
for (let i = 0; i < pceVideo.length; i += 4) {
  if (pceVideo[i] > pceVideo[i + 1] && pceVideo[i] > pceVideo[i + 2]) { pceHasRed = true; break }
}
assert(pceHasRed, 'PC Engine VDC/VCE output did not render expected palette color')
const pceAudio = new Float32Array(core.memory.buffer, core.omni_audio_ptr(), core.omni_audio_len())
assert(pceAudio.some((value) => Math.abs(value) > 0.001), 'PC Engine PSG output is silent')
assert(core.omni_save_state(0, 0) > 0, 'PC Engine save state is empty')
core.omni_unload()

core.omni_resources_clear()
stageBytes(0, pce, 'SuperGrafx HuCard')
ok(core.omni_load_staged(36), 'load staged SuperGrafx HuCard')
ok(core.omni_run_frame(), 'run SuperGrafx frame')
assert(core.omni_video_width() === 256 && core.omni_video_height() === 240, 'unexpected SuperGrafx surface')
const sgxVideo = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
assert(sgxVideo.some((value) => value !== 0), 'SuperGrafx dual-VDC output is blank')
assert(core.omni_save_state(0, 0) > 0, 'SuperGrafx save state is empty')
core.omni_unload()

const neoP = new Uint8Array(0x10000)
neoP.fill(0xff)
const neoS = new Uint8Array(0x10000)
neoS.fill(0xff, 32, 64)
const neoM = new Uint8Array(0x10000)
neoM.set([
  0x3e,0x00,0xd3,0x04, 0x3e,0x20,0xd3,0x05,
  0x3e,0x01,0xd3,0x04, 0x3e,0x00,0xd3,0x05,
  0x3e,0x07,0xd3,0x04, 0x3e,0x3e,0xd3,0x05,
  0x3e,0x08,0xd3,0x04, 0x3e,0x0f,0xd3,0x05, 0x76,
])
neoM.set([0xdb,0x00,0xd3,0x0c,0xed,0x45], 0x66)
const neoV1 = new Uint8Array(0x10000)
const neoC = new Uint8Array(0x40000)
const neoSections = [neoP, neoS, neoM, neoV1, new Uint8Array(0), neoC]
const neoLength = 4096 + neoSections.reduce((sum, section) => sum + section.byteLength, 0)
const neo = new Uint8Array(neoLength)
neo.set([0x4e,0x45,0x4f,0x01])
const neoHeader = new DataView(neo.buffer)
neoSections.forEach((section, index) => neoHeader.setUint32(4 + index * 4, section.byteLength, true))
neoHeader.setUint32(40, 0x0123, true)
let neoOffset = 4096
for (const section of neoSections) { neo.set(section, neoOffset); neoOffset += section.byteLength }

const neoBios = new Uint8Array(128 * 1024)
neoBios.fill(0x4e)
const neoBiosView = new DataView(neoBios.buffer)
neoBiosView.setUint32(0, 0x0010ff00, false)
neoBiosView.setUint32(4, 0x00c00100, false)
for (let vector = 24; vector <= 27; vector++) neoBiosView.setUint32(vector * 4, 0x00c00200, false)
const neoWords = [
  0x33fc,0x0f00,0x0040,0x003e,
  0x33fc,0x7002,0x003c,0x0000,
  0x33fc,0x1001,0x003c,0x0002,
  0x60fe,
]
neoWords.forEach((word, index) => neoBiosView.setUint16(0x100 + index * 2, word, false))
neoBiosView.setUint16(0x200, 0x4e73, false)
core.omni_resources_clear()
stageBytes(0, neo, 'Neo Geo NEO1 cartridge')
stageBytes(1, neoBios, 'Neo Geo BIOS')
ok(core.omni_load_staged(35), 'load staged Neo Geo cartridge')
ok(core.omni_run_frame(), 'run Neo Geo frame')
assert(core.omni_video_width() === 320 && core.omni_video_height() === 224, 'unexpected Neo Geo surface')
const neoVideo = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
let neoHasRed = false
for (let i = 0; i < neoVideo.length; i += 4) {
  if (neoVideo[i] > neoVideo[i + 1] && neoVideo[i] > neoVideo[i + 2]) { neoHasRed = true; break }
}
assert(neoHasRed, 'Neo Geo FIX/palette output did not render expected color')
const neoAudio = new Float32Array(core.memory.buffer, core.omni_audio_ptr(), core.omni_audio_len())
assert(neoAudio.some((value) => Math.abs(value) > 0.0001), 'Neo Geo YM2610 SSG output is silent')
assert(core.omni_persistent_len(5, 0) === 0x10000, 'unexpected Neo Geo backup RAM length')
assert(core.omni_persistent_len(6, 0) === 0x800, 'unexpected Neo Geo memory-card length')
assert(core.omni_save_state(0, 0) > 0, 'Neo Geo save state is empty')
core.omni_unload()

assert(core.omni_can_launch(10) === 1, 'Atari 2600 foundation should be launchable for development')
const atari = new Uint8Array(0x1000)
atari.fill(0xea)
const atariProgram = [
  0x78, 0xd8, 0xa2, 0xff, 0x9a,
  0xa9, 0x00, 0x85, 0x01,
  0xa9, 0x2e, 0x85, 0x09,
  0xa9, 0x4e, 0x85, 0x08,
  0xa9, 0xf0, 0x85, 0x0d,
  0xa9, 0xff, 0x85, 0x0e, 0x85, 0x0f,
  0xa9, 0x04, 0x85, 0x15,
  0xa9, 0x08, 0x85, 0x17,
  0xa9, 0x0f, 0x85, 0x19,
  0xa9, 0x00, 0x85, 0x02,
  0x4c, 0x27, 0xf0,
]
atari.set(atariProgram)
atari.set([0x00, 0xf0, 0x00, 0xf0, 0x00, 0xf0], 0x0ffa)
core.omni_resources_clear()
stageBytes(0, atari, 'Atari 2600 cartridge')
ok(core.omni_load_staged(10), 'load staged Atari 2600 cartridge')
for (let frame = 0; frame < 2; frame++) ok(core.omni_run_frame(), `run Atari 2600 frame ${frame}`)
const atariWidth = core.omni_video_width()
const atariHeight = core.omni_video_height()
assert(atariWidth === 160 && atariHeight === 192, `unexpected Atari 2600 surface ${atariWidth}x${atariHeight}`)
const atariVideo = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
assert(atariVideo.some((value) => value !== 0), 'Atari 2600 framebuffer is empty')
assert(core.omni_audio_rate() === 48_000, 'unexpected Atari 2600 audio sample rate')
assert(core.omni_audio_channels() === 2, 'unexpected Atari 2600 audio channel count')
const atariStateSize = core.omni_save_state(0, 0)
assert(atariStateSize > 0, 'Atari 2600 save state is empty')
core.omni_unload()

assert(core.omni_can_launch(11) === 1, 'Atari 5200 foundation should be launchable for development')
const atari5200Bios = new Uint8Array(0x800)
atari5200Bios.fill(0xea)
const atari5200Boot = [
  0x78, 0xd8, 0xa9, 0x3a, 0x8d, 0x18, 0xc0,
  0xa9, 0x84, 0x8d, 0x1a, 0xc0,
  0xa9, 0x20, 0x8d, 0x00, 0xd4,
  0xa9, 0x00, 0x8d, 0x02, 0xd4,
  0xa9, 0x20, 0x8d, 0x03, 0xd4,
  0xa9, 0x40, 0x8d, 0x0e, 0xd4,
  0x4c, 0x00, 0x40,
]
atari5200Bios.set(atari5200Boot)
atari5200Bios.set([0x00, 0xf8, 0x00, 0xf8, 0x00, 0xf8], 0x7fa)
const atari5200 = new Uint8Array(0x8000)
atari5200.fill(0xea)
atari5200.set([
  0xa9, 0x08, 0x8d, 0x00, 0xe8,
  0xa9, 0xaf, 0x8d, 0x01, 0xe8,
  0x4c, 0x0a, 0x40,
])
core.omni_resources_clear()
stageBytes(0, atari5200, 'Atari 5200 cartridge')
stageBytes(1, atari5200Bios, 'Atari 5200 BIOS')
ok(core.omni_load_staged(11), 'load staged Atari 5200 cartridge')
for (let frame = 0; frame < 2; frame++) ok(core.omni_run_frame(), `run Atari 5200 frame ${frame}`)
const atari5200Width = core.omni_video_width()
const atari5200Height = core.omni_video_height()
assert(
  atari5200Width === 320 && atari5200Height === 192,
  `unexpected Atari 5200 surface ${atari5200Width}x${atari5200Height}`,
)
const atari5200Video = new Uint8Array(
  core.memory.buffer,
  core.omni_video_ptr(),
  core.omni_video_len(),
)
assert(atari5200Video.some((value) => value !== 0), 'Atari 5200 framebuffer is empty')
assert(core.omni_audio_rate() === 48_000, 'unexpected Atari 5200 audio sample rate')
assert(core.omni_audio_channels() === 2, 'unexpected Atari 5200 audio channel count')
const atari5200Audio = new Float32Array(
  core.memory.buffer,
  core.omni_audio_ptr(),
  core.omni_audio_len(),
)
assert(atari5200Audio.some((value) => value > 0), 'Atari 5200 POKEY audio is silent')
assert(core.omni_save_state(0, 0) > 0, 'Atari 5200 save state is empty')
core.omni_unload()

assert(core.omni_can_launch(22) === 1, 'Atari 7800 foundation should be launchable for development')
const atari7800 = new Uint8Array(0xc000)
atari7800.fill(0xea)
const atari7800Program = [
  0x78, 0xd8,
  0xa9, 0x22, 0x85, 0x20,
  0xa9, 0x2e, 0x85, 0x21,
  0xa9, 0x4e, 0x85, 0x22,
  0xa9, 0x6e, 0x85, 0x23,
  0xa9, 0x00, 0x85, 0x30,
  0xa9, 0x18, 0x85, 0x2c,
  0xa9, 0x40, 0x85, 0x3c,
  0xa9, 0x04, 0x85, 0x15,
  0xa9, 0x08, 0x85, 0x17,
  0xa9, 0x0f, 0x85, 0x19,
  0x4c, 0x2a, 0x40,
]
atari7800.set(atari7800Program)
atari7800.set([0x00, 0x40, 0x00, 0x40, 0x00, 0x40], 0xbffa)
core.omni_resources_clear()
stageBytes(0, atari7800, 'Atari 7800 cartridge')
ok(core.omni_load_staged(22), 'load staged Atari 7800 cartridge')
for (let frame = 0; frame < 2; frame++) ok(core.omni_run_frame(), `run Atari 7800 frame ${frame}`)
const atari7800Width = core.omni_video_width()
const atari7800Height = core.omni_video_height()
assert(
  atari7800Width === 320 && atari7800Height === 240,
  `unexpected Atari 7800 surface ${atari7800Width}x${atari7800Height}`,
)
const atari7800Video = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
assert(
  atari7800Video.some((value, index) => index % 4 !== 3 && value !== 0),
  'Atari 7800 framebuffer has no RGB output',
)
const atari7800Audio = new Float32Array(
  core.memory.buffer,
  core.omni_audio_ptr(),
  core.omni_audio_len(),
)
assert(atari7800Audio.some((value) => value > 0), 'Atari 7800 TIA audio is silent')
assert(core.omni_save_state(0, 0) > 0, 'Atari 7800 save state is empty')
core.omni_unload()

assert(core.omni_can_launch(31) === 1, 'Genesis foundation should be launchable for development')
const genesis = new Uint8Array(0x40000)
genesis.fill(0xff)
const g16 = (offset, value) => { genesis[offset] = value >> 8; genesis[offset + 1] = value & 0xff }
const g32 = (offset, value) => { g16(offset, value >>> 16); g16(offset + 2, value & 0xffff) }
g32(0, 0x00ffff00)
g32(4, 0x00000200)
const genesisWords = [
  0x13fc, 0x0084, 0x00c0, 0x0011,
  0x13fc, 0x0010, 0x00c0, 0x0011,
  0x13fc, 0x0090, 0x00c0, 0x0011,
  0x60fe,
]
genesisWords.forEach((word, index) => g16(0x200 + index * 2, word))
core.omni_resources_clear()
stageBytes(0, genesis, 'Genesis cartridge')
ok(core.omni_load_staged(31), 'load staged Genesis cartridge')
for (let frame = 0; frame < 2; frame++) ok(core.omni_run_frame(), `run Genesis frame ${frame}`)
const genesisWidth = core.omni_video_width()
const genesisHeight = core.omni_video_height()
assert(genesisWidth === 320 && genesisHeight === 224, `unexpected Genesis surface ${genesisWidth}x${genesisHeight}`)
const genesisAudio = new Float32Array(core.memory.buffer, core.omni_audio_ptr(), core.omni_audio_len())
assert(genesisAudio.some((value) => Math.abs(value) > 0.001), 'Genesis PSG audio is silent')
assert(core.omni_save_state(0, 0) > 0, 'Genesis save state is empty')
core.omni_unload()

const sega32x = new Uint8Array(0x20000)
sega32x.fill(0xff)
const x16 = (offset, value) => { sega32x[offset] = (value >>> 8) & 0xff; sega32x[offset + 1] = value & 0xff }
const x32 = (offset, value) => { x16(offset, value >>> 16); x16(offset + 2, value & 0xffff) }
x32(0, 0x00ffff00)
x32(4, 0x00000400)
sega32x.set(new TextEncoder().encode('SEGA 32X'), 0x100)
sega32x.set(new TextEncoder().encode('OMNICORE '), 0x3c0)
x32(0x3d0, 1)
x32(0x3d4, 0x00001000)
x32(0x3d8, 0x00000000)
x32(0x3dc, 0x00000400)
x32(0x3e0, 0x06000000)
x32(0x3e4, 0x06000020)
x32(0x3e8, 0x06000200)
x32(0x3ec, 0x06000240)
const sega32xMain = [
  0x33fc,0x0003,0x00a1,0x5100,
  0x33fc,0x0000,0x00a1,0x5120,
  0x33fc,0x0000,0x00a1,0x5122,
  0x33fc,0x0000,0x00a1,0x5124,
  0x33fc,0x0000,0x00a1,0x5126,
  0x33fc,0x001f,0x00a1,0x5202,
  0x33fc,0x0081,0x00a1,0x5180,
  0x33fc,0x0001,0x00a1,0x518a,
  0x33fc,0x0100,0x0084,0x0000,
  0x13fc,0x0001,0x0084,0x0200,
  0x60fe,
]
sega32xMain.forEach((word, index) => x16(0x400 + index * 2, word))
const shMaster = [0xd103,0xe05a,0x2102,0xaffe,0x0009]
shMaster.forEach((word, index) => x16(0x1000 + index * 2, word))
x32(0x1010, 0x06000100)
const shSlave = [0xd103,0xe066,0x2102,0xaffe,0x0009]
shSlave.forEach((word, index) => x16(0x1020 + index * 2, word))
x32(0x1030, 0x06000104)
core.omni_resources_clear()
stageBytes(0, sega32x, 'Sega 32X cartridge')
ok(core.omni_load_staged(33), 'load staged Sega 32X cartridge')
ok(core.omni_run_frame(), 'run Sega 32X frame')
assert(core.omni_video_width() === 320 && core.omni_video_height() === 224, 'unexpected Sega 32X surface')
const sega32xVideo = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
assert(sega32xVideo[0] > sega32xVideo[1] && sega32xVideo[0] > sega32xVideo[2], 'Sega 32X framebuffer did not overlay the Genesis surface')
assert(core.omni_audio_rate() === 48_000 && core.omni_audio_channels() === 2, 'unexpected Sega 32X audio surface')
assert(core.omni_save_state(0, 0) > 0, 'Sega 32X save state is empty')
core.omni_unload()

const snes = new Uint8Array(0x8000)
snes.fill(0xea)
const snesProgram = [0x78,0xa9,0x0f,0x8d,0x00,0x21,0xa9,0x00,0x8d,0x21,0x21,0xa9,0x1f,0x8d,0x22,0x21,0xa9,0x00,0x8d,0x22,0x21,0x80,0xfe]
snes.set(snesProgram)
snes[0x7fd5] = 0x20
snes[0x7fd8] = 0x01
snes.set([0xff,0xff,0x00,0x00], 0x7fdc)
snes.set([0x00,0x80,0x00,0x80], 0x7ffc)
core.omni_resources_clear()
stageBytes(0, snes, 'SNES cartridge')
ok(core.omni_load_staged(30), 'load staged SNES cartridge')
for (let frame = 0; frame < 2; frame++) ok(core.omni_run_frame(), 'run SNES frame ' + frame)
assert(core.omni_video_width() === 256 && core.omni_video_height() === 224, 'unexpected SNES surface')
const snesVideo = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
assert(snesVideo[0] > 180, 'SNES PPU backdrop did not render')
assert(core.omni_audio_rate() === 32_000, 'unexpected SNES audio sample rate')
assert(core.omni_audio_channels() === 2, 'unexpected SNES audio channel count')
assert(core.omni_audio_len() > 0, 'SNES audio buffer is empty')
assert(core.omni_save_state(0, 0) > 0, 'SNES save state is empty')
core.omni_unload()

assert(core.omni_can_launch(40) === 1, 'PlayStation foundation should be launchable for development')
const ps1Bios = new Uint8Array(512 * 1024)
const ps1Words = [0x3c081f80,0x35081814,0x3c090800,0x35290001,0xad090000,0x3c090300,0xad090000,0x2508fffc,0x3c090200,0x352900ff,0xad090000,0x24090000,0xad090000,0x3c0900f0,0x35290140,0xad090000,0x0bf00010,0x00000000]
const ps1View = new DataView(ps1Bios.buffer)
ps1Words.forEach((word,index)=>ps1View.setUint32(index*4,word,true))
core.omni_resources_clear()
stageBytes(1, ps1Bios, 'PlayStation BIOS')
ok(core.omni_load_staged(40), 'load staged PlayStation BIOS')
ok(core.omni_run_frame(), 'run PlayStation frame')
assert(core.omni_video_width() === 320 && core.omni_video_height() === 240, 'unexpected PlayStation surface')
const ps1Video = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
assert(ps1Video[0] > 180, 'PlayStation GPU framebuffer did not render')
assert(core.omni_audio_rate() === 44_100 && core.omni_audio_channels() === 2, 'unexpected PlayStation audio surface')
assert(core.omni_save_state(0, 0) > 0, 'PlayStation save state is empty')
core.omni_unload()

const n64 = new Uint8Array(0x4000)
const n64View = new DataView(n64.buffer)
n64View.setUint32(0, 0x80371240, false)
const n64Program = [
  0x3c08a440,0x34090002,0xad090000,
  0x34091000,0xad090004,0x34090004,0xad090008,0x34090008,0xad090028,
  0x3c0a8000,0x354a1000,0x3409f801,0xa5490000,
  0x3c0a8000,0x354a2000,0x3c094000,0x3529c000,0xad490000,
  0x3c092000,0x3529e000,0xad490004,
  0x3c08a450,0x3409044f,0xad090010,0x34092000,0xad090000,0x34090008,0xad090004,
  0x34090001,0xad090008,
  0x1000ffff,0x00000000,
]
n64Program.forEach((word, index) => n64View.setUint32(0x40 + index * 4, word, false))
core.omni_resources_clear()
stageBytes(0, n64, 'Nintendo 64 cartridge')
ok(core.omni_load_staged(41), 'load staged Nintendo 64 cartridge')
ok(core.omni_run_frame(), 'run Nintendo 64 frame')
assert(core.omni_video_width() === 640 && core.omni_video_height() === 480, 'unexpected Nintendo 64 surface')
const n64Video = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
assert(n64Video[0] > 200 && n64Video[1] < 20, 'Nintendo 64 VI framebuffer did not render')
const n64Audio = new Float32Array(core.memory.buffer, core.omni_audio_ptr(), core.omni_audio_len())
assert(n64Audio.some((value) => Math.abs(value) > 0.2), 'Nintendo 64 AI audio output is silent')
assert(core.omni_persistent_len(5, 0) === 128 * 1024, 'unexpected Nintendo 64 save-memory length')
assert(core.omni_persistent_len(6, 0) === 32 * 1024, 'unexpected Nintendo 64 Controller Pak length')
assert(core.omni_save_state(0, 0) > 0, 'Nintendo 64 save state is empty')
core.omni_unload()

const saturnBios = new Uint8Array(512 * 1024)
const saturnView = new DataView(saturnBios.buffer)
saturnView.setUint32(0, 0x00000100, false)
saturnView.setUint32(4, 0x060ffff0, false)
saturnView.setUint16(0x100, 0x0009, false)
saturnView.setUint16(0x102, 0xaffe, false)
saturnView.setUint16(0x104, 0x0009, false)
core.omni_resources_clear()
stageBytes(1, saturnBios, 'Saturn BIOS')
ok(core.omni_load_staged(42), 'load staged Saturn BIOS')
ok(core.omni_run_frame(), 'run Saturn frame')
assert(core.omni_video_width() === 320 && core.omni_video_height() === 224, 'unexpected Saturn surface')
assert(core.omni_video_len() === 320 * 224 * 4, 'invalid Saturn framebuffer')
assert(core.omni_audio_rate() === 44_100 && core.omni_audio_channels() === 2, 'unexpected Saturn audio surface')
assert(core.omni_audio_len() > 0, 'Saturn audio buffer is empty')
assert(core.omni_persistent_len(5, 0) === 32 * 1024, 'unexpected Saturn backup RAM length')
assert(core.omni_save_state(0, 0) > 0, 'Saturn save state is empty')
core.omni_unload()

const jaguarCart = new Uint8Array(0x20000)
jaguarCart.fill(0xff)
const jaguarBios = new Uint8Array(128 * 1024)
jaguarBios.fill(0xff)
const jaguarView = new DataView(jaguarBios.buffer)
jaguarView.setUint32(0, 0x001fff00, false)
jaguarView.setUint32(4, 0x00e00100, false)
const jaguarWords = [
  0x33fc,0x0007,0x00f0,0x0028,
  0x33fc,0xf800,0x00f0,0x0058,
  0x33fc,0x2000,0x00f1,0xa14c,
  0x33fc,0xe000,0x00f1,0xa148,
  0x60fe,
]
jaguarWords.forEach((word, index) => jaguarView.setUint16(0x100 + index * 2, word, false))
core.omni_resources_clear()
stageBytes(0, jaguarCart, 'Jaguar cartridge')
stageBytes(1, jaguarBios, 'Jaguar BIOS')
ok(core.omni_load_staged(43), 'load staged Jaguar cartridge')
ok(core.omni_run_frame(), 'run Jaguar frame')
assert(core.omni_video_width() === 320 && core.omni_video_height() === 240, 'unexpected Jaguar surface')
const jaguarVideo = new Uint8Array(core.memory.buffer, core.omni_video_ptr(), core.omni_video_len())
assert(jaguarVideo[0] > jaguarVideo[1] && jaguarVideo[0] > jaguarVideo[2], 'Jaguar TOM background did not render')
const jaguarAudio = new Float32Array(core.memory.buffer, core.omni_audio_ptr(), core.omni_audio_len())
assert(jaguarAudio.some((value) => Math.abs(value) > 0.1), 'Jaguar Jerry DAC output is silent')
assert(core.omni_persistent_len(5, 0) === 128, 'unexpected Jaguar EEPROM length')
assert(core.omni_save_state(0, 0) > 0, 'Jaguar save state is empty')
core.omni_unload()

const threeDoBios = new Uint8Array(1024 * 1024)
new DataView(threeDoBios.buffer).setUint32(0, 0xeafffffe, true)
core.omni_resources_clear()
stageBytes(1, threeDoBios, '3DO BIOS')
ok(core.omni_load_staged(44), 'load staged 3DO BIOS')
ok(core.omni_run_frame(), 'run 3DO frame')
assert(core.omni_video_width() === 640 && core.omni_video_height() === 480, 'unexpected 3DO surface')
assert(core.omni_video_len() === 640 * 480 * 4, 'invalid 3DO framebuffer')
assert(core.omni_audio_rate() === 44_100 && core.omni_audio_channels() === 2, 'unexpected 3DO audio surface')
assert(core.omni_audio_len() > 0, '3DO audio buffer is empty')
assert(core.omni_persistent_len(5, 0) === 32 * 1024, 'unexpected 3DO NVRAM length')
assert(core.omni_save_state(0, 0) > 0, '3DO save state is empty')
core.omni_unload()

assert(core.omni_can_launch(50) === 1, 'Dreamcast foundation should be launchable for development')
const dreamcastBios = new Uint8Array(2 * 1024 * 1024)
const dreamcastBiosView = new DataView(dreamcastBios.buffer)
dreamcastBiosView.setUint16(0, 0xaffe, true)
dreamcastBiosView.setUint16(2, 0x0009, true)
const dreamcastDisc = new Uint8Array(4 * 2048)
for (let sector = 0; sector < 4; sector++) dreamcastDisc.fill(sector + 1, sector * 2048, (sector + 1) * 2048)
core.omni_resources_clear()
stageBytes(1, dreamcastBios, 'Dreamcast BIOS')
stageBytes(4, dreamcastDisc, 'Dreamcast disc')
ok(core.omni_load_staged(50), 'load staged Dreamcast')
ok(core.omni_run_frame(), 'run Dreamcast frame')
assert(core.omni_video_width() === 640 && core.omni_video_height() === 480, 'unexpected Dreamcast surface')
assert(core.omni_video_len() === 640 * 480 * 4, 'invalid Dreamcast framebuffer')
assert(core.omni_audio_rate() === 44_100 && core.omni_audio_channels() === 2, 'unexpected Dreamcast audio surface')
assert(core.omni_audio_len() > 0, 'Dreamcast audio buffer is empty')
assert(core.omni_persistent_len(5, 0) === 128 * 1024, 'unexpected Dreamcast flash length')
assert(core.omni_save_state(0, 0) > 0, 'Dreamcast save state is empty')
core.omni_unload()

const runFoundationImage = (platform, label, image, expectedWidth, expectedHeight) => {
  core.omni_resources_clear()
  stageBytes(0, image, label)
  ok(core.omni_load_staged(platform), `load staged ${label}`)
  ok(core.omni_run_frame(), `run ${label} frame`)
  assert(core.omni_video_width() === expectedWidth, `${label} width mismatch`)
  assert(core.omni_video_height() === expectedHeight, `${label} height mismatch`)
  assert(core.omni_audio_rate() === 48_000, `${label} audio rate mismatch`)
  assert(core.omni_audio_channels() === 2, `${label} audio channels mismatch`)
  assert(core.omni_save_state(0, 0) > 0, `${label} save state is empty`)
  core.omni_unload()
}

const activeTargets = [
  1, 2, 10, 11, 12, 13, 20, 21, 22,
  30, 31, 32, 33, 34, 35, 36,
  40, 41, 42, 43, 44,
  50, 51, 52, 53, 60, 61, 62, 70, 71,
]
assert(activeTargets.length === core.omni_target_count(), 'active target table length mismatch')
for (const platform of activeTargets) {
  assert(core.omni_can_launch(platform) === 1, `active platform ${platform} is not launchable`)
  assert(core.omni_support_level(platform) >= 1, `active platform ${platform} has no implementation`)
}

runFoundationImage(51, 'PlayStation 2 ELF', ps2Elf([0x1000ffff, 0]), 640, 448)
runFoundationImage(52, 'GameCube DOL', ppcDol([0x48000000]), 640, 480)
runFoundationImage(62, 'Wii DOL', ppcDol([0x48000000]), 640, 480)
runFoundationImage(53, 'Xbox XBE', xboxXbe(Uint8Array.from([0xeb, 0xfe])), 640, 480)
runFoundationImage(60, 'Xbox 360 PowerPC64 ELF', ppc64Elf([0x48000000]), 1280, 720)
runFoundationImage(61, 'PlayStation 3 PowerPC64 ELF', ppc64Elf([0x48000000]), 1280, 720)
runFoundationImage(70, 'Wii U PowerPC ELF', ppc32Elf([0x48000000]), 1280, 720)
runFoundationImage(71, 'Switch NRO', switchNro([0x14000000]), 1280, 720)

console.log(`OmniCore WASM smoke passed: ${bytes.byteLength} bytes, ${width}x${height}, ${audioLen} audio samples; all ${activeTargets.length} target graphs are launchable.`)
