import { readFileSync } from 'node:fs'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

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

assert(core.omni_can_launch(2) === 1, 'Pong should be launchable')
assert(core.omni_can_launch(20) === 1, 'NES foundation should be launchable for development')
assert(core.omni_can_launch(30) === 1, 'SNES foundation should be launchable for development')
assert(core.omni_is_targeted(30) === 1, 'SNES should remain an OmniCore target')
assert(core.omni_is_targeted(71) === 1, 'Switch should remain an OmniCore target')
assert(core.omni_is_targeted(72) === 0, 'PS4 should be outside the current target set')
assert(core.omni_is_targeted(73) === 0, 'Xbox One should be outside the current target set')
assert(core.omni_target_count() === 29, 'unexpected active target count')
const targetIds = Array.from({ length: core.omni_target_count() }, (_, index) => core.omni_target_at(index))
assert(targetIds.includes(71), 'Switch target is missing')
assert(!targetIds.includes(72) && !targetIds.includes(73), 'excluded generation-8 targets leaked into active list')

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
assert(core.omni_save_state(0, 0) > 0, 'SNES save state is empty')
core.omni_unload()

console.log(`OmniCore WASM smoke passed: ${bytes.byteLength} bytes, ${width}x${height}, ${audioLen} audio samples.`)
