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
assert(core.omni_can_launch(30) === 0, 'unimplemented SNES should not be launchable')

const nes = new Uint8Array(16 + 16 * 1024 + 8 * 1024)
nes.set([0x4e, 0x45, 0x53, 0x1a, 1, 1], 0)
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
const nesPtr = core.omni_alloc(nes.byteLength)
assert(nesPtr > 0, 'NES ROM allocation failed')
new Uint8Array(core.memory.buffer, nesPtr, nes.byteLength).set(nes)
ok(core.omni_load(20, nesPtr, nes.byteLength, 0, 0), 'load NES NROM')
core.omni_free(nesPtr, nes.byteLength)
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

console.log(`OmniCore WASM smoke passed: ${bytes.byteLength} bytes, ${width}x${height}, ${audioLen} audio samples.`)
