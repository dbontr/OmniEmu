import { mediaForFile, type Platform } from '../catalog'
import type { InputProfile } from '../input/mapping'
import { AudioScheduler } from './audio'
import { fingerprintFile, loadPersistentResource, savePersistentResource } from './persistence'
import { createVideoRenderer } from './video'

interface OmniExports extends WebAssembly.Exports {
  memory: WebAssembly.Memory
  omni_core_version(): number
  omni_support_level(platform: number): number
  omni_can_launch(platform: number): number
  omni_resources_clear(): void
  omni_resource_create(kind: number, slot: number, len: bigint): number
  omni_resource_create_streaming(kind: number, slot: number, len: bigint, maxChunks: number): number
  omni_resource_pending_start(kind: number, slot: number): bigint
  omni_resource_pending_end(kind: number, slot: number): bigint
  omni_resource_write(kind: number, slot: number, offset: bigint, ptr: number, len: number): number
  omni_load_staged(platform: number): number
  omni_alloc(len: number): number
  omni_free(ptr: number, len: number): void
  omni_load(platform: number, romPtr: number, romLen: number, biosPtr: number, biosLen: number): number
  omni_unload(): void
  omni_reset(): number
  omni_set_input(player: number, buttons: bigint): number
  omni_set_axis(player: number, axis: number, value: number): number
  omni_run_frame(): number
  omni_frame_rate(): number
  omni_video_ptr(): number
  omni_video_len(): number
  omni_video_width(): number
  omni_video_height(): number
  omni_audio_ptr(): number
  omni_audio_len(): number
  omni_audio_rate(): number
  omni_audio_channels(): number
  omni_last_error_ptr(): number
  omni_last_error_len(): number
  omni_save_state(ptr: number, capacity: number): number
  omni_load_state(ptr: number, len: number): number
  omni_persistent_len(kind: number, slot: number): number
  omni_read_persistent(kind: number, slot: number, ptr: number, capacity: number): number
  omni_write_persistent(kind: number, slot: number, ptr: number, len: number): number
}
export interface OmniLaunchRequest {
  platform: Platform
  game?: File | null
  bios?: File | null
  firmware?: File | null
  inputProfile: InputProfile
}

export interface OmniSession {
  canvas: HTMLCanvasElement
  reset(): void
  saveState(): Uint8Array
  loadState(bytes: Uint8Array): void
  dispose(): void
}

let exportsPromise: Promise<OmniExports> | null = null

async function loadExports(): Promise<OmniExports> {
  if (exportsPromise) return exportsPromise
  exportsPromise = (async () => {
    const url = `${import.meta.env.BASE_URL}omnicore.wasm`
    const response = await fetch(url)
    if (!response.ok) throw new Error(`Unable to load OmniCore (${response.status}).`)
    const bytes = await response.arrayBuffer()
    const instance = await WebAssembly.instantiate(bytes, {})
    const exports = instance.instance.exports as unknown as OmniExports
    if (!(exports.memory instanceof WebAssembly.Memory) || exports.omni_core_version() !== 1) {
      throw new Error('OmniCore ABI is incompatible with this frontend.')
    }
    return exports
  })()
  return exportsPromise
}
const PAD_BUTTONS: Record<string, number> = {
  BUTTON_1: 0, BUTTON_2: 1, BUTTON_3: 2, BUTTON_4: 3,
  LEFT_TOP_SHOULDER: 4, RIGHT_TOP_SHOULDER: 5,
  LEFT_BOTTOM_SHOULDER: 6, RIGHT_BOTTOM_SHOULDER: 7,
  SELECT: 8, START: 9, LEFT_STICK: 10, RIGHT_STICK: 11,
  DPAD_UP: 12, DPAD_DOWN: 13, DPAD_LEFT: 14, DPAD_RIGHT: 15,
}
const PAD_AXES: Record<string, [number, number]> = {
  'LEFT_STICK_X:+1': [0, 1], 'LEFT_STICK_X:-1': [0, -1],
  'LEFT_STICK_Y:+1': [1, 1], 'LEFT_STICK_Y:-1': [1, -1],
  'RIGHT_STICK_X:+1': [2, 1], 'RIGHT_STICK_X:-1': [2, -1],
  'RIGHT_STICK_Y:+1': [3, 1], 'RIGHT_STICK_Y:-1': [3, -1],
}

class InputScanner {
  private pressed = new Set<string>()
  private readonly profile: InputProfile
  private readonly pointerSurface: HTMLCanvasElement
  private pointerX = 0
  private pointerY = 0
  private pointerTouch = false
  private pointerClick = false

  constructor(profile: InputProfile, pointerSurface: HTMLCanvasElement) {
    this.profile = profile
    this.pointerSurface = pointerSurface
    this.pointerSurface.style.touchAction = 'none'
    window.addEventListener('keydown', this.keyDown)
    window.addEventListener('keyup', this.keyUp)
    window.addEventListener('blur', this.blur)
    this.pointerSurface.addEventListener('pointermove', this.pointerMove)
    this.pointerSurface.addEventListener('pointerenter', this.pointerMove)
    this.pointerSurface.addEventListener('pointerleave', this.pointerLeave)
    this.pointerSurface.addEventListener('pointerdown', this.pointerDown)
    this.pointerSurface.addEventListener('pointerup', this.pointerUp)
    this.pointerSurface.addEventListener('pointercancel', this.pointerLeave)
  }

  readButtons(player: number): bigint {
    let mask = 0n
    const pad = navigator.getGamepads?.()[player] ?? null
    for (const binding of this.profile.players[player] ?? []) {
      if (binding.index > 15) continue
      const strength = Math.max(this.keyboard(binding.keyboard) ? 1 : 0, this.gamepadStrength(pad, binding.gamepad))
      if (strength > 0.5) mask |= 1n << BigInt(binding.index)
    }
    if (player === 0) {
      if (this.pointerTouch) mask |= 1n << 32n
      if (this.pointerClick) mask |= 1n << 33n
    }
    return mask
  }

  readAxes(player: number): Int16Array {
    const pad = navigator.getGamepads?.()[player] ?? null
    const bindings = this.profile.players[player] ?? []
    const strength = (index: number) => {
      const binding = bindings.find((candidate) => candidate.index === index)
      if (!binding) return 0
      return Math.max(this.keyboard(binding.keyboard) ? 1 : 0, this.gamepadStrength(pad, binding.gamepad))
    }
    const axes = new Int16Array(8)
    axes[0] = signedAxis(strength(16) - strength(17))
    axes[1] = signedAxis(strength(18) - strength(19))
    axes[2] = signedAxis(strength(20) - strength(21))
    axes[3] = signedAxis(strength(22) - strength(23))
    axes[4] = triggerAxis(strength(12))
    axes[5] = triggerAxis(strength(13))
    if (player === 0) {
      axes[6] = signedAxis(this.pointerX)
      axes[7] = signedAxis(this.pointerY)
    }
    return axes
  }

  private keyboard(key: string) { return !!key && this.pressed.has(key.toLowerCase()) }

  private gamepadStrength(pad: Gamepad | null, token: string) {
    if (!pad || !token) return 0
    const button = PAD_BUTTONS[token]
    if (button !== undefined) return pad.buttons[button]?.value ?? 0
    const axis = PAD_AXES[token]
    if (!axis) return 0
    const directed = (pad.axes[axis[0]] ?? 0) * axis[1]
    const deadzone = 0.15
    if (directed <= deadzone) return 0
    return Math.min(1, (directed - deadzone) / (1 - deadzone))
  }

  private keyDown = (event: KeyboardEvent) => {
    const key = event.key.toLowerCase()
    if (this.isMappedKey(key)) { event.preventDefault(); this.pressed.add(key) }
  }
  private keyUp = (event: KeyboardEvent) => {
    const key = event.key.toLowerCase()
    if (this.isMappedKey(key)) { event.preventDefault(); this.pressed.delete(key) }
  }
  private updatePointer(event: PointerEvent) {
    const rect = this.pointerSurface.getBoundingClientRect()
    if (rect.width <= 0 || rect.height <= 0) return
    const x = Math.max(0, Math.min(1, (event.clientX - rect.left) / rect.width))
    const y = Math.max(0, Math.min(1, (event.clientY - rect.top) / rect.height))
    this.pointerX = x * 2 - 1
    this.pointerY = y * 2 - 1
    this.pointerTouch = event.pointerType === 'mouse'
      ? true
      : event.pressure > 0 || event.buttons !== 0
    this.pointerClick = (event.buttons & 1) !== 0
  }

  private pointerMove = (event: PointerEvent) => this.updatePointer(event)
  private pointerDown = (event: PointerEvent) => {
    event.preventDefault()
    this.updatePointer(event)
  }
  private pointerUp = (event: PointerEvent) => {
    event.preventDefault()
    this.updatePointer(event)
  }
  private pointerLeave = () => {
    this.pointerTouch = false
    this.pointerClick = false
  }
  private blur = () => {
    this.pressed.clear()
    this.pointerTouch = false
    this.pointerClick = false
  }
  private isMappedKey(key: string) {
    return Object.values(this.profile.players).some((bindings) =>
      bindings.some((binding) => binding.keyboard.toLowerCase() === key))
  }
  dispose() {
    window.removeEventListener('keydown', this.keyDown)
    window.removeEventListener('keyup', this.keyUp)
    window.removeEventListener('blur', this.blur)
    this.pointerSurface.removeEventListener('pointermove', this.pointerMove)
    this.pointerSurface.removeEventListener('pointerenter', this.pointerMove)
    this.pointerSurface.removeEventListener('pointerleave', this.pointerLeave)
    this.pointerSurface.removeEventListener('pointerdown', this.pointerDown)
    this.pointerSurface.removeEventListener('pointerup', this.pointerUp)
    this.pointerSurface.removeEventListener('pointercancel', this.pointerLeave)
    this.pressed.clear()
  }
}

const signedAxis = (value: number) =>
  Math.round(Math.max(-1, Math.min(1, value)) * 32767)

const triggerAxis = (value: number) =>
  Math.round(Math.max(0, Math.min(1, value)) * 32767)
function lastError(exports: OmniExports) {
  const ptr = exports.omni_last_error_ptr()
  const len = exports.omni_last_error_len()
  if (!ptr || !len) return 'OmniCore reported an unknown error.'
  return new TextDecoder().decode(new Uint8Array(exports.memory.buffer, ptr, len))
}

const STAGE_CHUNK_BYTES = 1024 * 1024
const STREAM_CACHE_CHUNKS = 512
const NO_PENDING_RANGE = -1n

async function writeFileRange(
  exports: OmniExports,
  kind: number,
  slot: number,
  file: File,
  start: number,
  end: number,
) {
  if (end <= start) return
  const bytes = new Uint8Array(await file.slice(start, end).arrayBuffer())
  const ptr = exports.omni_alloc(bytes.byteLength)
  if (!ptr) throw new Error(`OmniCore could not allocate ${bytes.byteLength} staging bytes.`)
  try {
    new Uint8Array(exports.memory.buffer, ptr, bytes.byteLength).set(bytes)
    requireOk(exports, exports.omni_resource_write(kind, slot, BigInt(start), ptr, bytes.byteLength))
  } finally {
    exports.omni_free(ptr, bytes.byteLength)
  }
}

async function stageFile(
  exports: OmniExports,
  kind: number,
  slot: number,
  file?: File | null,
  streaming = false,
) {
  if (!file) return
  if (streaming) {
    requireOk(
      exports,
      exports.omni_resource_create_streaming(kind, slot, BigInt(file.size), STREAM_CACHE_CHUNKS),
    )
    await writeFileRange(exports, kind, slot, file, 0, Math.min(file.size, STAGE_CHUNK_BYTES))
    return
  }
  requireOk(exports, exports.omni_resource_create(kind, slot, BigInt(file.size)))
  for (let offset = 0; offset < file.size; offset += STAGE_CHUNK_BYTES) {
    await writeFileRange(
      exports,
      kind,
      slot,
      file,
      offset,
      Math.min(file.size, offset + STAGE_CHUNK_BYTES),
    )
  }
}

async function hydratePendingResource(
  exports: OmniExports,
  kind: number,
  slot: number,
  file?: File | null,
) {
  if (!file) return false
  const start = exports.omni_resource_pending_start(kind, slot)
  if (start === NO_PENDING_RANGE) return false
  const end = exports.omni_resource_pending_end(kind, slot)
  if (end <= start || end > BigInt(file.size)) {
    throw new Error('OmniCore requested an invalid media range.')
  }
  const requestedStart = Number(start)
  const requestedEnd = Number(end)
  if (!Number.isSafeInteger(requestedStart) || !Number.isSafeInteger(requestedEnd)) {
    throw new Error('OmniCore requested a media range outside browser file limits.')
  }
  const windowStart = Math.floor(requestedStart / STAGE_CHUNK_BYTES) * STAGE_CHUNK_BYTES
  const windowEnd = Math.min(
    file.size,
    Math.max(requestedEnd, windowStart + STAGE_CHUNK_BYTES),
  )
  await writeFileRange(exports, kind, slot, file, windowStart, windowEnd)
  return true
}

function requireOk(exports: OmniExports, result: number) {
  if (result !== 0) throw new Error(lastError(exports))
}

async function loadStagedWithHydration(
  exports: OmniExports,
  platform: number,
  primaryKind: number,
  primaryFile?: File | null,
  streaming = false,
) {
  for (let attempt = 0; attempt < 128; attempt++) {
    const result = exports.omni_load_staged(platform)
    if (result === 0) return
    if (!streaming || !(await hydratePendingResource(exports, primaryKind, 0, primaryFile))) {
      throw new Error(lastError(exports))
    }
  }
  throw new Error('OmniCore requested too many media ranges while constructing the machine.')
}

const PERSISTENT_RESOURCE_KINDS = [5, 6, 7] as const
const PERSISTENT_RESOURCE_SLOTS = [0, 1] as const

interface PersistentResourceBinding {
  kind: number
  slot: number
  key: string
  last: Uint8Array | null
}

function readPersistent(exports: OmniExports, kind: number, slot: number): Uint8Array | null {
  const length = exports.omni_persistent_len(kind, slot)
  if (!length) return null
  const ptr = exports.omni_alloc(length)
  if (!ptr) throw new Error(`OmniCore could not allocate ${length} persistent bytes.`)
  try {
    requireOk(exports, exports.omni_read_persistent(kind, slot, ptr, length))
    return new Uint8Array(exports.memory.buffer, ptr, length).slice()
  } finally {
    exports.omni_free(ptr, length)
  }
}

function writePersistent(exports: OmniExports, kind: number, slot: number, bytes: Uint8Array) {
  const expected = exports.omni_persistent_len(kind, slot)
  if (!expected || bytes.byteLength !== expected) return false
  const ptr = exports.omni_alloc(bytes.byteLength)
  if (!ptr) throw new Error(`OmniCore could not allocate ${bytes.byteLength} persistent bytes.`)
  try {
    new Uint8Array(exports.memory.buffer, ptr, bytes.byteLength).set(bytes)
    requireOk(exports, exports.omni_write_persistent(kind, slot, ptr, bytes.byteLength))
    return true
  } finally {
    exports.omni_free(ptr, bytes.byteLength)
  }
}
function sameBytes(left: Uint8Array | null, right: Uint8Array) {
  if (!left || left.byteLength !== right.byteLength) return false
  for (let index = 0; index < right.byteLength; index++) {
    if (left[index] !== right[index]) return false
  }
  return true
}

export const runtimeAvailable = (platform: Platform) => platform.launchable === true

export async function launchOmniCore(host: HTMLElement, request: OmniLaunchRequest): Promise<OmniSession> {
  const exports = await loadExports()
  if (!exports.omni_can_launch(request.platform.omniCode)) {
    throw new Error(`${request.platform.name} does not have a runnable machine graph in OmniCore yet.`)
  }
  exports.omni_resources_clear()
  const primaryMedia = mediaForFile(request.platform, request.game?.name)
  const primaryKind = primaryMedia === 'disc' ? 4 : 0
  const streamingPrimary = primaryMedia === 'disc'
  try {
    await stageFile(exports, primaryKind, 0, request.game, streamingPrimary)
    await stageFile(exports, 1, 0, request.bios)
    await stageFile(exports, 2, 0, request.firmware)
    await loadStagedWithHydration(
      exports,
      request.platform.omniCode,
      primaryKind,
      request.game,
      streamingPrimary,
    )
  } catch (error) {
    exports.omni_resources_clear()
    throw error
  }

  const persistentResources: PersistentResourceBinding[] = []
  if (request.game) {
    try {
      const fingerprint = await fingerprintFile(request.game)
      for (const kind of PERSISTENT_RESOURCE_KINDS) {
        for (const slot of PERSISTENT_RESOURCE_SLOTS) {
          if (!exports.omni_persistent_len(kind, slot)) continue
          const key = `v1:${request.platform.omniCode}:${kind}:${slot}:${fingerprint}`
          try {
            const stored = await loadPersistentResource(key)
            if (stored) writePersistent(exports, kind, slot, stored)
            persistentResources.push({ kind, slot, key, last: readPersistent(exports, kind, slot) })
          } catch {
            // Keep other persistent resources active if one slot is unavailable or corrupt.
          }
        }
      }
    } catch {
      persistentResources.length = 0
    }
  }

  let persistenceWrite = Promise.resolve()
  const flushPersistence = () => {
    if (!persistentResources.length) return
    const writes: Array<{ resource: PersistentResourceBinding; bytes: Uint8Array }> = []
    for (const resource of persistentResources) {
      const bytes = readPersistent(exports, resource.kind, resource.slot)
      if (!bytes || sameBytes(resource.last, bytes)) continue
      writes.push({ resource, bytes })
    }
    if (!writes.length) return
    persistenceWrite = persistenceWrite
      .then(() => Promise.all(writes.map(async ({ resource, bytes }) => {
        await savePersistentResource(resource.key, bytes)
        resource.last = bytes
      })))
      .then(() => undefined)
      .catch(() => undefined)
  }

  const video = await createVideoRenderer()
  const audio = new AudioScheduler()
  const input = new InputScanner(request.inputProfile, video.canvas)
  host.replaceChildren(video.canvas)
  await audio.resume()

  let alive = true
  let disposed = false
  let frameHandle = 0
  let persistenceFrames = 0
  const visibilityHandler = () => { if (document.visibilityState === 'hidden') flushPersistence() }
  document.addEventListener('visibilitychange', visibilityHandler)
  const frameMs = 1000 / Math.max(1, exports.omni_frame_rate())
  let nextFrame = performance.now()
  const drawFrame = async () => {
    for (let player = 0; player < 4; player++) {
      requireOk(exports, exports.omni_set_input(player, input.readButtons(player)))
      const axes = input.readAxes(player)
      for (let axis = 0; axis < axes.length; axis++) {
        requireOk(exports, exports.omni_set_axis(player, axis, axes[axis] ?? 0))
      }
    }
    requireOk(exports, exports.omni_run_frame())
    if (streamingPrimary) {
      await hydratePendingResource(exports, primaryKind, 0, request.game)
    }
    persistenceFrames++
    if (persistenceFrames >= 300) { persistenceFrames = 0; flushPersistence() }
    const width = exports.omni_video_width()
    const height = exports.omni_video_height()
    const videoPtr = exports.omni_video_ptr()
    const videoLen = exports.omni_video_len()
    if (width && height && videoPtr && videoLen === width * height * 4) {
      video.draw(width, height, new Uint8Array(exports.memory.buffer, videoPtr, videoLen))
    }
    const audioPtr = exports.omni_audio_ptr()
    const audioLen = exports.omni_audio_len()
    const channels = exports.omni_audio_channels()
    if (audioPtr && audioLen && channels) {
      audio.schedule(new Float32Array(exports.memory.buffer, audioPtr, audioLen), exports.omni_audio_rate(), channels)
    }
  }

  const tick = async (now: number) => {
    if (!alive) return
    try {
      let frames = 0
      while (now >= nextFrame && frames < 2) {
        await drawFrame()
        nextFrame += frameMs
        frames++
      }
      if (now - nextFrame > frameMs * 4) nextFrame = now + frameMs
    } catch (error) {
      alive = false
      const message = error instanceof Error ? error.message : String(error)
      window.dispatchEvent(new CustomEvent('omnicore:error', { detail: { message } }))
    }
    if (alive) frameHandle = requestAnimationFrame((time) => { void tick(time) })
  }
  frameHandle = requestAnimationFrame((time) => { void tick(time) })
  return {
    canvas: video.canvas,
    reset() { requireOk(exports, exports.omni_reset()); nextFrame = performance.now() },
    saveState() {
      const size = exports.omni_save_state(0, 0)
      if (!size) throw new Error(lastError(exports))
      const ptr = exports.omni_alloc(size)
      if (!ptr) throw new Error('OmniCore could not allocate a save-state buffer.')
      try {
        const written = exports.omni_save_state(ptr, size)
        if (written !== size) throw new Error(lastError(exports))
        return new Uint8Array(exports.memory.buffer, ptr, size).slice()
      } finally { exports.omni_free(ptr, size) }
    },
    loadState(bytes: Uint8Array) {
      const ptr = exports.omni_alloc(bytes.byteLength)
      if (!ptr) throw new Error('OmniCore could not allocate a load-state buffer.')
      try {
        new Uint8Array(exports.memory.buffer, ptr, bytes.byteLength).set(bytes)
        requireOk(exports, exports.omni_load_state(ptr, bytes.byteLength))
      } finally { exports.omni_free(ptr, bytes.byteLength) }
    },
    dispose() {
      if (disposed) return
      disposed = true; alive = false; cancelAnimationFrame(frameHandle)
      document.removeEventListener('visibilitychange', visibilityHandler)
      flushPersistence()
      input.dispose(); audio.dispose(); video.dispose(); exports.omni_unload(); video.canvas.remove()
    },
  }
}
