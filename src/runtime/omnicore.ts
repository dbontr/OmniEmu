import type { Platform } from '../catalog'
import type { InputProfile } from '../input/mapping'

interface OmniExports extends WebAssembly.Exports {
  memory: WebAssembly.Memory
  omni_core_version(): number
  omni_support_level(platform: number): number
  omni_can_launch(platform: number): number
  omni_alloc(len: number): number
  omni_free(ptr: number, len: number): void
  omni_load(platform: number, romPtr: number, romLen: number, biosPtr: number, biosLen: number): number
  omni_unload(): void
  omni_reset(): number
  omni_set_input(player: number, buttons: bigint): number
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
}
export interface OmniLaunchRequest {
  platform: Platform
  game?: File | null
  bios?: File | null
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
class VideoRenderer {
  readonly canvas = document.createElement('canvas')
  private readonly gl: WebGL2RenderingContext
  private readonly texture: WebGLTexture
  private textureWidth = 0
  private textureHeight = 0

  constructor() {
    const gl = this.canvas.getContext('webgl2', { alpha: false, antialias: false, depth: false, stencil: false })
    if (!gl) throw new Error('OmniCore requires WebGL 2 for the unified video path.')
    this.gl = gl
    this.canvas.className = 'omnicore-canvas'
    const program = this.createProgram()
    const vao = gl.createVertexArray()
    if (!vao) throw new Error('Unable to create OmniCore video vertex array.')
    gl.bindVertexArray(vao)
    gl.useProgram(program)
    const vertices = gl.createBuffer()
    if (!vertices) throw new Error('Unable to create OmniCore video vertex buffer.')
    gl.bindBuffer(gl.ARRAY_BUFFER, vertices)
    gl.bufferData(gl.ARRAY_BUFFER, new Float32Array([-1, -1, 3, -1, -1, 3]), gl.STATIC_DRAW)
    const position = gl.getAttribLocation(program, 'a_position')
    gl.enableVertexAttribArray(position)
    gl.vertexAttribPointer(position, 2, gl.FLOAT, false, 0, 0)
    this.texture = gl.createTexture()!
    gl.bindTexture(gl.TEXTURE_2D, this.texture)
    gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1)
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST)
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST)
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE)
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)
    gl.uniform1i(gl.getUniformLocation(program, 'u_frame'), 0)
    gl.disable(gl.DEPTH_TEST)
    gl.disable(gl.CULL_FACE)
    gl.disable(gl.BLEND)
  }

  private createProgram() {
    const gl = this.gl
    const vertex = `#version 300 es
      in vec2 a_position; out vec2 v_uv;
      void main(){ gl_Position=vec4(a_position,0.0,1.0); v_uv=vec2((a_position.x+1.0)*0.5,1.0-(a_position.y+1.0)*0.5); }`
    const fragment = `#version 300 es
      precision mediump float; in vec2 v_uv; uniform sampler2D u_frame; out vec4 outColor;
      void main(){ outColor=texture(u_frame,v_uv); }`
    const compile = (type: number, shaderSource: string) => {
      const shader = gl.createShader(type)!
      gl.shaderSource(shader, shaderSource)
      gl.compileShader(shader)
      if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
        throw new Error(gl.getShaderInfoLog(shader) || 'Video shader failed.')
      }
      return shader
    }
    const program = gl.createProgram()!
    gl.attachShader(program, compile(gl.VERTEX_SHADER, vertex))
    gl.attachShader(program, compile(gl.FRAGMENT_SHADER, fragment))
    gl.linkProgram(program)
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      throw new Error(gl.getProgramInfoLog(program) || 'Video program failed.')
    }
    return program
  }

  draw(width: number, height: number, pixels: Uint8Array) {
    const gl = this.gl
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width
      this.canvas.height = height
    }
    gl.viewport(0, 0, width, height)
    gl.bindTexture(gl.TEXTURE_2D, this.texture)
    if (this.textureWidth !== width || this.textureHeight !== height) {
      this.textureWidth = width
      this.textureHeight = height
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, width, height, 0, gl.RGBA, gl.UNSIGNED_BYTE, null)
    }
    gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, width, height, gl.RGBA, gl.UNSIGNED_BYTE, pixels)
    gl.drawArrays(gl.TRIANGLES, 0, 3)
  }
}
class AudioScheduler {
  private context: AudioContext | null = null
  private cursor = 0

  async resume() {
    this.context ??= new AudioContext({ latencyHint: 'interactive' })
    if (this.context.state === 'suspended') await this.context.resume()
  }

  schedule(samples: Float32Array, sampleRate: number, channels: number) {
    if (!samples.length || !sampleRate || !channels) return
    const context = this.context
    if (!context) return
    const frames = Math.floor(samples.length / channels)
    const buffer = context.createBuffer(channels, frames, sampleRate)
    for (let channel = 0; channel < channels; channel++) {
      const target = buffer.getChannelData(channel)
      for (let frame = 0; frame < frames; frame++) target[frame] = samples[frame * channels + channel] ?? 0
    }
    const source = context.createBufferSource()
    source.buffer = buffer; source.connect(context.destination)
    const now = context.currentTime
    if (this.cursor < now || this.cursor - now > 0.18) this.cursor = now
    source.start(this.cursor)
    this.cursor += frames / sampleRate
  }

  dispose() {
    const context = this.context; this.context = null; this.cursor = 0
    if (context) void context.close()
  }
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
  constructor(profile: InputProfile) {
    this.profile = profile
    window.addEventListener('keydown', this.keyDown)
    window.addEventListener('keyup', this.keyUp)
    window.addEventListener('blur', this.blur)
  }

  read(player: number): bigint {
    let mask = 0n
    const pad = navigator.getGamepads?.()[player] ?? null
    for (const binding of this.profile.players[player] ?? []) {
      if (binding.index > 15) continue
      if (this.keyboard(binding.keyboard) || this.gamepad(pad, binding.gamepad)) mask |= 1n << BigInt(binding.index)
    }
    return mask
  }
  private keyboard(key: string) { return !!key && this.pressed.has(key.toLowerCase()) }
  private gamepad(pad: Gamepad | null, token: string) {
    if (!pad || !token) return false
    const button = PAD_BUTTONS[token]
    if (button !== undefined) return pad.buttons[button]?.pressed ?? false
    const axis = PAD_AXES[token]
    if (!axis) return false
    const value = pad.axes[axis[0]] ?? 0
    return axis[1] > 0 ? value > 0.5 : value < -0.5
  }
  private keyDown = (event: KeyboardEvent) => {
    const key = event.key.toLowerCase()
    if (this.isMappedKey(key)) { event.preventDefault(); this.pressed.add(key) }
  }
  private keyUp = (event: KeyboardEvent) => {
    const key = event.key.toLowerCase()
    if (this.isMappedKey(key)) { event.preventDefault(); this.pressed.delete(key) }
  }
  private blur = () => this.pressed.clear()
  private isMappedKey(key: string) {
    return Object.values(this.profile.players).some((bindings) => bindings.some((binding) => binding.keyboard.toLowerCase() === key))
  }
  dispose() {
    window.removeEventListener('keydown', this.keyDown)
    window.removeEventListener('keyup', this.keyUp)
    window.removeEventListener('blur', this.blur)
    this.pressed.clear()
  }
}
function lastError(exports: OmniExports) {
  const ptr = exports.omni_last_error_ptr()
  const len = exports.omni_last_error_len()
  if (!ptr || !len) return 'OmniCore reported an unknown error.'
  return new TextDecoder().decode(new Uint8Array(exports.memory.buffer, ptr, len))
}

async function copyFile(exports: OmniExports, file?: File | null) {
  if (!file) return { ptr: 0, len: 0, free: () => undefined }
  const source = new Uint8Array(await file.arrayBuffer())
  const ptr = exports.omni_alloc(source.byteLength)
  if (!ptr) throw new Error(`OmniCore could not allocate ${source.byteLength} bytes.`)
  new Uint8Array(exports.memory.buffer, ptr, source.byteLength).set(source)
  return {
    ptr,
    len: source.byteLength,
    free: () => exports.omni_free(ptr, source.byteLength),
  }
}

function requireOk(exports: OmniExports, result: number) {
  if (result !== 0) throw new Error(lastError(exports))
}
export const runtimeAvailable = (platform: Platform) => platform.launchable === true

export async function launchOmniCore(host: HTMLElement, request: OmniLaunchRequest): Promise<OmniSession> {
  const exports = await loadExports()
  if (!exports.omni_can_launch(request.platform.omniCode)) {
    throw new Error(`${request.platform.name} does not have a runnable machine graph in OmniCore yet.`)
  }
  const rom = await copyFile(exports, request.game)
  const bios = await copyFile(exports, request.bios)
  try {
    requireOk(exports, exports.omni_load(request.platform.omniCode, rom.ptr, rom.len, bios.ptr, bios.len))
  } finally {
    rom.free(); bios.free()
  }

  const video = new VideoRenderer()
  const audio = new AudioScheduler()
  const input = new InputScanner(request.inputProfile)
  host.replaceChildren(video.canvas)
  await audio.resume()

  let alive = true
  let disposed = false
  let frameHandle = 0
  const frameMs = 1000 / Math.max(1, exports.omni_frame_rate())
  let nextFrame = performance.now()
  const drawFrame = () => {
    for (let player = 0; player < 4; player++) requireOk(exports, exports.omni_set_input(player, input.read(player)))
    requireOk(exports, exports.omni_run_frame())
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

  const tick = (now: number) => {
    if (!alive) return
    try {
      let frames = 0
      while (now >= nextFrame && frames < 2) { drawFrame(); nextFrame += frameMs; frames++ }
      if (now - nextFrame > frameMs * 4) nextFrame = now + frameMs
    } catch (error) {
      alive = false
      const message = error instanceof Error ? error.message : String(error)
      window.dispatchEvent(new CustomEvent('omnicore:error', { detail: { message } }))
    }
    if (alive) frameHandle = requestAnimationFrame(tick)
  }
  frameHandle = requestAnimationFrame(tick)
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
      input.dispose(); audio.dispose(); exports.omni_unload(); video.canvas.remove()
    },
  }
}
