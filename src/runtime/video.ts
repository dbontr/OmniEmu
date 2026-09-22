export interface VideoRenderer {
  readonly canvas: HTMLCanvasElement
  draw(width: number, height: number, pixels: Uint8Array): void
  dispose(): void
}

interface GpuAdapterLike {
  requestDevice(): Promise<GpuDeviceLike>
}

interface GpuApiLike {
  requestAdapter(): Promise<GpuAdapterLike | null>
  getPreferredCanvasFormat(): string
}

interface GpuTextureLike {
  createView(): unknown
  destroy(): void
}

interface GpuPipelineLike {
  getBindGroupLayout(index: number): unknown
}

interface GpuRenderPassLike {
  setPipeline(pipeline: GpuPipelineLike): void
  setBindGroup(index: number, bindGroup: unknown): void
  draw(vertexCount: number): void
  end(): void
}

interface GpuCommandEncoderLike {
  beginRenderPass(descriptor: unknown): GpuRenderPassLike
  finish(): unknown
}

interface GpuDeviceLike {
  queue: {
    writeTexture(destination: unknown, data: Uint8Array, layout: unknown, size: unknown): void
    submit(commandBuffers: unknown[]): void
  }
  createShaderModule(descriptor: unknown): unknown
  createSampler(descriptor: unknown): unknown
  createRenderPipeline(descriptor: unknown): GpuPipelineLike
  createTexture(descriptor: unknown): GpuTextureLike
  createBindGroup(descriptor: unknown): unknown
  createCommandEncoder(): GpuCommandEncoderLike
}

interface GpuCanvasContextLike {
  configure(descriptor: unknown): void
  getCurrentTexture(): GpuTextureLike
}

const GPU_TEXTURE_COPY_DST = 0x02
const GPU_TEXTURE_BINDING = 0x04

function gpuApi(): GpuApiLike | null {
  return (navigator as Navigator & { gpu?: GpuApiLike }).gpu ?? null
}

class WebGpuVideoRenderer implements VideoRenderer {
  readonly canvas: HTMLCanvasElement
  private texture: GpuTextureLike | null = null
  private bindGroup: unknown = null
  private textureWidth = 0
  private textureHeight = 0
  private upload = new Uint8Array(0)
  private readonly device: GpuDeviceLike
  private readonly context: GpuCanvasContextLike
  private readonly pipeline: GpuPipelineLike
  private readonly sampler: unknown

  private constructor(
    canvas: HTMLCanvasElement,
    device: GpuDeviceLike,
    context: GpuCanvasContextLike,
    pipeline: GpuPipelineLike,
    sampler: unknown,
  ) {
    this.canvas = canvas
    this.device = device
    this.context = context
    this.pipeline = pipeline
    this.sampler = sampler
    this.canvas.className = 'omnicore-canvas'
  }

  static async create(): Promise<WebGpuVideoRenderer | null> {
    const gpu = gpuApi()
    if (!gpu) return null
    const adapter = await gpu.requestAdapter()
    if (!adapter) return null
    const device = await adapter.requestDevice()
    const canvas = document.createElement('canvas')
    const context = canvas.getContext('webgpu') as unknown as GpuCanvasContextLike | null
    if (!context) return null
    const format = gpu.getPreferredCanvasFormat()
    context.configure({ device, format, alphaMode: 'opaque' })
    const module = device.createShaderModule({ code: `
      struct VertexOut {
        @builtin(position) position: vec4f,
        @location(0) uv: vec2f,
      }
      @vertex fn vertexMain(@builtin(vertex_index) index: u32) -> VertexOut {
        var positions = array<vec2f, 3>(vec2f(-1.0, -1.0), vec2f(3.0, -1.0), vec2f(-1.0, 3.0));
        var out: VertexOut;
        let position = positions[index];
        out.position = vec4f(position, 0.0, 1.0);
        out.uv = vec2f((position.x + 1.0) * 0.5, 1.0 - (position.y + 1.0) * 0.5);
        return out;
      }
      @group(0) @binding(0) var frameSampler: sampler;
      @group(0) @binding(1) var frameTexture: texture_2d<f32>;
      @fragment fn fragmentMain(in: VertexOut) -> @location(0) vec4f {
        return textureSample(frameTexture, frameSampler, in.uv);
      }
    ` })
    const sampler = device.createSampler({ minFilter: 'nearest', magFilter: 'nearest' })
    const pipeline = device.createRenderPipeline({
      layout: 'auto',
      vertex: { module, entryPoint: 'vertexMain' },
      fragment: { module, entryPoint: 'fragmentMain', targets: [{ format }] },
      primitive: { topology: 'triangle-list' },
    })
    return new WebGpuVideoRenderer(canvas, device, context, pipeline, sampler)
  }

  draw(width: number, height: number, pixels: Uint8Array) {
    if (this.canvas.width !== width || this.canvas.height !== height) {
      this.canvas.width = width
      this.canvas.height = height
    }
    if (!this.texture || this.textureWidth !== width || this.textureHeight !== height) {
      this.texture?.destroy()
      this.textureWidth = width
      this.textureHeight = height
      this.texture = this.device.createTexture({
        size: { width, height, depthOrArrayLayers: 1 },
        format: 'rgba8unorm',
        usage: GPU_TEXTURE_COPY_DST | GPU_TEXTURE_BINDING,
      })
      this.bindGroup = this.device.createBindGroup({
        layout: this.pipeline.getBindGroupLayout(0),
        entries: [
          { binding: 0, resource: this.sampler },
          { binding: 1, resource: this.texture.createView() },
        ],
      })
    }
    const sourceBytesPerRow = width * 4
    const bytesPerRow = (sourceBytesPerRow + 255) & ~255
    let upload = pixels
    if (bytesPerRow !== sourceBytesPerRow) {
      const required = bytesPerRow * height
      if (this.upload.byteLength !== required) this.upload = new Uint8Array(required)
      for (let row = 0; row < height; row++) {
        const sourceStart = row * sourceBytesPerRow
        const targetStart = row * bytesPerRow
        this.upload.set(pixels.subarray(sourceStart, sourceStart + sourceBytesPerRow), targetStart)
      }
      upload = this.upload
    }
    this.device.queue.writeTexture(
      { texture: this.texture },
      upload,
      { bytesPerRow, rowsPerImage: height },
      { width, height, depthOrArrayLayers: 1 },
    )
    const encoder = this.device.createCommandEncoder()
    const pass = encoder.beginRenderPass({
      colorAttachments: [{
        view: this.context.getCurrentTexture().createView(),
        clearValue: { r: 0, g: 0, b: 0, a: 1 },
        loadOp: 'clear',
        storeOp: 'store',
      }],
    })
    pass.setPipeline(this.pipeline)
    pass.setBindGroup(0, this.bindGroup)
    pass.draw(3)
    pass.end()
    this.device.queue.submit([encoder.finish()])
  }

  dispose() {
    this.texture?.destroy()
    this.texture = null
    this.bindGroup = null
  }
}

class WebGlVideoRenderer implements VideoRenderer {
  readonly canvas = document.createElement('canvas')
  private readonly gl: WebGL2RenderingContext
  private readonly texture: WebGLTexture
  private readonly program: WebGLProgram
  private readonly vao: WebGLVertexArrayObject
  private readonly vertices: WebGLBuffer
  private textureWidth = 0
  private textureHeight = 0

  constructor() {
    const gl = this.canvas.getContext('webgl2', {
      alpha: false,
      antialias: false,
      depth: false,
      stencil: false,
    })
    if (!gl) throw new Error('OmniCore requires WebGL 2 when WebGPU is unavailable.')
    this.gl = gl
    this.canvas.className = 'omnicore-canvas'
    const program = this.createProgram()
    const vao = gl.createVertexArray()
    if (!vao) {
      gl.deleteProgram(program)
      throw new Error('Unable to create OmniCore video vertex array.')
    }
    gl.bindVertexArray(vao)
    gl.useProgram(program)
    const vertices = gl.createBuffer()
    if (!vertices) {
      gl.deleteVertexArray(vao)
      gl.deleteProgram(program)
      throw new Error('Unable to create OmniCore video vertex buffer.')
    }
    this.program = program
    this.vao = vao
    this.vertices = vertices
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
    const compile = (type: number, source: string) => {
      const shader = gl.createShader(type)!
      gl.shaderSource(shader, source)
      gl.compileShader(shader)
      if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
        throw new Error(gl.getShaderInfoLog(shader) || 'Video shader failed.')
      }
      return shader
    }
    const program = gl.createProgram()!
    const vertexShader = compile(gl.VERTEX_SHADER, vertex)
    const fragmentShader = compile(gl.FRAGMENT_SHADER, fragment)
    gl.attachShader(program, vertexShader)
    gl.attachShader(program, fragmentShader)
    gl.linkProgram(program)
    gl.deleteShader(vertexShader)
    gl.deleteShader(fragmentShader)
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      const error = gl.getProgramInfoLog(program) || 'Video program failed.'
      gl.deleteProgram(program)
      throw new Error(error)
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
    gl.texSubImage2D(
      gl.TEXTURE_2D,
      0,
      0,
      0,
      width,
      height,
      gl.RGBA,
      gl.UNSIGNED_BYTE,
      pixels,
    )
    gl.drawArrays(gl.TRIANGLES, 0, 3)
  }

  dispose() {
    this.gl.deleteTexture(this.texture)
    this.gl.deleteBuffer(this.vertices)
    this.gl.deleteVertexArray(this.vao)
    this.gl.deleteProgram(this.program)
  }
}

export async function createVideoRenderer(): Promise<VideoRenderer> {
  try {
    const webGpu = await WebGpuVideoRenderer.create()
    if (webGpu) return webGpu
  } catch {
    // WebGL remains the deterministic compatibility path when WebGPU setup fails.
  }
  return new WebGlVideoRenderer()
}
