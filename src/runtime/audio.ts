const WORKLET_NAME = 'omniemu-audio-output'

const WORKLET_SOURCE = `
class OmniAudioProcessor extends AudioWorkletProcessor {
  constructor() {
    super()
    this.queue = []
    this.segment = null
    this.position = 0
    this.port.onmessage = ({ data }) => {
      if (data.type === 'reset') {
        this.queue.length = 0
        this.segment = null
        this.position = 0
        return
      }
      if (data.type !== 'audio') return
      this.queue.push({ samples: new Float32Array(data.buffer), rate: data.rate, channels: data.channels })
      while (this.queue.length > 8) this.queue.shift()
    }
  }

  nextSegment() {
    this.segment = this.queue.shift() ?? null
  }

  process(_inputs, outputs) {
    const output = outputs[0]
    const left = output[0]
    const right = output[1] ?? left
    left.fill(0)
    if (right !== left) right.fill(0)
    for (let frame = 0; frame < left.length; frame++) {
      if (!this.segment) this.nextSegment()
      while (this.segment) {
        const frames = this.segment.samples.length / this.segment.channels
        if (this.position < frames) break
        this.position -= frames
        this.nextSegment()
      }
      if (!this.segment) break
      const { samples, rate, channels } = this.segment
      const frames = samples.length / channels
      const base = Math.floor(this.position)
      const next = Math.min(base + 1, frames - 1)
      const fraction = this.position - base
      const sample = (index, channel) => samples[index * channels + (channels === 1 ? 0 : channel)] ?? 0
      left[frame] = sample(base, 0) + (sample(next, 0) - sample(base, 0)) * fraction
      right[frame] = sample(base, 1) + (sample(next, 1) - sample(base, 1)) * fraction
      this.position += rate / sampleRate
    }
    return true
  }
}
registerProcessor('${WORKLET_NAME}', OmniAudioProcessor)
`

class BufferSourceOutput {
  private cursor = 0
  private readonly context: AudioContext

  constructor(context: AudioContext) {
    this.context = context
  }

  schedule(samples: Float32Array, sampleRate: number, channels: number) {
    const frames = Math.floor(samples.length / channels)
    const buffer = this.context.createBuffer(channels, frames, sampleRate)
    for (let channel = 0; channel < channels; channel++) {
      const target = buffer.getChannelData(channel)
      for (let frame = 0; frame < frames; frame++) {
        target[frame] = samples[frame * channels + channel] ?? 0
      }
    }
    const source = this.context.createBufferSource()
    source.buffer = buffer
    source.connect(this.context.destination)
    const now = this.context.currentTime
    if (this.cursor < now || this.cursor - now > 0.18) this.cursor = now
    source.start(this.cursor)
    this.cursor += frames / sampleRate
  }
}
export class AudioScheduler {
  private context: AudioContext | null = null
  private worklet: AudioWorkletNode | null = null
  private fallback: BufferSourceOutput | null = null

  async resume() {
    this.context ??= new AudioContext({ latencyHint: 'interactive' })
    if (this.context.state === 'suspended') await this.context.resume()
    if (this.worklet || this.fallback) return
    if (this.context.audioWorklet && typeof AudioWorkletNode !== 'undefined') {
      const url = URL.createObjectURL(new Blob([WORKLET_SOURCE], { type: 'text/javascript' }))
      try {
        await this.context.audioWorklet.addModule(url)
        this.worklet = new AudioWorkletNode(this.context, WORKLET_NAME, {
          numberOfInputs: 0,
          numberOfOutputs: 1,
          outputChannelCount: [2],
        })
        this.worklet.connect(this.context.destination)
        return
      } catch {
        this.worklet = null
      } finally {
        URL.revokeObjectURL(url)
      }
    }
    this.fallback = new BufferSourceOutput(this.context)
  }

  schedule(samples: Float32Array, sampleRate: number, channels: number) {
    if (!samples.length || !sampleRate || (channels !== 1 && channels !== 2)) return
    if (this.worklet) {
      const copy = samples.slice()
      this.worklet.port.postMessage(
        { type: 'audio', buffer: copy.buffer, rate: sampleRate, channels },
        [copy.buffer],
      )
      return
    }
    this.fallback?.schedule(samples, sampleRate, channels)
  }

  dispose() {
    const context = this.context
    this.worklet?.port.postMessage({ type: 'reset' })
    this.worklet?.disconnect()
    this.context = null
    this.worklet = null
    this.fallback = null
    if (context) void context.close()
  }
}
