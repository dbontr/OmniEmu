import { createHash } from 'node:crypto'
import { closeSync, createReadStream, existsSync, openSync, readFileSync, readSync, statSync, writeFileSync } from 'node:fs'
import { dirname, isAbsolute, join, resolve } from 'node:path'
import { fileURLToPath } from 'node:url'
import { performance } from 'node:perf_hooks'
import { compatibilityFixture } from './compatibility-fixtures.mjs'

const root = dirname(dirname(fileURLToPath(import.meta.url)))
const CHUNK_BYTES = 1024 * 1024
const NO_PENDING_RANGE = -1n
const DEFAULT_STREAM_CACHE_CHUNKS = 512
const RESOURCE_KINDS = Object.freeze({
  game: 0,
  bios: 1,
  firmware: 2,
  keys: 3,
  disc: 4,
  storage: 5,
  memoryCard: 6,
  nand: 7,
})

function usage() {
  console.error('usage: node scripts/compatibility-runner.mjs <manifest.json> [--report <report.json>]')
  process.exit(2)
}

const args = process.argv.slice(2)
if (args.length === 0 || args[0].startsWith('-')) usage()
const manifestPath = resolve(args[0])
let reportPath = null
for (let index = 1; index < args.length; index++) {
  if (args[index] === '--report' && args[index + 1]) {
    reportPath = resolve(args[++index])
  } else {
    usage()
  }
}

function invariant(condition, message) {
  if (!condition) throw new Error(message)
}

function parseManifest(path) {
  const parsed = JSON.parse(readFileSync(path, 'utf8'))
  invariant(parsed?.schema === 'omniemu-compatibility/1', 'manifest schema must be omniemu-compatibility/1')
  invariant(Array.isArray(parsed.cases) && parsed.cases.length > 0, 'manifest must contain at least one case')
  return parsed
}

function resolveResourcePath(manifestDirectory, path) {
  invariant(typeof path === 'string' && path.length > 0, 'resource path must be a non-empty string')
  return isAbsolute(path) ? path : resolve(manifestDirectory, path)
}

function resourceKind(value) {
  if (Number.isInteger(value) && value >= 0 && value <= 7) return value
  invariant(typeof value === 'string' && Object.hasOwn(RESOURCE_KINDS, value), `unknown resource kind: ${value}`)
  return RESOURCE_KINDS[value]
}

async function sha256File(path) {
  const hash = createHash('sha256')
  for await (const chunk of createReadStream(path)) hash.update(chunk)
  return hash.digest('hex')
}

function sha256Bytes(bytes) {
  return createHash('sha256').update(bytes).digest('hex')
}

const wasmPath = join(root, 'public', 'omnicore.wasm')
invariant(existsSync(wasmPath), 'public/omnicore.wasm is missing; run npm run core:build first')
const { instance } = await WebAssembly.instantiate(readFileSync(wasmPath), {})
const core = instance.exports
invariant(core.omni_core_version() === 1, `unsupported OmniCore ABI ${core.omni_core_version()}`)

function lastError() {
  const ptr = core.omni_last_error_ptr()
  const len = core.omni_last_error_len()
  if (!ptr || !len) return 'OmniCore reported an unknown error'
  return new TextDecoder().decode(new Uint8Array(core.memory.buffer, ptr, len))
}

function requireOk(result, operation) {
  if (result !== 0) throw new Error(`${operation}: ${lastError()}`)
}

function writeRange(resource, start, end) {
  invariant(Number.isSafeInteger(start) && Number.isSafeInteger(end) && start >= 0 && end >= start, 'invalid resource range')
  if (start === end) return
  const length = end - start
  let bytes
  if (resource.bytes) {
    bytes = resource.bytes.subarray(start, end)
  } else {
    bytes = Buffer.allocUnsafe(length)
    const read = readSync(resource.fd, bytes, 0, length, start)
    invariant(read === length, `${resource.label}: short read ${read}/${length}`)
  }
  const ptr = core.omni_alloc(length)
  invariant(ptr > 0, `${resource.label}: OmniCore allocation failed for ${length} bytes`)
  try {
    new Uint8Array(core.memory.buffer, ptr, length).set(bytes)
    requireOk(core.omni_resource_write(resource.kind, resource.slot, BigInt(start), ptr, length), `stage ${resource.label}`)
  } finally {
    core.omni_free(ptr, length)
  }
}

async function prepareResources(caseDefinition, manifestDirectory) {
  const resources = []
  try {
    for (const [index, definition] of (caseDefinition.resources ?? []).entries()) {
      const label = definition.label ?? `resource ${index}`
      const hasPath = typeof definition.path === 'string' && definition.path.length > 0
      const hasFixture = typeof definition.fixture === 'string' && definition.fixture.length > 0
      invariant(hasPath !== hasFixture, `${caseDefinition.name}: ${label} must define exactly one of path or fixture`)

      let path = null
      let bytes = null
      let size
      let fd = null
      if (hasFixture) {
        bytes = compatibilityFixture(definition.fixture)
        size = bytes.byteLength
      } else {
        path = resolveResourcePath(manifestDirectory, definition.path)
        invariant(existsSync(path), `${caseDefinition.name}: ${label} does not exist`)
        const stat = statSync(path)
        invariant(stat.isFile(), `${caseDefinition.name}: ${label} is not a file`)
        invariant(Number.isSafeInteger(stat.size), `${caseDefinition.name}: ${label} exceeds Node file-size limits`)
        size = stat.size
        fd = openSync(path, 'r')
      }

      const kind = resourceKind(definition.kind)
      const slot = definition.slot ?? 0
      invariant(Number.isInteger(slot) && slot >= 0, `${caseDefinition.name}: ${label} has an invalid resource slot`)
      if (definition.sha256) {
        const actual = bytes ? sha256Bytes(bytes) : await sha256File(path)
        invariant(actual === String(definition.sha256).toLowerCase(), `${caseDefinition.name}: SHA-256 mismatch for ${label}`)
      }
      const streaming = definition.streaming ?? (kind === RESOURCE_KINDS.disc || kind === RESOURCE_KINDS.nand)
      const resource = { kind, slot, path, bytes, label, streaming, size, fd }
      resources.push(resource)
      if (streaming) {
        const cacheChunks = definition.cacheChunks ?? DEFAULT_STREAM_CACHE_CHUNKS
        invariant(Number.isInteger(cacheChunks) && cacheChunks > 0, `${caseDefinition.name}: ${label} has an invalid streaming cache size`)
        requireOk(core.omni_resource_create_streaming(kind, slot, BigInt(size), cacheChunks), `create ${resource.label}`)
        writeRange(resource, 0, Math.min(size, CHUNK_BYTES))
      } else {
        requireOk(core.omni_resource_create(kind, slot, BigInt(size)), `create ${resource.label}`)
        for (let offset = 0; offset < size; offset += CHUNK_BYTES) {
          writeRange(resource, offset, Math.min(size, offset + CHUNK_BYTES))
        }
      }
    }
    return resources
  } catch (error) {
    closeResources(resources)
    throw error
  }
}

function closeResources(resources) {
  for (const resource of resources) {
    if (resource.fd !== null) closeSync(resource.fd)
  }
}

function hydratePending(resources) {
  let hydrated = 0
  for (const resource of resources) {
    if (!resource.streaming) continue
    const startValue = core.omni_resource_pending_start(resource.kind, resource.slot)
    if (startValue === NO_PENDING_RANGE) continue
    const endValue = core.omni_resource_pending_end(resource.kind, resource.slot)
    invariant(endValue > startValue && endValue <= BigInt(resource.size), `${resource.label}: OmniCore requested an invalid range`)
    const requestedStart = Number(startValue)
    const requestedEnd = Number(endValue)
    invariant(Number.isSafeInteger(requestedStart) && Number.isSafeInteger(requestedEnd), `${resource.label}: pending range exceeds Node limits`)
    const start = Math.floor(requestedStart / CHUNK_BYTES) * CHUNK_BYTES
    const end = Math.min(resource.size, Math.max(requestedEnd, start + CHUNK_BYTES))
    writeRange(resource, start, end)
    hydrated++
  }
  return hydrated
}

function applyInput(caseDefinition) {
  for (let player = 0; player < 4; player++) {
    const definition = caseDefinition.input?.[player] ?? {}
    const buttons = definition.buttons == null ? 0n : BigInt(definition.buttons)
    requireOk(core.omni_set_input(player, buttons), `set player ${player + 1} buttons`)
    const axes = Array.isArray(definition.axes) ? definition.axes : []
    for (let axis = 0; axis < Math.min(axes.length, 8); axis++) {
      requireOk(core.omni_set_axis(player, axis, Number(axes[axis] ?? 0)), `set player ${player + 1} axis ${axis}`)
    }
  }
}

function runFrames(count, resources, caseDefinition) {
  for (let frame = 0; frame < count; frame++) {
    applyInput(caseDefinition)
    requireOk(core.omni_run_frame(), `run frame ${frame}`)
    let rounds = 0
    while (hydratePending(resources) > 0) {
      rounds++
      invariant(rounds <= 64, `${caseDefinition.name}: media hydration failed to converge`)
    }
  }
}

function saveState() {
  const length = core.omni_save_state(0, 0)
  invariant(length > 0, 'save state reports zero bytes')
  const ptr = core.omni_alloc(length)
  invariant(ptr > 0, `save-state allocation failed for ${length} bytes`)
  try {
    const written = core.omni_save_state(ptr, length)
    invariant(written === length, `save state wrote ${written}/${length} bytes`)
    return new Uint8Array(core.memory.buffer, ptr, length).slice()
  } finally {
    core.omni_free(ptr, length)
  }
}

function loadState(bytes) {
  const ptr = core.omni_alloc(bytes.byteLength)
  invariant(ptr > 0, `restore allocation failed for ${bytes.byteLength} bytes`)
  try {
    new Uint8Array(core.memory.buffer, ptr, bytes.byteLength).set(bytes)
    requireOk(core.omni_load_state(ptr, bytes.byteLength), 'restore state')
  } finally {
    core.omni_free(ptr, bytes.byteLength)
  }
}

function captureSurface() {
  const width = core.omni_video_width()
  const height = core.omni_video_height()
  const videoLength = core.omni_video_len()
  const video = videoLength > 0
    ? new Uint8Array(core.memory.buffer, core.omni_video_ptr(), videoLength).slice()
    : new Uint8Array()
  const audioSamples = core.omni_audio_len()
  const audio = audioSamples > 0
    ? new Uint8Array(core.memory.buffer, core.omni_audio_ptr(), audioSamples * Float32Array.BYTES_PER_ELEMENT).slice()
    : new Uint8Array()
  return {
    width,
    height,
    videoBytes: video.byteLength,
    videoNonzero: video.some((value) => value !== 0),
    videoSha256: sha256Bytes(video),
    audioRate: core.omni_audio_rate(),
    audioChannels: core.omni_audio_channels(),
    audioSamples,
    audioSha256: sha256Bytes(audio),
  }
}

function expectationFailures(actual, expected = {}) {
  const failures = []
  const compare = (key, actualValue, expectedValue) => {
    if (expectedValue !== undefined && actualValue !== expectedValue) failures.push(`${key}: expected ${expectedValue}, got ${actualValue}`)
  }
  compare('video.width', actual.width, expected.video?.width)
  compare('video.height', actual.height, expected.video?.height)
  compare('video.sha256', actual.videoSha256, expected.video?.sha256)
  compare('audio.rate', actual.audioRate, expected.audio?.rate)
  compare('audio.channels', actual.audioChannels, expected.audio?.channels)
  compare('audio.sha256', actual.audioSha256, expected.audio?.sha256)
  if (expected.video?.nonzero === true && !actual.videoNonzero) failures.push('video framebuffer is entirely zero')
  if (expected.video?.nonzero === false && actual.videoNonzero) failures.push('video framebuffer is unexpectedly nonzero')
  if (expected.audio?.minSamples !== undefined && actual.audioSamples < expected.audio.minSamples) {
    failures.push(`audio.samples: expected at least ${expected.audio.minSamples}, got ${actual.audioSamples}`)
  }
  if (expected.audio?.maxSamples !== undefined && actual.audioSamples > expected.audio.maxSamples) {
    failures.push(`audio.samples: expected at most ${expected.audio.maxSamples}, got ${actual.audioSamples}`)
  }
  return failures
}

async function runCase(definition, manifestDirectory) {
  invariant(typeof definition.name === 'string' && definition.name.length > 0, 'compatibility case requires a name')
  invariant(Number.isInteger(definition.platform) && definition.platform > 0, `${definition.name}: invalid platform id`)
  const minimumSupport = definition.supportAtLeast ?? 1
  const actualSupport = core.omni_support_level(definition.platform)
  invariant(core.omni_can_launch(definition.platform) === 1, `${definition.name}: platform ${definition.platform} is not launchable`)
  invariant(actualSupport >= minimumSupport, `${definition.name}: support level ${actualSupport} is below required ${minimumSupport}`)
  const frames = definition.frames ?? 60
  const replayFrames = definition.determinism?.replayFrames ?? 2
  invariant(Number.isInteger(frames) && frames >= 0, `${definition.name}: frames must be a nonnegative integer`)
  invariant(Number.isInteger(replayFrames) && replayFrames >= 0, `${definition.name}: replayFrames must be a nonnegative integer`)

  core.omni_resources_clear()
  const resources = await prepareResources(definition, manifestDirectory)
  try {
    requireOk(core.omni_load_staged(definition.platform), `load ${definition.name}`)
    const start = performance.now()
    runFrames(frames, resources, definition)
    const elapsedMs = performance.now() - start
    const surface = captureSurface()
    const failures = expectationFailures(surface, definition.expect)

    let deterministic = null
    if (definition.determinism?.enabled !== false && replayFrames > 0) {
      const state = saveState()
      runFrames(replayFrames, resources, definition)
      const first = captureSurface()
      loadState(state)
      runFrames(replayFrames, resources, definition)
      const second = captureSurface()
      const videoDeterministic = first.videoSha256 === second.videoSha256
      const audioDeterministic = first.audioSha256 === second.audioSha256
      deterministic = videoDeterministic && audioDeterministic
      if (!videoDeterministic) {
        failures.push(`save-state replay video mismatch: ${first.videoSha256} != ${second.videoSha256}`)
      }
      if (!audioDeterministic) {
        failures.push(`save-state replay audio mismatch: ${first.audioSha256} != ${second.audioSha256}`)
      }
    }

    const hostFramesPerSecond = elapsedMs > 0 ? frames * 1000 / elapsedMs : null
    if (definition.expect?.minHostFps !== undefined && hostFramesPerSecond < definition.expect.minHostFps) {
      failures.push(`host fps: expected at least ${definition.expect.minHostFps}, got ${hostFramesPerSecond.toFixed(2)}`)
    }

    const persistent = []
    for (const expected of definition.expect?.persistent ?? []) {
      const kind = resourceKind(expected.kind)
      const slot = expected.slot ?? 0
      const length = core.omni_persistent_len(kind, slot)
      persistent.push({ kind, slot, length })
      if (expected.length !== undefined && length !== expected.length) {
        failures.push(`persistent ${kind}:${slot}: expected ${expected.length} bytes, got ${length}`)
      }
    }

    return {
      name: definition.name,
      platform: definition.platform,
      supportLevel: actualSupport,
      frames,
      replayFrames,
      elapsedMs: Number(elapsedMs.toFixed(3)),
      hostFramesPerSecond: hostFramesPerSecond == null ? null : Number(hostFramesPerSecond.toFixed(3)),
      deterministic,
      surface,
      persistent,
      pass: failures.length === 0,
      failures,
    }
  } finally {
    core.omni_unload()
    closeResources(resources)
    core.omni_resources_clear()
  }
}

const manifest = parseManifest(manifestPath)
const manifestDirectory = dirname(manifestPath)
const results = []
for (const definition of manifest.cases) {
  try {
    results.push(await runCase(definition, manifestDirectory))
  } catch (error) {
    core.omni_unload()
    core.omni_resources_clear()
    results.push({
      name: definition?.name ?? '<unnamed>',
      platform: definition?.platform ?? null,
      pass: false,
      failures: [error instanceof Error ? error.message : String(error)],
    })
  }
}

const report = {
  schema: 'omniemu-compatibility-report/1',
  coreAbi: core.omni_core_version(),
  coreWasmSha256: await sha256File(wasmPath),
  cases: results,
  passed: results.filter((result) => result.pass).length,
  failed: results.filter((result) => !result.pass).length,
}

const serialized = `${JSON.stringify(report, null, 2)}\n`
if (reportPath) writeFileSync(reportPath, serialized)
process.stdout.write(serialized)
if (report.failed > 0) process.exitCode = 1
