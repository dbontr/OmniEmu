const CUE_MAGIC = new TextEncoder().encode('OMCUE001')
const MAX_CUE_BYTES = 1024 * 1024
const MAX_CUE_FILES = 99
const encoder = new TextEncoder()
const utf8Decoder = new TextDecoder('utf-8', { fatal: true })
const legacyDecoder = new TextDecoder('windows-1252')

export interface PreparedGameFile {
  file: File
  sourceFiles: number
}

const normalizeName = (name: string) => name.replace(/\\/g, '/').split('/').pop()?.toLowerCase() ?? name.toLowerCase()
const isCue = (file: File) => file.name.toLowerCase().endsWith('.cue')

function cueReferences(text: string) {
  const references: string[] = []
  for (const rawLine of text.split(/\r?\n/)) {
    const line = rawLine.trim().replace(/^\uFEFF/, '')
    if (!/^FILE\s/i.test(line)) continue
    const rest = line.slice(4).trim()
    const match = rest.match(/^"([^"]+)"\s+\S+/) ?? rest.match(/^(\S+)\s+\S+/)
    if (!match?.[1]) throw new Error(`Invalid CUE FILE directive: ${line}`)
    references.push(match[1])
  }
  return references
}
function selectedTrackFiles(files: readonly File[], references: readonly string[], cueName: string) {
  const byName = new Map<string, File>()
  for (const file of files) {
    if (isCue(file)) continue
    const key = normalizeName(file.name)
    if (byName.has(key)) throw new Error(`Two selected track files have the same name: ${file.name}`)
    byName.set(key, file)
  }

  const ordered: File[] = []
  const seen = new Set<string>()
  for (const reference of references) {
    const key = normalizeName(reference)
    if (seen.has(key)) continue
    const file = byName.get(key)
    if (!file) throw new Error(`The CUE sheet references ${reference}, but that file was not selected.`)
    if (!file.size) throw new Error(`The CUE track file ${file.name} is empty.`)
    ordered.push(file)
    seen.add(key)
  }
  if (!ordered.length) throw new Error('The CUE sheet does not reference any track files.')

  const sbiFiles = files.filter((file) => file.name.toLowerCase().endsWith('.sbi'))
  if (sbiFiles.length > 1) throw new Error('Select at most one SBI subchannel file per CUE sheet.')
  if (sbiFiles.length === 1) {
    const sbi = sbiFiles[0]
    const cueStem = normalizeName(cueName).replace(/\.cue$/, '')
    const sbiStem = normalizeName(sbi.name).replace(/\.sbi$/, '')
    if (cueStem !== sbiStem) throw new Error(`SBI file ${sbi.name} must have the same base name as ${cueName}.`)
    if (!sbi.size) throw new Error(`The SBI file ${sbi.name} is empty.`)
    ordered.push(sbi)
  }

  if (ordered.length > MAX_CUE_FILES) throw new Error(`CUE sets are limited to ${MAX_CUE_FILES} companion files.`)
  return ordered
}
function makeCueContainer(cue: File, cueBytes: Uint8Array, tracks: readonly File[]) {
  const names = tracks.map((file) => encoder.encode(file.name))
  if (names.some((name) => !name.length || name.length > 0xffff)) {
    throw new Error('A CUE track filename is too long for OmniCore.')
  }
  const directoryBytes = names.reduce((total, name) => total + 2 + name.length + 16, 0)
  const dataStart = 16 + cueBytes.length + directoryBytes
  const header = new Uint8Array(dataStart)
  const view = new DataView(header.buffer)
  header.set(CUE_MAGIC, 0)
  view.setUint32(8, cueBytes.length, true)
  view.setUint32(12, tracks.length, true)
  header.set(cueBytes, 16)

  let directoryOffset = 16 + cueBytes.length
  let dataOffset = BigInt(dataStart)
  for (let index = 0; index < tracks.length; index++) {
    const file = tracks[index]
    const name = names[index]
    view.setUint16(directoryOffset, name.length, true)
    directoryOffset += 2
    header.set(name, directoryOffset)
    directoryOffset += name.length
    view.setBigUint64(directoryOffset, BigInt(file.size), true)
    view.setBigUint64(directoryOffset + 8, dataOffset, true)
    directoryOffset += 16
    dataOffset += BigInt(file.size)
  }

  const lastModified = Math.max(cue.lastModified, ...tracks.map((track) => track.lastModified))
  return new File([header, ...tracks], cue.name, {
    type: 'application/x-omni-cue',
    lastModified,
  })
}

export async function prepareGameFiles(files: readonly File[]): Promise<PreparedGameFile> {
  if (!files.length) throw new Error('No game file was selected.')
  const cues = files.filter(isCue)
  if (!cues.length) {
    if (files.length !== 1) throw new Error('Select one game file, or one CUE sheet with all referenced track files.')
    return { file: files[0], sourceFiles: 1 }
  }
  if (cues.length !== 1) throw new Error('Select exactly one CUE sheet at a time.')
  const cue = cues[0]
  if (!cue.size || cue.size > MAX_CUE_BYTES) {
    throw new Error(`CUE sheets must be between 1 byte and ${MAX_CUE_BYTES} bytes.`)
  }
  const sourceBytes = new Uint8Array(await cue.arrayBuffer())
  let cueText: string
  let cueBytes = sourceBytes
  try {
    cueText = utf8Decoder.decode(sourceBytes)
  } catch {
    cueText = legacyDecoder.decode(sourceBytes)
    cueBytes = encoder.encode(cueText)
  }
  if (cueBytes.length > MAX_CUE_BYTES) throw new Error('The normalized CUE sheet is too large.')
  const references = cueReferences(cueText)
  const tracks = selectedTrackFiles(files, references, cue.name)
  return {
    file: makeCueContainer(cue, cueBytes, tracks),
    sourceFiles: tracks.length + 1,
  }
}
