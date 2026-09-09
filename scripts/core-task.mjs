import { copyFileSync, existsSync } from 'node:fs'
import { spawnSync } from 'node:child_process'
import { dirname, join } from 'node:path'
import { fileURLToPath } from 'node:url'

const root = dirname(dirname(fileURLToPath(import.meta.url)))
const task = process.argv[2] ?? 'build'
const allowed = new Set(['build', 'test', 'fmt', 'clippy'])
if (!allowed.has(task)) throw new Error(`Unknown OmniCore task: ${task}`)

const manifest = join(root, 'core', 'Cargo.toml')
const output = join(root, 'public', 'omnicore.wasm')
const built = join(root, 'core', 'target', 'wasm32-unknown-unknown', 'release', 'omnicore.wasm')

function cargoArgs(manifestPath) {
  if (task === 'test') return ['test', '--manifest-path', manifestPath]
  if (task === 'fmt') return ['fmt', '--manifest-path', manifestPath, '--', '--check']
  if (task === 'clippy') return ['clippy', '--manifest-path', manifestPath, '--all-targets', '--', '-D', 'warnings']
  return ['build', '--manifest-path', manifestPath, '--release', '--target', 'wasm32-unknown-unknown']
}

const localCargo = spawnSync('cargo', ['--version'], { stdio: 'ignore', shell: false })
let result
if (localCargo.status === 0) {
  result = spawnSync('cargo', cargoArgs(manifest), { cwd: root, stdio: 'inherit', shell: false })
} else if (process.platform === 'win32') {
  const match = /^([A-Za-z]):[\\/](.*)$/.exec(root)
  if (!match) throw new Error(`Cannot translate Windows path for WSL: ${root}`)
  const wslRoot = `/mnt/${match[1].toLowerCase()}/${match[2].replaceAll('\\', '/')}`
  const wslManifest = `${wslRoot}/core/Cargo.toml`
  const args = cargoArgs(wslManifest).map((arg) => `'${arg}'`).join(' ')
  const command = `if [ -f "$HOME/.cargo/env" ]; then . "$HOME/.cargo/env"; fi; cargo ${args}`
  result = spawnSync('wsl', ['sh', '-lc', command], { cwd: root, stdio: 'inherit', shell: false })
} else {
  throw new Error('Cargo is required to build OmniCore.')
}

if (result.status !== 0) process.exit(result.status ?? 1)
if (task === 'build') {
  if (!existsSync(built)) throw new Error(`OmniCore build did not produce ${built}`)
  copyFileSync(built, output)
  console.log(`OmniCore WASM: ${output}`)
}
