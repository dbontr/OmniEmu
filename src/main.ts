import './styles.css'
import { PLATFORMS, candidatesForFile, platformById, type Platform } from './catalog'
import { launchOmniCore, runtimeAvailable, type OmniSession } from './runtime/omnicore'
import { defaultProfile, loadInputProfile, primaryBindings, saveInputProfile, type InputProfile } from './input/mapping'

type MessageKind = 'idle' | 'loading' | 'ready' | 'running' | 'error'

interface AppState {
  platform: Platform
  game: File | null
  bios: File | null
  session: OmniSession | null
  input: InputProfile
  message: string
  messageKind: MessageKind
}

const firstRunnable = PLATFORMS.find(runtimeAvailable) ?? PLATFORMS[0]
const state: AppState = {
  platform: firstRunnable,
  game: null,
  bios: null,
  session: null,
  input: loadInputProfile(),
  message: 'Drop a game file or choose one from disk.',
  messageKind: 'idle',
}

const app = document.querySelector<HTMLDivElement>('#app')!
app.innerHTML = shellMarkup()

const el = <T extends Element>(selector: string) => document.querySelector<T>(selector)!
const stage = el<HTMLElement>('#player-stage')
const host = el<HTMLElement>('#emulator-host')
const idle = el<HTMLElement>('#player-idle')
const inspector = el<HTMLElement>('#inspector')
const gameInput = el<HTMLInputElement>('#game-input')
const biosInput = el<HTMLInputElement>('#bios-input')
const platformSelect = el<HTMLSelectElement>('#platform-select')

renderPlatformSidebar()
renderPlatformSelect()
refreshUi()

window.addEventListener('gamepadconnected', refreshCapabilities)
window.addEventListener('gamepaddisconnected', refreshCapabilities)
window.addEventListener('omnicore:error', (event) => {
  const detail = (event as CustomEvent<{ message?: string }>).detail
  state.messageKind = 'error'
  state.message = detail?.message || 'OmniCore stopped because of an emulation error.'
  refreshPlayerBar()
})
function shellMarkup() {
  const mascot = `${import.meta.env.BASE_URL}emu.svg`
  return `
  <div class="app-shell">
    <header class="topbar">
      <div class="brand"><img src="${mascot}" alt=""><div><strong>OmniEmu</strong><small>one core · every console</small></div></div>
      <label class="button file-button">Open game<input id="game-input" type="file"></label>
      <button class="button" id="details-button" type="button"><span class="long">Game &amp; system </span>details</button>
      <div class="topbar-actions">
        <button class="button" id="controls-button" type="button">Controllers</button>
        <button class="button" id="fullscreen-button" type="button">Fullscreen</button>
      </div>
    </header>
    <div class="workspace">
      <aside class="sidebar" aria-label="Console generations">
        <div class="sidebar-title">Console generations</div>
        <div class="support-legend"><span class="r"><i></i>Playable</span><span class="e"><i></i>Core foundation</span><span><i></i>Planned</span></div>
        <div id="generation-list"></div>
      </aside>
      <main class="player-column">
        <section class="player-stage" id="player-stage">
          <div class="emulator-host hidden" id="emulator-host"></div>
          <div class="player-empty" id="player-idle">
            <img src="${mascot}" alt="OmniEmu's emu mascot">
            <h1>Load a game.</h1>
            <p>Games stay on this device. One OmniCore WebAssembly engine owns the machine, timing, video, audio, saves, and global controller model across every console.</p>
            <label class="drop-zone" id="drop-zone"><strong>Drop a ROM, disc image, or archive here</strong><span>or click to choose a local game</span><input class="sr-only" id="drop-input" type="file"></label>
          </div>
        </section>
        <footer class="player-bar">
          <div class="player-game"><strong id="player-title">No game loaded</strong><small id="player-status">Waiting for a local game file</small></div>
          <div class="actions"><button class="button primary" id="launch-button" type="button" disabled>Launch</button><button class="button danger" id="stop-button" type="button" disabled>Stop</button></div>
        </footer>
      </main>
      <aside class="inspector" id="inspector" aria-label="Game and system details">
        <section><div class="selection-summary"><strong id="system-name"></strong><small id="system-meta"></small></div><div class="chip-row" id="system-chips"></div><div id="system-note"></div></section>
        <section>
          <h2>Game</h2>
          <div class="field"><label for="platform-select">Console</label><select id="platform-select"></select></div>
          <div class="field"><label>Selected file</label><p id="game-file">No game selected</p></div>
          <div class="field" id="bios-field"><label>BIOS / firmware</label><label class="button file-button"><span id="bios-label">Choose BIOS file</span><input id="bios-input" type="file"></label></div>
          <div id="detection-note"></div>
        </section>
        <section><h2>Acceleration</h2><div class="status-grid" id="capabilities"></div></section>
        <section><h2>Privacy</h2><p>Game and BIOS files stay local. OmniEmu copies them directly into the single OmniCore WebAssembly memory and never uploads them.</p></section>
      </aside>
    </div>
    <div id="modal-root"></div>
  </div>`
}
function renderPlatformSidebar() {
  const root = el<HTMLElement>('#generation-list')
  root.innerHTML = Array.from({ length: 8 }, (_, index) => index + 1).map((generation) => {
    const platforms = PLATFORMS.filter((platform) => platform.generation === generation)
    const playable = platforms.filter((platform) => platform.tier === 'playable').length
    return `<section class="generation"><button type="button"><span>Generation ${generation}</span><small>${playable}/${platforms.length} playable</small></button><div class="platform-list">${platforms.map((platform) => platformButton(platform)).join('')}</div></section>`
  }).join('')
  root.querySelectorAll<HTMLButtonElement>('[data-platform]').forEach((button) => button.addEventListener('click', () => selectPlatform(button.dataset.platform ?? '')))
}
function platformButton(platform: Platform) {
  return `<button class="platform-row ${platform.tier}" data-platform="${platform.id}" type="button" title="${escapeHtml(platform.vendor)} · ${escapeHtml(platform.years)}"><span>${escapeHtml(platform.name)}</span><i class="dot" aria-hidden="true"></i></button>`
}

function renderPlatformSelect() {
  platformSelect.innerHTML = PLATFORMS.map((platform) => `<option value="${platform.id}">Gen ${platform.generation} · ${escapeHtml(platform.name)}${platform.tier === 'playable' ? '' : ' — ' + platform.tier}</option>`).join('')
  platformSelect.addEventListener('change', () => selectPlatform(platformSelect.value))
}
function selectPlatform(id: string) {
  const platform = platformById(id)
  if (!platform) return
  state.platform = platform
  state.bios = null
  biosInput.value = ''
  if (state.game) state.message = `${platform.name} selected for ${state.game.name}.`
  refreshUi()
}
const dropInput = el<HTMLInputElement>('#drop-input')
const dropZone = el<HTMLElement>('#drop-zone')

gameInput.addEventListener('change', () => gameInput.files?.[0] && chooseGame(gameInput.files[0]))
dropInput.addEventListener('change', () => dropInput.files?.[0] && chooseGame(dropInput.files[0]))
biosInput.addEventListener('change', () => {
  state.bios = biosInput.files?.[0] ?? null
  state.message = state.bios ? `${state.bios.name} selected for ${state.platform.name}.` : 'Firmware selection cleared.'
  refreshUi()
})

dropZone.addEventListener('dragover', (event) => { event.preventDefault(); dropZone.classList.add('drag') })
dropZone.addEventListener('dragleave', () => dropZone.classList.remove('drag'))
dropZone.addEventListener('drop', (event) => {
  event.preventDefault()
  dropZone.classList.remove('drag')
  const file = event.dataTransfer?.files?.[0]
  if (file) chooseGame(file)
})
function chooseGame(file: File) {
  stopSession(false)
  state.game = file
  const candidates = candidatesForFile(file.name)
  const runnable = candidates.find((platform) => runtimeAvailable(platform))
  if (runnable) state.platform = runnable
  state.bios = null
  biosInput.value = ''
  state.messageKind = 'idle'
  state.message = candidates.length > 1
    ? `${file.name} matches ${candidates.length} systems. Verify the console before launch.`
    : candidates.length === 1
      ? `${candidates[0].name} detected from the file type.`
      : 'File type was not recognized. Choose the console manually.'
  refreshUi()
}
function refreshUi() {
  document.querySelectorAll<HTMLElement>('[data-platform]').forEach((row) => row.classList.toggle('active', row.dataset.platform === state.platform.id))
  platformSelect.value = state.platform.id
  el<HTMLElement>('#system-name').textContent = state.platform.name
  el<HTMLElement>('#system-meta').textContent = `Generation ${state.platform.generation} · ${state.platform.vendor} · ${state.platform.years}`
  el<HTMLElement>('#game-file').textContent = state.game ? `${state.game.name} · ${formatBytes(state.game.size)}` : 'No game selected'
  el<HTMLElement>('#bios-label').textContent = state.bios ? state.bios.name : 'Choose local BIOS file'

  const chipRoot = el<HTMLElement>('#system-chips')
  chipRoot.innerHTML = `<span class="chip ${state.platform.tier}">${state.platform.tier}</span>${state.platform.extensions.slice(0, 5).map((ext) => `<span class="chip">.${ext}</span>`).join('')}`

  const biosField = el<HTMLElement>('#bios-field')
  biosField.classList.toggle('hidden', !state.platform.bios)
  const note = el<HTMLElement>('#system-note')
  note.innerHTML = platformNotice(state.platform)
  updateDetectionNote()
  refreshCapabilities()
  refreshPlayerBar()
}
function platformNotice(platform: Platform) {
  const parts: string[] = []
  if (platform.tier === 'planned') parts.push('This machine is assigned a permanent OmniCore platform id, but its hardware graph is not implemented yet. Launch remains disabled until it runs inside the same OmniCore binary.')
  if (platform.tier === 'foundation') parts.push('The shared OmniCore CPU/kernel substrate for this machine family exists, but the complete console hardware graph is not playable yet.')
  if (platform.bios === 'required') parts.push('This console requires a local BIOS/firmware file supplied by the user.')
  if (platform.note) parts.push(platform.note)
  return parts.length ? `<div class="notice ${platform.tier === 'planned' ? 'warning' : ''}">${parts.map(escapeHtml).join(' ')}</div>` : ''
}

function updateDetectionNote() {
  const root = el<HTMLElement>('#detection-note')
  if (!state.game) { root.innerHTML = ''; return }
  const candidates = candidatesForFile(state.game.name)
  if (candidates.length <= 1) { root.innerHTML = ''; return }
  const names = candidates.slice(0, 5).map((candidate) => candidate.name).join(', ')
  root.innerHTML = `<div class="notice">Ambiguous file extension. Possible systems: ${escapeHtml(names)}${candidates.length > 5 ? ', …' : ''}</div>`
}
function refreshCapabilities() {
  const root = document.querySelector<HTMLElement>('#capabilities')
  if (!root) return
  const webgl2 = !!document.createElement('canvas').getContext('webgl2')
  const gamepads = navigator.getGamepads?.().filter(Boolean).length ?? 0
  const checks: Array<[string, boolean, string]> = [
    ['OmniCore WASM', typeof WebAssembly !== 'undefined', 'single binary'],
    ['WebGL 2', webgl2, webgl2 ? 'hardware rendering' : 'legacy fallback'],
    ['WASM threads', crossOriginIsolated && typeof SharedArrayBuffer !== 'undefined', crossOriginIsolated ? 'available' : 'single-thread fallback'],
    ['WebGPU API', 'gpu' in navigator, 'future unified GPU path'],
    ['Gamepads', gamepads > 0, gamepads ? `${gamepads} connected` : 'none connected'],
  ]
  root.innerHTML = checks.map(([label, ok, detail]) => `<span>${label}</span><strong class="${ok ? 'good' : 'warn'}">${escapeHtml(detail)}</strong>`).join('')
}

function refreshPlayerBar() {
  el<HTMLElement>('#player-title').textContent = state.game ? state.game.name.replace(/\.[^.]+$/, '') : state.platform.romless ? state.platform.name : 'No game loaded'
  el<HTMLElement>('#player-status').textContent = state.message
  const hasGame = !!state.game || state.platform.romless === true
  const canLaunch = hasGame && runtimeAvailable(state.platform) && (state.platform.bios !== 'required' || !!state.bios)
  el<HTMLButtonElement>('#launch-button').disabled = !canLaunch || state.messageKind === 'loading'
  el<HTMLButtonElement>('#stop-button').disabled = !state.session
}
el<HTMLButtonElement>('#launch-button').addEventListener('click', launchSelectedGame)
el<HTMLButtonElement>('#stop-button').addEventListener('click', () => stopSession(true))
el<HTMLButtonElement>('#details-button').addEventListener('click', () => inspector.classList.toggle('open'))
el<HTMLButtonElement>('#controls-button').addEventListener('click', openControllerModal)
el<HTMLButtonElement>('#fullscreen-button').addEventListener('click', async () => {
  try {
    if (document.fullscreenElement) await document.exitFullscreen()
    else await stage.requestFullscreen()
  } catch {
    state.message = 'Fullscreen is unavailable in this browser context.'
    refreshPlayerBar()
  }
})
async function launchSelectedGame() {
  if ((!state.game && !state.platform.romless) || !runtimeAvailable(state.platform)) return
  if (state.platform.bios === 'required' && !state.bios) {
    state.messageKind = 'error'
    state.message = `${state.platform.name} requires a local BIOS/firmware file before launch.`
    refreshPlayerBar()
    return
  }

  stopSession(false)
  state.messageKind = 'loading'
  state.message = `Starting ${state.platform.name} in OmniCore…`
  idle.classList.add('hidden')
  host.classList.remove('hidden')
  try {
    state.session = await launchOmniCore(host, { platform: state.platform, game: state.game, bios: state.bios, inputProfile: state.input })
    state.messageKind = 'running'
    state.message = `${state.platform.name} running inside the single OmniCore WebAssembly engine.`
  } catch (error) {
    host.classList.add('hidden')
    idle.classList.remove('hidden')
    state.messageKind = 'error'
    state.message = error instanceof Error ? error.message : String(error)
  }
  refreshPlayerBar()
}

function stopSession(showMessage: boolean) {
  state.session?.dispose()
  state.session = null
  host.replaceChildren()
  host.classList.add('hidden')
  idle.classList.remove('hidden')
  if (showMessage) {
    state.messageKind = 'idle'
    state.message = state.game ? `${state.game.name} is ready to launch again.` : 'Waiting for a local game file.'
  }
  refreshPlayerBar()
}
const GAMEPAD_OPTIONS = [
  '', 'BUTTON_1', 'BUTTON_2', 'BUTTON_3', 'BUTTON_4',
  'LEFT_TOP_SHOULDER', 'RIGHT_TOP_SHOULDER', 'LEFT_BOTTOM_SHOULDER', 'RIGHT_BOTTOM_SHOULDER',
  'SELECT', 'START', 'LEFT_STICK', 'RIGHT_STICK', 'DPAD_UP', 'DPAD_DOWN', 'DPAD_LEFT', 'DPAD_RIGHT',
  'LEFT_STICK_X:+1', 'LEFT_STICK_X:-1', 'LEFT_STICK_Y:+1', 'LEFT_STICK_Y:-1',
  'RIGHT_STICK_X:+1', 'RIGHT_STICK_X:-1', 'RIGHT_STICK_Y:+1', 'RIGHT_STICK_Y:-1',
]

function openControllerModal() {
  const root = el<HTMLElement>('#modal-root')
  let draft = structuredClone(state.input)
  let player = 0
  root.innerHTML = controllerModalMarkup(player, draft)
  bindModal()

  function bindModal() {
    root.querySelector<HTMLButtonElement>('[data-close]')?.addEventListener('click', close)
    root.querySelector<HTMLButtonElement>('[data-cancel]')?.addEventListener('click', close)
    root.querySelector<HTMLSelectElement>('#mapping-player')?.addEventListener('change', (event) => {
      player = Number((event.target as HTMLSelectElement).value)
      root.innerHTML = controllerModalMarkup(player, draft)
      bindModal()
    })
    root.querySelector<HTMLButtonElement>('[data-reset]')?.addEventListener('click', () => {
      draft.players[player] = structuredClone(defaultProfile().players[player])
      root.innerHTML = controllerModalMarkup(player, draft)
      bindModal()
    })
    root.querySelector<HTMLButtonElement>('[data-save]')?.addEventListener('click', () => {
      state.input = draft
      saveInputProfile(state.input)
      state.message = 'Global controller mapping saved. It will apply to every new emulator session.'
      refreshPlayerBar()
      close()
    })
    bindMappingRows()
  }
  function bindMappingRows() {
    root.querySelectorAll<HTMLInputElement>('[data-key-index]').forEach((input) => {
      input.addEventListener('keydown', (event) => {
        event.preventDefault()
        const index = Number(input.dataset.keyIndex)
        const binding = draft.players[player]?.find((item) => item.index === index)
        if (!binding) return
        binding.keyboard = event.key === 'Backspace' || event.key === 'Delete' ? '' : normalizeKey(event.key)
        input.value = binding.keyboard || '—'
      })
    })
    root.querySelectorAll<HTMLSelectElement>('[data-pad-index]').forEach((select) => {
      select.addEventListener('change', () => {
        const index = Number(select.dataset.padIndex)
        const binding = draft.players[player]?.find((item) => item.index === index)
        if (binding) binding.gamepad = select.value
      })
    })
  }

  function close() { root.replaceChildren() }
}

function normalizeKey(key: string) {
  if (key === ' ') return ' '
  if (key === 'Shift') return 'shift'
  return key.toLowerCase()
}
function controllerModalMarkup(player: number, profile: InputProfile) {
  const rows = primaryBindings(profile, player).map((binding) => {
    const options = GAMEPAD_OPTIONS.map((option) => `<option value="${escapeAttr(option)}"${option === binding.gamepad ? ' selected' : ''}>${option || 'Unmapped'}</option>`).join('')
    return `<div class="mapping-row"><strong>${escapeHtml(binding.label)}</strong><input data-key-index="${binding.index}" value="${escapeAttr(binding.keyboard || '—')}" readonly title="Press a key; Backspace clears"><select data-pad-index="${binding.index}">${options}</select></div>`
  }).join('')
  return `<div class="modal-backdrop"><section class="modal" role="dialog" aria-modal="true" aria-labelledby="mapping-title">
    <header><div><h2 id="mapping-title">Global controller mapping</h2><p>One logical controller profile is translated by OmniCore across every console machine.</p></div><button class="button close" type="button" data-close>Close</button></header>
    <div class="mapping-table">
      <div class="field"><label for="mapping-player">Player</label><select id="mapping-player">${[0,1,2,3].map((index) => `<option value="${index}"${index === player ? ' selected' : ''}>Player ${index + 1}</option>`).join('')}</select></div>
      <div class="mapping-head"><span>Logical control</span><span>Keyboard</span><span>Gamepad</span></div>${rows}
    </div>
    <footer><button class="button" type="button" data-reset>Reset player</button><button class="button" type="button" data-cancel>Cancel</button><button class="button primary" type="button" data-save>Save global mapping</button></footer>
  </section></div>`
}

function formatBytes(bytes: number) {
  if (bytes < 1024) return `${bytes} B`
  if (bytes < 1024 ** 2) return `${(bytes / 1024).toFixed(1)} KiB`
  if (bytes < 1024 ** 3) return `${(bytes / 1024 ** 2).toFixed(1)} MiB`
  return `${(bytes / 1024 ** 3).toFixed(2)} GiB`
}

function escapeHtml(value: string) { return value.replace(/[&<>'"]/g, (char) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;' }[char] ?? char)) }
function escapeAttr(value: string) { return escapeHtml(value) }
