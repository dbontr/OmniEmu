import type { Platform } from '../catalog'
import type { InputProfile } from '../input/mapping'
import { toEmulatorJsControls } from '../input/mapping'

const DATA_ROOT = 'https://cdn.emulatorjs.org/4.2.3/data/'

export interface LaunchRequest {
  platform: Platform
  game: File
  bios?: File | null
  inputProfile: InputProfile
  autoStart?: boolean
}

export interface EmulatorSession {
  iframe: HTMLIFrameElement
  dispose(): void
}

const escapeScriptJson = (value: unknown) => JSON.stringify(value).replace(/</g, '\\u003c')

export function createEmulatorSession(host: HTMLElement, request: LaunchRequest): EmulatorSession {
  if (!request.platform.core) throw new Error(`${request.platform.name} does not have a browser core yet.`)

  const gameUrl = URL.createObjectURL(request.game)
  const biosUrl = request.bios ? URL.createObjectURL(request.bios) : ''
  const controls = toEmulatorJsControls(request.inputProfile)
  const iframe = document.createElement('iframe')
  iframe.className = 'emulator-frame'
  iframe.title = `${request.platform.name} emulator`
  iframe.allow = 'autoplay; fullscreen; gamepad; clipboard-write'
  iframe.setAttribute('allowfullscreen', '')

  const config = {
    core: request.platform.core,
    name: request.game.name.replace(/\.[^.]+$/, ''),
    gameUrl,
    biosUrl,
    threads: crossOriginIsolated && typeof SharedArrayBuffer !== 'undefined',
    controls,
    autoStart: request.autoStart ?? true,
  }

  iframe.srcdoc = buildFrameDocument(config)
  host.replaceChildren(iframe)

  return {
    iframe,
    dispose() {
      iframe.remove()
      URL.revokeObjectURL(gameUrl)
      if (biosUrl) URL.revokeObjectURL(biosUrl)
    },
  }
}
function buildFrameDocument(config: {
  core: string
  name: string
  gameUrl: string
  biosUrl: string
  threads: boolean
  controls: ReturnType<typeof toEmulatorJsControls>
  autoStart: boolean
}) {
  const payload = escapeScriptJson(config)
  return `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<style>
html,body,#game{width:100%;height:100%;margin:0;background:#050706;overflow:hidden}
body{font-family:system-ui,sans-serif;color:#fff}
#boot{position:fixed;inset:0;display:grid;place-items:center;background:#050706;color:#aeb7b0;font-size:12px;z-index:5}
</style>
</head>
<body>
<div id="game"></div><div id="boot">Preparing emulator…</div>
<script>
const cfg=${payload};
window.EJS_player='#game';
window.EJS_core=cfg.core;
window.EJS_gameName=cfg.name;
window.EJS_gameUrl=cfg.gameUrl;
window.EJS_biosUrl=cfg.biosUrl;
window.EJS_pathtodata='${DATA_ROOT}';
window.EJS_threads=cfg.threads;
window.EJS_defaultControls=cfg.controls;
window.EJS_startOnLoaded=cfg.autoStart;
window.EJS_color='#83bf6a';
window.EJS_backgroundColor='#050706';
window.EJS_alignStartButton='center';
window.EJS_AdTimer=-1;
window.EJS_askBeforeExit=false;
window.EJS_fixedSaveInterval=10000;
window.EJS_ready=function(){document.getElementById('boot')?.remove();parent.postMessage({type:'omniemu:ready'},'*')};
window.EJS_onGameStart=function(){parent.postMessage({type:'omniemu:started'},'*')};
window.EJS_onExit=function(){parent.postMessage({type:'omniemu:exit'},'*')};
window.addEventListener('error',function(event){parent.postMessage({type:'omniemu:error',message:event.message},'*')});
<\/script>
<script src="${DATA_ROOT}loader.js"><\/script>
</body>
</html>`
}
