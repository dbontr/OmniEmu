# OmniEmu Architecture

## Design constraints

OmniEmu is a static GitHub Pages application. There is no required application server, no ROM upload service, and no persistent backend. The host shell must stay small while emulator backends may be large, so all heavy runtimes are lazy-loaded per session.

The runtime is intentionally multi-backend. Console generations do not share one CPU, GPU, timing model, media model, or emulator implementation. The stable product surface is therefore the OmniEmu adapter contract, not a universal emulation core.

## Layers

1. **Shell** — navigation, local-file picking, system detection, status, fullscreen, settings.
2. **Catalog** — console generations, formats, support tier, required local system files, preferred adapter.
3. **Input profile** — one four-player logical control map persisted in browser local storage.
4. **Adapter registry** — resolves a selected console to a browser runtime implementation.
5. **Runtime sandbox** — one disposable iframe/session per game.
6. **Upstream emulator backend** — WebAssembly/RetroArch today; dedicated high-end adapters later.
7. **Browser services** — WebAssembly, WebGL/WebGPU, Web Audio, Gamepad API, IndexedDB, File/Blob APIs.

## Adapter contract

`src/runtime/adapter.ts` owns runtime selection. An adapter answers whether it supports a platform and creates a disposable session from a local game file, optional local system file, and the global input profile.

The first adapter is `libretro-wasm`, implemented by `src/runtime/emulatorjs.ts`. It creates an isolated iframe and loads a pinned runtime from the EmulatorJS CDN only after the user launches a compatible game.
## Input model

The shell exposes logical controls rather than console-specific USB/HID layouts. Player 1–4 mappings are stored once, then converted to the backend's control schema at session creation. This keeps keyboard and controller configuration global while allowing a backend to translate logical A/B/X/Y, shoulders, triggers, sticks, D-pad, Start/Select, and emulator hotkeys into its native input model.

Player 1 receives keyboard defaults. Players 2–4 default to gamepads only so one keyboard does not accidentally drive every controller port.

## Performance model

- Avoid large application frameworks and keep the host shell below the cost of one emulator core.
- Compile/run CPU emulation in WebAssembly whenever a backend supports it.
- Use WebGL 2 for established cores and allow WebGPU-specific adapters without changing the shell contract.
- Enable `SharedArrayBuffer`/WASM threads when cross-origin isolation is available.
- Destroy the previous iframe before launching another core so memory, audio nodes, event handlers, and global backend state are released together.
- Cache/persist only through browser-local facilities; never require uploading game images to an application server.

GitHub Pages does not expose arbitrary response-header configuration. `coi-serviceworker` is vendored as a same-origin service worker to establish the isolation required by threaded WebAssembly on compatible browsers. The UI reports the actual isolation state instead of assuming it succeeded.

## File model

Single-file cartridge images and single-file disc formats such as CHD/ISO are the simplest path. Multi-track CUE/BIN and directory-based games need a future `GameBundle` abstraction that preserves relative file names and can materialize an archive or virtual filesystem for the selected backend. Large media must be streamed or chunked rather than copied repeatedly through JavaScript heaps.

No commercial game content, proprietary system files, or console secrets belong in the repository.
