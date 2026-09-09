# OmniEmu Roadmap

## Phase 0 — foundation — implemented

- GitHub Pages/Vite shell.
- Emu bird brand and mascot.
- Generation 1–8 platform catalog.
- Local game and BIOS selection.
- Extension-based system detection plus manual override.
- Pinned browser libretro adapter.
- Disposable iframe runtime isolation.
- Global four-player keyboard/gamepad profile.
- Thread/WebGL/WebGPU/gamepad capability reporting.
- Cross-origin-isolation service worker.
- Automated GitHub Pages deployment.
- Brand-isolation verification.

## Phase 1 — make generations 2–5 boringly reliable

- Verify every advertised core with legal test/homebrew software.
- Add per-system preferred-core overrides and fallback cores.
- Add ZIP/7z handling and a `GameBundle` abstraction for CUE/BIN and multi-file discs.
- Add persistent game library handles where the File System Access API is available, with IndexedDB metadata fallback.
- Unify save RAM/state import/export and add automatic crash-safe save checkpoints.
- Build controller capture/calibration, dead-zone settings, per-device aliases, rumble routing, and hot-plug recovery.

## Phase 2 — generation 6 first

1. **Dreamcast:** integrate and reproducibly build the 2026 Flycast WebAssembly JIT as a dedicated adapter; target full-speed WebGL2 first, then WebGPU experiments.
2. **PlayStation 2:** integrate the Play! browser build behind its own adapter and compatibility gate; do not claim universal PS2 compatibility until representative suites pass.
3. **GameCube/Wii:** evaluate and harden a Dolphin WebAssembly/WebGPU path before exposing it as runnable.
4. **Original Xbox:** track xemu and related research; keep roadmap-only until a credible browser backend exists.
## Phase 3 — generation 7

- Xbox 360: research Xenia-to-WebAssembly/WebGPU feasibility; no false compatibility claims.
- PlayStation 3: research RPCS3 browser feasibility and the cost of translating its JIT/Vulkan assumptions.
- Wii: share the GameCube adapter family once the Dolphin browser path is production-grade.

## Phase 4 — generation 8

- Wii U and Switch: lower priority than making generations 1–7 excellent; require reproducible browser backends and user-owned system material.
- PlayStation 4 and Xbox One: intentionally lowest priority. Keep them in the catalog/adapter contract so the architecture does not need redesign if viable browser cores emerge.

## Phase 5 — first-generation hardware simulations

Many first-generation machines do not load ROM cartridges. Add dedicated timing-accurate simulations for representative Odyssey/Pong/Telstar-style hardware and expose their game/mode selectors through the same launch surface.

## Acceptance bar for `ready`

A console may move to `ready` only after the adapter has deterministic builds, a pinned upstream revision, legal regression software, crash recovery, global input integration, save persistence where applicable, representative compatibility evidence, and acceptable frame/audio pacing on current desktop Chromium and Firefox. Safari/iOS support is measured separately because browser JIT/thread/GPU constraints differ.
