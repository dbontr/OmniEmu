# OmniEmu

OmniEmu is a GitHub Pages-first, local-file, all-in-one browser emulation shell. Its goal is one fast interface for console generations 1 through 8, with a single controller profile, automatic system detection, on-demand emulator backends, and no game-content hosting.

The mascot is an emu bird. The geometric emu artwork in `public/emu.svg` is the project mark.

## What works in the first foundation

- Static GitHub Pages deployment with a very small TypeScript/Vite shell.
- Local ROM/disc-image selection and drag/drop. Game files are opened as browser object URLs rather than uploaded by OmniEmu.
- Automatic console candidates from file extension plus manual console override.
- A pinned EmulatorJS 4.2.3 iframe adapter for supported WebAssembly/RetroArch cores.
- Browser acceleration detection for WebAssembly, WebGL 2, cross-origin-isolated WASM threads, WebGPU availability, and connected gamepads.
- Global 4-player controller mapping stored once in `localStorage` and supplied to every new emulator session.
- Optional/required BIOS selection kept local to the browser session.
- Console catalog spanning generations 1–8, including explicit roadmap entries when no credible browser backend is connected yet.
- A branding verification check that prevents legacy project naming from entering this repository.

## Current backend coverage

The initial production adapter targets cartridge and fifth-generation systems already exposed through established browser cores: Atari 2600/5200/7800, ColecoVision, NES, Master System, SNES, Genesis/Mega Drive, Sega CD/32X, PC Engine/TurboGrafx-16, Neo Geo-class arcade cores, PlayStation, Nintendo 64, Saturn, Jaguar, and 3DO.

Not every game supported by a native emulator is guaranteed to work in a browser. Compatibility depends on the upstream core, browser, device performance, game format, and required user-supplied system files.
## Why generations 6–8 are different

The browser shell is not the limiting problem for later consoles. The limiting problem is the availability of fast, accurate browser-native emulator backends. Dreamcast now has a credible 2026 WebAssembly JIT project, and Play! has an experimental PlayStation 2 web build, so those are the first high-end adapters to pursue. GameCube/Wii require a mature browser Dolphin path. Original Xbox, Xbox 360, PlayStation 3, Wii U, PlayStation 4, Xbox One, and Switch do not currently have equivalent production browser backends that OmniEmu can responsibly label universal.

OmniEmu therefore treats console support as an adapter contract rather than pretending one emulator core can cover every machine. A console becomes `ready` only when its adapter is reproducible, legal test content boots, input works through the global mapping layer, saves survive reloads, and representative games meet performance/correctness gates.

First-generation consoles are also unusual: many are dedicated analog/digital game machines with no conventional ROM cartridge. They belong in OmniEmu as hardware simulations rather than as ROM loaders.

## Architecture

```text
local file(s)
    ↓
format detector → console resolver → runtime adapter registry
                                      ↓
                 ┌────────────────────┼────────────────────┐
                 │                    │                    │
          libretro/WASM       Dreamcast WASM       dedicated adapters
                 │                    │                    │
                 └────────── global input bus ─────────────┘
                                      ↓
                         canvas/audio/save services
```

The main page never embeds all emulators. It loads the selected backend on demand inside a disposable iframe, so switching consoles does not leak runtime globals or keep unnecessary WebAssembly heaps alive.
## Development

Requires Node.js 24+.

```bash
npm install
npm run dev
npm run verify
```

`npm run verify` checks project-brand isolation, type-checks, and produces the GitHub Pages build.

## Performance strategy

- WebAssembly cores instead of JavaScript CPU emulation for performance-critical systems.
- WebGL 2 rendering today; adapter contract leaves room for WebGPU-native renderers.
- Cross-origin isolation through a same-origin service worker on GitHub Pages so supported cores can use `SharedArrayBuffer`/WASM threads.
- On-demand core loading and disposable iframe isolation.
- No framework runtime in the shell; the production app bundle is intentionally small.
- Global input mapping is resolved before launch rather than rebuilt independently in each console UI.

## Content policy

OmniEmu does not ship commercial games, proprietary BIOS/firmware, console keys, or other protected system files. Users are responsible for providing game/system files they are permitted to use. The project is an emulator frontend and compatibility layer, not a game-content repository.

See `ARCHITECTURE.md`, `ROADMAP.md`, and `REFERENCES.md` for the implementation plan and upstream projects under evaluation.
