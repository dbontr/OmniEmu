# OmniEmu

OmniEmu is an experimental browser emulator built around one runtime rule:

> Every console is implemented inside the same `omnicore.wasm` emulator binary.

The browser does not download or select a different emulator core per console. Platform selection is data passed into OmniCore. CPU engines, device models, buses, timing, video, audio, media handling, input, and state logic are compiled together and share one ABI.

The emu bird is OmniEmu's mascot and project mark.

## Current implementation

The unified runtime is already executing real machine graphs:

- Rust `cdylib` built as one optimized `omnicore.wasm`.
- Permanent platform ids covering generations 1 through 8.
- 64-bit shared address-space primitives with little- and big-endian accesses for later 32/64-bit machines.
- Exact rational clock domains plus a deterministic event scheduler; emulated time never depends on browser wall time.
- Versioned, console-tagged deterministic save-state codec.
- One RGBA video path rendered with WebGL 2.
- One interleaved `f32` audio path through Web Audio.
- One global four-player logical keyboard/Gamepad controller ABI.
- Reusable NMOS 6502 execution engine covering the official instruction set.
- Playable built-in Home Pong/Telstar-class hardware simulation with video, audio, input, reset, and save states.
- A runnable NES development machine using the same OmniCore binary: 6502 CPU, CPU bus, controllers, OAM DMA, PPU registers/timing, background/sprite rendering, NMI, CHR RAM, PRG RAM, and mapper 0/2/3 cartridge switching.

## What "one core" means

Consoles do not contain identical hardware, so a universal emulator cannot be one generic CPU/GPU algorithm. OmniCore instead provides a common kernel and reusable hardware primitives, then constructs the selected console's real hardware graph inside the same executable.

Source files remain separated for maintainability, but they are not runtime plugins or independently loaded emulator modules. They are statically linked into one program. A console-specific implementation is simply another machine graph compiled into OmniCore and reached through the same `omni_load(platform_id, ...)` entry point.

Shared hardware is implemented once where it is genuinely shared. The 6502-family substrate can serve Atari and Nintendo systems; later Z80, 65816, 68000, MIPS, PowerPC, ARM, x86, SH, DSP, vector and GPU engines will live under the same kernel. Accuracy-sensitive devices remain console-specific rather than being forced through a false abstraction.

## Support and launchability

Compatibility state and developer launchability are intentionally separate:

- `playable` — a machine is exposed as a normal supported system.
- `foundation` — substantial OmniCore hardware exists but compatibility is incomplete.
- `planned` — the platform id and resource model exist, but the machine graph is not implemented.

A foundation machine may be launchable for development without being advertised as broadly compatible. NES currently works this way: mapper 0 (NROM), mapper 2 (UxROM), and mapper 3 (CNROM) paths execute inside OmniCore, while APU accuracy, additional mappers, PPU edge cases, save states, and a representative game corpus still need to be completed before the console can move to `playable`.

The project target is eventually to accept a user's legally obtained game/system files and run them through this one core across generations 1 through 8. The repository does not claim that every commercial game works at the current revision.

## Build

Requirements are Node.js 24+ and a current Rust toolchain with the `wasm32-unknown-unknown` target. On Windows, the build helper can use Rust through WSL when native Cargo is unavailable.

```text
npm install
npm run verify
```

`npm run verify` performs brand-isolation checks, Rust unit tests, a release WebAssembly build, direct ABI/runtime smoke tests for the built-in machine and NES path, TypeScript checking, and the production Vite build.

Useful commands:

```text
npm run core:test
npm run core:build
npm run core:smoke
npm run dev
```

GitHub Pages rebuilds OmniCore from source on every deployment rather than committing a generated WASM artifact.

## Local content

ROMs, disc images, BIOS files, keys, and firmware are user-supplied. Game/system bytes stay local to the browser and are passed to OmniCore; OmniEmu does not upload them. The project does not distribute copyrighted games, proprietary firmware, or console keys.

See `ARCHITECTURE.md` for the unified runtime design and `ROADMAP.md` for the implementation sequence.
