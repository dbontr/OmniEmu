# OmniEmu

OmniEmu is an experimental browser emulator built around one runtime rule:

> Every targeted console is implemented inside the same `omnicore.wasm` emulator binary.

The browser does not download or select a different emulator core per console. Platform selection is data passed into OmniCore. CPU engines, device models, buses, timing, video, audio, media handling, input, execution caches, and state logic are compiled together and share one ABI.

The emu bird is OmniEmu's mascot and project mark.

## Target set

OmniCore now has explicit hardware blueprints for **29 target systems across generations 1–8**, from Magnavox Odyssey/Home Pong through Wii U and Nintendo Switch. PlayStation 4 and Xbox One keep reserved platform ids so old data cannot collide with future ids, but they are intentionally outside the active target set.

A hardware blueprint is not a compatibility claim. It records the guest CPU/accelerator domains, endian/address-width requirements, memory model, graphics class, media model, firmware/key requirements, persistence needs, and high-end execution requirements that the common core must satisfy.

## Current implementation

The unified runtime is already executing real machine graphs and now has substantially broader substrate:

- Rust `cdylib` built as one optimized `omnicore.wasm`.
- 64-bit shared physical interconnect with little- and big-endian 8/16/32/64-bit accesses.
- 16–64-bit virtual-memory/MMU layer with page permissions, mapping generations, and translation faults.
- Exact rational clock domains plus deterministic event scheduling.
- 256-source priority/mask interrupt controller.
- Generic DMA engine with fixed/increment/decrement addressing.
- Versioned, console-tagged deterministic save-state codec.
- JIT-neutral guest IR, per-ISA translated-block cache, MMU-generation validation, and self-modifying-code invalidation primitives.
- GPU command queue with bounded upload memory plus generation-aware translated-shader caching for the future WebGPU path.
- Chunked 64-bit resource store for games, BIOS, firmware, keys, discs, NAND, storage, and memory cards.
- Sparse random-access/block media primitives whose core data model can represent multi-gigabyte images without allocating every block.
- Sparse persistent block storage suitable for memory cards, flash/NAND images, and virtual hard disks.
- One RGBA video path rendered with WebGL 2 and one interleaved `f32` audio path through Web Audio.
- One global four-player controller ABI with 64 digital bits and 8 signed 16-bit analog axes per player; keyboard and Gamepad mappings feed the same logical controls.
- Reusable NMOS 6502 execution engine covering the official instruction set.
- Playable built-in Home Pong/Telstar-class hardware simulation with video, audio, input, reset, and save states.
- Runnable NES development machine with CPU/PPU bus, controllers, OAM DMA, NMI/IRQ, background/sprite rendering, four mirroring modes, pulse/triangle/noise/DMC audio, deterministic save states, battery-backed persistence, and mapper 0/1/2/3/4/7/11/66 switching.

The browser stages local files into OmniCore in bounded chunks rather than first duplicating them into one temporary WASM allocation. The same resource ABI supports multiple firmware, key, disc, and storage slots for later machines. Machine-owned persistent resources have a generic ABI; NES battery RAM is automatically restored and checkpointed in IndexedDB using a content-derived game fingerprint. The current launch path still stages the complete selected file; true host-backed demand paging for very large disc/package media is a later implementation step.

## What "one core" means

Consoles do not contain identical hardware, so a universal emulator cannot be one generic CPU/GPU algorithm. OmniCore provides a common kernel and reusable hardware primitives, then constructs the selected console's real hardware graph inside the same executable.

Source files remain separated for maintainability, but they are not runtime plugins or independently loaded emulator modules. They are statically linked into one program. A console implementation is another machine graph compiled into OmniCore and reached through the same platform/resource ABI.

The execution architecture is deliberately heterogeneous. A PS2 blueprint can contain R5900, R3000-class IOP, and vector-unit domains; PS3 can contain PowerPC and SPU domains; Switch can use a 64-bit ARM domain. They still share the same scheduler, MMU/interconnect abstractions, resource system, input/state contract, and WebAssembly runtime.

## Support and launchability

Compatibility state and developer launchability are intentionally separate:

- `playable` — a machine is exposed as a normal supported system.
- `foundation` — substantial real OmniCore hardware exists but compatibility is incomplete.
- `planned` — the platform has a target blueprint, but the machine graph is not complete.
NES is currently `foundation`: NROM, MMC1, UxROM, CNROM, MMC3, AxROM, Color Dreams, and GxROM execute inside OmniCore; APU channels including DMC, deterministic save states, NES 2.0 sizing, and battery persistence are live. It still needs substantially more mapper coverage, cycle-accurate PPU/MMC3 edge behavior, unofficial CPU-opcode compatibility, and a representative legal compatibility corpus before it can move to `playable`.

The project target is broad compatibility for the 29 active systems, not a claim that all of those commercial libraries work today.

## Build and verification

Requirements are Node.js 24+ and a current Rust toolchain with `wasm32-unknown-unknown`, rustfmt, and clippy. On Windows, the build helper can use Rust through WSL when native Cargo is unavailable.

```text
npm install
npm run verify
```

`npm run verify` runs brand-isolation checks, rustfmt, clippy with warnings denied, the Rust test suite, an optimized WASM build, direct WASM ABI/runtime smoke tests, TypeScript checking, and the production Vite build.

Useful commands:

```text
npm run core:test
npm run core:fmt
npm run core:clippy
npm run core:build
npm run core:smoke
npm run dev
```

GitHub Pages rebuilds OmniCore from source on every deployment rather than committing a generated WASM artifact.

## Local content

ROMs, disc images, BIOS files, keys, and firmware are user-supplied. Game/system bytes stay local to the browser and are passed to OmniCore; OmniEmu does not upload them. The project does not distribute copyrighted games, proprietary firmware, or console keys.

See `ARCHITECTURE.md` for the unified runtime design and `ROADMAP.md` for the machine implementation sequence.
