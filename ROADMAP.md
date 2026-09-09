# OmniEmu Roadmap

The goal is one increasingly complete emulator, not a launcher for separate emulator products. Every milestone preserves the single `omnicore.wasm` runtime invariant.

## Stage 0 — universal kernel foundation

Established:

- One Rust/WebAssembly emulator binary and one browser bridge.
- Permanent platform-id namespace for generations 1–8.
- Deterministic event scheduler and exact rational clock-domain conversion.
- 64-bit shared address-space abstraction with explicit endianness for later 32/64-bit machines.
- Unified video, audio, input, error, reset and machine lifecycle ABI.
- Versioned platform-tagged save-state codec.
- Browser WebGL 2 / Web Audio / Gamepad path.
- Direct release-WASM smoke tests in the normal build.
- First playable ROM-less machine: Home Pong/Telstar class.
- Reusable official-opcode NMOS 6502 execution substrate.
- First ROM machine graph: NES with CPU bus, controllers, DMA, PPU/NMI/rendering and mapper 0/2/3 support.

## Stage 1 — finish the first cartridge families

The NES development graph is launchable but remains `foundation`. Finish it before claiming broad compatibility:

1. NES APU pulse/triangle/noise/DMC channels and frame counter.
2. Cycle-sensitive PPU behavior, sprite evaluation, scrolling edge cases and timing validation.
3. Mapper framework expansion: MMC1, MMC3/MMC6, AxROM, MMC5 and other high-coverage boards, then less common mappers.
4. NES deterministic save states and persistent cartridge RAM.
5. Automated legal/homebrew compatibility corpus and browser performance gates.

In parallel, reuse the same 6502-family substrate for Atari 2600, Atari 5200 and Atari 7800 rather than creating new emulator runtimes.

## Stage 2 — generations 2 through 4

Add reusable Z80, 65C816 and 68000-family CPU engines plus VDP, PSG/FM, cartridge and optical-media primitives, then complete:

- ColecoVision and Intellivision.
- Master System.
- SNES / Super Famicom.
- Genesis / Mega Drive, Sega CD and 32X.
- PC Engine / TurboGrafx family.
- Neo Geo.

Expansion chips are devices inside the loaded machine graph. They never select a different browser emulator.

## Stage 3 — fifth generation

Build the shared MIPS/geometry/DSP/GPU/media substrate needed for PlayStation, Nintendo 64, Saturn, Jaguar and 3DO.

This is where OmniCore needs a reusable dynamic-translation layer, reference interpreters, stronger worker isolation and WebGPU acceleration. Fast paths must be continuously checked against deterministic reference execution.

## Stage 4 — sixth generation

Bring Dreamcast, PlayStation 2, GameCube and original Xbox into OmniCore using shared JIT, MMU, shader-translation, optical-media and asynchronous-I/O infrastructure instead of four independent applications.

Large media must become host-backed/streamed rather than copied wholesale into WebAssembly memory.

## Stage 5 — seventh and eighth generations

Target Wii, Xbox 360, PlayStation 3, Wii U, Switch, PlayStation 4 and Xbox One only after the shared high-end architecture is proven. PS4 and Xbox One remain lowest priority, but their ids are reserved now so they do not require a runtime redesign.

High-end work requires:

- scalable multi-core guest scheduling;
- portable JIT/dynamic translation plus deterministic interpreter fallbacks;
- guest MMU and virtual-memory models;
- GPU command decoding and shader translation;
- WebGPU-first rendering;
- WebAssembly SIMD/threads and shared-memory workers;
- AudioWorklet-based low-latency mixing;
- user-provided lawful firmware/keys where required;
- virtual storage and multi-gigabyte disc/package streaming.

## Compatibility gate

A platform moves to `playable` only when the same OmniCore build passes deterministic unit tests plus representative legal software covering boot, CPU, graphics, sound, input, media, reset, state/persistence and sustained frame pacing.

A platform approaches a universal-compatibility claim only after a large automated corpus demonstrates it. A title merely reaching its first frame is not sufficient evidence.
