# OmniEmu Roadmap

The goal is one increasingly complete emulator, not a launcher for separate emulator products. Every milestone preserves the single `omnicore.wasm` runtime invariant.

## Stage 0 — universal kernel foundation

Established:

- One Rust/WebAssembly emulator binary and one browser bridge.
- Permanent platform-id namespace for generations 1–8.
- Deterministic event scheduler and exact rational clock-domain conversion.
- 64-bit physical interconnect plus a 16–64-bit paged MMU with permissions and mapping generations.
- 256-source interrupt controller and generic DMA engine.
- JIT-neutral IR, per-ISA code cache, MMU-generation validation and invalidation primitives.
- 29 active hardware blueprints spanning generations 1–8; PS4/Xbox One are reserved but out of scope.
- Typed chunked resources for game/disc/BIOS/firmware/keys/NAND/storage plus sparse block media.
- Bounded GPU command queue and translated-shader cache foundation.
- Unified video, audio, input, error, reset and machine lifecycle ABI.
- Versioned platform-tagged save-state codec.
- Browser WebGL 2 / Web Audio / Gamepad path.
- Direct release-WASM smoke tests in the normal build.
- First playable ROM-less machine: Home Pong/Telstar class.
- Atari 2600 development graph: shared 6502/6507 addressing, RIOT RAM/I/O/timer, color-clocked TIA graphics/collisions/audio, WSYNC, input, save states, and 2K/4K/F8/F6/F4 cartridges.
- Reusable official-opcode NMOS 6502 execution substrate.
- Reusable Z80 execution substrate with indexed/extended families, interrupts, block I/O, and deterministic state support.
- First ROM machine graph: NES with CPU/PPU buses, controllers, OAM/DMC DMA, NMI/IRQ/rendering, APU audio, deterministic save states, battery persistence, NES 2.0 sizing, and mapper 0/1/2/3/4/7/11/66 support.
- Second ROM machine graph: Master System with Z80, Sega banked ROM/SRAM, Mode 4 VDP rendering, corrected 3:2 CPU/dot timing, frame/line interrupts, V/H counters, scroll locks, two-player controls, SN76489 audio, save states, and persistent SRAM.
- Third ROM machine graph: ColecoVision reusing the Z80/SN76489 with TMS9918 Graphics I/II rendering, sprites, VDP NMI, BIOS/cartridge mapping, controllers and save states.

## Stage 1 — finish the first cartridge families

The NES development graph is launchable but remains `foundation`. Finish it before claiming broad compatibility:

1. Refine APU/DMC timing: frame-counter write delay, DMA alignment, analog filtering, and IRQ edge cases.
2. Rework PPU execution toward fetch/cycle accuracy: sprite evaluation, scroll transfers, odd-frame timing, and mapper-visible address activity.
3. Replace coarse MMC3 scanline IRQ timing with PPU A12-edge observation and expand board coverage beyond mapper 0/1/2/3/4/7/11/66.
4. Add stable unofficial NMOS 6502 opcodes used by commercial software and strengthen NES 2.0/submapper behavior.
5. Build an automated legal/homebrew compatibility corpus with visual/audio/state determinism and browser performance gates.

Atari 2600 now has a launchable development graph. Continue its timing/bankswitch/peripheral compatibility while reusing the same 6502-family substrate for Atari 5200 and Atari 7800 rather than creating new emulator runtimes.

## Stage 2 — generations 2 through 4

The reusable Z80 and first Sega VDP/PSG machine graph are now established. Continue with 65C816 and 68000-family CPU engines plus broader VDP, PSG/FM, cartridge and optical-media primitives, then complete:

- Finish ColecoVision keypad/peripheral/timing compatibility, then Intellivision.
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

The 64-bit chunked resource/block-media layer is now in place; machine implementations must consume it directly so large media never requires whole-image materialization.

## Stage 5 — seventh and eighth generations

Target Wii, Xbox 360, PlayStation 3, Wii U and Switch after the shared high-end architecture is proven. PlayStation 4 and Xbox One are explicitly outside the current scope; their numeric ids remain reserved only to avoid future data-format collisions.

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
