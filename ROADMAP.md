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
- Atari 5200 development graph: shared NMOS 6502, 16 KiB RAM and cartridge/BIOS map, ANTIC display-list/scanline foundation, GTIA colors/triggers, POKEY analog/keypad/audio/IRQ, WSYNC, and save states.
- Atari 7800 development graph: shared NMOS 6502/SALLY execution, documented RAM/register mirrors, PIA/TIA foundation, MARIA DLL/display-list rendering and DLI/WSYNC, A78 parsing, linear/SuperGame cartridges, optional POKEY, and save states.
- Reusable NMOS 6502 execution substrate with the official set, common stable undocumented families, and selectable decimal arithmetic for NMOS versus NES-class behavior.
- Reusable Z80 execution substrate with indexed/extended families, interrupts, block I/O, and deterministic state support.
- Reusable W65C816 execution substrate with all 256 opcode paths, native/emulation modes, 8/16-bit widths, 24-bit addressing, decimal arithmetic, interrupts, and block moves.
- Reusable SPC700 execution substrate with all 256 opcode decode paths, direct-page/bit operations, 16-bit YA arithmetic, multiply/divide, branches/calls, and save-state serialization.
- Reusable Motorola 68000 execution substrate with big-endian 24-bit addressing, effective-address decoding, exceptions/interrupts, arithmetic/control/shift families, and deterministic state support.
- Reusable MIPS R3000A-class execution substrate with branch/load delay slots, COP0 exception/interrupt state, HI/LO arithmetic, unaligned merge loads/stores, and deterministic pipeline serialization for the PlayStation path.
- First ROM machine graph: NES with CPU/PPU buses, controllers, OAM/DMC DMA, NMI/IRQ/rendering, APU audio, deterministic save states, battery persistence, NES 2.0 sizing, and mapper 0/1/2/3/4/7/11/66 support.
- Second ROM machine graph: Master System with Z80, Sega banked ROM/SRAM, Mode 4 VDP rendering, corrected 3:2 CPU/dot timing, frame/line interrupts, V/H counters, scroll locks, two-player controls, SN76489 audio, save states, and persistent SRAM.
- Third ROM machine graph: ColecoVision reusing the Z80/SN76489 with TMS9918 Graphics I/II rendering, sprites, VDP NMI, BIOS/cartridge mapping, controllers and save states.
- Fourth ROM machine graph: Genesis / Mega Drive combining 68000 + Z80, VDP, YM2612, SN76489, controllers, SMD decoding, SRAM persistence, interrupts, and deterministic state.
- Fifth ROM machine graph: SNES / Super Famicom using W65C816 with LoROM/HiROM, WRAM/I/O/controllers, PPU VRAM/CGRAM/OAM and Mode 0/1 BG1 rendering, vblank/NMI, SRAM persistence, and deterministic state.

## Stage 1 — finish the first cartridge families

The NES development graph is launchable but remains `foundation`. Finish it before claiming broad compatibility:

1. Refine APU/DMC timing: frame-counter write delay, DMA alignment, analog filtering, and IRQ edge cases.
2. Rework PPU execution toward fetch/cycle accuracy: sprite evaluation, scroll transfers, odd-frame timing, and mapper-visible address activity.
3. Replace coarse MMC3 scanline IRQ timing with PPU A12-edge observation and expand board coverage beyond mapper 0/1/2/3/4/7/11/66.
4. Stable undocumented NMOS 6502 opcode families are now implemented; continue with unstable edge cases, bus-visible timing details, and stronger NES 2.0/submapper behavior.
5. Build an automated legal/homebrew compatibility corpus with visual/audio/state determinism and browser performance gates.

Atari 2600, Atari 5200 and Atari 7800 now have launchable development graphs. Continue their timing, display, bankswitch and peripheral compatibility in the shared 6502-family substrate; Atari 7800 specifically still needs cycle-faithful MARIA DMA/modes and advanced mapper coverage.

## Stage 2 — generations 2 through 4

The reusable Z80, W65C816 and Motorola 68000 engines plus launchable Sega and SNES machine graphs are now established. General SNES DMA and direct/indirect HDMA table execution are now implemented. SPC700, S-SMP and the first BRR-based eight-voice S-DSP audio path are established; next refine Gaussian interpolation/envelope/pitch modulation/echo-FIR, then cycle-faithful HDMA scheduling and deeper PPU timing/modes, deepen Genesis VDP/YM2612 timing, and broaden cartridge/optical-media primitives before completing:

- Finish ColecoVision keypad/peripheral/timing compatibility, then Intellivision.
- Master System.
- SNES / Super Famicom.
- Genesis / Mega Drive, Sega CD and 32X.
- PC Engine / TurboGrafx family.
- Neo Geo.

Expansion chips are devices inside the loaded machine graph. They never select a different browser emulator.

## Stage 3 — fifth generation

The MIPS R3000A-class CPU substrate is now established. Next construct the PlayStation memory/BIOS/GPU/DMA/interrupt machine graph, then add the remaining reusable 32-bit CPU, geometry/DSP and fixed-function GPU paths for Nintendo 64, Saturn, Jaguar and 3DO.

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
