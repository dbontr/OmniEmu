# OmniEmu Architecture

## Runtime invariant

OmniEmu ships one emulator runtime: `omnicore.wasm`.

The browser never chooses among unrelated emulator backends. Platform selection is a numeric argument to one WebAssembly instance. A machine is constructed inside OmniCore, executes through the same input/frame/video/audio/state ABI, and is destroyed before another machine is loaded.

Source code is split into files for maintainability, but console implementations are not plugins. Everything is statically linked into the same optimized binary.

## Unified layers

1. **Browser shell** — local game/system files, console selection, global controller editor and presentation.
2. **OmniCore browser bridge** — one WASM load, frame pacing, memory transfer, WebGL 2, Web Audio and Gamepad translation.
3. **Stable C ABI** — console-independent entry points used by every machine.
4. **Kernel** — deterministic scheduling, exact clock domains, logical input, video/audio buffers and machine lifecycle.
5. **Universal interconnect** — 64-bit address-space contract with 8/16/32/64-bit accesses and explicit endianness, alongside specialized fast buses for small CPUs.
6. **Reusable hardware engines** — CPUs, memory, cartridge/media logic, later DSP/GPU/vector blocks.
7. **Machine graphs** — the actual devices and address maps of each console, compiled into OmniCore.
8. **State codec** — versioned deterministic binary state tagged with its platform id to prevent cross-console restores.

## Stable ABI

The current ABI exposes:

```text
omni_core_version
omni_support_level
omni_can_launch
omni_load / omni_unload / omni_reset
omni_set_input / omni_run_frame
omni_video_* / omni_audio_*
omni_save_state / omni_load_state
omni_last_error_*
```

`omni_load(platform_id, rom, bios)` is the sole machine-selection entry point. `omni_can_launch` is distinct from the compatibility tier so partially implemented machines can be exercised during development without being advertised as fully playable.

Platform ids are permanent once published. That lets save states, compatibility data, regression fixtures and browser storage refer to a machine without relying on source filenames.

Later systems will need multiple firmware files, optical-disc streaming, keys, writable storage and larger media. Those resources will be added to the generic OmniCore host/resource ABI rather than by introducing separate emulator runtimes.

## Deterministic time

Emulated hardware must not read browser wall time. The kernel provides two timing primitives:

- a stable integer event scheduler with deterministic ordering at equal timestamps;
- exact rational `ClockDomain` conversion that carries fractional phase between calls instead of accumulating floating-point drift.

`requestAnimationFrame` only determines when the browser asks OmniCore for another completed frame. It does not determine how many emulated CPU, GPU or peripheral cycles occur.

This separation is required for reproducible regression tests, rewind, save states, debugging, deterministic replays and eventual netplay.

## Address spaces

Early CPUs use specialized narrow buses when that is the fastest and clearest implementation. OmniCore also has a shared 64-bit `AddressSpace` contract for later machines. It provides checked 8/16/32/64-bit accesses and explicit little/big-endian conversion, so 32-bit and 64-bit consoles do not require a new top-level runtime model.

Memory-mapped devices, DMA engines, GPU register files and high-address physical maps can all sit behind this interconnect while machine-specific fast paths remain possible where accuracy or performance requires them.

## Shared hardware strategy

Reusable hardware is shared only when the real consoles share it:

- NMOS 6502/6507 family: Atari and NES-family work.
- Z80 family: Master System, ColecoVision and related machines.
- 65816: SNES-family work.
- 68000 family: Genesis, Neo Geo and related components.
- MIPS family: PlayStation-class machines.
- PowerPC, ARM, x86, SH, DSP, vector and GPU engines are added under the same kernel as later generations are reached.

Accuracy-sensitive devices remain machine-specific where sharing would be incorrect. The objective is one emulator executable with reusable hardware, not pretending every console has equivalent internals.

## Current machine proof points

Home Pong/Telstar-class hardware is fully constructed inside OmniCore and exercises the common frame, input, audio, video, reset and state paths without a ROM.

The NES development graph is the first ROM-driven proof: it combines the shared 6502 engine with an NES CPU bus, controller ports, OAM DMA, PPU register state/timing, NMI, background and sprite rendering, CHR/PRG memory and mapper 0/2/3 switching. It is intentionally still marked `foundation` while APU, additional mappers, PPU edge behavior and compatibility validation are unfinished.

## Browser acceleration path

Today, OmniCore emits one RGBA framebuffer and interleaved `f32` audio buffer. The browser bridge uploads video through WebGL 2 and schedules audio through Web Audio. The global logical controller ABI is a 64-bit mask per player.

The high-end path keeps the same one-core rule while moving expensive work toward WebAssembly SIMD/threads, workers, shared memory, AudioWorklet and WebGPU. Large optical/package media will use a generic host-backed streaming resource interface rather than forcing multi-gigabyte files to become console-specific JavaScript backends.

## Playable gate

A console moves to `playable` only when its OmniCore machine passes deterministic builds, legal regression software, input, video/audio, reset/state behavior, representative compatibility tests and acceptable performance in current browsers.

An external emulator existing somewhere is not sufficient. If the machine does not run inside `omnicore.wasm`, it is not OmniEmu support.
