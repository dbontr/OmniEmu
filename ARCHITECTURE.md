# OmniEmu Architecture

## Runtime invariant

OmniEmu ships one emulator runtime: `omnicore.wasm`.

The browser never chooses among unrelated emulator backends. Platform selection is a numeric argument to one WebAssembly instance. A machine is constructed inside OmniCore, executes through the same input/frame/video/audio/state ABI, and is destroyed before another machine is loaded.

Source code is split into files for maintainability, but console implementations are not plugins. Everything is statically linked into the same optimized binary.

## Unified layers

1. **Browser shell** — local game/system files, console selection, global controller editor and presentation.
2. **OmniCore browser bridge** — one WASM load, frame pacing, memory transfer, WebGL 2, Web Audio and Gamepad translation.
3. **Stable C ABI** — console-independent entry points used by every machine.
4. **Hardware blueprint registry** — 29 active target systems describe guest ISA domains, address width/endian requirements, media, firmware/keys, graphics class and persistence needs.
5. **Kernel** — deterministic scheduling, exact clock domains, logical input, video/audio buffers and machine lifecycle.
6. **Memory/interrupt fabric** — 64-bit physical interconnect, 16–64-bit virtual MMU, permissions, DMA, and 256-source interrupt controller.
7. **Execution layer** — reference interpreters plus JIT-neutral guest IR, per-ISA block caches, MMU-generation checks and invalidation.
8. **Media/storage layer** — typed chunked resources, random-access block media and sparse writable storage.
9. **Graphics layer** — bounded GPU command queue and generation-sensitive translated-shader cache feeding the future WebGPU backend.
10. **Machine graphs** — actual devices and address maps for each console, all compiled into OmniCore.
11. **State codec** — versioned deterministic binary state tagged with its platform id to prevent cross-console restores.

## Stable ABI

The current ABI exposes:

```text
omni_core_version
omni_support_level
omni_can_launch / omni_is_targeted / omni_target_count / omni_target_at
omni_resources_clear
omni_resource_create / omni_resource_write
omni_load_staged
omni_load / omni_unload / omni_reset
omni_set_input / omni_set_axis / omni_run_frame
omni_video_* / omni_audio_*
omni_save_state / omni_load_state
omni_last_error_*
```

`omni_load_staged(platform_id)` is the general machine-selection path after typed resources have been staged. The older `omni_load(platform_id, rom, bios)` remains as a compact compatibility wrapper. `omni_can_launch` is distinct from `omni_is_targeted`: a console may have a complete core blueprint long before its machine graph is runnable.

Platform ids are permanent once published. That lets save states, compatibility data, regression fixtures and browser storage refer to a machine without relying on source filenames.

The generic resource ABI supports multiple games/discs, BIOS, firmware, keys, NAND, storage and memory-card slots. Resources use 64-bit declared sizes and bounded chunk staging. A machine-owned persistent-resource ABI lets running machines expose writable storage independently of save states; NES battery RAM is the first end-to-end consumer and is checkpointed to IndexedDB. Random-access/sparse block views remain the path for future multi-gigabyte host-backed media.

## Deterministic time

Emulated hardware must not read browser wall time. The kernel provides two timing primitives:

- a stable integer event scheduler with deterministic ordering at equal timestamps;
- exact rational `ClockDomain` conversion that carries fractional phase between calls instead of accumulating floating-point drift.

`requestAnimationFrame` only determines when the browser asks OmniCore for another completed frame. It does not determine how many emulated CPU, GPU or peripheral cycles occur.

This separation is required for reproducible regression tests, rewind, save states, debugging, deterministic replays and eventual netplay.

## Address spaces

Early CPUs use specialized narrow buses when that is the fastest and clearest implementation. OmniCore also has a shared 64-bit `AddressSpace` contract plus a 16–64-bit paged MMU for later machines. It provides checked 8/16/32/64-bit accesses, explicit little/big-endian conversion, read/write/execute permissions, mapping generations, and fail-closed translation faults.

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

The NES development graph is the first ROM-driven proof. It combines the shared 6502 engine with CPU/PPU buses, controller ports, OAM and DMC DMA paths, NMI/IRQ delivery, background/sprite rendering, pulse/triangle/noise/DMC audio, NES 2.0 sizing, deterministic save states, battery persistence, and mapper 0/1/2/3/4/7/11/66 support. It remains `foundation` because precise PPU fetch/sprite timing, MMC3 A12-edge behavior, unofficial CPU opcodes, additional boards, and large compatibility-corpus validation are unfinished.

## Browser acceleration path

Today, OmniCore emits one RGBA framebuffer and interleaved `f32` audio buffer. The browser bridge uploads video through WebGL 2 and schedules audio through Web Audio. The global logical controller ABI provides a 64-bit digital mask plus eight signed 16-bit analog axes per player, including sticks, analog triggers and auxiliary axes.

The high-end path keeps the same one-core rule while moving expensive work toward WebAssembly SIMD/threads, workers, shared memory, AudioWorklet and WebGPU. OmniCore now has the JIT-neutral block cache/invalidation layer, bounded GPU command/shader caches, and sparse 64-bit media/storage primitives needed to build those paths without introducing console-specific JavaScript runtimes.

## Playable gate

A console moves to `playable` only when its OmniCore machine passes deterministic builds, legal regression software, input, video/audio, reset/state behavior, representative compatibility tests and acceptable performance in current browsers.

An external emulator existing somewhere is not sufficient. If the machine does not run inside `omnicore.wasm`, it is not OmniEmu support.
