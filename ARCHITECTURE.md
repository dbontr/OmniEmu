# OmniEmu Architecture

## Runtime invariant

OmniEmu ships one emulator runtime: `omnicore.wasm`.

The browser never chooses among unrelated emulator backends. Platform selection is a numeric argument to one WebAssembly instance. A machine is constructed inside OmniCore, executes through the same input/frame/video/audio/state ABI, and is destroyed before another machine is loaded.

Source code is split into files for maintainability, but console implementations are not plugins. Everything is statically linked into the same optimized binary.

## Unified layers

1. **Browser shell** — local game/system files, console selection, global controller editor and presentation.
2. **OmniCore browser bridge** — one WASM load, frame pacing, local resource hydration, WebGPU-first video with WebGL 2 fallback, AudioWorklet-first audio with Web Audio fallback, and Gamepad translation.
3. **Stable C ABI** — console-independent entry points used by every machine.
4. **Hardware blueprint registry** — 30 active target systems describe guest ISA domains, address width/endian requirements, media, firmware/keys, graphics class and persistence needs.
5. **Kernel** — deterministic scheduling, exact clock domains, logical input, video/audio buffers and machine lifecycle.
6. **Memory/interrupt fabric** — 64-bit physical interconnect, 16–64-bit virtual MMU, permissions, DMA, and 256-source interrupt controller.
7. **Execution layer** — reference interpreters plus JIT-neutral guest IR, per-ISA block caches, MMU-generation checks and invalidation.
8. **Media/storage layer** — typed chunked resources, bounded live paged media, random-access block views and sparse writable storage.
9. **Graphics layer** — bounded GPU command queue plus generation-sensitive shader translation/cache feeding the browser rendering path.
10. **Machine graphs** — actual devices and address maps for each console, all compiled into OmniCore.
11. **State codec** — versioned deterministic binary state tagged with its platform id to prevent cross-console restores.

## Stable ABI

The current ABI exposes:

```text
omni_core_version
omni_support_level
omni_can_launch / omni_is_targeted / omni_target_count / omni_target_at
omni_resources_clear
omni_resource_create / omni_resource_create_streaming / omni_resource_write
omni_resource_pending_start / omni_resource_pending_end
omni_load_staged
omni_load / omni_unload / omni_reset
omni_set_input / omni_set_axis / omni_run_frame
omni_video_* / omni_audio_*
omni_save_state / omni_load_state
omni_last_error_*
```

`omni_load_staged(platform_id)` is the general machine-selection path after typed resources have been staged. The older `omni_load(platform_id, rom, bios)` remains as a compact compatibility wrapper. `omni_can_launch` is distinct from `omni_is_targeted`: a console may have a complete core blueprint long before its machine graph is runnable.

Platform ids are permanent once published. That lets save states, compatibility data, regression fixtures and browser storage refer to a machine without relying on source filenames.

The generic resource ABI supports multiple games/discs, BIOS, firmware, keys, NAND, storage and memory-card slots. Resources use 64-bit declared sizes and bounded chunk staging. Streaming resources share live paged data with an already-created machine, report missing byte ranges through the ABI, and let the browser hydrate bounded windows from the original local `File`. A machine-owned persistent-resource ABI exposes writable storage independently of save states. The browser discovers every currently exposed storage, memory-card and NAND slot and checkpoints each resource independently to IndexedDB under a content-derived game fingerprint.

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

- NMOS 6502/6507 family: Atari and NES-family work; configurable direct/stack page bases are reused by the HuC6280 compatibility layer.
- HuC6280 family: PC Engine and SuperGrafx execution with MPR mapping, HuC6280/65C02 extensions, timer/IRQ and CPU speed switching; SuperGrafx reuses the same CPU/PSG/VCE substrate while adding a second HuC6270 and HuC6202 VPC.
- Z80 family: Master System, ColecoVision and related machines.
- CP1610 family: Intellivision CPU execution and word-oriented bus behavior.
- 65C816: a reusable 24-bit W65C816 engine is now implemented with all opcode decode paths; SNES timing and machine devices are the next layer.
- 68000 family: Genesis, Neo Geo and related components.
- MIPS family: PlayStation R3000A, Nintendo 64 R4300-class, and PlayStation 2 R5900/Emotion Engine execution use separate generation-appropriate cores under the same deterministic machine/state contracts; the R5900 includes 128-bit GPR/MMI foundations and an explicit HLE direct-map mode used only by the PS2 development boot path.
- Nintendo 64 RSP: one reusable scalar/vector signal-processor engine owns 4 KiB DMEM/IMEM execution, COP0 device access, vector arithmetic and deterministic microcode state.
- Jaguar Tom/Jerry RISC: one reusable 64-opcode execution engine parameterized for GPU- and DSP-specific operations, shared by the Jaguar machine graph.
- SuperH: SH-2 remains a big-endian console substrate for 32X/Saturn, while Dreamcast uses a dedicated little-endian SH-4 core with banked privileged registers, saved exception state and floating-point register banks.
- ARM: 3DO uses the ARM60/ARMv3 engine; Dreamcast AICA uses a separate ARM7TDMI/ARMv4T engine with ARM/Thumb state switching, halfword/signed transfers and long multiply.
- PowerPC: the reusable 750/Gekko core executes GameCube and Wii-class 32-bit PowerPC state, including paired-single arithmetic, comparisons, merge operations, PSQ quantized loads/stores, GQR0-7 and HID2 controls. Wii U reuses the same integer execution family for Espresso development paths. A separate reusable 64-bit big-endian PowerPC substrate serves Xbox 360 Xenon and PlayStation 3 PPE development execution.
- x86: a reusable 32-bit Pentium-class interpreter provides ModR/M+SIB addressing, integer/control/stack/string foundations, CPUID/RDTSC and deterministic state for the original Xbox development graph.
- AArch64: a reusable little-endian ARMv8-A interpreter provides branches, integer data processing, addressing, pair and scalar loads/stores, literal loads and SVC handling for the Switch NRO development graph.
- Sparse high-capacity memory: multi-gigabyte guest DRAM/VRAM spaces allocate host pages only when touched and serialize only populated pages, allowing Xbox 360, PS3, Wii U and Switch development graphs to preserve real capacity boundaries without dense browser allocations.

Accuracy-sensitive devices remain machine-specific where sharing would be incorrect. The objective is one emulator executable with reusable hardware, not pretending every console has equivalent internals.

## Current machine proof points

Magnavox Odyssey is a dedicated discrete-logic machine graph rather than a CPU emulator. Its game cards select combinations of player, ball, wall, gate and English circuits; the machine is intentionally silent and requires no ROM.

Home Pong/Telstar-class hardware is fully constructed inside OmniCore and exercises the common frame, input, audio, video, reset and state paths without a ROM.

Atari 2600 is the first shared-6502 reuse outside Nintendo: OmniCore constrains the CPU through a 6507-style 13-bit bus, models RIOT RAM/I/O/timing, advances TIA video at three color clocks per CPU cycle, renders playfield/player/missile/ball objects with collision latches, honors WSYNC stalls, synthesizes the two TIA audio channels, and implements common 2K/4K/F8/F6/F4 cartridge schemes.

Atari 5200 now reuses the same NMOS 6502 substrate in a different machine graph with 16 KiB RAM, cartridge/BIOS mapping, ANTIC scanline and display-list execution, GTIA color/trigger state, POKEY analog controllers/keypad/audio/IRQ, WSYNC and deterministic state. ANTIC/GTIA mode and DMA timing are intentionally still foundation-level.

Atari 7800 extends that reuse into the SALLY/MARIA generation: the machine models the documented RAM/register mirrors, PIA, TIA audio/input, MARIA display-list-list and object-header rendering, DLI/WSYNC foundations, A78 metadata, linear and SuperGame banking, and optional POKEY address variants. MARIA DMA stealing, full pixel modes, advanced mappers, BIOS/security behavior and broad cartridge validation remain foundation work.

The NES development graph is the first ROM-driven proof. It combines the shared 6502 engine with CPU/PPU buses, controller ports, OAM and DMC DMA paths, NMI/IRQ delivery, background/sprite rendering, pulse/triangle/noise/DMC audio, NES 2.0 sizing, deterministic save states, battery persistence, and mapper 0/1/2/3/4/7/11/66 support. It remains `foundation` because precise PPU fetch/sprite timing, MMC3 A12-edge behavior, unstable/edge-case undocumented CPU behavior, additional boards, and large compatibility-corpus validation are unfinished. Stable undocumented NMOS opcode families are now implemented, while the RP2A03 path explicitly disables decimal arithmetic.

The Master System development graph is the second ROM-driven proof and the first reuse of the Z80-family substrate. It combines the shared Z80 with Sega ROM/SRAM mapping, Mode 4 tile/sprite rendering, frame and line interrupts, horizontal/vertical scroll locks, controller ports, SN76489 audio, deterministic state, and cartridge persistence. It remains `foundation` while cycle-level VDP behavior, legacy modes, mapper variants, FM audio, and broad compatibility validation are unfinished.

ColecoVision is the third ROM-driven machine and demonstrates reuse across vendors: the same Z80 and SN76489 execute alongside a reusable TMS9918 device, an 8 KiB user BIOS, the Coleco memory/I/O map, controller multiplexing, Graphics I/II rendering, sprites, VDP NMI and deterministic state.

Intellivision uses the reusable CP1610 execution engine with user-supplied EXEC and GROM images, 16-bit system RAM, 8-bit scratch/GRAM regions and documented GRAM aliases. The STIC path implements background cards, color-stack/foreground-background modes, MOB drawing and collision registers; the AY-3-8914 path models register masks, tone, noise, envelopes and controller ports. Intellicart segmented images and deterministic state are supported. STIC bus arbitration/timing, expansion peripherals and broad cartridge validation remain foundation work.

Genesis / Mega Drive is the first 16-bit machine graph. It combines the reusable Motorola 68000 and Z80 engines with the console bus map, bus-request/reset behavior, controller ports, VDP VRAM/CRAM/VSRAM and plane/sprite rendering foundation, YM2612 FM/DAC/timers, SN76489 audio, SMD decoding, cartridge SRAM persistence, the official Super Street Fighter II 512 KiB bank mapper and deterministic state. VDP DMA/cycle timing, full YM2612 fidelity, mapper breadth and compatibility validation remain foundation work.

PC Engine / TurboGrafx-16 reuses the configurable 6502 substrate only where the HuC6280 is genuinely compatible, while a dedicated HuC6280 layer owns MPR translation, direct/stack pages, extension opcodes, timer/IRQ and speed state. Its machine graph connects HuC6270/HuC6260 video and PSG audio to a controller/cartridge layer that keeps ordinary HuCards direct-mapped while profile-gating Street Fighter II's 2.5 MiB bank window and alternating six-button scan protocol plus Populous' 32 KiB cartridge RAM. The System Card/CD-ROM device owns SCSI transfer, backup RAM, Arcade Card 2 MiB DRAM and its four programmable access ports, ADPCM memory/DMA, MSM5205-compatible decode/playback, fader state and CD-DA playback. Disc geometry and CUE/BINARY/WAVE parsing use the shared `cd_image` layer also consumed by PlayStation, while controller-specific command, IRQ, SubQ and timing behavior remains machine-owned. All machine and CD playback state participates in deterministic serialization.

SuperGrafx extends the PC Engine machine family without duplicating its shared hardware. The first HuC6270 remains the primary timing source; a second independent HuC6270 is exposed through the documented SuperGrafx memory window, while a HuC6202 VPC owns the two priority registers, two 10-bit windows, ST-port VDC selection and final dual-VDC pixel composition. Both VDC interrupt outputs feed the shared HuC6280 IRQ1 boundary, and the VPC/VDC profile is serialized as part of deterministic machine state.

Neo Geo combines the existing 68000 and Z80 execution engines in a different 16-bit machine graph. NEO1 cartridge parsing keeps P/S/M/V/C regions distinct; the board map provides BIOS/cartridge vector switching, work/backup/card memory, video RAM and palette banks, and the sound path connects command NMI/reply handling to YM2610 SSG and ADPCM-A/B sample engines. Protected cartridge logic, FM fidelity and exact raster timing remain machine-specific work.

SNES / Super Famicom now provides the first W65C816 machine graph. OmniCore maps LoROM/HiROM cartridges and SRAM across the 24-bit bus, models 128 KiB WRAM and CPU/PPU I/O, serial controllers, vblank/NMI, VRAM/CGRAM/OAM register paths and Mode 0/1 BG1 tile rendering. General DMA and direct/indirect HDMA table execution now use the eight channel registers and B-bus patterns, including repeat and line-counter behavior. The SPC700 is integrated through 64 KiB S-SMP RAM, bidirectional CPU ports, control semantics, three timers and the DSP register interface; OmniCore HLEs only the IPL transfer handshake so it does not bundle proprietary IPL firmware. The S-DSP foundation now decodes BRR blocks and mixes eight 32 kHz stereo voices with pitch, key events, envelope/noise and ENDX state into the common audio ABI. Gaussian interpolation, exact envelope/pitch-modulation behavior, echo/FIR, cycle-faithful HDMA timing, remaining PPU modes/sprites/windows/color math and enhancement chips are still foundation work.

PlayStation is the first fifth-generation machine graph. It uses the reusable MIPS R3000A execution core with mirrored system RAM, scratchpad and a user-provided 512 KiB BIOS, routes COP0 external interrupts through the console interrupt controller, and models BIOS memory-control programming, the 4 KiB instruction cache/cache-isolation path, native partial-width MMIO stores, and COP2/GTE execution with CPU overlap, delayed transport, and hardware-measured input-latch timing for perspective, clipping, depth, and non-aliasing lighting commands. The board DMA controller arbitrates enabled channels by DPCR priority, routes MDEC/GPU/CD-ROM/SPU/OTC transfers, applies DICR mask/master/acknowledge behavior, waits for CD-ROM BFRD, honors RAM_SIZE windows and raises DMA bus errors. The machine also integrates GPU VRAM/display/rasterization with measured command-busy intervals and readiness handshakes, root counters, DualShock-class SIO0, memory-card persistence, streamed multitrack CD-ROM/XA/CD-DA, MDEC, and a 24-voice SPU with deterministic serialization. Aliasing/multi-read GTE latch edges, DMA/CPU bus-cycle arbitration, full asynchronous GPU FIFO/command arbitration, sub-scanline MDEC/timer timing, exact SPU reverb edges, compressed disc containers and broad retail compatibility validation remain machine-specific work.

Nintendo 64 uses a dedicated R4300-class interpreter with 64-bit integer state, big-endian accesses, COP0/TLB/Count/Compare/LL-SC behavior and COP1 floating-point register/control state. COP1 now handles 32/64-bit register moves and load/store paths, single/double arithmetic and square root, comparisons and condition branches, integer conversions, Status.CU1 unusable exceptions, Status.FR register views, and deterministic serialization. The console graph HLEs only the PIF boot handoff, copies the cartridge IPL3 into SP memory, and then executes cartridge code normally. Its RCP foundation includes scalar/vector RSP execution, 8 MiB RDRAM, SP/DP/MI/VI/AI/PI/SI registers and DMA, Joybus controllers and Controller Pak, RDP color-image/fill commands, VI framebuffer presentation, AI sample delivery, persistent save storage and deterministic state. Bit-exact VR4300 FP exception/subnormal/NaN behavior, complete vector edge semantics, full RDP rasterization/texturing/blending, CIC/PIF variants and exact timing remain foundation work.

Atari Jaguar combines the shared 68000 with one reusable Jaguar RISC engine instantiated as Tom GPU and Jerry DSP. The board maps 2 MiB DRAM, a 6 MiB cartridge window, user-supplied 128 KiB boot ROM, Tom/Jerry register and local-RAM windows, the active-low controller matrix, stereo DAC state and EEPROM persistence. The video foundation traverses Tom bitmap and scaled-bitmap object entries into the shared RGBA surface. Blitter fidelity, complete object-processor semantics, RISC interrupt pipelines, Jerry timing and bus arbitration remain machine-specific work.

Dreamcast is the first sixth-generation machine graph. Its main CPU is a dedicated little-endian SH-4 interpreter with privileged register banks, fixed-vector exception state and floating-point foundations; AICA owns a separate ARM7TDMI/ARMv4T interpreter with ARM/Thumb execution. The board maps the 2 MiB boot ROM, 128 KiB flash, 16 MiB main RAM, 8 MiB PowerVR VRAM and 2 MiB sound RAM, routes HOLLY interrupts, executes Maple controller DMA, produces AICA PCM, presents PowerVR framebuffer state, streams GD-ROM sectors through the common resource pager and preserves deterministic machine/flash state. Full PowerVR2 tile/raster behavior, AICA DSP/ADPCM, SH-4 MMU/cache/peripherals, GD-ROM command/media breadth and cycle-level timing remain machine-specific work.

PlayStation 2 is the second sixth-generation proof point. It runs a dedicated 128-bit R5900/Emotion Engine core over 32 MiB EE RAM and the 16 KiB scratchpad, exposes the EE hardware/GS address windows, routes INTC/timer state into the CPU interrupt line, and supports an explicit HLE development boot from raw ELF32/MIPS or ISO9660 `SYSTEM.CNF`/`BOOT2`. The initial GIF/GS path consumes packed A+D and image-transfer qwords into 4 MiB GS VRAM and presents PSMCT32 framebuffer state. Memory-card persistence and deterministic state are integrated. The IOP/SIF/SIO2, vector units, complete GS raster/swizzle/texture semantics, CDVD, SPU2, BIOS boot and exact timing remain machine-specific work.

GameCube is the third sixth-generation proof point and the first PowerPC machine. The Gekko substrate extends the reusable 750-class interpreter with paired-single/GQR/HID2 state; the machine maps 24 MiB MEM1, 16 MiB ARAM and Flipper PI/DI/SI/VI/DSP register foundations, HLE-loads DOL sections from raw executable or disc data, streams DVD DMA through the common sparse resource layer, translates GameCube controller state through SI, presents YUYV XFB output, produces audio-DMA samples, and serializes deterministic state. EXI channels 0 and 1 expose independent Slot A and Slot B memory cards with transfer-complete/device interrupts, DMA, guest-visible read/program/erase behavior and persistent backing. Channel 0 also exposes the IPL device, including optional user-supplied 2 MiB IPL dumps, raw boot-code descrambling, embedded font reads, SRAM access and deterministic RTC progression. Full GX/Flipper graphics, DSP microcode, additional EXI devices and exact bus/timing behavior remain machine-specific work.

Wii is the first seventh-generation launchable foundation. It reuses the Broadway-compatible PowerPC/Gekko engine, maps 24 MiB MEM1 plus 64 MiB MEM2 with physical/cached/uncached aliases, exposes the Flipper-compatible and Hollywood PPC register windows, and implements the documented Broadway↔Starlet IPC message/control/interrupt boundary. The current HLE loader accepts homebrew DOLs only; YUYV XFB presentation, a fixed-rate audio surface and deterministic state are live. IOS/Starlet execution and services, retail encrypted-disc/NAND paths, Bluetooth/Wii Remote, complete Hollywood/GX/DSP behavior and exact timing remain machine-specific work.

Xbox completes sixth-generation machine breadth. Its XBE development path uses the reusable 32-bit x86 interpreter, 64 MiB shared RAM, XBE section/entry decoding, NV2A and MCPX apertures, PCRTC framebuffer presentation and deterministic state. Retail kernel/BIOS boot, NV2A graphics, USB, audio, IDE/DVD/FATX, networking and exact timing remain machine-specific work.

Xbox 360 uses three Xenon core objects backed by the reusable 64-bit big-endian PowerPC interpreter and 512 MiB sparse unified memory. A bounded ELF64 development path proves code execution and state replay without requiring proprietary XEX/kernel data. SMT, VMX128, hypervisor/kernel boot, Xenos/eDRAM, chipset devices and timing remain.

PlayStation 3 uses the same 64-bit PowerPC substrate only for the Cell PPE domain, with separate sparse XDR and RSX local memory plus seven visible SPU local-store regions. The development path accepts PowerPC64 ELF input. SPU execution/MFC/EIB, firmware/hypervisor/SELF boot, RSX graphics and complete devices remain machine-specific work.

Wii U reuses the 32-bit PowerPC execution family across three Espresso core objects while keeping a distinct sparse 32 MiB MEM1 and 2 GiB MEM2 map. A bounded ELF32 development path proves execution and deterministic state. Cafe OS/RPX/RPL, IOSU, Latte/GX2, audio, input and storage protocols remain.

Nintendo Switch uses a dedicated little-endian AArch64 interpreter and four application-core state objects over sparse 4 GiB DRAM. The development loader validates standalone NRO text/rodata/data layout, BSS and homebrew entry-state conventions. The Horizon HLE foundation covers the libnx startup paths for service management, applet/window control, time, filesystem setup, HID shared-memory/Npad input, and VI/Binder display-layer initialization. Retail Horizon service breadth, NSO/NCA/NPDM and package loading, Maxwell/NVN graphics, full audio/storage/networking, and production multicore scheduling/timing remain.

All 30 active target platforms therefore have an in-OmniCore launchable machine graph. Launchable breadth is separate from the compatibility gate below.

## Browser acceleration path

OmniCore emits one RGBA framebuffer and interleaved `f32` audio buffer. The browser bridge prefers WebGPU for framebuffer presentation and falls back to WebGL 2. Audio is delivered through an AudioWorklet when available, with a Web Audio buffer-source fallback. The global logical controller ABI provides a 64-bit digital mask plus eight signed 16-bit analog axes per player.

The same one-core rule extends to high-end systems. The shared foundation includes deterministic multi-CPU scheduling boundaries, JIT-neutral translated-block caching and invalidation, bounded GPU command/shader translation caches, WebGPU presentation, AudioWorklet delivery, 64-bit sparse storage, and demand-hydrated media ranges. Later console work supplies the ISA/device translators and machine-specific GPU semantics rather than adding separate browser emulator runtimes.

## Playable gate

A console moves to `playable` only when its OmniCore machine passes deterministic builds, legal regression software, input, video/audio, reset/state behavior, representative compatibility tests and acceptable performance in current browsers.

An external emulator existing somewhere is not sufficient. If the machine does not run inside `omnicore.wasm`, it is not OmniEmu support.
