# References

OmniEmu is implemented as its own single Rust/WebAssembly emulator core. The following standards and documentation families guide the implementation; they are references, not runtime dependencies.

## Browser platform

- WebAssembly core specification and JavaScript API.
- WebGL 2 / OpenGL ES 3.0 rendering model.
- Web Audio API and AudioWorklet model.
- Gamepad API.
- WebGPU specification for the future high-end rendering path.
- SharedArrayBuffer and WebAssembly threads/SIMD browser capabilities.

## Hardware research

Machine implementations should be grounded in original hardware manuals, chip documentation, publicly documented file/container formats, and independently verifiable test ROMs. Where community hardware documentation is used, behavior should be confirmed against test suites rather than copied as an assumption.

## Project rule

Third-party emulators may be consulted for public compatibility research and differential testing where their licenses permit it, but OmniEmu does not download or select those emulators at runtime. Console implementations must execute through `omnicore.wasm` before the UI marks them playable.
