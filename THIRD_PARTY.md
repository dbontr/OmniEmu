# Third-party software

OmniEmu's emulation runtime is built from this repository's `core/` source. It does not fetch third-party console emulator cores at runtime.

## coi-serviceworker

The static site uses `coi-serviceworker` to provide cross-origin isolation on hosts such as GitHub Pages where application code cannot configure HTTP response headers directly.

License: MIT. The installed package license and source attribution remain available under `node_modules/coi-serviceworker` during development.

## Build tooling

TypeScript and Vite are development/build dependencies. Rust's standard toolchain compiles OmniCore to `wasm32-unknown-unknown`.

## User content

Games, BIOS/firmware, console keys, and other proprietary system content are not bundled with OmniEmu. Users must supply their own lawful local copies where a machine requires them.
