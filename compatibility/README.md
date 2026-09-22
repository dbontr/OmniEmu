# Compatibility corpus

OmniEmu uses a manifest-driven compatibility runner to distinguish a runnable machine foundation from demonstrated software compatibility. The runner executes the same optimized `public/omnicore.wasm` artifact used by the browser; it does not load another emulator core.

The repository contains only the ROM-free `smoke.json` gate. Copyrighted games, proprietary firmware, console keys, and private compatibility corpora must remain outside the repository. A local manifest may reference user-owned files by absolute path or by a path relative to that manifest.

## Run a corpus

Build OmniCore first, then pass a manifest to the runner:

```text
npm run core:build
node scripts/compatibility-runner.mjs C:\path\to\my-corpus.json --report C:\path\to\report.json
```

`npm run verify` also runs `compatibility/smoke.json`. The repository smoke corpus covers all 29 active target graphs with built-in machines or synthetic ROM-free fixtures and verifies deterministic save-state replay, expected video/audio surfaces, and persistence contracts where applicable.

## Manifest contract

A manifest uses schema `omniemu-compatibility/1` and contains one or more cases. Each case identifies a permanent numeric platform id, its resources, the number of frames to execute, optional fixed controller input, and objective expectations.

```json
{
  "schema": "omniemu-compatibility/1",
  "cases": [
    {
      "name": "My lawful NES regression",
      "platform": 20,
      "supportAtLeast": 1,
      "frames": 600,
      "resources": [
        {
          "kind": "game",
          "path": "C:\\Games\\owned-test.nes",
          "sha256": "<known-sha256>"
        }
      ],
      "determinism": {
        "replayFrames": 30
      },
      "expect": {
        "video": {
          "width": 256,
          "height": 240,
          "nonzero": true,
          "sha256": "<optional-final-frame-sha256>"
        },
        "audio": {
          "rate": 48000,
          "channels": 2,
          "minSamples": 1,
          "sha256": "<optional-final-buffer-sha256>"
        },
        "minHostFps": 60,
        "persistent": [
          {
            "kind": "storage",
            "slot": 0,
            "length": 8192
          }
        ]
      }
    }
  ]
}
```

Resource kinds are `game`, `bios`, `firmware`, `keys`, `disc`, `storage`, `memoryCard`, and `nand`. A numeric resource kind `0` through `7` is also accepted. Slots default to zero.

Disc and NAND resources stream by default. The runner stages them through the same bounded resource ABI used by the browser and hydrates requested ranges on demand. Other resources are staged in 1 MiB chunks, so the compatibility harness does not need a second full-size WASM copy of a large file.

A resource `sha256` is optional but strongly recommended for a durable corpus. It makes a result meaningful without placing proprietary bytes in the repository. Reports contain case results and hashes but do not copy source file paths or source bytes.

## Promotion gates

`foundation` means a machine graph is runnable. Promotion to `playable` requires a representative legal corpus that covers boot, CPU behavior, video, sound, input, media, reset, save-state/persistence behavior, deterministic replay, and sustained browser performance. A `validated` claim should additionally require broad software coverage and hardware/reference-emulator differential tests for timing-sensitive behavior.

Compatibility manifests should be split by platform and capability so a regression identifies the hardware area that failed. Final-frame hashes are useful for stable deterministic tests, but they should be supplemented by longer-run and targeted hardware tests rather than treated as proof of universal accuracy.
