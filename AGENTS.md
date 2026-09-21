# Project Development Guide

## Context

- Target: STM32L051C8, Cortex-M0+, 64 KB Flash and 8 KB RAM.
- Firmware: C11 and assembly, CMake/Ninja, arm-none-eabi GCC.
- HAL and LL are intentionally mixed. Preserve the selected peripheral API.
- Use Chinese for project discussion unless the user requests another language.
- Follow the user's requested scope. A planning or review task does not authorize
  implementing every issue discovered during that task.

## Ownership and Compatibility

- `RFID_CC1101_433MHz_V2.ioc` owns CubeMX peripheral configuration. Preserve
  generated-file USER CODE sections and keep application logic in owned modules.
  Update generated configuration through the IOC when changing hardware setup.
- `Core/Src/adxl362_app.c` owns sensor setup and compatibility wrappers;
  `adxl362_behavior.c` owns FIFO parsing and behavior classification.
- `Drivers/ADI/no-OS/` contains the pinned ADI driver. Preserve upstream files and
  license headers; platform adaptations belong in `Core/Src/no_os_*.c`.
- `Core/Src/cc1101_driver.c` owns SPI1, registers, FIFO and radio operations;
  `cc1101_app.c` owns RFID payload assembly. Keep `Core/Inc/cc1101.h` compatible
  unless the requested task includes a public API change.
- Preserve RF register values, payload lengths/offsets, CRC coverage and byte
  order, EEPROM layout, RTC periods, RNG behavior, battery/reset fields and
  interrupt polarity unless the task explicitly changes them.
- The TI provenance is recorded in `Drivers/TI/CC1101_SWRC021_REFERENCE.md`.
- Register/FIFO waits must be bounded. Propagate failures before consuming read
  buffers; recording an error alone does not make a partial transfer valid.
- Keep interrupt-shared state volatile and account for events arriving between
  foreground reads and clears. Avoid unbounded allocation on this RAM budget.
- Preserve unrelated work and vendored code. Add abstractions only when they
  simplify the affected module.

## Build

Run from the repository root with CMake, Ninja and arm-none-eabi GCC on PATH:

```powershell
cmake --preset Debug
cmake --build --preset Debug --parallel 1
```

For an isolated verification directory:

```powershell
cmake -S . -B build/codex-verify -G Ninja -DCMAKE_BUILD_TYPE=Debug '-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake'
cmake --build build/codex-verify --parallel 1
```

Quote toolchain definitions in PowerShell. If Ninja is not resolved, pass its
installed path as a quoted `-DCMAKE_MAKE_PROGRAM=...` argument. Tool locations
belong to the local environment, not portable CMake source files. One build job
also avoids concurrent depfile access in this Dropbox workspace.

## Verification

- Scale tests to the change. Configuration/documentation changes need config
  validation and diff checks; a workflow migration may also establish a build
  baseline. Do not repeatedly rebuild unchanged firmware.
- Firmware changes require a successful relevant build and a review of warnings
  and memory usage. Compilation alone does not verify radio or sensor hardware.
- For behavior changes, use `Tools/adxl362_pc/` to compile the production
  algorithm. Follow its README and preserve 25 Hz sampling and 150-sample blocks.
  Compare per-sample outputs and classification counts against the current
  pre-change baseline. Archived `original/` sources are historical references,
  not automatically the desired current behavior.
- Representative fixtures: `chuanxi.csv`, the 2025-08-20 16:44 panting recording,
  and `02 1004 20230315 1819-2219 29W.csv`, all under `Doc/data/`.
- Radio changes require payload/CRC checks and short/long TX plus RX error-path
  coverage. Report hardware smoke tests separately from host/static checks.
- Put local binaries, maps and test outputs under ignored `build/` or `output/`.
  Record missing toolchains, failed tests and unperformed hardware checks plainly.

## Codex Defaults

`.codex/config.toml` selects GPT-6 Astra with high reasoning for this project.
It does not change the firmware or add an API dependency. Keep credentials,
machine-specific paths and permission overrides out of this shared config.
Model defaults do not by themselves prove an already-running task changed model.
