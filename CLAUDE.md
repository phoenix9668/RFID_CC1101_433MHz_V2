# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Firmware for a battery-powered, neck-mounted cattle RFID collar. An STM32L051C8Tx (Cortex-M0+, 64 KB flash / 8 KB RAM) reads an ADXL362 accelerometer over SPI2, classifies cattle behaviour on-device, and periodically broadcasts a summary over 433 MHz via a CC1101 transceiver on SPI1.

Uses both the HAL and the LL driver sets (`USE_HAL_DRIVER` + `USE_FULL_LL_DRIVER` are both defined).

## Build

CMake + Ninja + `arm-none-eabi-gcc`, driven by presets. The toolchain file is `cmake/gcc-arm-none-eabi.cmake`.

```bash
cmake --preset Debug && cmake --build --preset Debug
```

Output lands in `build/Debug/RFID_CC1101_433MHz_V2.elf`. Swap `Debug` for `Release` (`-Os -g0`) as needed. Check the size fits the part:

```bash
arm-none-eabi-size build/Debug/RFID_CC1101_433MHz_V2.elf
```

The linker prints memory-region usage on every build; the budget is tight (~61% flash, ~65% RAM as of the CMake port), so watch it when adding code.

There are no unit tests and no host-runnable test target — a successful cross-compile and link is the only automated check. CI (`.github/workflows/build.yml`) runs exactly the two commands above on `panting` pushes/PRs.

Day-to-day development is through the official **STM32 VS Code Extension**, which drives the same CMake presets and provides ST-Link debug via `.vscode/launch.json`. `.clangd` points IntelliSense at `build/Debug/compile_commands.json`, so configure at least once before expecting working completion.

## Regenerating peripheral code

`RFID_CC1101_433MHz_V2.ioc` is a real STM32CubeMX project with `ProjectManager.TargetToolchain=CMake`. Regenerating from CubeMX rewrites the MX peripheral init files (`Core/Src/{adc,crc,gpio,iwdg,rtc,spi,usart}.c`, `stm32l0xx_hal_msp.c`, `stm32l0xx_it.c`) **and** `cmake/stm32cubemx/CMakeLists.txt`. Application logic lives inside `/* USER CODE BEGIN|END */` markers and survives regeneration — anything written outside those markers in a generated file will be lost.

Note the split: `cmake/stm32cubemx/CMakeLists.txt` is CubeMX-owned and lists the generated + HAL/LL sources; the root `CMakeLists.txt` is hand-maintained and lists only the application sources (`adxl362.c`, `cc1101.c`, `rand_bytes_gen.c`). Add new application files to the root file.

CubeMX headless/CLI generation (`-q <script>`) crashes on macOS with a Java NPE in this version — regenerate through the GUI.

## Architecture

### Duty-cycled main loop

`main()` initialises peripherals then spins in a loop that is, by design, asleep most of the time. `HAL_PWR_EnterSTOPMode` is entered whenever no work is pending, and SPI peripherals are `MX_SPIx_Init()`/`MX_SPIx_DeInit()`'d around each use rather than left running — power budget dominates the design. An independent watchdog (IWDG) is reloaded on every path through the loop; any long-running code you add must keep reloading it.

Three event flags, all set from ISRs, drive everything:

- **`rtc.tenSecIndex`** — RTC wakeup timer fires ~every 10 s (`Core/Src/rtc.c`, `HAL_RTCEx_WakeUpTimerEventCallback`). Every 6th tick (~60 s) the current `action_classify` counters are snapshotted into `step.*Array[step.stepStage]` and mirrored into data EEPROM.
- **`rtc.twentyMinIndex`** — every 120 ticks (~20 min): advance `step.stepStage`, sample battery via ADC, generate 32 random bytes, assemble and transmit the RF packet, then tear the SPI/CRC peripherals back down. A `rand() % 100` ms jitter is applied before transmit so a herd of collars doesn't collide on air.
- **`step.fifoOverrun`** — ADXL362 FIFO watermark interrupt: drain the FIFO over SPI2 and run `ADXL362FifoProcess()`.

### Behaviour classification

`Core/Src/adxl362.c` reads raw 12-bit three-axis samples out of the ADXL362 FIFO, de-interleaves them (the leading 2 bits of each sample's high byte tag the axis, and the FIFO may start mid-triplet — the code realigns), then classifies activity into six mutually exclusive buckets held in `action_classify_t`: `rest / ingestion / movement / climb / ruminate / other`.

`_Original_Data_Algorithm` in `Core/Inc/main.h` selects between algorithm variants and changes how much FIFO is read (1024 vs 900 bytes) and which peripherals must be live during processing. It is currently `1`.

`step_t` holds a 12-slot ring (`_STEP_LOOPNUM`) of per-behaviour counters — 12 × 20 min = 4 hours of history — which is what actually goes on air. `stepStage` and the counter arrays are persisted to data EEPROM (`DATAEEPROM_Program`, base `EEPROM_START_ADDR`) so a reset mid-deployment doesn't lose accumulated behaviour.

### RF packet

`CC1101SendHandler()` in `Core/Src/cc1101.c` builds `cc1101.sendBuffer` positionally. The layout is expressed inline as running offsets rather than a struct, so changing any size constant shifts everything downstream:

```
[0 .. _RFID_SIZE)                6 B   device ID (device.deviceCode1..6)
+ 32 B                                 RandomString — filler/obfuscation, NOT key material
+ 12 × 2 B × 6                         rest, ingestion, movement, climb, ruminate, other
                                       (big-endian uint16 per stage, one block per behaviour)
+ 1 B                                  step.stepStage
+ _BATTERY_SIZE (2 B)                  adc.avgValue
+ _RESETCNT_SIZE (2 B)                 resetCnt
+ _CRC32_SIZE (4 B)                    inverted STM32 hardware CRC32 over everything above
```

The CRC is `~HAL_CRC_Calculate(...)` over the payload — the STM32 CRC peripheral needs specific config to match standard CRC32; see the link in `README.md`.

### RNG

`Core/Src/rand_bytes_gen.c` fills `RandomString[32]` using xoshiro128++ seeded from `HAL_GetTick()`, the ADC average, and the 96-bit unique device ID at `UID_BASE`. This replaced a Keil-only precompiled ST cryptolib that could not link under GCC. It is deliberately **not** a CSPRNG: the output is only ever used as RF-packet filler, never as key material. Keep it that way unless the packet format changes.

## Debug output

`rfid_printf()` compiles to nothing unless `_RFID_PRINT_DEBUG` is `1` in `Core/Inc/main.h`. `_DEBUG` is a separate switch that enables the UART device-provisioning path (`Set_DeviceInfo()`, which programs the device ID from an `ABCD`-prefixed UART frame), LED blinking, and disables STOP mode — the collar will not sleep with `_DEBUG == 1`.

## Branches

`panting` is the active branch and the only one CI builds. It and `main` have **diverged in source layout**: `main` carries a refactor onto ADI's no-OS ADXL362 driver (`Core/Src/adxl362_app.c`, `adxl362_behavior.c`, `no_os_*.c`, `Drivers/ADI/`), while `panting` keeps the original self-contained `Core/Src/adxl362.c`. Their root `CMakeLists.txt` source lists differ accordingly, so don't cherry-pick build files between them without checking.

The Keil MDK-ARM project has been removed from `panting`; CMake is the only build system.

## Repository notes

The working tree lives inside a Dropbox folder, so files may change under you from another machine. `Doc/data/` and `Doc/Power Consumption Data/` are large data sets deliberately kept out of git (Dropbox-backed only). `Doc/Paper/` is tracked and holds the heat-stress/panting algorithm research: the source paper, collar recordings, and two validation efforts under `Doc/Paper/validation/` — the original one plus an independent re-verification (`独立复核报告.md`) that found the original's headline result was confounded by comparing across different animals.
