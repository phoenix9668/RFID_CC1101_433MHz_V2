# V3.8 Firmware Rebuild

Base: `34d57c2a93b57979fecb929067e49df47c17c14d`.
Branch: `codex/v38-rebuild-from-34d57c2`.
No later firmware implementation or later panting classifier is merged.

## Development

Use STM32CubeIDE for Visual Studio Code 3.10.0. This rebuild uses STM32CubeCLT
1.18.0, GNU Arm GCC 13.3.1, CMake 3.28.1 and Ninja. HAL/LL remains the
base STM32CubeL0 V1.12.2. Installing a newer ST bundle does not change the
compiler selected by the local presets.

Run `Tools/setup-local.ps1` with your CubeClt directory, native HostGcc executable,
Python executable and optional CsvData directory. It generates ignored
`.local/tools.json` and `CMakeUserPresets.json`; do not commit these files.
Example (replace the environment variables with your installed tool paths):

```powershell
./Tools/setup-local.ps1 -CubeClt $env:STM32_CUBE_CLT -HostGcc $env:HOST_GCC -Python $env:PYTHON -CsvData $env:RFID_CSV_DATA
./Tools/open-vscode.ps1
./Tools/build.ps1 -Preset Debug
./Tools/verify.ps1
```

Select `Debug-local`, `Release-local` or `Diagnostics-local` in the ST CMake
preset selector. The first two use production timing and disable UART logs.
Diagnostics also uses real timing; it adds bounded UART logs, not faster
report intervals or raw-data radio packets. An additional opt-in
`RFID_DIAGNOSTIC_HOLD_AWAKE` bench switch isolates STOP effects; it is OFF in
every preset and rejected outside Diagnostics. Build outputs are ELF, HEX, BIN,
MAP, per-source stack usage and compile_commands.json under the selected build
directory. The Firmware task uses the same preset-aware build script.

`.settings/ide.store.json` matches ST's official project creator output for
the BOM part STM32L051C8T6; its source type identifies the CubeMX boundary.
The existing IOC uses STM32L051C8T3, the same memory/peripheral target with a
different temperature suffix. No device pack startup or HAL replaces the
pinned base libraries.

## Debugging Safety

Two `stlinkgdbtarget` configurations are provided: build/download/debug and
symbols only. Open VS Code using the script above so both select the same CLT
as the command-line build. Symbols-only does not download or request a reset;
attachment can still halt execution and may fail while production code is in STOP.

Back up first with `Tools/backup-board.ps1` and preserve its complete manifest.
It reads Flash, EEPROM twice, raw option bytes and decoded option bytes.
Private identifiers, probe serials, UART captures and dumps stay in `.local/`.
The standalone `Tools/st-debug-smoke.cjs` checks the backup hashes and checks
ELF load segments are entirely within internal Flash before a download.
A VS Code download must be used only after the same backup has been verified.
If old firmware ran after the original backup, supply an explicitly captured
same-device pre-download EEPROM image with `--legacy-reference` to the smoke
script. Keep the original backup too; never replace it with the new reference.
Use `Tools/inspect-eeprom.cjs dump.bin reference.bin` to inspect CRC-valid journal
records and compare the old region without printing the device identifier.

ST GDB Server 7.10.0 binds 0.0.0.0 even when serverHost is localhost; that setting
is a client destination, not a binding restriction. Both entry points now
require a dedicated deny-only Windows Firewall rule before starting.
An administrator must run `Tools/debug-network.ps1 -Install` once. It blocks
non-loopback inbound TCP for the exact configured ST server; it does not
remove rules or open ports. Normal launches only validate the active rule.
The installation and a subsequent protected debug session remain unverified
on this machine until administrator setup is available.

Semihosting, RTOS proxy and live watch are disabled. Breakpoint sessions can
retain low-power debug and freeze the watchdog; they are NOT current tests.
For production measurements use Release, independently reset/run, disconnect
SWD and UART, and verify no debug freeze remains. Do not accept debug-session
current readings as production results.

## Ownership

- `Core/`: generated peripheral initialization, ST system source and pinned startup.
- `Platform/`: pin ownership, bounded buses, ADI adapter, radio, EEPROM and STOP.
- `App/behavior`: original integer six-class algorithm, private singleton state.
- `App/fifo_parser`: tagged XYZ assembly; rejects invalid or incomplete samples.
- `App/history`: six counters, twelve windows and rollover.
- `App/protocol`: hardware-independent 191-byte compatibility contract.
- `App/storage`: power-cut tolerant journal and one-time legacy import.
- `App/app`: event claiming, acquisition priority, reporting and checkpoint policy.
- `Tests/legacy`: test-only source extracted from the exact base.

IRQ handlers only acknowledge hardware and record events. Claiming events and
the final pre-STOP check run with saved/restored PRIMASK. RTC time keeps report
progress across STOP; SysTick is only for awake peripheral deadlines.
RTC wakeup is at most 10 seconds and is shortened to meet the next checkpoint
or window boundary. FIFO watermark interrupts normally cause a read
around every 6 seconds. Checkpoint and rollover can drain a partial FIFO.
Each 25 valid XYZ samples retains the original classification output and delay.
FIFO parsing loss or overflow resets algorithm history rather than bridging
the gap with fabricated samples.

The nominal FIFO occupancy timestamps determine the reporting boundary:
a classification ending exactly at 20 minutes belongs to the completed window.
ADXL362 oscillator tolerance means real totals need not be exactly 1200.
Do not resample or fabricate seconds to force that total.

A complete window is journaled, copied into history, and the next slot becomes
current before the exact report snapshot is encoded. TX has a bounded deadline.
There is no ACK or endless retry; failed reports remain represented by the
history carried in later reports. Sensor failures retry at 10-second intervals.
An EEPROM write failure disables further journal writes until reboot, preserving
the last valid record; RAM acquisition/reporting continue. The 60-second loss
target does not apply to an EEPROM fault or to data still awaiting acquisition.

## Regeneration

Generate CubeMX output into a staging directory, inspect the diff, and merge
only the generated Core/IOC changes. Do not replace the root CMakeLists, App or
Platform with generated files. The active entry point and interrupts live in
Platform, so a newly generated Core/main.c must not be added to the target.
The IOC tracks 4 MHz SPI, long ADC sampling, MCO disabled and the RTC reload.

Original Keil project/crypto dependencies are recoverable from the base commit.
The diagnostic UART is read-only; the old manufacturing identity-write command
is not a production feature and is not implemented here. Provisioning needs a
separate explicitly authorized workflow.

See [sources](SOURCES.md), [protocol/storage](PROTOCOL_STORAGE.md),
[hardware](HARDWARE.md) and [verification](VERIFICATION.md).
