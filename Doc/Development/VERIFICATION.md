# Verification Record

Date: 2026-09-21. Base: 34d57c2a93b57979fecb929067e49df47c17c14d.

## Software Passed

- Clean Debug, Release and Diagnostics builds in build/final-clean using
  CLT1.18.0 / GCC13.3.1 / CMake3.28.1. No compiler/linker warnings in that run.
- Five CTest targets pass: behavior_synthetic, core_test, sensor_test,
  radio_test, app_test. New native tests use assertions and -Werror; the
  mechanically extracted legacy algorithm is compiled separately.
- The base algorithm matches for 13200 synthetic seconds and 15738 CSV seconds.
  CSV parsing explicitly selects x/y/z, including the wide CSV.
  Source/result hashes and six-class totals are in csv-regression.json.
- Exact 191-byte encoding matches the base CC1101SendHandler with fixed inputs
  and an independent model of the original STM32 CRC peripheral.
- Official ADI 900/1024-byte chunking and each chunk failure, bad ID/reinit,
  malformed/partial FIFO data and overrun handling pass mocked tests.
- RF tests cover the exact base 47-byte table; 1/59/60/61/120/121/180/191/240/254
  payloads; 255 rejection; SO stuck before/after SRES; SPI failure; TX timeout
  and underflow; RX streaming, CRC/address rejection, timeout and overflow.
- EEPROM tests cover all 117 initial import write boundaries, torn words,
  nine stages of record reuse, CRC corruption, sequence wrap and invalid legacy
  stage/progress/per-class/total counts. Old offsets 0..23F stay unchanged.
- A 4h20 SIMULATION covers 13 reports, twelve-window wrap, failed TX,
  reset/checkpoint recovery and events arriving during EEPROM writes.
  This is not an uninterrupted 4h20 physical board run.
- A one-millisecond deadline test covers unaligned boot/checkpoint timing.
  RTC wakeup is re-armed before STOP, with a maximum ten-second interval and
  a shorter interval when the next checkpoint/window deadline is nearer.
- Each of the first two staged milestones was separately exported from the
  Git index and built/tested before its commit.
- PowerShell scripts and JSON settings parsed; Node scripts passed syntax checks.
  Diagnostics-local was built with the same script/preset used by the VS Code task.

## Memory

Current linked bytes:

| Preset | Flash | RAM including 2048-byte minimum stack reservation |
| --- | ---: | ---: |
| Debug | 22232 | 4408 |
| Release | 20228 | 4400 |
| Diagnostics | 26216 | 4816 |

Capacity is 65536 Flash / 8192 RAM. There is no heap. The linker asserts at least
2048 bytes between static data and the stack top; unallocated RAM is extra
headroom, not measured runtime stack consumption.

Diagnostics .su reports 528 bytes for the official FIFO helper, 360 for
storage_restore, 200 for log_state and 128 for process_fifo. Individual frames
are not a complete call-chain bound, especially including libc and exceptions.
Runtime stack watermark/worst-case interrupt stress remain pending.

The downloaded final Diagnostics-local ELF SHA256 is
2E597FAB4C147A9BE897C6BDAD7A4D209D969B9CF81CCAAD322D7CC415E5A68E.
HEX SHA256 is 6A823BA02515CF614E70F79850F2E62D5BDE56AD750DAEA67C706877982D6866.
Symbols include build paths, so an ELF hash from a different source directory
can differ. Vendor hashes are separately fixed in SOURCES.md.

## Physical Evidence

- Under-reset SWD at 100 kHz: Cortex-M0+, device ID 417, revision X,
  64 KiB Flash, approximately 2.93-2.97 V.
- Flash 64 KiB, EEPROM 2 KiB (two identical reads), raw and decoded option bytes
  backed up with SHA256 before the first download. All private files are ignored.
- RDP AA (level 0), software IWDG, BOR off. No option bytes were changed.
- The earliest old image showed invalid progress/counts; user approved the
  validation policy. Old firmware continued running between that backup and
  the first download, so its legacy progress/reset/statistics changed in that
  interval. That old writer activity must not be attributed to the new journal.
- A first ST stlinkgdbtarget download verified successfully and halted at main.
  A network audit found an all-interface listener; the session was closed.
  Subsequent USB probe failures recovered after disconnecting USB and board
  leads. No probe firmware update, driver change or RDP change was attempted.
- Runtime checks then used the official ST Programmer (no network listener)
  and receive-only UART. Final normal-STOP Diagnostics was downloaded, verified,
  hard-reset and left running. The temporary awake-only experiment is not active.
- ADXL362 IDs AD/1D/F2 and every configured register readback passed repeatedly.
  FIFO transfers run continuously without reported parsing, SPI or overrun errors.
- A restored window crossed its real 20-minute boundary at RTC t=180004 ms,
  having booted with 1020 seconds already saved. The next slot became 9.
  This is a resumed real boundary, NOT a new uninterrupted 20-minute test.
- CC1101 PARTNUM=00, VERSION=14 and critical configuration readbacks passed.
  The 191-byte TX state machine returned tx-ok; sensor/radio/storage errors
  remained 0/0/0. There is no independent base-station receipt yet.
- Deadline-aligned checkpoints were observed at t=60000, 120004, 240004 and
  300004 ms, including partial FIFO drains at those deadlines.
- Post-window EEPROM contained 36 valid CRC records. The newest checkpoint
  was stage 9 / elapsed 120 / reset 26 / counts [95,0,0,0,0,0].
  Device identity and bytes 0..23F matched the post-old-firmware reference.
  Raw option bytes still matched the original backup.
- After subsequent hardware reset/final download, UART reported stage 9 /
  elapsed 120 / reset 28, confirming restoration of saved progress. Read-only
  under-reset sessions themselves can restart execution on CLI disconnect.

## Sampling Rate Gate: NOT PASSED

Both normal STOP and awake-only comparison runs observed 453 FIFO words
(151 XYZ samples) roughly every 7.65 seconds: approximately 19.75 samples/s,
despite FILTER_CTL=51 and POWER_CTL=02. The awake-only run read these registers
every ten seconds; they remained correct with successful SPI reads. SysTick,
RTC and host timestamps agree; RTC PRER=007F00FF, WUTR=20479 and LSE selection
were read back. Disabling STOP did not remove the discrepancy.

Consequently, retaining 25 input samples per classification currently yields
about 0.79 classifications per real second on this board; the two-minute
checkpoint's 95 outputs agrees with that observation. The requirement of one
classification per real second is NOT physically accepted. No resampling,
threshold change, undocumented register tweak or shorter window masks it.

The source of the ODR discrepancy is unresolved. Further evidence should
include a reference board/sensor or logic-analyzer measurement of data-ready
and SPI, checking actual sensor supply/clock and comparison with equivalent
base firmware. This is not sufficient evidence to declare a defective or
counterfeit sensor. See the ADI source/datasheet links in SOURCES.md.

## Remaining Acceptance

The user has confirmed a DSLogic U3Pro16; Windows USB enumeration and the
installed DSView 1.3.2 are confirmed. At the user's request, probe wiring and
waveform capture are deferred to a later session. ADXL362_CAPTURE.md records
the next passive timing/SPI tests. No new firmware was flashed in this step.
The base station is not currently on site, so receipt tests are explicitly
deferred, not passed or inferred from the collar's tx-ok result.

- Resolve measured ODR and confirm real six-second acquisition.
- Validate 900 and 1024 byte worst-case FIFO paths physically, including
  intentional overflow/error recovery; regular observed reads were 906 bytes.
- Observe short packets and RX error paths physically.
- Confirm exact receipt/decoding by the unchanged base station.
- Run at least 4h20 uninterrupted plus reset/power-cut scenarios.
- Measure stack watermark, ADC accuracy, sleep/current/charge against an
  equivalent production baseline with debug and UART disconnected.
- Install/verify the narrow Windows Firewall restriction before any further
  ST interactive debug session; protected symbols-only attachment is untested.
- Verify VS Code GUI preset selection, indexing and artifact navigation.
  Metadata was validated against ST's creator, but GUI clicks are not claimed.

A pre-existing broken refs/codex/turn-diffs checkpoint caused automatic Git
maintenance to report an error after the first successful commit. Subsequent
commits disabled automatic maintenance per command. No unrelated refs were
deleted or repaired. Original main checkout and user modifications are preserved.

## Rollback

Keep the complete private backup outside disposable build output. Match the
connected device identity and backup hashes first. Program only the original
Flash image at 08000000 and verify; do not mass erase or change options.
EEPROM restoration is a separate destructive decision because it replaces new
statistics. Restoring only Flash leaves the preserved old layout available.
Never use read-unprotect, alter RDP or erase identity as a connection workaround.
The original source remains available at the base SHA.
