# GPT-6 Astra Development Workflow

## Scope and Configuration

This is an STM32L051 firmware repository with no OpenAI API integration found in
the inspected application and tool sources. The migration configures the coding
workflow. The existing user default was already `gpt-6-astra` with `high`
reasoning; `.codex/config.toml` now pins those values for this project.

`AGENTS.md` records the toolchain, module ownership, compatibility requirements
and verification workflow. Credentials, providers and permission settings are
not changed. Host-only PC runner portability changes described below enable the
verification on the installed MinGW runtime.

Validation used the installed Codex app-server with `--strict-config`, the
`initialize` handshake, and `config/read` with `cwd` set to this repository and
`includeLayers: true`. In the actual user environment it returned:

```text
model: gpt-6-astra
model_reasoning_effort: high
origin of both settings: project .codex/config.toml
```

The bundled model catalog also lists Astra and supports `high`. The isolated
shell sandbox uses a different Windows account and skips untrusted project
layers, so its initial config result was not used as the user's effective
configuration. No trust setting was changed by this task.

This verifies loaded defaults, not the identity of an already-running task or
availability on another account. Explicit task/model overrides still apply.
To remove the project pin, remove its two model settings and use the desired
model in the client. The previous user default was also Astra, so removing this
file alone does not switch back to an older model.

Official references:

- [Astra migration guidance](https://developers.openai.com/api/docs/guides/latest-model)
- [Codex models](https://learn.chatgpt.com/docs/models)
- [Project configuration and precedence](https://learn.chatgpt.com/docs/config-file/config-basic)

## Firmware Baseline

Baseline source revision: `3ae1254`. Firmware source, radio payload assembly,
vendor drivers, IOC, linker script and CMake files are unchanged in this task.

The new build directory was configured and built with STM32CubeCLT 1.18.0,
CMake 3.28 and arm-none-eabi GCC 13.3.1:

```powershell
cmake -S . -B build/codex-astra-verify -G Ninja -DCMAKE_BUILD_TYPE=Debug '-DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake' '-DCMAKE_MAKE_PROGRAM=C:/ST/STM32CubeCLT_1.18.0/Ninja/bin/ninja.exe'
cmake --build build/codex-astra-verify --parallel 1
```

All 50 build steps passed, with no compiler warnings or linker errors in the
successful run. RAM: 5960 / 8192 bytes. Flash: 50940 / 65536 bytes. The initial
unquoted PowerShell toolchain argument was split incorrectly; the command above
is the verified form.

## Behavior Baseline

The PC runner compiles the unchanged production `adxl362_behavior.c`. Native
MinGW GCC 6.2.0 from the installed Vivado tools linked successfully after adding
the Windows-only `pc_getline` compatibility implementation. POSIX builds still
use the system `getline`. The reader test passed empty input, CRLF, blank lines,
a 4096-character line requiring growth, a final line without newline, and EOF
on a reused buffer. Both native binaries compiled with `-Wall -Wextra -Werror`.

See `Tools/adxl362_pc/README.md` for portable commands. This machine's native
compiler was:

```text
C:/Xilinx/Vivado/2020.2/tps/mingw/6.2.0/win64.o/nt/bin/gcc.exe
```

The old MinGW runtime could not open the Chinese input filename. The fixture
was copied to `build/codex-astra-verify/panting-input.csv`, and its SHA256 was
verified against the original before accepting the run. All outputs remain in
the ignored `build/codex-astra-verify/` directory.

| Input in Doc/data | Output name | Input rows | Output rows | Discarded tail |
| --- | --- | ---: | ---: | ---: |
| chuanxi.csv | chuanxi.csv | 19709 | 19650 | 59 |
| 喘息20250820 1644.csv | panting-20250820-1644.csv | 77846 | 77700 | 146 |
| 02 1004 20230315 1819-2219 29W.csv | 29w.csv | 296249 | 296100 | 149 |

Counts are per sample, with one classification repeated over each 25-sample
second. Tail rows below a full 150-sample FIFO block are intentionally ignored.

| Behavior | chuanxi | 2025-08-20 16:44 | 29W |
| --- | ---: | ---: | ---: |
| 0 initial delay | 325 | 400 | 400 |
| 1 rest | 425 | 10825 | 94550 |
| 2 ingestion | 0 | 2250 | 0 |
| 3 movement | 200 | 6000 | 25 |
| 4 climb | 0 | 0 | 0 |
| 5 ruminate | 0 | 2600 | 76800 |
| 6 other | 1925 | 31675 | 56275 |
| 7 breath | 16775 | 23950 | 68050 |

The full `29w.csv` output is byte-identical to the existing
`02_1004_20230315_1819-2219_29W_latest_result_precommit.csv` and
`02_1004_20230315_1819-2219_29W_latest_result.csv`. The other two runs establish
current baselines; they are not claimed as comparisons against older outputs.

SHA256 fingerprints, in the table order above:

```text
Inputs:
8AAD22FDF6B5DC8E786F94A0F0795D84E77073F30B2D59422878CA5D41BC6AB5
0EA049C7F6CB7E1AB4756C44C0F11EB56A948E7E1FB56F84B5792584E1AEB9BF
CB0B90E91E14584BEA23927021697E684C849C3F3C149562064D185CA158B789

Outputs:
9F18412E48FAB2E42386FEFD9DFFB5F876AC9F5FA4C0323B3CAE2C9B251EBFC1
48CC239B91FEA094B9564FBD3670960380A157806C772CE236FD524ECCC9D542
A8CB232566BD8F76D2F4D4665F0F242A80835319AB8CBE330481B19E80217AEE

Core/Src/adxl362_behavior.c:
E89F0156F2391EDCD154C03911702A32DEAC2E73CCB55E3BD3DA450DCB213831
```

## CC1101 Review Exercise

This static review inspected timeout/error propagation and GDO event handling
at revision `3ae1254`. Findings below remain open; the workflow migration does
not modify radio firmware.

1. **P1: RX transfer failures can consume uninitialized status bytes.**
   `Core/Src/cc1101_driver.c:478` declares an uninitialized `status[2]`.
   `CC1101ReadMultiReg()` returns void and may exit before filling the buffer on
   SO/SPI timeout (`:202`). `CC1101RecPacket()` nevertheless reads RSSI and CRC
   bits at `:507` through `:513` and may return a positive length. A follow-up
   should propagate each transfer result and reject incomplete reads before
   examining the payload or status; zero-initialization alone is insufficient.
   Verify with failures before the status burst and between its two bytes.

2. **P1: CRC failure can still dispatch a received command.**
   `Core/Src/cc1101_driver.c:516` records CRC failure and returns `1`. At
   `Core/Src/cc1101_app.c:73` every nonzero length enters command parsing without
   checking that error or a minimum payload length. A corrupted packet whose
   device fields and command still match can therefore return `0xC0` as if
   valid. A follow-up must distinguish successful payload length from failure
   and check the required fields are present before dispatch. Test a matching
   command with CRC clear and truncated packets alongside valid packets.

3. **P2: TX completion can be lost when foreground code clears a shared flag.**
   `Core/Src/cc1101_driver.c:438` unconditionally clears `rxCatch` just before
   waiting for completion. `Core/Src/stm32l0xx_it.c:193` sets the same flag on
   either configured EXTI edge. Completion arriving before that clear is lost;
   start and end events may also collapse into one flag while the foreground
   is delayed. A successful transmission can then be reported as a timeout.
   Verify both edges arriving before polling and completion during refill;
   preserve the configured hardware polarity when designing the correction.

Hardware register readbacks, SPI fault injection and short/long over-the-air
TX/RX were not performed. The build and behavior baselines do not close these
radio findings. No claim of complete CC1101 correctness is made.
