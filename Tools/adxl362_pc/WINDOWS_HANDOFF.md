# ADXL362 Windows Handoff

## What To Transfer

This workspace contains the current ADXL362 algorithm work and a PC runner for
validating firmware behavior from CSV data.

Key files:

- `Core/Src/adxl362.c`
- `Core/Inc/adxl362.h`
- `Tools/adxl362_pc/`
- `Doc/data/02 1004 20230315 1819-2219 29W.csv`
- `Doc/data/chuanxi.csv`

Original firmware backup:

- `Tools/adxl362_pc/original/adxl362.c`
- `Tools/adxl362_pc/original/adxl362.h`

## Current Verified Result

Input:

```text
Doc/data/02 1004 20230315 1819-2219 29W.csv
```

Old algorithm output:

```text
Tools/adxl362_pc/02_1004_20230315_1819-2219_29W_original_result.csv
```

New algorithm output:

```text
Tools/adxl362_pc/02_1004_20230315_1819-2219_29W_latest_result.csv
```

Key behavior comparison:

```text
ingestion(2): old=0, new=0, per-sample mismatch=0
movement(3):  old=25, new=25, per-sample mismatch=0
climb(4):     old=0, new=0, per-sample mismatch=0
```

Overall change:

```text
old 5 -> new 7: 68050 samples
old 5 -> new 5: 76800 samples
all other classifications stay the same
```

## Windows Setup

Recommended tools:

- Git for Windows
- MSYS2 or MinGW-w64 with `gcc` and `make`

In MSYS2 MinGW64, install tools if needed:

```sh
pacman -S --needed mingw-w64-x86_64-gcc make
```

Build and run latest algorithm:

```sh
cd Tools/adxl362_pc
make clean all
./build/adxl362_pc "../../Doc/data/02 1004 20230315 1819-2219 29W.csv" \
  "02_1004_20230315_1819-2219_29W_latest_result_windows.csv"
```

Verbose diagnostics:

```sh
./build/adxl362_pc --verbose "../../Doc/data/02 1004 20230315 1819-2219 29W.csv" \
  "02_1004_20230315_1819-2219_29W_latest_result_windows.csv" \
  > printf_output_windows.txt
```

## Notes

- `Tools/adxl362_pc/result.csv` was not used for the 29W comparison and should
  not be overwritten unless intentionally testing a small default output.
- `Tools/adxl362_pc/.gitignore` ignores generated CSV files. If you want to
  move result CSVs through Git, add them with `git add -f`.
- The PC runner compiles the firmware `Core/Src/adxl362.c` directly. It is not a
  separate reimplementation of the algorithm.
