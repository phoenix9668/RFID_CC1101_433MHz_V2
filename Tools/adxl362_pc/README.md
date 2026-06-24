# ADXL362 PC algorithm runner

This harness compiles the production `Core/Src/adxl362_behavior.c` and
`Core/Inc/adxl362.h` directly. The firmware algorithm and its data structures
are not copied or modified. PC-only headers replace STM32 peripherals, and
`src/main.c` converts decoded CSV `x,y,z` samples back into the ADXL362 FIFO
byte format expected by `ADXL362FifoProcess()`.

## Data contract

- CSV header must contain `x`, `y`, and `z`.
- Values must be integer ADXL362 12-bit samples in `-2048..2047`.
- Sampling rate is 25 Hz.
- One firmware FIFO call consumes 150 samples (6 seconds / 900 bytes).
- An incomplete final 150-sample block is reported and ignored.
- CSV fields containing embedded commas are not supported by this small runner.

## Build and run

```sh
cd Tools/adxl362_pc
make
./build/adxl362_pc ../../Doc/data/chuanxi.csv result.csv
```

Use the firmware's `rfid_printf` diagnostics:

```sh
./build/adxl362_pc --verbose ../../Doc/data/chuanxi.csv result.csv
```

Build with AddressSanitizer and UndefinedBehaviorSanitizer:

```sh
make sanitize
./build-sanitize/adxl362_pc ../../Doc/data/chuanxi.csv result.csv
```

The output preserves each processed input row and appends a `behavior`
column. The behavior number is repeated for the corresponding 25 samples:

- `0`: initial delay
- `1`: rest
- `2`: ingestion
- `3`: movement
- `4`: climb
- `5`: ruminate
- `6`: other
- `7`: breath
