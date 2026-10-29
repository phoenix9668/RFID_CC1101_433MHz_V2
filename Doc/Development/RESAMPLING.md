# Scheme 2: 50 Hz acquisition, 25 Hz behavior input

Status: hardware-tested candidate. Real FIFO data have been replayed against the
board outputs, and the latest 16 consecutive sealed windows on the tested V3.8
board each contain 1200 counts. Behavior accuracy, base-station reception and
production energy remain unqualified. See the
[2026-10-27 window-count validation](../ADXL362_Window_Count_Validation/README.md).
The classifier in `App/Src/behavior.c`, RF payload, EEPROM layout and 20-minute
RTC window are unchanged.

## Acquisition and time reconstruction

- Normal firmware selects FILTER_CTL=0x52 (4 g, 50 Hz, HALF_BW). POWER_CTL remains 0x02.
- ODR/EXTCLK bench images deliberately retain 0x51 so existing clock experiments remain reproducible.
- ACT/INACT register bytes are unchanged. Their physical timing changes with ODR;
  these detectors do not drive the current application's FIFO classification path.
- Keep the 450-word/150-XYZ watermark. The observed slow part is expected to reach
  it in about 3.8 seconds at the 50 Hz setting, NOT 6 seconds. Healthy 50 Hz parts
  reach it in about 3 seconds. Full FIFO capacity is only 512 words.
- `sample_clock` observes RTC milliseconds and the words actually drained. The
  first batch is discarded as a time anchor. At least 2 seconds establish the
  initial rate; later rate telemetry is updated over spans of at least 20 seconds.
- Accept 32..65 Hz with one-sample occupancy-quantization tolerance. An unsupported
  rate invalidates the calibration rather than silently assuming 39.5 or 50 Hz.
- Within each drained batch, assign uniform word times between consecutive read
  snapshots. This is a reconstruction, not individual hardware sample timestamps.
  It assumes no FIFO loss and bounded service latency; timing jitter is spread over
  a batch. Long-span observed ODR is diagnostic and sets the fallback read deadline.
- A deadline equivalent to approximately 160 samples handles a missing watermark
  interrupt. Before calibration it is 2.4 seconds; after calibration it is capped
  at 4.8 seconds. A deadline read drains even a below-watermark FIFO. The 160 versus
  170-full-sample margin assumes a slowly varying rate; a large abrupt rate change
  can still overrun, which discards/restarts rather than fabricating samples.
- A forced drain (checkpoint, window end or deadline) of fewer than 15 words is
  deferred to the next watermark batch. One to four samples cannot be timed
  against the previous read: their reconstructed spacing leaves the 10..40 ms
  window and would reset the resampler and classifier. Deferred samples are
  counted in whichever window is open when they are processed; sealed windows
  are never backfilled. The fault was found with the timing sweep described
  below; on the bench it was masked by the 169 ms batch processing time.
- At startup and faults, clear parser, resampler and behavior history. Genuine
  gaps and overrun samples are not interpolated into valid data.

## Resampler

- Fixed memory, no malloc, no floating point or trigonometry on the MCU.
- A 48-entry XYZ/time ring and a 321-entry int16 kernel lookup.
- Kaiser-windowed sinc, cutoff 8 Hz, beta 5, support +/-320 ms, 1 ms coefficient
  grid with coefficient interpolation. Regenerate with `Tools/generate-resampler-kernel.py`.
- SciPy `firwin(641, 8, fs=1000, window=('kaiser', 5))` supplies the kernel. Runtime
  normalizes the sampled weights for exact DC gain; accumulation uses int64.
- Arithmetic is narrowed where the exact result provably fits: tap distances
  below 320 ms use 32-bit division, products of a 16-bit weight and a 16-bit
  sample are formed in 32 bits before the 64-bit accumulation, and batch word
  times use a 32-bit product for spans up to 8.38 s. Outputs are bit-identical
  to the previous implementation (differential test of 6.8 million pushes plus
  the full workbook replay).
- Output signal times lie on a continuous 40 ms grid. The first center is a whole
  RTC second with complete filter support. There is no endpoint extrapolation.
- An output needs 320 ms of future support. Production accounting uses availability
  time, preserving the legacy classifier's emission-time convention instead of
  writing back into sealed 20-minute windows. The old classifier also has its own
  historical correction delay. This is not a promise of exact event-time labels.
- Valid steady streams produce 25 outputs/s and one classification/s. Startup,
  reacquisition, gaps and unfinished seconds are not forced to total 1200.
- This FIR is an antialiasing candidate, NOT an exact compensation for the original
  analog response. No thresholds were fitted to make legacy agreement look better.

## Measured processing time

Measured on the V3.8 board on 2026-10-26 with the bench-only PB10 marker
(`RFID_BENCH_TIMING_PIN`), which is high only while drained words are
processed. At the observed 39.3 Hz, one 453-word batch took 169.2 ms with the
previous arithmetic and flash prefetch off, and 133.6 ms with prefetch on.
Prefetch is therefore enabled in `SystemClock_Config`. With the narrowed
arithmetic above, the same marker measured 45.3 ms per 453-word batch (15 batches,
44.9..45.4 ms), about 3.7 times faster than the original prefetch-off code.
The earlier instruction-level host simulation predicted 87..95 ms for the
previous arithmetic and underestimated its board cost even with prefetch, so
use it only for relative comparisons. Energy per batch still has to be measured.

## Software verification

`ctest --test-dir build/resample-verify/host --output-on-failure` includes:

- Unmodified classifier versus the 34d57c2 reference.
- Production 50 Hz configuration and unchanged diagnostic 25 Hz baseline.
- DC and signed-limit inputs, exact 40 ms output spacing, 32/39.502107/50/57.5/65 Hz,
  passband tones at 2/4 Hz, rejection at 12.5/14 Hz, gaps and RTC wrap.
- Four independent 4h20 scheduling runs: 39.502 Hz, 50 Hz, 65 Hz and missing FIFO IRQ.
  Startup has explicit missing counts; all 12 subsequent windows have 1200 counts.
- `app_timing_test`: event-level sweeps of the sensor rate from 32 to 65 Hz in
  0.01 Hz steps with the RTC's 1/256 s resolution, 1/2048 s wakeup ticks, 6 ms
  EEPROM word writes and a modelled processing time per word (0.374, 0.295,
  0.13 and 0 ms). Any resampler or sample-clock reset, FIFO overrun or
  shortfall of classifications fails. The previous app.c fails this sweep at
  0.13 and 0 ms per word.
- Existing protocol, CRC, storage interruption, reset, radio and driver tests.

Inspect GCC .su files for static stack estimates. The linker continues to reserve
2 KiB for stack. MCU timing, stack high-water and energy are not measured by host tests.

## Historical Excel replay

Requirements: Python, numpy, scipy, openpyxl; native CMake/GCC build of the host tests.

```text
python Tools/validate-resampling.py --data-root <original-project>/Doc/MATLAB \
  --host-build build/resample-verify/host --output <analysis-directory>
```

The source settings were 25 Hz, but their historical physical rates are unmeasured.
The tool assumes 25 Hz and uses `resample_poly` to construct high-rate scenarios.
These synthetic inputs do not validate the real 50 Hz sensor bandwidth or motion
above the original recording bandwidth. All workbooks remain read-only; hashes,
units assumptions, index discontinuities and rail values are recorded in manifest.json.

Output includes raw per-second legacy labels, paired labels, per-class counts and
full transition matrices. Filter-only 25 Hz is a control, not production mode.
Agreement is not accuracy. No human ground truth exists in these workbooks.

### Per-class acceptance

The user-defined gates replace a requirement for 100% overall agreement:
movement (class 3) and climb (class 4) >=95%; ingestion (class 2) and ruminate
(class 5) >=90%. Rest/other have no acceptance threshold. For now, require BOTH
reference recall (matched / original class seconds) and reference precision
(matched / candidate class seconds). This is a conservative interpretation of
per-behavior agreement, not a new classifier training target.

`validate-resampling.py` writes `acceptance.json` automatically. To reevaluate
existing replay outputs without scientific Python dependencies:

```text
python Tools/behavior_acceptance.py --results <analysis-directory>/results.json
python -B Tests/test_behavior_acceptance.py
```

The evaluator independently reconstructs every matrix from the included CSV
seconds, checks counts, and saves input/evaluator hashes. It pools seconds only
within each scenario, also reports per-record results, and never treats missing
classes as perfect agreement. Add `--require-pass` for exit code 2 when any
non-control pooled scenario fails or lacks coverage. A pass is only an observed
agreement gate; small samples and synthetic inputs cannot establish field accuracy.

Current 39.502107 Hz synthetic scenario: ingestion recall 92.96%, precision
93.89%; movement 100%/100% on just 11 reference seconds. Climb and ruminate have
no reference/candidate seconds, so overall acceptance is incomplete. In
vofa_9.15.11 alone, ingestion recall is 89.83%, below 90% (50 Hz scenario: 89.96%).
Pooled acceptance therefore does not mean every recording meets the threshold.

## Required before deployment

1. Extend the true 50 Hz-setting recordings to motion and fault conditions with
   independent RTC/logic-analyzer time. Static board/host output comparison and
   16 consecutive sealed 1200-count windows are now verified; continue checking
   STOP wakeups, gaps, duplicated output times, backlog and rate resets under
   the remaining operating conditions.
2. Compare filter-only and complete conversion on motion data; collect labelled
   cases for all six classes. Current replay has no reference climb/ruminate outputs.
3. Measure computation time, stack high-water and 20-minute charge with Release
   firmware, debug/UART disconnected. Compare to the same production-function baseline.
4. Confirm filter choice and threshold compatibility before calling this production-ready.

## References

- ADI ADXL362 Rev. G, pp. 19, 35: https://www.analog.com/media/en/technical-documentation/data-sheets/adxl362.pdf
- SciPy FIR construction: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.firwin.html
- SciPy offline reconstruction: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.resample_poly.html
