# Scheme 2: historical collar-data replay

## Scope and limits

- The user confirmed that the source collars were configured at 25 Hz. Actual historical ODR is unknown.
- All time axes below assume 25 Hz. These are algorithm agreement results, NOT classification accuracy.
- No human labels or real 50 Hz recordings are available. High-rate streams are constructed using SciPy resample_poly.
- The 25 Hz scenario is a filter-only control, not the selected 50 Hz production configuration.
- The simulation cannot recover missing high-frequency motion or reproduce the hardware filter change at 50 Hz.
- XYZ values are retained in their original integer units. Axis/range calibration across recordings is unverified.
- Index resets split segments. No interpolation across gaps; warmup (<20 s) and the final second are excluded.
- The unchanged C classifier is checked against the 34d57c2 legacy implementation on every complete 150-sample block.

## Per-record comparison

| Record | Simulated input Hz | Compared s | Changed s | Agreement |
|---|---:|---:|---:|---:|
| vofa_9.15.11_Sheet1_seg1 | 25.000000 | 1979 | 218 | 88.98% |
| vofa_9.15.11_Sheet1_seg1 | 39.502053 | 1979 | 221 | 88.83% |
| vofa_9.15.11_Sheet1_seg1 | 50.000000 | 1979 | 217 | 89.03% |
| vofa_9.15.11_Sheet1_seg1 | 57.500000 | 1979 | 216 | 89.09% |
| vofa_9.15.12_Sheet1_seg1 | 25.000000 | 1964 | 169 | 91.40% |
| vofa_9.15.12_Sheet1_seg1 | 39.502053 | 1964 | 172 | 91.24% |
| vofa_9.15.12_Sheet1_seg1 | 50.000000 | 1964 | 175 | 91.09% |
| vofa_9.15.12_Sheet1_seg1 | 57.500000 | 1964 | 169 | 91.40% |
| vofa_9.15.3_Sheet1_seg1 | 25.000000 | 1979 | 167 | 91.56% |
| vofa_9.15.3_Sheet1_seg1 | 39.502053 | 1979 | 163 | 91.76% |
| vofa_9.15.3_Sheet1_seg1 | 50.000000 | 1979 | 164 | 91.71% |
| vofa_9.15.3_Sheet1_seg1 | 57.500000 | 1979 | 171 | 91.36% |
| example_data_Sheet1_seg1 | 25.000000 | 12 | 1 | 91.67% |
| example_data_Sheet1_seg1 | 39.502053 | 12 | 1 | 91.67% |
| example_data_Sheet1_seg1 | 50.000000 | 12 | 1 | 91.67% |
| example_data_Sheet1_seg1 | 57.500000 | 12 | 1 | 91.67% |
| example_data_Sheet2_seg1 | 25.000000 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg1 | 39.502053 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg1 | 50.000000 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg1 | 57.500000 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg2 | 25.000000 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg2 | 39.502053 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg2 | 50.000000 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg2 | 57.500000 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg3 | 25.000000 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg3 | 39.502053 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg3 | 50.000000 | 0 | 0 | n/a: too short |
| example_data_Sheet2_seg3 | 57.500000 | 0 | 0 | n/a: too short |

## Six-class counts on the common intervals

Class order: rest, ingestion, movement, climb, ruminate, other. Original outputs are a comparator, not ground truth.

| Simulated input | Baseline counts | Scheme 2 counts | Changed / compared |
|---|---|---|---|
| 25.0 Hz | [617, 3155, 11, 0, 0, 2151] | [723, 3120, 11, 0, 0, 2080] | 555 / 5934 |
| 39.502107 Hz | [617, 3155, 11, 0, 0, 2151] | [729, 3124, 11, 0, 0, 2070] | 557 / 5934 |
| 50.0 Hz | [617, 3155, 11, 0, 0, 2151] | [722, 3117, 11, 0, 0, 2084] | 557 / 5934 |
| 57.5 Hz | [617, 3155, 11, 0, 0, 2151] | [720, 3119, 11, 0, 0, 2084] | 557 / 5934 |

## Interpretation

A rate-normalization path is implemented, not a newly trained behavior classifier. The 8 Hz Kaiser FIR deliberately removes high-frequency content before conversion; it is not claimed to reproduce the exact original analog filter.
Agreement quantifies sensitivity to this processing. A disagreement is not necessarily an error; agreement is not proof of correctness.
Short segments remain in the inventory and output files but do not contribute to agreement. Partial final seconds are not padded.
Inspect results.json for full 6x6 transition matrices and CSV files for every classified second.
acceptance.json checks both per-class reference recall and precision: movement/climb >=95%, ingestion/ruminate >=90%. Zero support is unassessable, never a pass; pooled and per-record results are separate. These are agreement gates, not accuracy guarantees.
Real 50 Hz collar recordings, RTC rate estimation, STOP/wake timing, FIFO overrun behavior and current consumption still require board validation before deployment.
