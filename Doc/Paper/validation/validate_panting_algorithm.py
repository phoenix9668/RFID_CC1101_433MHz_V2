#!/usr/bin/env python3
"""Validate the paper's panting/rumination spectral rule on the supplied CSV files.

The paper's complete classifier cannot be reproduced from the PDF alone because
step 8 delegates the initial other/rumination/eating classifier to reference
[30]. This script therefore validates the explicit spectral reclassification
rule:

    F_1_2 = max(|FFT| in [1, 2) Hz) / mean(|FFT| in [1, 2) Hz)
    F_2_3 = max(|FFT| in [2, 3) Hz) / mean(|FFT| in [2, 3) Hz)
    panting signature if F_1_2 > F_2_3

The primary interpretation follows the paper's displayed three-axis energy
formula by applying the FFT to sqrt(x^2 + y^2 + z^2). Alternative signal and
window interpretations are included because the PDF does not specify the FFT
axis, taper, overlap, or the treatment of the 2 Hz band boundary.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


SAMPLE_RATE_HZ = 25.0
PAPER_SAMPLE_RATE_HZ = 10.0
WINDOW_SECONDS = 90
PAPER_BANDS_HZ = ((1.0, 2.0), (2.0, 3.0))

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = Path(__file__).resolve().parents[3]
PAPER_DIR = REPO_ROOT / "Doc" / "Paper"
DEFAULT_OUTPUT_DIR = SCRIPT_DIR / "output"


@dataclass(frozen=True)
class RecordingSpec:
    file_name: str
    recording: str
    expected_behavior: str
    encoding: str
    xyz_columns: tuple[str, str, str] | None = ("x", "y", "z")

    @property
    def path(self) -> Path:
        return PAPER_DIR / self.file_name


RECORDINGS = (
    RecordingSpec(
        "05项圈 5826牛 0302 1549  反刍.csv",
        "反刍对照",
        "rumination",
        "ascii",
        None,
    ),
    RecordingSpec(
        "喘息20250819 1622.csv",
        "喘息 2025-08-19 16:22",
        "panting",
        "gb18030",
    ),
    RecordingSpec(
        "喘息20250820 1644.csv",
        "喘息 2025-08-20 16:44",
        "panting",
        "gb18030",
    ),
    RecordingSpec(
        "喘息20250820 1755.csv",
        "喘息 2025-08-20 17:55",
        "panting",
        "gb18030",
    ),
)

DUPLICATE_PREFIX_REFERENCE = "喘息20250820 1644.csv"
DUPLICATE_PREFIX_TRIMMED = "喘息20250820 1755.csv"


@dataclass
class LoadedRecording:
    spec: RecordingSpec
    frame: pd.DataFrame
    xyz: np.ndarray
    raw_rows: int
    omitted_initial_rows: int


def load_recording(spec: RecordingSpec) -> LoadedRecording:
    frame = pd.read_csv(spec.path, encoding=spec.encoding)
    if spec.xyz_columns is None:
        xyz_frame = frame.iloc[:, :3].copy()
        xyz_frame.columns = ["x", "y", "z"]
    else:
        missing = [column for column in spec.xyz_columns if column not in frame.columns]
        if missing:
            raise ValueError(f"{spec.file_name} missing required columns: {missing}")
        xyz_frame = frame.loc[:, list(spec.xyz_columns)].copy()

    xyz_frame = xyz_frame.apply(pd.to_numeric, errors="coerce")
    if xyz_frame.isna().any(axis=None):
        bad_rows = int(xyz_frame.isna().any(axis=1).sum())
        raise ValueError(f"{spec.file_name} has {bad_rows} rows with invalid x/y/z values")

    xyz = xyz_frame.to_numpy(dtype=float)
    omitted_initial_rows = int(len(xyz) > 0 and np.all(xyz[0] == 0))
    if omitted_initial_rows:
        xyz = xyz[1:]

    return LoadedRecording(
        spec=spec,
        frame=frame,
        xyz=xyz,
        raw_rows=len(frame),
        omitted_initial_rows=omitted_initial_rows,
    )


def common_prefix_rows(first: np.ndarray, second: np.ndarray) -> int:
    comparable_rows = min(len(first), len(second))
    if comparable_rows == 0:
        return 0
    unequal = np.flatnonzero(np.any(first[:comparable_rows] != second[:comparable_rows], axis=1))
    return int(unequal[0]) if len(unequal) else comparable_rows


def taper_values(length: int, taper: str) -> np.ndarray:
    if taper == "rectangular":
        return np.ones(length)
    if taper == "hann":
        return np.hanning(length)
    raise ValueError(f"Unsupported taper: {taper}")


def signal_from_xyz(xyz: np.ndarray, method: str) -> tuple[np.ndarray, str]:
    if method == "raw_magnitude":
        signal = np.linalg.norm(xyz, axis=1)
        return signal - signal.mean(), "magnitude"

    centered = xyz - xyz.mean(axis=0, keepdims=True)

    if method == "dominant_range_axis":
        axis_index = int(np.ptp(centered, axis=0).argmax())
        return centered[:, axis_index], "xyz"[axis_index]

    if method == "dominant_rms_axis":
        axis_index = int(np.std(centered, axis=0).argmax())
        return centered[:, axis_index], "xyz"[axis_index]

    if method == "pca_projection":
        _, _, right_vectors = np.linalg.svd(centered, full_matrices=False)
        return centered @ right_vectors[0], "pca1"

    if method == "difference_magnitude":
        differences = np.diff(xyz, axis=0, prepend=xyz[:1])
        signal = np.linalg.norm(differences, axis=1)
        return signal - signal.mean(), "difference_magnitude"

    raise ValueError(f"Unsupported signal method: {method}")


def spectral_metrics(
    xyz_window: np.ndarray,
    sample_rate_hz: float,
    signal_method: str,
    taper: str,
) -> dict[str, float | str | bool]:
    signal, selected_signal = signal_from_xyz(xyz_window, signal_method)
    amplitude = np.abs(np.fft.rfft(signal * taper_values(len(signal), taper)))
    frequency = np.fft.rfftfreq(len(signal), d=1.0 / sample_rate_hz)

    band_metrics: list[tuple[float, float, float, float]] = []
    for low_hz, high_hz in PAPER_BANDS_HZ:
        band_mask = (frequency >= low_hz) & (frequency < high_hz)
        band_amplitude = amplitude[band_mask]
        band_frequency = frequency[band_mask]
        if not len(band_amplitude):
            raise ValueError(
                f"No FFT bins in band [{low_hz}, {high_hz}) at "
                f"{sample_rate_hz} Hz and {len(signal)} samples"
            )
        peak_index = int(band_amplitude.argmax())
        band_mean = float(band_amplitude.mean())
        normalized_peak = float(band_amplitude[peak_index] / band_mean) if band_mean else math.inf
        band_metrics.append(
            (
                normalized_peak,
                float(band_frequency[peak_index]),
                float(band_amplitude[peak_index]),
                band_mean,
            )
        )

    f_1_2, peak_1_2_hz, peak_1_2_amplitude, mean_1_2_amplitude = band_metrics[0]
    f_2_3, peak_2_3_hz, peak_2_3_amplitude, mean_2_3_amplitude = band_metrics[1]
    return {
        "selected_signal": selected_signal,
        "f_1_2": f_1_2,
        "f_2_3": f_2_3,
        "spectral_ratio": f_1_2 / f_2_3,
        "peak_1_2_hz": peak_1_2_hz,
        "peak_1_2_bpm": peak_1_2_hz * 60.0,
        "peak_2_3_hz": peak_2_3_hz,
        "peak_2_3_bpm": peak_2_3_hz * 60.0,
        "peak_1_2_amplitude": peak_1_2_amplitude,
        "mean_1_2_amplitude": mean_1_2_amplitude,
        "peak_2_3_amplitude": peak_2_3_amplitude,
        "mean_2_3_amplitude": mean_2_3_amplitude,
        "criterion_pass": bool(f_1_2 > f_2_3),
    }


def prepare_xyz(
    loaded: dict[str, LoadedRecording],
    duplicate_prefix_rows: int,
    deduplicate: bool,
) -> dict[str, np.ndarray]:
    prepared = {name: item.xyz for name, item in loaded.items()}
    if deduplicate:
        prepared[DUPLICATE_PREFIX_TRIMMED] = prepared[DUPLICATE_PREFIX_TRIMMED][
            duplicate_prefix_rows:
        ]
    return prepared


def score_recordings(
    loaded: dict[str, LoadedRecording],
    duplicate_prefix_rows: int,
    *,
    sample_rate_hz: float = SAMPLE_RATE_HZ,
    window_seconds: int = WINDOW_SECONDS,
    signal_method: str = "raw_magnitude",
    taper: str = "rectangular",
    offset_seconds: int = 0,
    deduplicate: bool = True,
) -> pd.DataFrame:
    prepared = prepare_xyz(loaded, duplicate_prefix_rows, deduplicate)
    window_samples = int(round(sample_rate_hz * window_seconds))
    offset_samples = int(round(sample_rate_hz * offset_seconds))
    rows: list[dict[str, object]] = []

    for spec in RECORDINGS:
        xyz = prepared[spec.file_name]
        for window_index, start in enumerate(
            range(offset_samples, len(xyz) - window_samples + 1, window_samples)
        ):
            metrics = spectral_metrics(
                xyz[start : start + window_samples],
                sample_rate_hz,
                signal_method,
                taper,
            )
            rows.append(
                {
                    "file_name": spec.file_name,
                    "recording": spec.recording,
                    "expected_behavior": spec.expected_behavior,
                    "window_index": window_index,
                    "start_sample_after_dedup": start,
                    "start_seconds_after_dedup": start / sample_rate_hz,
                    "window_seconds": window_seconds,
                    "sample_rate_hz": sample_rate_hz,
                    "signal_method": signal_method,
                    "taper": taper,
                    **metrics,
                }
            )

    return pd.DataFrame(rows)


def wilson_interval(successes: int, trials: int, z: float = 1.959963984540054) -> tuple[float, float]:
    if trials == 0:
        return (math.nan, math.nan)
    proportion = successes / trials
    denominator = 1.0 + z * z / trials
    center = (proportion + z * z / (2.0 * trials)) / denominator
    half_width = (
        z
        * math.sqrt(
            proportion * (1.0 - proportion) / trials + z * z / (4.0 * trials * trials)
        )
        / denominator
    )
    return center - half_width, center + half_width


def aggregate_performance(window_scores: pd.DataFrame) -> dict[str, float | int]:
    positive = window_scores[window_scores["expected_behavior"] == "panting"]
    negative = window_scores[window_scores["expected_behavior"] == "rumination"]
    positive_passes = int(positive["criterion_pass"].sum())
    negative_rejections = int((~negative["criterion_pass"]).sum())
    positive_windows = len(positive)
    negative_windows = len(negative)
    positive_rate = positive_passes / positive_windows if positive_windows else math.nan
    negative_rejection_rate = (
        negative_rejections / negative_windows if negative_windows else math.nan
    )
    positive_interval = wilson_interval(positive_passes, positive_windows)
    negative_interval = wilson_interval(negative_rejections, negative_windows)
    return {
        "panting_windows": positive_windows,
        "panting_windows_passing": positive_passes,
        "panting_window_pass_rate": positive_rate,
        "panting_window_pass_rate_wilson_low": positive_interval[0],
        "panting_window_pass_rate_wilson_high": positive_interval[1],
        "rumination_windows": negative_windows,
        "rumination_windows_rejected": negative_rejections,
        "rumination_window_rejection_rate": negative_rejection_rate,
        "rumination_window_rejection_rate_wilson_low": negative_interval[0],
        "rumination_window_rejection_rate_wilson_high": negative_interval[1],
        "balanced_descriptive_rate": (positive_rate + negative_rejection_rate) / 2.0,
    }


def recording_summary(window_scores: pd.DataFrame) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for (file_name, recording, expected_behavior), group in window_scores.groupby(
        ["file_name", "recording", "expected_behavior"], sort=False
    ):
        passes = int(group["criterion_pass"].sum())
        rows.append(
            {
                "file_name": file_name,
                "recording": recording,
                "expected_behavior": expected_behavior,
                "windows": len(group),
                "criterion_passes": passes,
                "criterion_pass_rate": passes / len(group),
                "median_spectral_ratio": float(group["spectral_ratio"].median()),
                "q25_spectral_ratio": float(group["spectral_ratio"].quantile(0.25)),
                "q75_spectral_ratio": float(group["spectral_ratio"].quantile(0.75)),
                "median_peak_1_2_bpm": float(group["peak_1_2_bpm"].median()),
            }
        )
    return pd.DataFrame(rows)


def sensitivity_analysis(
    loaded: dict[str, LoadedRecording],
    duplicate_prefix_rows: int,
) -> pd.DataFrame:
    cases = (
        {
            "check": "primary",
            "detail": "25 Hz, 90 s, raw magnitude, rectangular taper, deduplicated",
        },
        {
            "check": "taper",
            "detail": "Hann taper",
            "taper": "hann",
        },
        {
            "check": "signal_interpretation",
            "detail": "dominant peak-to-peak axis",
            "signal_method": "dominant_range_axis",
        },
        {
            "check": "signal_interpretation",
            "detail": "dominant RMS axis",
            "signal_method": "dominant_rms_axis",
        },
        {
            "check": "signal_interpretation",
            "detail": "PCA first component",
            "signal_method": "pca_projection",
        },
        {
            "check": "signal_interpretation",
            "detail": "magnitude of first differences",
            "signal_method": "difference_magnitude",
        },
        {
            "check": "window_length",
            "detail": "30 s windows",
            "window_seconds": 30,
        },
        {
            "check": "window_length",
            "detail": "60 s windows",
            "window_seconds": 60,
        },
        {
            "check": "window_length",
            "detail": "180 s windows",
            "window_seconds": 180,
        },
        {
            "check": "window_alignment",
            "detail": "90 s windows offset by 30 s",
            "offset_seconds": 30,
        },
        {
            "check": "window_alignment",
            "detail": "90 s windows offset by 60 s",
            "offset_seconds": 60,
        },
        {
            "check": "deduplication",
            "detail": "shared prefix retained",
            "deduplicate": False,
        },
        {
            "check": "sampling_rate",
            "detail": "incorrectly assume paper's 10 Hz for 25 Hz data",
            "sample_rate_hz": PAPER_SAMPLE_RATE_HZ,
        },
    )

    rows: list[dict[str, object]] = []
    for case in cases:
        parameters = {
            "sample_rate_hz": SAMPLE_RATE_HZ,
            "window_seconds": WINDOW_SECONDS,
            "signal_method": "raw_magnitude",
            "taper": "rectangular",
            "offset_seconds": 0,
            "deduplicate": True,
        }
        parameters.update({key: value for key, value in case.items() if key in parameters})
        scores = score_recordings(loaded, duplicate_prefix_rows, **parameters)
        performance = aggregate_performance(scores)
        rows.append(
            {
                "check": case["check"],
                "detail": case["detail"],
                **parameters,
                **performance,
            }
        )
    return pd.DataFrame(rows)


def data_quality_summary(
    loaded: dict[str, LoadedRecording],
    duplicate_prefix_rows: int,
) -> pd.DataFrame:
    rows: list[dict[str, object]] = []
    for spec in RECORDINGS:
        item = loaded[spec.file_name]
        frame = item.frame
        xyz_frame = pd.DataFrame(item.xyz, columns=["x", "y", "z"])
        deduplicated_rows = (
            duplicate_prefix_rows if spec.file_name == DUPLICATE_PREFIX_TRIMMED else 0
        )
        analysis_rows = len(item.xyz) - deduplicated_rows
        rows.append(
            {
                "file_name": spec.file_name,
                "recording": spec.recording,
                "expected_behavior": spec.expected_behavior,
                "raw_rows": item.raw_rows,
                "usable_xyz_rows": len(item.xyz),
                "analysis_rows_after_cross_file_dedup": analysis_rows,
                "omitted_initial_zero_rows": item.omitted_initial_rows,
                "deduplicated_prefix_rows": deduplicated_rows,
                "analysis_duration_minutes_at_25_hz": analysis_rows / SAMPLE_RATE_HZ / 60.0,
                "exact_duplicate_rows_within_file": int(frame.duplicated().sum()),
                "duplicate_xyz_rows_within_file": int(xyz_frame.duplicated().sum()),
                "null_xyz_rows": int(xyz_frame.isna().any(axis=1).sum()),
                "out_of_adxl362_12bit_range_rows": int(
                    ((xyz_frame < -2048) | (xyz_frame > 2047)).any(axis=1).sum()
                ),
                "timestamp_column_present": any(
                    token in str(column).lower()
                    for column in frame.columns
                    for token in ("time", "date", "时间", "日期")
                ),
                "column_count": len(frame.columns),
            }
        )
    return pd.DataFrame(rows)


def run_firmware_comparison(
    loaded: dict[str, LoadedRecording],
    duplicate_prefix_rows: int,
) -> tuple[pd.DataFrame, str | None]:
    runner_dir = REPO_ROOT / "Tools" / "adxl362_pc"
    runner = runner_dir / "build" / "adxl362_pc"

    try:
        subprocess.run(
            ["make", "-C", str(runner_dir)],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError) as error:
        return pd.DataFrame(), f"Firmware runner build failed: {error}"

    rows: list[dict[str, object]] = []
    with tempfile.TemporaryDirectory(prefix="panting-firmware-") as temporary_directory:
        temporary_path = Path(temporary_directory)
        for spec in RECORDINGS:
            input_path = spec.path
            input_encoding = spec.encoding
            if spec.xyz_columns is None:
                input_path = temporary_path / "rumination_xyz.csv"
                normalized = pd.DataFrame(
                    np.rint(loaded[spec.file_name].xyz).astype(int),
                    columns=["x", "y", "z"],
                )
                normalized.to_csv(input_path, index=False)
                input_encoding = "ascii"

            output_path = temporary_path / f"{spec.recording}.csv"
            try:
                subprocess.run(
                    [str(runner), str(input_path), str(output_path)],
                    check=True,
                    capture_output=True,
                    text=True,
                )
                output = pd.read_csv(output_path, encoding=input_encoding)
            except (OSError, subprocess.CalledProcessError, UnicodeError, pd.errors.ParserError) as error:
                return pd.DataFrame(), f"Firmware runner failed for {spec.file_name}: {error}"

            behaviors = pd.to_numeric(output["behavior"], errors="raise")
            trimmed_rows = 0
            if spec.file_name == DUPLICATE_PREFIX_TRIMMED:
                trimmed_rows = duplicate_prefix_rows + loaded[spec.file_name].omitted_initial_rows
                behaviors = behaviors.iloc[min(trimmed_rows, len(behaviors)) :]

            breath_samples = int((behaviors == 7).sum())
            rows.append(
                {
                    "file_name": spec.file_name,
                    "recording": spec.recording,
                    "expected_behavior": spec.expected_behavior,
                    "processed_samples_after_dedup": len(behaviors),
                    "trimmed_duplicate_output_rows": trimmed_rows,
                    "breath_samples": breath_samples,
                    "breath_sample_rate": breath_samples / len(behaviors) if len(behaviors) else math.nan,
                }
            )

    return pd.DataFrame(rows), None


def write_json(path: Path, payload: dict[str, object]) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument(
        "--skip-firmware",
        action="store_true",
        help="Skip the separate comparison against the current C firmware classifier.",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    loaded = {spec.file_name: load_recording(spec) for spec in RECORDINGS}
    duplicate_prefix_rows = common_prefix_rows(
        loaded[DUPLICATE_PREFIX_REFERENCE].xyz,
        loaded[DUPLICATE_PREFIX_TRIMMED].xyz,
    )

    primary_scores = score_recordings(loaded, duplicate_prefix_rows)
    primary_performance = aggregate_performance(primary_scores)
    by_recording = recording_summary(primary_scores)
    sensitivity = sensitivity_analysis(loaded, duplicate_prefix_rows)
    quality = data_quality_summary(loaded, duplicate_prefix_rows)

    primary_scores.to_csv(output_dir / "window_scores.csv", index=False)
    by_recording.to_csv(output_dir / "recording_summary.csv", index=False)
    sensitivity.to_csv(output_dir / "sensitivity_analysis.csv", index=False)
    quality.to_csv(output_dir / "data_quality_summary.csv", index=False)

    firmware_error: str | None = "Skipped by request"
    firmware = pd.DataFrame()
    if not args.skip_firmware:
        firmware, firmware_error = run_firmware_comparison(loaded, duplicate_prefix_rows)
        if not firmware.empty:
            firmware.to_csv(output_dir / "firmware_comparison.csv", index=False)

    summary: dict[str, object] = {
        "question": (
            "Do the supplied files support the PDF paper's explicit F_1_2 > F_2_3 "
            "panting-versus-rumination spectral rule?"
        ),
        "overall_assessment": "Share with caveats / not a full correctness validation",
        "primary_method": {
            "actual_sample_rate_hz": SAMPLE_RATE_HZ,
            "paper_sample_rate_hz": PAPER_SAMPLE_RATE_HZ,
            "window_seconds": WINDOW_SECONDS,
            "window_overlap": "none",
            "signal": "sqrt(x^2 + y^2 + z^2), mean removed",
            "taper": "rectangular",
            "bands_hz": ["[1, 2)", "[2, 3)"],
            "decision": "F_1_2 > F_2_3",
            "cross_file_deduplication": (
                f"Removed {duplicate_prefix_rows} shared cleaned samples from "
                f"{DUPLICATE_PREFIX_TRIMMED}"
            ),
        },
        "primary_results": primary_performance,
        "data_quality": {
            "duplicate_prefix_rows_after_initial_zero_removal": duplicate_prefix_rows,
            "duplicate_prefix_seconds": duplicate_prefix_rows / SAMPLE_RATE_HZ,
            "duplicate_prefix_minutes": duplicate_prefix_rows / SAMPLE_RATE_HZ / 60.0,
            "timestamp_columns_present": False,
            "file_level_labels_only": True,
            "manual_respiration_rate_labels_present": False,
            "temperature_humidity_labels_present": False,
        },
        "firmware_comparison_error": firmware_error,
        "limitations": [
            (
                "The PDF does not include the referenced step-8 classifier thresholds/model, "
                "so the full other/rumination/eating-to-heat-stress pipeline is not reproducible."
            ),
            (
                "File names provide recording-level labels only; there are no timestamped visual "
                "respiration counts or annotated panting intervals, so window-level sensitivity "
                "and specificity cannot be established."
            ),
            (
                "The recordings contain no animal identifier, ambient temperature, humidity, or "
                "THI, so the heat-stress claim cannot be validated from these files."
            ),
            (
                "Windows from the same recording are serially correlated; nominal Wilson intervals "
                "do not account for clustering and are descriptive only."
            ),
            (
                "The PDF leaves FFT axis selection, taper, overlap, and the exact interpretation of "
                "the three-axis energy calculation underspecified; sensitivity checks show that "
                "these choices materially affect results."
            ),
            (
                "The current firmware uses a separate 25 Hz, 6 s time-domain sine detector and is "
                "not a faithful implementation of the paper's 90 s FFT harmonic-ratio rule."
            ),
        ],
    }
    write_json(output_dir / "validation_summary.json", summary)

    print(f"output_dir={output_dir}")
    print(f"duplicate_prefix_rows={duplicate_prefix_rows}")
    print(
        "primary: "
        f"panting_pass={primary_performance['panting_windows_passing']}/"
        f"{primary_performance['panting_windows']} "
        f"({primary_performance['panting_window_pass_rate']:.1%}), "
        f"rumination_rejected={primary_performance['rumination_windows_rejected']}/"
        f"{primary_performance['rumination_windows']} "
        f"({primary_performance['rumination_window_rejection_rate']:.1%})"
    )
    if firmware_error:
        print(f"firmware_comparison={firmware_error}")
    elif not firmware.empty:
        print("firmware_comparison=completed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
