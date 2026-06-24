#!/usr/bin/env python3
"""Run the ADXL362 original-data behavior classifier on decoded CSV samples."""

from __future__ import annotations

import argparse
import csv
import math
from collections import Counter
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable


SAMPLE_RATE_HZ = 25
SECONDS_PER_FIFO = 6
SAMPLES_PER_FIFO = SAMPLE_RATE_HZ * SECONDS_PER_FIFO
MEM_ROWS = 18
MEM_COLS = 4

SINE_WAVE_MIN_FREQ = 1.0
SINE_WAVE_MAX_FREQ = 1.7
SINE_WAVE_MIN_AMPLITUDE = 30
PLATEAU_THRESHOLD = 5
MIN_PLATEAU_COUNT = 3
MAX_PLATEAU_WINDOW = 3
MAX_PEAK_COUNT = 20
MAX_VALLEY_COUNT = 20
MAX_PERIOD_COUNT = 11
STD_DEV_THRESHOLD = 0.3
SMOOTH_WINDOW_SIZE = 5
EXTREMA_WINDOW_SIZE = 3

ACTION_LABELS = {
    0: "initial_delay",
    1: "rest",
    2: "ingestion",
    3: "movement",
    4: "climb",
    5: "ruminate",
    6: "other",
    7: "breath",
}


@dataclass
class AxisSample:
    x: int
    y: int
    z: int


@dataclass
class ExtremePoint:
    value: int
    index: int


@dataclass
class SineDetection:
    peaks: list[ExtremePoint] = field(default_factory=list)
    valleys: list[ExtremePoint] = field(default_factory=list)
    periods: list[int] = field(default_factory=list)
    is_rising: int = 0
    is_sine_wave: int = 0
    frequency: float = 0.0
    amplitude: int = 0


@dataclass
class ThresholdJudge:
    low: int = 0
    normal: int = 0
    abovenormal: int = 0
    high: int = 0


@dataclass
class AxisAccumulator:
    x: int = 0
    y: int = 0
    z: int = 0


@dataclass
class ClassifierState:
    memory_array: list[list[int]] = field(
        default_factory=lambda: [[0 for _ in range(MEM_COLS)] for _ in range(MEM_ROWS)]
    )
    memory_index: int = 0
    diff_previous: AxisSample = field(default_factory=lambda: AxisSample(0, 0, 0))


def c_div(numerator: int, denominator: int) -> int:
    """Match C integer division, which truncates toward zero."""

    if denominator == 0:
        raise ZeroDivisionError("division by zero")
    sign = -1 if (numerator < 0) != (denominator < 0) else 1
    return sign * (abs(numerator) // abs(denominator))


def parse_int(value: str, column: str, row_number: int) -> int:
    try:
        return int(value)
    except ValueError as exc:
        raise ValueError(f"row {row_number}: column {column!r} is not an integer: {value!r}") from exc


def detect_sine_wave(samples: list[AxisSample]) -> SineDetection:
    sine = SineDetection()
    x_values = [sample.x for sample in samples]
    y_values = [sample.y for sample in samples]

    x_max_val = max(x_values)
    x_min_val = min(x_values)
    y_max_val = max(y_values)
    y_min_val = min(y_values)

    if abs(x_max_val - x_min_val) <= 2 * abs(y_max_val - y_min_val):
        return sine

    smoothed_x: list[int] = []
    half_window = SMOOTH_WINDOW_SIZE // 2
    for i in range(len(samples)):
        window_sum = 0
        count = 0
        for j in range(-half_window, half_window + 1):
            idx = i + j
            if 0 <= idx < len(samples):
                window_sum += samples[idx].x
                count += 1
        smoothed_x.append(c_div(window_sum, count))

    for i in range(len(samples)):
        if i == 0:
            continue

        if i >= MAX_PLATEAU_WINDOW:
            is_plateau = True
            plateau_value_sum = 0
            plateau_count = 0

            for j in range(-MAX_PLATEAU_WINDOW, 0):
                if abs(smoothed_x[i + j] - smoothed_x[i + j + 1]) > PLATEAU_THRESHOLD:
                    is_plateau = False
                    break
                plateau_value_sum += smoothed_x[i + j]
                plateau_count += 1

            if is_plateau and plateau_count >= MIN_PLATEAU_COUNT:
                plateau_value = c_div(plateau_value_sum, plateau_count)
                before_plateau = 0
                if i > MAX_PLATEAU_WINDOW + 2:
                    before_plateau = smoothed_x[i - MAX_PLATEAU_WINDOW - 2]
                after_plateau = smoothed_x[i]

                if before_plateau < plateau_value and after_plateau < plateau_value:
                    if sine.is_rising and len(sine.peaks) < MAX_PEAK_COUNT:
                        sine.peaks.append(
                            ExtremePoint(plateau_value, i - c_div(MAX_PLATEAU_WINDOW, 2))
                        )
                        if len(sine.peaks) >= 2 and len(sine.periods) < MAX_PERIOD_COUNT:
                            sine.periods.append(sine.peaks[-1].index - sine.peaks[-2].index)
                        sine.is_rising = 0
                elif before_plateau > plateau_value and after_plateau > plateau_value:
                    if not sine.is_rising and len(sine.valleys) < MAX_VALLEY_COUNT:
                        sine.valleys.append(
                            ExtremePoint(plateau_value, i - c_div(MAX_PLATEAU_WINDOW, 2))
                        )
                        sine.is_rising = 1

        if i >= EXTREMA_WINDOW_SIZE and i < len(samples) - EXTREMA_WINDOW_SIZE:
            is_peak = True
            for j in range(-EXTREMA_WINDOW_SIZE, EXTREMA_WINDOW_SIZE + 1):
                if j != 0 and smoothed_x[i + j] > smoothed_x[i]:
                    is_peak = False
                    break

            if is_peak and sine.is_rising and len(sine.peaks) < MAX_PEAK_COUNT:
                too_close = False
                for peak in sine.peaks:
                    if abs(i - peak.index) < EXTREMA_WINDOW_SIZE:
                        too_close = True
                        if smoothed_x[i] > peak.value:
                            peak.value = smoothed_x[i]
                            peak.index = i
                        break

                if not too_close:
                    sine.peaks.append(ExtremePoint(smoothed_x[i], i))
                    if len(sine.peaks) >= 2 and len(sine.periods) < MAX_PERIOD_COUNT:
                        sine.periods.append(sine.peaks[-1].index - sine.peaks[-2].index)
                sine.is_rising = 0

            is_valley = True
            for j in range(-EXTREMA_WINDOW_SIZE, EXTREMA_WINDOW_SIZE + 1):
                if j != 0 and smoothed_x[i + j] < smoothed_x[i]:
                    is_valley = False
                    break

            if is_valley and not sine.is_rising and len(sine.valleys) < MAX_VALLEY_COUNT:
                too_close = False
                for valley in sine.valleys:
                    if abs(i - valley.index) < EXTREMA_WINDOW_SIZE:
                        too_close = True
                        if smoothed_x[i] < valley.value:
                            valley.value = smoothed_x[i]
                            valley.index = i
                        break

                if not too_close:
                    sine.valleys.append(ExtremePoint(smoothed_x[i], i))
                sine.is_rising = 1

    sine.peaks.sort(key=lambda point: point.index)
    sine.valleys.sort(key=lambda point: point.index)
    sine.periods = []
    for i in range(1, len(sine.peaks)):
        if len(sine.periods) >= MAX_PERIOD_COUNT:
            break
        sine.periods.append(sine.peaks[i].index - sine.peaks[i - 1].index)

    if len(sine.peaks) >= 3 and len(sine.valleys) >= 3 and sine.periods:
        avg_period = sum(sine.periods) / len(sine.periods)
        sine.frequency = SAMPLE_RATE_HZ / avg_period

        avg_peak = c_div(sum(point.value for point in sine.peaks), len(sine.peaks))
        avg_valley = c_div(sum(point.value for point in sine.valleys), len(sine.valleys))
        sine.amplitude = avg_peak - avg_valley

        if (
            SINE_WAVE_MIN_FREQ <= sine.frequency <= SINE_WAVE_MAX_FREQ
            and sine.amplitude > SINE_WAVE_MIN_AMPLITUDE
        ):
            peak_std = math.sqrt(
                sum((point.value - avg_peak) * (point.value - avg_peak) for point in sine.peaks)
                / len(sine.peaks)
            )
            valley_std = math.sqrt(
                sum(
                    (point.value - avg_valley) * (point.value - avg_valley)
                    for point in sine.valleys
                )
                / len(sine.valleys)
            )

            if (
                peak_std < sine.amplitude * STD_DEV_THRESHOLD
                and valley_std < sine.amplitude * STD_DEV_THRESHOLD
            ):
                sine.is_sine_wave = 1

    return sine


def apply_memory_post_processing(
    state: ClassifierState,
    action: int,
    avg_x: int,
    avg_sum: int,
    avg_range: int,
    is_sine_wave: int,
) -> int:
    memory = state.memory_array
    memory_index = state.memory_index
    memory[memory_index][0] = action
    memory[memory_index][1] = avg_x
    memory[memory_index][2] = avg_sum
    memory[memory_index][3] = avg_range

    mid_index = memory_index - 9 if memory_index >= 9 else memory_index + 9

    if memory[mid_index][0] == 3:
        movement_cnt = sum(1 for row in memory if row[0] == 3)
        if movement_cnt == 1:
            rest_cnt1 = 0
            rest_cnt2 = 0

            if mid_index < memory_index:
                rest_cnt1 += sum(1 for i in range(mid_index, memory_index) if memory[i][0] == 1)
                rest_cnt2 += sum(1 for i in range(0, mid_index) if memory[i][0] == 1)
                rest_cnt2 += sum(1 for i in range(memory_index, MEM_ROWS) if memory[i][0] == 1)
            elif memory_index < mid_index:
                rest_cnt2 += sum(1 for i in range(memory_index, mid_index) if memory[i][0] == 1)
                rest_cnt1 += sum(1 for i in range(0, memory_index) if memory[i][0] == 1)
                rest_cnt1 += sum(1 for i in range(mid_index, MEM_ROWS) if memory[i][0] == 1)

            if rest_cnt1 >= 4 and rest_cnt2 >= 4:
                memory[mid_index][0] = 1

    if memory[memory_index][0] == 4:
        movement_cnt = sum(1 for row in memory if row[0] == 3)
        climb_cnt = sum(1 for row in memory if row[0] == 4)

        if movement_cnt < 4:
            memory[memory_index][0] = 6
        elif climb_cnt >= 2:
            memory[memory_index][0] = 3

    state.memory_index = 0 if memory_index >= MEM_ROWS - 1 else memory_index + 1

    ingestion_cnt = sum(1 for row in memory if row[0] == 2)
    if ingestion_cnt >= 2:
        for row in memory:
            if row[0] == 3:
                row[0] = 6

    rest_cnt = sum(1 for row in memory if row[0] == 1)
    if rest_cnt <= 4 and is_sine_wave == 0:
        deta_a_cnt = sum(1 for row in memory if 130 < row[2] < 700)
        jicha_cnt = sum(1 for row in memory if row[3] < 120)

        if deta_a_cnt >= 14 and jicha_cnt >= 14:
            eighteen_average = c_div(sum(row[1] for row in memory), MEM_ROWS)
            sum_eighteen_average = sum(abs(eighteen_average - row[1]) for row in memory)

            if sum_eighteen_average <= 400 and eighteen_average < 150:
                for row in memory:
                    row[0] = 5

    memory_index_o = 0 if state.memory_index == MEM_ROWS - 1 else state.memory_index + 1
    return memory[memory_index_o][0]


def classify_fifo(samples: list[AxisSample], state: ClassifierState) -> list[int]:
    sine_detection = detect_sine_wave(samples)
    outputs: list[int] = []
    threshold_judge = ThresholdJudge()
    average_info = AxisAccumulator()
    sum_info = AxisAccumulator()

    x_max_val = -2000
    x_min_val = 2000
    y_max_val = -2000
    y_min_val = 2000
    z_max_val = -2000
    z_min_val = 2000

    for i, sample in enumerate(samples):
        previous = state.diff_previous
        state.diff_previous = AxisSample(sample.x, sample.y, sample.z)

        x_max_val = max(x_max_val, sample.x)
        x_min_val = min(x_min_val, sample.x)
        y_max_val = max(y_max_val, sample.y)
        y_min_val = min(y_min_val, sample.y)
        z_max_val = max(z_max_val, sample.z)
        z_min_val = min(z_min_val, sample.z)

        average_info.x += sample.x

        diff_x = sample.x - previous.x
        diff_y = sample.y - previous.y
        diff_z = sample.z - previous.z
        sum_info.x += abs(diff_x)
        sum_info.y += abs(diff_y)
        sum_info.z += abs(diff_z)

        abs_diff_x = abs(diff_x)
        if abs_diff_x <= 10:
            threshold_judge.low += 1
        elif abs_diff_x <= 100:
            threshold_judge.normal += 1
        elif abs_diff_x <= 200:
            threshold_judge.abovenormal += 1
        else:
            threshold_judge.high += 1

        if (i + 1) % SAMPLE_RATE_HZ == 0:
            avg_x = c_div(average_info.x, SAMPLE_RATE_HZ)
            avg_sum = c_div(sum_info.x + sum_info.y + sum_info.z, 3)
            avg_range = c_div(
                (x_max_val - x_min_val) + (y_max_val - y_min_val) + (z_max_val - z_min_val),
                3,
            )

            if sine_detection.is_sine_wave == 1:
                action = 7
            elif threshold_judge.low >= 24:
                action = 1
            elif (
                (threshold_judge.normal + threshold_judge.abovenormal) > 11
                and threshold_judge.high == 0
                and avg_x >= 200
            ):
                action = 2
            elif avg_x > -200 and avg_x < 100 and avg_sum > 400 and avg_range > 150:
                action = 3
            elif threshold_judge.high > 0 and avg_x <= -200:
                action = 4
            else:
                action = 6

            outputs.append(
                apply_memory_post_processing(
                    state,
                    action,
                    avg_x,
                    avg_sum,
                    avg_range,
                    sine_detection.is_sine_wave,
                )
            )

            threshold_judge = ThresholdJudge()
            average_info = AxisAccumulator()
            sum_info = AxisAccumulator()
            x_max_val = -2000
            x_min_val = 2000
            y_max_val = -2000
            y_min_val = 2000
            z_max_val = -2000
            z_min_val = 2000

    return outputs


def classify_samples(samples: list[AxisSample]) -> list[int]:
    state = ClassifierState()
    behaviors: list[int] = []

    full_sample_count = (len(samples) // SAMPLES_PER_FIFO) * SAMPLES_PER_FIFO
    for start in range(0, full_sample_count, SAMPLES_PER_FIFO):
        fifo_outputs = classify_fifo(samples[start : start + SAMPLES_PER_FIFO], state)
        if len(fifo_outputs) != SECONDS_PER_FIFO:
            raise RuntimeError(f"expected {SECONDS_PER_FIFO} outputs, got {len(fifo_outputs)}")
        for behavior in fifo_outputs:
            behaviors.extend([behavior] * SAMPLE_RATE_HZ)

    return behaviors


def read_csv_samples(path: Path, encoding: str) -> tuple[list[str], list[dict[str, str]], list[AxisSample]]:
    with path.open("r", encoding=encoding, newline="") as input_file:
        reader = csv.DictReader(input_file)
        if not reader.fieldnames:
            raise ValueError(f"{path} has no header row")

        missing_columns = [column for column in ("x", "y", "z") if column not in reader.fieldnames]
        if missing_columns:
            raise ValueError(f"{path} is missing required columns: {', '.join(missing_columns)}")

        rows: list[dict[str, str]] = []
        samples: list[AxisSample] = []
        for row_number, row in enumerate(reader, start=2):
            rows.append(row)
            samples.append(
                AxisSample(
                    parse_int(row["x"], "x", row_number),
                    parse_int(row["y"], "y", row_number),
                    parse_int(row["z"], "z", row_number),
                )
            )

    return list(reader.fieldnames), rows, samples


def write_csv_with_behaviors(
    path: Path,
    encoding: str,
    fieldnames: Iterable[str],
    rows: list[dict[str, str]],
    behaviors: list[int],
) -> None:
    output_fieldnames = list(fieldnames)
    if "behavior" not in output_fieldnames:
        output_fieldnames.append("behavior")

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding=encoding, newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=output_fieldnames, extrasaction="ignore")
        writer.writeheader()
        for row, behavior in zip(rows, behaviors):
            output_row = dict(row)
            output_row["behavior"] = str(behavior)
            writer.writerow(output_row)


def default_path(file_name: str) -> Path:
    return Path(__file__).resolve().parent / file_name


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Classify decoded ADXL362 x/y/z CSV samples using the firmware behavior logic."
    )
    parser.add_argument("--input", type=Path, default=default_path("chuanxi.csv"))
    parser.add_argument("--output", type=Path, default=default_path("chuanxi_with_behavior.csv"))
    parser.add_argument("--encoding", default="gbk")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    fieldnames, rows, samples = read_csv_samples(args.input, args.encoding)
    behaviors = classify_samples(samples)
    kept_rows = len(behaviors)
    discarded_rows = len(rows) - kept_rows

    write_csv_with_behaviors(args.output, args.encoding, fieldnames, rows[:kept_rows], behaviors)

    counts = Counter(behaviors)
    print(f"input_rows={len(rows)}")
    print(f"output_rows={kept_rows}")
    print(f"discarded_tail_rows={discarded_rows}")
    print(f"output={args.output}")
    print("behavior_counts:")
    for behavior in sorted(ACTION_LABELS):
        count = counts.get(behavior, 0)
        print(f"  {behavior} ({ACTION_LABELS[behavior]}): {count}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
