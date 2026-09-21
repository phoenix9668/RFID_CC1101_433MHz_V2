"""CS/IRQ edge timing only. Never decode 4 MHz SPI from a 1 MHz capture."""

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from analyze_adxl362_dsl import edges, load_capture


def summarize(signal, rate, active_high):
    if rate <= 0 or len(signal) < 2 or np.any(signal > 1):
        raise ValueError("Expected binary samples and a positive sample rate")
    active, inactive = (1, 0) if active_high else (0, 1)
    starts = edges(signal, inactive, active)
    ends = edges(signal, active, inactive)
    # Discard incomplete pulse widths, not real start-to-start intervals.
    end_index = np.searchsorted(ends, starts)
    complete = end_index < len(ends)
    widths = ends[end_index[complete]] - starts[complete]
    result = {
        "high_samples": int(np.count_nonzero(signal)),
        "active_at_start": bool(signal[0] == active),
        "active_at_end": bool(signal[-1] == active),
        "pulse_starts": len(starts), "pulse_ends": len(ends),
        "complete_pulses": len(widths),
        "first_start_sample": int(starts[0]) if len(starts) else None,
        "last_start_sample": int(starts[-1]) if len(starts) else None,
        "start_samples": starts.tolist(),
        "frequency_hz": None, "period_us": None, "width_us": None,
    }
    for name, values in (("period_us", np.diff(starts)), ("width_us", widths)):
        if len(values):
            result[name] = {"min": float(values.min()*1e6/rate),
                            "mean": float(values.mean()*1e6/rate),
                            "max": float(values.max()*1e6/rate)}
    if len(starts) > 1:
        result["frequency_hz"] = float((len(starts)-1)*rate/(starts[-1]-starts[0]))
    return result


def segment_timings(signal, rate, gap_ms=500):
    if gap_ms <= 0 or rate <= 0:
        raise ValueError("Expected positive rate and segment gap")
    starts = edges(signal, 0, 1)
    if not len(starts):
        return []
    groups = np.split(starts, np.flatnonzero(np.diff(starts) > rate*gap_ms/1000)+1)
    result = []
    for group in groups:
        periods = np.diff(group)
        # Same fixed trims for every phase, independent of the measured rate.
        steady = group[(group >= group[0]+rate*0.5) & (group <= group[-1]-rate*0.1)]
        stable_periods = np.diff(steady)
        result.append({
            "first_sample": int(group[0]), "last_sample": int(group[-1]),
            "start_s": float(group[0]/rate), "end_s": float(group[-1]/rate),
            "pulse_starts": len(group),
            "frequency_hz": float((len(group)-1)*rate/(group[-1]-group[0])) if len(periods) else None,
            "period_us": {"min": float(periods.min()*1e6/rate),
                          "mean": float(periods.mean()*1e6/rate),
                          "max": float(periods.max()*1e6/rate)} if len(periods) else None,
            "leading_gap_observed": bool(group[0] > rate*gap_ms/1000),
            "trailing_gap_observed": bool(len(signal)-1-group[-1] > rate*gap_ms/1000),
            "steady": {
                "trim_head_ms": 500, "trim_tail_ms": 100,
                "pulse_starts": len(steady),
                "first_sample": int(steady[0]) if len(steady) else None,
                "last_sample": int(steady[-1]) if len(steady) else None,
                "frequency_hz": float((len(steady)-1)*rate/(steady[-1]-steady[0])) if len(stable_periods) else None,
                "period_us": {"min": float(stable_periods.min()*1e6/rate),
                              "mean": float(stable_periods.mean()*1e6/rate),
                              "max": float(stable_periods.max()*1e6/rate)} if len(stable_periods) else None,
            },
        })
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--segments", action="store_true", help="Split IRQ phases at gaps over 500 ms; labels require UART")
    args = parser.parse_args()
    _, session, rate, signals = load_capture(args.capture, timing_only=True)
    result = {"sha256": hashlib.sha256(args.capture.read_bytes()).hexdigest(),
              "sample_rate": rate, "samples": len(signals[0]),
              "duration_s": len(signals[0])/rate,
              "operation_mode": session.get("Operation Mode"),
              "threshold": session.get("Threshold Level"),
              "cs": summarize(signals[0], rate, False),
              "irq": summarize(signals[4], rate, True),
              "limitation": "Edge timing only; CS frequency is not independent DATA_READY evidence."}
    if args.segments:
        result["irq_segments"] = segment_timings(signals[4], rate)
        result["irq"]["frequency_hz"] = None
        result["irq"]["period_us"] = None
        result["limitation"] += " Standby gaps excluded from segment rates; do not assign nominal ODR from measured rate alone."
    text = json.dumps(result, indent=2)
    if args.output:
        # Never replace an earlier measurement silently.
        with args.output.open("x", encoding="utf-8") as output:
            output.write(text + "\n")
    else:
        print(text)


if __name__ == "__main__":
    main()
