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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path)
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
    text = json.dumps(result, indent=2)
    if args.output:
        # Never replace an earlier measurement silently.
        with args.output.open("x", encoding="utf-8") as output:
            output.write(text + "\n")
    else:
        print(text)


if __name__ == "__main__":
    main()
