"""D5 reference-clock / D4 DATA_READY evidence. Does not decode low-rate SPI."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from analyze_adxl362_dsl import edges, load_capture
from analyze_adxl362_timing import segment_timings


def distribution(values):
    unique, counts = np.unique(values, return_counts=True)
    return [[int(value), int(count)] for value, count in zip(unique, counts)]


def reference_bursts(clock, rate):
    rises, falls = edges(clock, 0, 1), edges(clock, 1, 0)
    if not len(rises): return [], rises
    groups = np.split(rises, np.flatnonzero(np.diff(rises) > rate*0.002)+1)
    result = []
    for group in groups:
        periods = np.diff(group)
        index = np.searchsorted(falls, group)
        complete = index < len(falls)
        widths = falls[index[complete]]-group[complete]
        # Discard only the last width of a gated burst; retain every full period.
        widths = widths[:max(0, len(group)-1)]
        freq = float((len(group)-1)*rate/(group[-1]-group[0])) if len(periods) else None
        is_clock = len(group) >= 32
        expected = rate/32000
        low_widths = periods-widths if len(widths) == len(periods) else np.array([])
        width_ok = (len(widths) == len(periods) and len(low_widths) > 0 and
                    np.all(widths >= expected*0.25) and np.all(widths <= expected*0.75) and
                    np.all(low_widths >= expected*0.25) and np.all(low_widths <= expected*0.75))
        result.append({
            "first_sample": int(group[0]), "last_sample": int(group[-1]),
            "rising_edges": len(group), "clock_candidate": is_clock,
            "frequency_hz": freq,
            "period_samples": distribution(periods),
            "high_width_samples": distribution(widths),
            "low_width_samples": distribution(low_widths),
            "continuous_32khz_candidate": bool(is_clock and width_ok and abs(freq/32000-1)<0.01 and
                np.all(periods >= expected*0.75) and np.all(periods <= expected*1.25)),
        })
    return result, rises


def analyze_signals(signals, rate):
    if len(signals) != 6 or rate < 1_000_000:
        raise ValueError("Requires D0-D5 and at least 1 MS/s")
    if len({len(s) for s in signals}) != 1:
        raise ValueError("Mismatched channel lengths")
    bursts, clock_rises = reference_bursts(signals[5], rate)
    irq_rises = edges(signals[4], 0, 1)
    cs_falls, cs_rises = edges(signals[0], 1, 0), edges(signals[0], 0, 1)
    irq_falls = edges(signals[4], 1, 0)
    phases = segment_timings(signals[4], rate)
    for phase in phases:
        steady = phase["steady"]
        phase["reference_relation"] = "insufficient_steady_data"
        if steady["first_sample"] is None or steady["pulse_starts"] < 3: continue
        first, last = steady["first_sample"], steady["last_sample"]
        selected = irq_rises[(irq_rises >= first) & (irq_rises <= last)]
        matching = [b for b in bursts if b["clock_candidate"] and
                    b["first_sample"] <= first and b["last_sample"] >= last]
        overlap = [b for b in bursts if b["clock_candidate"] and
                   b["last_sample"] >= first and b["first_sample"] <= last]
        phase["reference_relation"] = "continuous_reference" if matching else (
            "reference_boundary_or_gap" if overlap else "no_reference_observed")
        if matching:
            cycles = np.diff(np.searchsorted(clock_rises, selected))
            phase["clock_cycles_per_data_ready"] = distribution(cycles)
            phase["reference_hz"] = matching[0]["frequency_hz"]
            phase["expected_odr_hz_at_25_setting"] = matching[0]["frequency_hz"]/2048
            phase["reference_continuity_ok"] = matching[0]["continuous_32khz_candidate"]
        # A complete SPI acknowledgement must sit inside each steady IRQ pulse.
        good = 0
        for rise in selected:
            f = np.searchsorted(irq_falls, rise)
            c = np.searchsorted(cs_falls, rise)
            if f == len(irq_falls) or c == len(cs_falls): continue
            end = irq_falls[f]
            ce = np.searchsorted(cs_rises, cs_falls[c])
            if (ce < len(cs_rises) and rise < cs_falls[c] < cs_rises[ce] < end and
                np.searchsorted(cs_falls, end)-c == 1): good += 1
        phase["steady_irqs_with_one_complete_cs_ack"] = good
    return {"sample_rate_hz": rate, "sample_count": len(signals[0]),
            "reference_bursts": bursts, "irq_phases": phases,
            "limits": "UART must confirm clock selection and FILTER_CTL=0x51. "
                      "Short startup IRQ groups and partial phases are retained. "
                      "Digital timing only; 1 MHz cannot establish SPI bytes or analog edge quality."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    header, session, rate, signals = load_capture(args.capture, timing_only=True, channel_count=6)
    result = analyze_signals(signals, rate)
    result.update(sha256=hashlib.sha256(args.capture.read_bytes()).hexdigest(),
                  header_trigger_time_unix_ms=header.getint("header", "trigger time"),
                  threshold_v=session["Threshold Level"])
    text = json.dumps(result, indent=2)
    if args.output:
        with args.output.open("x", encoding="utf-8") as stream: stream.write(text+"\n")
    else: print(text)


if __name__ == "__main__": main()
