"""Read-only Excel ingestion and real C replay. Synthetic high-rate scenarios only.

Requires numpy, scipy and openpyxl. Never writes input workbooks.
"""
import argparse
import csv
import hashlib
import io
import json
from fractions import Fraction
from pathlib import Path
import re
import subprocess

import numpy as np
import openpyxl
from scipy.signal import resample_poly

from behavior_acceptance import write_acceptance


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def segments(path):
    workbook = openpyxl.load_workbook(path, read_only=True, data_only=True)
    try:
        for sheet in workbook:
            rows = sheet.iter_rows(values_only=True)
            header = [str(v).strip().lower() for v in next(rows)]
            indices = [header.index(a) for a in ('x', 'y', 'z')]
            n_index = header.index('n') if 'n' in header else None
            blocks, block, previous, breaks = [], [], None, []
            for row_number, row in enumerate(rows, 2):
                xyz = [row[i] for i in indices]
                if not all(isinstance(v, (int, float)) and np.isfinite(v) and int(v) == v
                           and -2048 <= v <= 2047 for v in xyz):
                    raise ValueError(f'{path.name}/{sheet.title}:{row_number}: invalid XYZ {xyz}')
                serial = row[n_index] if n_index is not None else None
                label = re.fullmatch(r'samples\[(\d+)\]\s*', str(row[0]))
                if label:
                    serial = int(label[1])
                if serial is not None and previous is not None and serial != previous + 1:
                    blocks.append(block)
                    block = []
                    breaks.append(row_number)
                block.append(xyz)
                previous = serial
            if block:
                blocks.append(block)
            yield sheet.title, [np.array(b, dtype=np.int16) for b in blocks], breaks
    finally:
        workbook.close()


def replay(exe, samples, rate, filtered=False):
    times = np.rint(np.arange(len(samples)) * 1e6 / rate).astype(np.int64)
    data = ''.join(f'{t},{x},{y},{z}\n' for t, (x, y, z) in zip(times, samples))
    cmd = [str(exe)] + (['--filtered'] if filtered else [])
    run = subprocess.run(cmd, input=data, text=True, capture_output=True, check=True)
    output = (np.loadtxt(io.StringIO(run.stdout), delimiter=',', dtype=np.int64, ndmin=2)
              if run.stdout.strip() else np.empty((0, 5), dtype=np.int64))
    labels = output[output[:, 4] >= 0]
    return output, {int(round((r[0] + 40000) / 1e6)): int(r[4]) for r in labels}


def class_counts(values):
    return [sum(v == c for v in values) for c in range(1, 7)]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--data-root', type=Path, required=True)
    ap.add_argument('--host-build', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    exe = args.host_build.resolve() / 'Tests/resampler_replay.exe'
    legacy = args.host_build.resolve() / 'Tests/behavior_regression.exe'
    root = Path(__file__).resolve().parents[1]
    manifest = {'source_rate_hz_assumed': 25, 'source_odr_setting_confirmed': 25,
                'source_actual_rate_measured': False, 'human_labels': False,
                'high_rate_inputs': 'synthetic reconstruction, NOT new collar measurements',
                'sources': [], 'code_sha256': {str(p.relative_to(root)): digest(p) for p in
                    [root/'App/Src/behavior.c', root/'App/Src/resampler.c',
                     root/'App/Src/resampler_kernel.h', Path(__file__)]},
                'executable_sha256': digest(exe)}
    summaries = []
    for path in sorted(args.data_root.rglob('*.xlsx')):
        original_hash = digest(path)
        for sheet, blocks, breaks in segments(path):
            source_id = path.stem + '_' + sheet
            manifest['sources'].append({'file': str(path.relative_to(args.data_root)),
                'sheet': sheet, 'sha256': original_hash, 'samples': sum(map(len, blocks)),
                'segment_lengths': list(map(len, blocks)), 'index_break_rows': breaks,
                'xyz_min': np.concatenate(blocks).min(axis=0).tolist(),
                'xyz_max': np.concatenate(blocks).max(axis=0).tolist(),
                'rail_values': int(np.isin(np.concatenate(blocks), [-2048, 2047]).sum())})
            for segment, raw in enumerate(blocks):
                ident = f'{source_id}_seg{segment+1}'
                full_blocks = len(raw) // 150 * 150
                legacy_check = 'not enough for a 150-sample legacy block'
                if full_blocks:
                    check = subprocess.run([str(legacy)], input=raw[:full_blocks].astype('<i2').tobytes(),
                                           capture_output=True, check=True)
                    legacy_check = check.stderr.decode().strip()
                original, original_labels = replay(exe, raw, 25)
                baseline_path = args.output / (ident + '_original.csv')
                with baseline_path.open('w', newline='', encoding='utf-8') as handle:
                    writer = csv.writer(handle)
                    writer.writerow(['end_second_nominal', 'class_raw_0_is_warmup'])
                    writer.writerows(original_labels.items())
                for rate in [25.0, 39.502107, 50.0, 57.5]:
                    ratio = Fraction(rate / 25).limit_denominator(1000)
                    constructed_rate = 25 * ratio.numerator / ratio.denominator
                    high = np.rint(resample_poly(raw.astype(float), ratio.numerator,
                                                ratio.denominator, axis=0, padtype='line'))
                    # Simulated ADXL output must obey the physical signed 12-bit range.
                    clips = int(((high < -2048) | (high > 2047)).sum())
                    high = np.clip(high, -2048, 2047).astype(np.int16)
                    filtered, labels = replay(exe, high, constructed_rate, True)
                    # Exclude warmup, FIR edges and unfinished seconds identically.
                    common = [s for s in sorted(labels) if s in original_labels and
                              s >= 20 and s <= len(raw)/25 - 1 and
                              1 <= labels[s] <= 6 and 1 <= original_labels[s] <= 6]
                    old = [original_labels[s] for s in common]
                    new = [labels[s] for s in common]
                    matrix = np.zeros((6, 6), dtype=int)
                    for a, b in zip(old, new): matrix[a-1, b-1] += 1
                    changed = sum(a != b for a, b in zip(old, new))
                    summary = {'source': ident, 'samples': len(raw), 'nominal_seconds': len(raw)/25,
                        'input_rate_simulated': constructed_rate, 'rate_requested': rate,
                        'compared_seconds': len(common), 'different_seconds': changed,
                        'agreement': (1-changed/len(common)) if common else None,
                        'baseline_counts_common': class_counts(old), 'candidate_counts_common': class_counts(new),
                        'class_matrix': matrix.tolist(), 'legacy_equivalence': legacy_check,
                        'synthetic_clipped_axis_values': clips,
                        'candidate_output_samples': len(filtered)}
                    summaries.append(summary)
                    name = ident + f'_scheme2_{rate:g}.csv'
                    with (args.output / name).open('w', newline='', encoding='utf-8') as handle:
                        writer = csv.writer(handle)
                        writer.writerow(['end_second_nominal', 'original_class', 'scheme2_class', 'included_in_comparison'])
                        included = set(common)
                        for s in sorted(set(original_labels) | set(labels)):
                            writer.writerow([s, original_labels.get(s, ''), labels.get(s, ''), int(s in included)])
                    print(ident, rate, 'compared=', len(common), 'changed=', changed, flush=True)
        assert original_hash == digest(path), 'Input workbook changed during analysis'
    (args.output / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    (args.output / 'results.json').write_text(json.dumps(summaries, indent=2), encoding='utf-8')
    write_acceptance(args.output / 'results.json')
    text = ['# Scheme 2: historical collar-data replay', '',
            '## Scope and limits', '',
            '- The user confirmed that the source collars were configured at 25 Hz. Actual historical ODR is unknown.',
            '- All time axes below assume 25 Hz. These are algorithm agreement results, NOT classification accuracy.',
            '- No human labels or real 50 Hz recordings are available. High-rate streams are constructed using SciPy resample_poly.',
            '- The 25 Hz scenario is a filter-only control, not the selected 50 Hz production configuration.',
            '- The simulation cannot recover missing high-frequency motion or reproduce the hardware filter change at 50 Hz.',
            '- XYZ values are retained in their original integer units. Axis/range calibration across recordings is unverified.',
            '- Index resets split segments. No interpolation across gaps; warmup (<20 s) and the final second are excluded.',
            '- The unchanged C classifier is checked against the 34d57c2 legacy implementation on every complete 150-sample block.',
            '', '## Per-record comparison', '',
            '| Record | Simulated input Hz | Compared s | Changed s | Agreement |',
            '|---|---:|---:|---:|---:|']
    for r in summaries:
        agreement = f"{100*r['agreement']:.2f}%" if r['agreement'] is not None else 'n/a: too short'
        text.append(f"| {r['source']} | {r['input_rate_simulated']:.6f} | {r['compared_seconds']} | {r['different_seconds']} | {agreement} |")
    text += ['', '## Six-class counts on the common intervals', '',
             'Class order: rest, ingestion, movement, climb, ruminate, other. Original outputs are a comparator, not ground truth.', '',
             '| Simulated input | Baseline counts | Scheme 2 counts | Changed / compared |', '|---|---|---|---|']
    for rate in [25.0, 39.502107, 50.0, 57.5]:
        selected = [r for r in summaries if r['rate_requested'] == rate]
        a = np.sum([r['baseline_counts_common'] for r in selected], axis=0).tolist()
        b = np.sum([r['candidate_counts_common'] for r in selected], axis=0).tolist()
        changed = sum(r['different_seconds'] for r in selected)
        n = sum(r['compared_seconds'] for r in selected)
        text.append(f'| {rate} Hz | {a} | {b} | {changed} / {n} |')
    text += ['', '## Interpretation', '',
             'A rate-normalization path is implemented, not a newly trained behavior classifier. The 8 Hz Kaiser FIR deliberately removes high-frequency content before conversion; it is not claimed to reproduce the exact original analog filter.',
             'Agreement quantifies sensitivity to this processing. A disagreement is not necessarily an error; agreement is not proof of correctness.',
             'Short segments remain in the inventory and output files but do not contribute to agreement. Partial final seconds are not padded.',
             'Inspect results.json for full 6x6 transition matrices and CSV files for every classified second.',
             'acceptance.json checks both per-class reference recall and precision: movement/climb >=95%, ingestion/ruminate >=90%. Zero support is unassessable, never a pass; pooled and per-record results are separate. These are agreement gates, not accuracy guarantees.',
             'Real 50 Hz collar recordings, RTC rate estimation, STOP/wake timing, FIFO overrun behavior and current consumption still require board validation before deployment.']
    (args.output / 'README.md').write_text('\n'.join(text)+'\n', encoding='utf-8')


if __name__ == '__main__':
    main()
