"""Compare an RFID_BENCH_RAW_DUMP UART log with a host replay of the same words.

The board streams one "d t n s rate" line per FIFO batch, the batch words as
"w ..." lines, then "r center_ms x y z" per resampler output, "c ts class 0 0"
per classification and "x t i code 0" per reset. This tool extracts the d/w
lines after each boot, feeds them to Tests/fifo_dump_replay.exe and compares
the board's d/x/r/c lines with the host's, line by line. It also counts
classifications between consecutive "window" log lines and writes the
reconstructed samples as CSV. Standard library only.

    python Tools/compare-fifo-dump.py --log capture.txt --exe build/<host>/Tests/fifo_dump_replay.exe --out <dir>
"""
import argparse
import json
import subprocess
from pathlib import Path

TAGS = ('d ', 'w ', 'x ', 'r ', 'c ')


def segments(lines):
    """Split the log at 'boot ' lines; yield (boot_line, lines after it)."""
    current, boot = [], None
    for line in lines:
        if line.startswith('boot '):
            if boot is not None:
                yield boot, current
            boot, current = line, []
        elif boot is not None:
            current.append(line)
    if boot is not None:
        yield boot, current


def analyze(boot, lines, exe, out_dir, index):
    board = [l for l in lines if l[:2] in TAGS]
    feed = '\n'.join(l for l in board if l[:2] in ('d ', 'w ')) + '\n'
    run = subprocess.run([str(exe), '--samples'], input=feed, text=True, capture_output=True)
    host_lines = run.stdout.splitlines()
    host = [l for l in host_lines if l[:2] in ('d ', 'x ', 'r ', 'c ')]
    samples = [l.split()[1:] for l in host_lines if l.startswith('s ')]
    board_cmp = [l for l in board if l[:2] != 'w ']
    mismatches = []
    for i, (a, b) in enumerate(zip(board_cmp, host)):
        if a.split() != b.split():
            mismatches.append({'index': i, 'board': a, 'host': b})
            if len(mismatches) >= 20:
                break
    # Classifications per sealed window, in board order.
    per_window, count = [], 0
    for line in lines:
        if line.startswith('c '):
            count += 1
        elif line.startswith('window '):
            per_window.append(count)
            count = 0
    report = {
        'boot': boot, 'batches': sum(1 for l in board if l.startswith('d ')),
        'words': sum(len(l.split()) - 1 for l in board if l.startswith('w ')),
        'board_lines': len(board_cmp), 'host_lines': len(host),
        'resampler_outputs': sum(1 for l in board_cmp if l.startswith('r ')),
        'classifications': sum(1 for l in board_cmp if l.startswith('c ')),
        'resets': [l for l in board_cmp if l.startswith('x ')][:20],
        'classifications_per_sealed_window': per_window, 'open_window_classifications': count,
        'mismatches': mismatches, 'replay_exit': run.returncode, 'replay_stderr': run.stderr[-500:],
        'samples': len(samples),
    }
    with (out_dir / f'samples-{index}.csv').open('w', newline='', encoding='utf-8') as fh:
        fh.write('t_us,x,y,z\n')
        fh.writelines(','.join(s) + '\n' for s in samples)
    return report


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--log', type=Path, required=True)
    ap.add_argument('--exe', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    lines = args.log.read_text(encoding='utf-8', errors='replace').splitlines()
    reports = [analyze(boot, seg, args.exe.resolve(), args.out, i)
               for i, (boot, seg) in enumerate(segments(lines)) if any(l.startswith('d ') for l in seg)]
    (args.out / 'report.json').write_text(json.dumps(reports, indent=1), encoding='utf-8')
    ok = bool(reports)
    for i, r in enumerate(reports):
        status = 'OK' if not r['mismatches'] and r['replay_exit'] == 0 and r['board_lines'] == r['host_lines'] else 'MISMATCH'
        ok &= status == 'OK'
        print(f"segment {i}: {r['boot']}")
        print(f"  batches {r['batches']}, words {r['words']}, samples {r['samples']}, resampler outputs "
              f"{r['resampler_outputs']}, classifications {r['classifications']}, resets {len(r['resets'])}")
        print(f"  board lines {r['board_lines']} vs host lines {r['host_lines']}; mismatches {len(r['mismatches'])} -> {status}")
        if r['replay_exit']:
            print('  replay error:', r['replay_stderr'].strip())
        for m in r['mismatches'][:5]:
            print(f"  #{m['index']}: board '{m['board']}' host '{m['host']}'")
        print(f"  classifications per sealed window: {r['classifications_per_sealed_window']} (open: {r['open_window_classifications']})")
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
