"""Compare the original and rebuilt C algorithms on recorded CSV samples."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import subprocess

p = argparse.ArgumentParser()
p.add_argument("--exe", type=Path, required=True)
p.add_argument("--data", type=Path, required=True)
p.add_argument("--out", type=Path, required=True)
a = p.parse_args()
a.out.mkdir(parents=True, exist_ok=True)
names = ["chuanxi.csv", "\u5598\u606f20250820 1644.csv", "02 1004 20230315 1819-2219 29W.csv"]
reports = []
for number, name in enumerate(names):
    source = a.data / name
    raw = source.read_bytes()
    try:
        text = raw.decode("utf-8-sig")
    except UnicodeDecodeError:
        text = raw.decode("gb18030")
    samples = bytearray()
    for row in csv.DictReader(text.splitlines()):
        try:
            xyz = [int(row[axis]) for axis in ("x", "y", "z")]
        except (ValueError, KeyError, TypeError) as error:
            raise ValueError(f"Invalid XYZ columns in {name}: {row}") from error
        if len(xyz) != 3 or any(n < -2048 or n > 2047 for n in xyz):
            raise ValueError(f"Invalid sample in {name}: {row}")
        samples.extend(struct.pack("<hhh", *xyz))
    run = subprocess.run([str(a.exe.resolve())], input=samples, capture_output=True, check=True)
    (a.out / f"behavior-{number}.csv").write_bytes(run.stdout)
    reports.append({"source": name, "source_sha256": hashlib.sha256(raw).hexdigest(),
                    "samples": len(samples) // 6, "tail_samples": (len(samples) // 6) % 150,
                    "result_sha256": hashlib.sha256(run.stdout).hexdigest(),
                    "result": run.stderr.decode().strip()})
(a.out / "summary.json").write_text(json.dumps(reports, ensure_ascii=True, indent=2), encoding="utf-8")
print(json.dumps(reports, ensure_ascii=True, indent=2))
