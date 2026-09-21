"""Read-only mode-0 SPI analysis of unpacked DSView v3 logic archives.

Requires NumPy. Channels D0-D4 are CS, SCK, MOSI, MISO, INT2.
Only the explicit per-channel L-n/block format is supported, not sparse/RLE
archives. DSView v1.3.2 LogicSnapshot::get_sample_self numbers samples LSB first
inside little-endian words; SPI bytes on the wire are independently MSB first.
"""

import argparse
import configparser
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np


def edges(signal, before, after):
    return np.flatnonzero((signal[:-1] == before) & (signal[1:] == after)) + 1


def load_capture(path):
    with zipfile.ZipFile(path) as archive:
        header = configparser.ConfigParser()
        header.read_string(archive.read("header").decode("utf-8-sig"))
        session = json.loads(archive.read("session"))
        if header.getint("version", "version") != 3:
            raise ValueError("Only DSView v3 is supported")
        if session.get("DeviceMode") != 0 or session.get("Enable RLE Compress") != 0:
            raise ValueError("Only uncompressed logic data is supported")
        if session.get("Using External Clock") != 0 or session.get("Filter Targets") != 0:
            raise ValueError("Expected unfiltered, internally clocked capture")
        count = header.getint("header", "total samples")
        rate = int(session["Sample rate"])
        if not 0 < count <= 30_000_000 or count != int(session["Sample count"]):
            raise ValueError("Inconsistent sample count or more than 30M samples")
        if rate < 40_000_000:
            raise ValueError("Use at least 40 MS/s for nominal 4 MHz SPI")
        blocks = header.getint("header", "total blocks")
        names = archive.namelist()
        if len(names) != len(set(names)) or not 1 <= blocks <= 64:
            raise ValueError("Duplicate ZIP entries or invalid block count")
        signals = []
        enabled = {c["index"] for c in session["channel"] if c["enabled"]}
        for channel in range(5):
            expected = [f"L-{channel}/{i}" for i in range(blocks)]
            found = {name for name in names if name.startswith(f"L-{channel}/")}
            if channel not in enabled or found != set(expected):
                raise ValueError(f"Missing/unsupported channel {channel} blocks")
            size = sum(archive.getinfo(name).file_size for name in expected)
            if size != (count + 7) // 8:
                raise ValueError(f"Channel {channel}: unexpected packed byte count")
            packed = b"".join(archive.read(name) for name in expected)
            signals.append(np.unpackbits(np.frombuffer(packed, np.uint8),
                                         bitorder="little")[:count])
    return header, session, rate, signals


def decode_spi(signals, rate):
    cs, sck, mosi, miso, irq = signals
    if not cs[0] or not cs[-1]:
        raise ValueError("Capture truncates an active CS transaction")
    starts, ends = edges(cs, 1, 0), edges(cs, 0, 1)
    rises, falls = edges(sck, 0, 1), edges(sck, 1, 0)
    transactions = []
    for start, end in zip(starts, ends):
        clocks = rises[(rises > start) & (rises < end)]
        falling = falls[(falls > start) & (falls < end)]
        if sck[start] or sck[end] or len(clocks) != len(falling):
            raise ValueError("Incomplete clocks or non-mode-0 clock idle level")
        if not len(clocks) or len(clocks) % 8:
            raise ValueError("CS transaction contains an incomplete SPI byte")
        periods = np.diff(clocks.reshape(-1, 8), axis=1).ravel()
        if periods.min() < 8 or (falling - clocks).min() < 3:
            raise ValueError("Clock glitches or insufficient sampling resolution")
        tx = np.packbits(mosi[clocks], bitorder="big").tobytes()
        rx = np.packbits(miso[clocks], bitorder="big").tobytes()
        unstable = sum(int(np.count_nonzero((line[clocks-1] != line[clocks]) |
                                             (line[clocks] != line[clocks+1])))
                       for line in (mosi, miso))
        if unstable:
            raise ValueError("Data changes within one sample of a sampling edge")
        item = {
            "start_sample": int(start), "end_sample": int(end),
            "start_ms": float(start * 1000 / rate),
            "end_ms": float(end * 1000 / rate),
            "bytes": len(tx), "mosi_hex": tx.hex(" "), "miso_hex": rx.hex(" "),
            "clock_hz": float(rate / periods.mean()),
            "within_byte_period_samples": [int(periods.min()), int(periods.max())],
        }
        if tx[0] in (0x0a, 0x0b) and len(tx) >= 3:
            item.update(command="write" if tx[0] == 0x0a else "read",
                        address=tx[1], data_hex=(tx if tx[0] == 0x0a else rx)[2:].hex(" "))
        elif tx[0] == 0x0d:
            if (len(rx)-1) % 2:
                raise ValueError("Odd FIFO payload: do not join discarded half words")
            item.update(command="fifo", data_hex=rx[1:].hex(" "))
        else:
            raise ValueError(f"Unexpected ADXL362 command/length: {tx.hex(' ')}")
        transactions.append(item)
    return transactions, edges(irq, 0, 1).tolist(), edges(irq, 1, 0).tolist()


def fifo_summary(transactions):
    groups, chunks = [], []
    for item in transactions + [{"command": "end"}]:
        if item["command"] == "fifo":
            chunks.append(bytes.fromhex(item["data_hex"]))
        elif chunks:
            words = np.frombuffer(b"".join(chunks), dtype="<u2")
            tags = words >> 14
            values = (words & 0x3fff).astype(np.int32)
            values[values >= 0x2000] -= 0x4000
            groups.append({
                "chunk_bytes": list(map(len, chunks)), "words": len(words),
                "tag_counts_xyzt": np.bincount(tags, minlength=4).tolist(),
                "starts_with_x": bool(len(tags) and tags[0] == 0),
                "xyz_order_errors": int(np.count_nonzero(tags[1:] != (tags[:-1]+1) % 3)),
                "invalid_sign_extensions": int(np.count_nonzero(
                    (words & 0x3000) != np.where(words & 0x800, 0x3000, 0))),
                "values_first_12": values[:12].tolist(),
            })
            chunks = []
    return groups


def analyze(path):
    header, session, rate, signals = load_capture(path)
    transactions, rising, falling = decode_spi(signals, rate)
    return {
        "file": Path(path).name, "sha256": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        "sample_rate_hz": rate, "sample_count": len(signals[0]),
        "header_trigger_time_unix_ms": header.getint("header", "trigger time"),
        "header_trigger_sample": header.getint("header", "trigger pos"),
        "threshold_v": float(session["Threshold Level"]),
        "irq_rising_samples": rising, "irq_falling_samples": falling,
        "transactions": transactions, "fifo_groups": fifo_summary(transactions),
        "limits": "Digital trace only; not analog signal integrity, ODR or firmware-stack acceptance.",
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    try:
        print(json.dumps(analyze(args.capture), indent=2))
    except (ValueError, KeyError, OSError, zipfile.BadZipFile, configparser.Error) as error:
        parser.exit(1, f"Unsupported/invalid capture: {error}\n")
