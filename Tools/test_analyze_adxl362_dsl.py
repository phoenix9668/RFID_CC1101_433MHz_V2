"""Synthetic mode-0 signals; no hardware or captured proprietary data required."""

import json
from pathlib import Path
import tempfile
import unittest
import zipfile
import numpy as np
from analyze_adxl362_dsl import decode_spi, fifo_summary, load_capture


def waveform(tx=b"\x0b\x0c\x00\x00", rx=b"\x00\x00\xc5\x01"):
    lines = [[1, 0, 0, 0, 0]] * 20
    lines += [[0, 0, 0, 0, 0]] * 8
    for a, b in zip(tx, rx):
        for bit in range(7, -1, -1):
            data = [0, 0, (a >> bit) & 1, (b >> bit) & 1, 0]
            lines += [data] * 6
            lines += [[0, 1, data[2], data[3], 0]] * 6
        lines += [[0, 0, a & 1, b & 1, 0]] * 20
    lines += [[0, 0, 0, 0, 0]] * 8 + [[1, 0, 0, 0, 0]] * 20
    return np.array(lines, dtype=np.uint8).T.copy()


class DecodeTests(unittest.TestCase):
    def test_count_read(self):
        transactions, _, _ = decode_spi(waveform(), 48_000_000)
        self.assertEqual(transactions[0]["data_hex"], "c5 01")
        self.assertEqual(transactions[0]["clock_hz"], 4_000_000)

    def test_fifo_sign_and_tags(self):
        tx = b"\x0d" + bytes(6)
        rx = b"\x00\xff\x3f\x02\x40\xfd\xbf"
        transactions, _, _ = decode_spi(waveform(tx, rx), 48_000_000)
        summary = fifo_summary(transactions)[0]
        self.assertEqual(summary["values_first_12"], [-1, 2, -3])
        self.assertEqual(summary["xyz_order_errors"], 0)
        self.assertEqual(summary["invalid_sign_extensions"], 0)

    def test_truncated_cs(self):
        with self.assertRaisesRegex(ValueError, "truncates"):
            decode_spi(waveform()[:, :-25], 48_000_000)

    def test_incomplete_byte(self):
        lines = waveform()
        rising = np.flatnonzero(np.diff(lines[1].astype(int)) == 1) + 1
        lines[1, rising[-1]:rising[-1]+6] = 0
        with self.assertRaisesRegex(ValueError, "incomplete SPI byte"):
            decode_spi(lines, 48_000_000)

    def test_odd_fifo_rejected(self):
        with self.assertRaisesRegex(ValueError, "Odd FIFO"):
            decode_spi(waveform(b"\x0d\x00", b"\x00\x00"), 48_000_000)

    def test_setup_hold_ambiguity(self):
        lines = waveform()
        rising = np.flatnonzero(np.diff(lines[1].astype(int)) == 1)[0] + 1
        lines[2, rising] ^= 1
        with self.assertRaisesRegex(ValueError, "Data changes"):
            decode_spi(lines, 48_000_000)

    def test_tag_discontinuity_reported(self):
        summary = fifo_summary([{"command": "fifo", "data_hex": "01 00 02 80"}])[0]
        self.assertEqual(summary["xyz_order_errors"], 1)


class ArchiveTests(unittest.TestCase):
    def capture(self, path, rate=48_000_000, truncate=False, channels=5):
        signals = waveform()
        if channels == 6:
            signals = np.vstack([signals, np.arange(signals.shape[1], dtype=np.uint8) % 2])
        count = signals.shape[1]
        session = {"DeviceMode": 0, "Enable RLE Compress": 0,
                   "Using External Clock": 0, "Filter Targets": 0,
                   "Sample rate": str(rate), "Sample count": str(count),
                   "channel": [{"index": i, "enabled": True} for i in range(channels)]}
        with zipfile.ZipFile(path, "w") as archive:
            archive.writestr("header", f"[version]\nversion=3\n[header]\n"
                              f"total samples={count}\ntotal blocks=2\n")
            archive.writestr("session", json.dumps(session))
            for i, line in enumerate(signals):
                packed = np.packbits(line, bitorder="little").tobytes()
                if truncate and i == 3:
                    packed = packed[:-1]
                split = len(packed)//2
                archive.writestr(f"L-{i}/1", packed[split:])
                archive.writestr(f"L-{i}/0", packed[:split])
        return signals

    def test_numeric_block_order_and_bit_order(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"test.dsl"
            expected = self.capture(path)
            _, _, _, actual = load_capture(path)
            np.testing.assert_array_equal(actual, expected)

    def test_undersampled_spi_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"test.dsl"
            self.capture(path, rate=1_000_000)
            with self.assertRaisesRegex(ValueError, "at least 40"):
                load_capture(path)

    def test_short_channel_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"test.dsl"
            self.capture(path, truncate=True)
            with self.assertRaisesRegex(ValueError, "packed byte count"):
                load_capture(path)

    def test_timing_only_explicit_opt_in(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"test.dsl"
            expected = self.capture(path, rate=1_000_000)
            _, _, rate, actual = load_capture(path, timing_only=True)
            self.assertEqual(rate, 1_000_000)
            np.testing.assert_array_equal(actual, expected)

    def test_sixth_channel_is_explicit_and_required(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"test.dsl"
            self.capture(path)
            with self.assertRaisesRegex(ValueError, "channel 5"):
                load_capture(path, timing_only=True, channel_count=6)
            expected = self.capture(path, channels=6)
            _, _, _, actual = load_capture(path, timing_only=True, channel_count=6)
            np.testing.assert_array_equal(actual, expected)
            _, _, _, original = load_capture(path)
            np.testing.assert_array_equal(original, expected[:5])


if __name__ == "__main__":
    unittest.main()
