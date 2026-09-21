import unittest
import numpy as np
from analyze_adxl362_timing import summarize, segment_timings


class TimingTests(unittest.TestCase):
    def test_known_period(self):
        signal = np.zeros(120, dtype=np.uint8)
        for start in (10, 50, 90):
            signal[start:start+3] = 1
        result = summarize(signal, 1000, True)
        self.assertEqual(result["frequency_hz"], 25)
        self.assertEqual(result["complete_pulses"], 3)
        self.assertEqual(result["width_us"]["mean"], 3000)
        self.assertEqual(summarize(1-signal, 1000, False)["frequency_hz"], 25)

    def test_no_edges_is_not_zero_hz(self):
        for value in (0, 1):
            result = summarize(np.full(100, value, dtype=np.uint8), 1000, True)
            self.assertIsNone(result["frequency_hz"])
            self.assertEqual(result["pulse_starts"], 0)
            self.assertEqual(result["high_samples"], 100*value)

    def test_partial_pulses(self):
        signal = np.zeros(100, dtype=np.uint8)
        signal[:5] = 1
        signal[40:43] = 1
        signal[80:] = 1
        result = summarize(signal, 1000, True)
        self.assertEqual(result["frequency_hz"], 25)
        self.assertEqual(result["complete_pulses"], 1)
        self.assertTrue(result["active_at_start"] and result["active_at_end"])

    def test_single_pulse(self):
        result = summarize(np.array([0, 1, 0], dtype=np.uint8), 1000, True)
        self.assertIsNone(result["frequency_hz"])
        self.assertEqual(result["complete_pulses"], 1)

    def test_bad_input(self):
        with self.assertRaises(ValueError):
            summarize(np.array([0, 2]), 1000, True)
        with self.assertRaises(ValueError):
            summarize(np.array([0, 1]), 0, True)

    def test_sweep_segments_exclude_standby(self):
        signal = np.zeros(4000, dtype=np.uint8)
        for base, period in ((600, 40), (1800, 20), (2800, 10)):
            for i in range(10):
                signal[base+i*period:base+i*period+2] = 1
        groups = segment_timings(signal, 1000)
        self.assertEqual([g["frequency_hz"] for g in groups], [25, 50, 100])
        self.assertTrue(all(g["leading_gap_observed"] and g["trailing_gap_observed"] for g in groups))

    def test_segment_edges_and_absence(self):
        signal = np.zeros(1000, dtype=np.uint8)
        self.assertEqual(segment_timings(signal, 1000), [])
        signal[1:3] = 1
        signal[998:] = 1
        groups = segment_timings(signal, 1000)
        self.assertFalse(groups[0]["leading_gap_observed"])
        self.assertFalse(groups[1]["trailing_gap_observed"])
        self.assertTrue(all(g["frequency_hz"] is None for g in groups))

    def test_fixed_settling_trim_keeps_raw_result(self):
        signal = np.zeros(12000, dtype=np.uint8)
        signal[601:603] = 1
        for start in range(610, 10610, 40):
            signal[start:start+2] = 1
        result = segment_timings(signal, 1000)[0]
        self.assertNotEqual(result["frequency_hz"], 25)
        self.assertEqual(result["steady"]["frequency_hz"], 25)
        self.assertLess(result["steady"]["pulse_starts"], result["pulse_starts"])
        short = segment_timings(np.array([0, 1, 0]), 1000)[0]
        self.assertIsNone(short["steady"]["frequency_hz"])


if __name__ == "__main__":
    unittest.main()
