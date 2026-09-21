import unittest
import numpy as np
from analyze_adxl362_timing import summarize


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


if __name__ == "__main__":
    unittest.main()
