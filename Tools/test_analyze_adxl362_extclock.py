import unittest
import numpy as np
from analyze_adxl362_extclock import analyze_signals, reference_bursts


class ExternalClockTests(unittest.TestCase):
    rate = 1_024_000

    def signals(self, cycles=2048):
        size = self.rate*2
        clock = ((np.arange(size)//16) % 2).astype(np.uint8)
        irq = np.zeros(size, np.uint8)
        cs = np.ones(size, np.uint8)
        for rise in range(64000, size-100, cycles*32):
            irq[rise:rise+100] = 1
            cs[rise+20:rise+40] = 0
        return [cs, np.zeros(size, np.uint8), np.zeros(size, np.uint8),
                np.zeros(size, np.uint8), irq, clock]

    def test_exact_external(self):
        result = analyze_signals(self.signals(), self.rate)
        burst, phase = result["reference_bursts"][0], result["irq_phases"][0]
        self.assertEqual(burst["frequency_hz"], 32000)
        self.assertTrue(burst["continuous_32khz_candidate"])
        self.assertEqual(phase["expected_odr_hz_at_25_setting"], 15.625)
        self.assertEqual(phase["steady"]["frequency_hz"], 15.625)
        self.assertEqual(phase["clock_cycles_per_data_ready"][0][0], 2048)
        self.assertEqual(phase["steady_irqs_with_one_complete_cs_ack"], phase["steady"]["pulse_starts"])

    def test_wrong_division_is_not_hidden(self):
        phase = analyze_signals(self.signals(1620), self.rate)["irq_phases"][0]
        self.assertEqual(phase["clock_cycles_per_data_ready"][0][0], 1620)
        self.assertNotEqual(phase["steady"]["frequency_hz"], 15.625)

    def test_missing_clock_pulse(self):
        signals = self.signals()
        signals[5][1_000_000:1_000_032] = 0
        result = analyze_signals(signals, self.rate)
        self.assertFalse(result["reference_bursts"][0]["continuous_32khz_candidate"])
        self.assertFalse(result["irq_phases"][0]["reference_continuity_ok"])

    def test_narrow_clock_pulses_rejected(self):
        signals = self.signals()
        signals[5] = (np.arange(len(signals[5])) % 32 == 16).astype(np.uint8)
        burst = analyze_signals(signals, self.rate)["reference_bursts"][0]
        self.assertEqual(burst["frequency_hz"], 32000)
        self.assertFalse(burst["continuous_32khz_candidate"])

    def test_internal_or_disconnected_probe(self):
        signals = self.signals()
        signals[5][:] = 0
        phase = analyze_signals(signals, self.rate)["irq_phases"][0]
        self.assertEqual(phase["reference_relation"], "no_reference_observed")
        self.assertNotIn("expected_odr_hz_at_25_setting", phase)

    def test_reference_boundary(self):
        signals = self.signals()
        signals[5][self.rate:] = 0
        phase = analyze_signals(signals, self.rate)["irq_phases"][0]
        self.assertEqual(phase["reference_relation"], "reference_boundary_or_gap")

    def test_single_activity_pulse_is_not_clock(self):
        clock = np.zeros(10000, np.uint8)
        clock[200:600] = 1
        bursts, _ = reference_bursts(clock, self.rate)
        self.assertFalse(bursts[0]["clock_candidate"])

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError): analyze_signals(self.signals()[:5], self.rate)
        with self.assertRaises(ValueError): analyze_signals(self.signals(), 1000)


if __name__ == "__main__": unittest.main()
