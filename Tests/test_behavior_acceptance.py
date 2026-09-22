"""Host-only tests; no scientific Python dependencies needed."""
import importlib.util
from pathlib import Path
import unittest


SPEC = importlib.util.spec_from_file_location(
    'behavior_acceptance', Path(__file__).resolve().parents[1] / 'Tools/behavior_acceptance.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class AcceptanceTest(unittest.TestCase):
    def test_empty_is_not_perfect(self):
        result = MODULE.evaluate([[0] * 6 for _ in range(6)])
        self.assertEqual(result['status'], 'unassessable')
        self.assertIsNone(result['classes'][3]['recall'])
        self.assertIsNone(result['classes'][3]['precision'])

    def test_exact_thresholds_pass(self):
        matrix = [[0] * 6 for _ in range(6)]
        for class_id, threshold in MODULE.THRESHOLDS_PERCENT.items():
            i = class_id - 1
            matrix[i][i] = threshold
            matrix[i][5] = 100 - threshold
            matrix[5][i] = 100 - threshold
        self.assertEqual(MODULE.evaluate(matrix)['status'], 'pass')

    def test_one_sided_errors_fail(self):
        for missed, extra in ((6, 0), (0, 6)):
            matrix = [[0] * 6 for _ in range(6)]
            matrix[2][2], matrix[2][5], matrix[5][2] = 94, missed, extra
            self.assertEqual(MODULE.evaluate(matrix)['classes'][2]['status'], 'fail')

    def test_rounding_cannot_create_pass(self):
        matrix = [[0] * 6 for _ in range(6)]
        matrix[2][2], matrix[2][5] = 94999, 5001
        self.assertEqual(MODULE.evaluate(matrix)['classes'][2]['status'], 'fail')

    def test_absent_one_side_fails(self):
        for a, b in ((3, 5), (5, 3)):
            matrix = [[0] * 6 for _ in range(6)]
            matrix[a][b] = 1
            self.assertEqual(MODULE.evaluate(matrix)['classes'][3]['status'], 'fail')

    def test_invalid_matrix(self):
        with self.assertRaises(ValueError):
            MODULE.evaluate([[0]])
        matrix = [[0] * 6 for _ in range(6)]
        matrix[0][0] = -1
        with self.assertRaises(ValueError):
            MODULE.evaluate(matrix)


if __name__ == '__main__':
    unittest.main()
