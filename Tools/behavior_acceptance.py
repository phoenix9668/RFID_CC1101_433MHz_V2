"""Per-class agreement gates against legacy outputs, not labelled accuracy."""
import argparse
import csv
import hashlib
import json
from pathlib import Path


CLASSES = ('rest', 'ingestion', 'movement', 'climb', 'ruminate', 'other')
THRESHOLDS_PERCENT = {2: 90, 3: 95, 4: 95, 5: 90}


def evaluate(matrix):
    if (len(matrix) != 6 or any(len(row) != 6 for row in matrix)
            or any(type(n) is not int or n < 0 for row in matrix for n in row)):
        raise ValueError('Expected a nonnegative integer 6x6 matrix')
    classes = []
    for i, name in enumerate(CLASSES):
        reference = sum(matrix[i])
        candidate = sum(row[i] for row in matrix)
        matched = matrix[i][i]
        threshold = THRESHOLDS_PERCENT.get(i + 1)
        status = 'not_required'
        if threshold is not None:
            # Compare integers, not rounded percentages displayed in the report.
            if any(n and 100 * matched < threshold * n for n in (reference, candidate)):
                status = 'fail'
            elif not reference or not candidate:
                status = 'unassessable'
            else:
                status = 'pass'
        classes.append({'class_id': i + 1, 'name': name,
                        'threshold_percent': threshold,
                        'reference_seconds': reference, 'candidate_seconds': candidate,
                        'matched_seconds': matched,
                        'recall': matched / reference if reference else None,
                        'precision': matched / candidate if candidate else None,
                        'status': status})
    statuses = {c['status'] for c in classes}
    status = ('fail' if 'fail' in statuses else
              'unassessable' if 'unassessable' in statuses else 'pass')
    return {'status': status, 'classes': classes}


def write_acceptance(results_path):
    results_path = Path(results_path)
    content = results_path.read_bytes()
    records = json.loads(content)
    if not records:
        raise ValueError('No replay records')
    totals, per_record, csv_hashes, seen = {}, [], {}, set()
    for record in records:
        rate, source = record['rate_requested'], record['source']
        key = (source, rate)
        if key in seen:
            raise ValueError(f'Duplicate record: {key}')
        seen.add(key)
        # Independently reconcile every included second with the saved matrix.
        path = results_path.parent / f'{source}_scheme2_{rate:g}.csv'
        csv_content = path.read_bytes()
        csv_hashes[path.name] = hashlib.sha256(csv_content).hexdigest()
        matrix = [[0] * 6 for _ in range(6)]
        seconds = set()
        for row in csv.DictReader(csv_content.decode('utf-8').splitlines()):
            if row['included_in_comparison'] != '1':
                continue
            second = int(row['end_second_nominal'])
            a, b = int(row['original_class']), int(row['scheme2_class'])
            if second in seconds or not (1 <= a <= 6 and 1 <= b <= 6):
                raise ValueError(f'Invalid or duplicate included label in {path.name}')
            seconds.add(second)
            matrix[a - 1][b - 1] += 1
        if (matrix != record['class_matrix']
                or [sum(row) for row in matrix] != record['baseline_counts_common']
                or [sum(row[i] for row in matrix) for i in range(6)] != record['candidate_counts_common']
                or len(seconds) != record['compared_seconds']
                or len(seconds) - sum(matrix[i][i] for i in range(6)) != record['different_seconds']):
            raise ValueError(f'Matrix/count reconciliation failed: {path.name}')
        per_record.append({'source': source, 'rate_requested': rate, **evaluate(matrix)})
        total = totals.setdefault(rate, [[0] * 6 for _ in range(6)])
        for i in range(6):
            for j in range(6):
                total[i][j] += matrix[i][j]
    report = {
        'interpretation': 'Reference agreement, NOT accuracy; no human labels. High-rate inputs are synthetic.',
        'gate': 'Both reference recall and reference precision must meet the class threshold.',
        'aggregation': 'Pool seconds within each rate, never across synthetic rate scenarios. Per-record gates are also reported.',
        'coverage': 'Undefined metrics never pass. Small positive support is reported, not a statistical guarantee.',
        'results_sha256': hashlib.sha256(content).hexdigest(),
        'evaluator_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'verified_csv_sha256': csv_hashes,
        'scenarios': [{'rate_requested': rate, 'filter_only_control': rate == 25.0,
                       'class_matrix': matrix, **evaluate(matrix)}
                      for rate, matrix in sorted(totals.items())],
        'per_record': per_record,
    }
    output = results_path.with_name('acceptance.json')
    output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--results', type=Path, required=True)
    parser.add_argument('--require-pass', action='store_true',
                        help='Exit 2 unless every non-control pooled scenario passes all required classes')
    args = parser.parse_args()
    report = write_acceptance(args.results)
    for scenario in report['scenarios']:
        print(f"{scenario['rate_requested']} Hz: {scenario['status']}")
        for result in scenario['classes']:
            if result['threshold_percent'] is not None:
                recall = result['recall']
                precision = result['precision']
                print(f"  {result['name']}: reference={result['reference_seconds']}, "
                      f"candidate={result['candidate_seconds']}, matched={result['matched_seconds']}, "
                      f"recall={recall}, precision={precision}, {result['status']}")
    if args.require_pass and any(s['status'] != 'pass' for s in report['scenarios']
                                 if not s['filter_only_control']):
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
