#!/usr/bin/env python3
"""Reproducible small-input differential checks against the preserved v9 binary."""
import argparse
import json
import random
import sys
from pathlib import Path
from validate import run, common_args

ROOT = Path(__file__).resolve().parents[1]

def cases(count):
    rng = random.Random(20261004)
    for index in range(count):
        mode = index % 4
        depth = 1 + (index // 4) % 3
        if mode == 3:
            points = []
            grid = ['2 2']
            lines = []
            circles = []
        else:
            points = rng.sample([(0, 0), (1, 0), (0, 1), (1, 1), (2, 0),
                                 (-1, 1), (0.5, 0.25), (1.25, -0.5)], 3 + index % 2)
            grid = []
            lines = [(0, 1, 0)] if index % 3 == 0 else []
            circles = [(0, 0, 1)] if index % 5 == 0 else []
        goal_type = (index // 3) % 3
        if goal_type == 0:
            goal = ['0 0 1', rng.choice(['0.5 0.5', '0.3333333333333333 0.2', '0 0'])]
        elif goal_type == 1:
            goal = ['1 0 0', rng.choice(['0 1 0.5', '1 1 1', '1 0 0'])]
        else:
            goal = ['0 1 0', rng.choice(['0 0 1', '0.5 0.5 0.5'])]
        text = [str(depth), str(mode), *grid,
                f'{len(points)} {len(lines)} 0 0 {len(circles)}',
                *[' '.join(map(str, p)) for p in points],
                *[' '.join(map(str, l)) for l in lines],
                *[' '.join(map(str, c)) for c in circles], *goal]
        yield index, ('\n'.join(text) + '\n').encode()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--exe', required=True)
    ap.add_argument('--baseline', required=True)
    ap.add_argument('--count', type=int, default=80)
    ap.add_argument('--output', type=Path)
    ap.add_argument('--time-limit', type=float, default=10)
    args = ap.parse_args()
    results = []
    for index, data in cases(args.count):
        flags = common_args(1, '1e-11' if index % 2 else '1e-13', args.time_limit)
        if index % 4 == 0:
            flags.append('--dedup-candidates')
        if index % 7 == 0:
            flags.append('--no-symmetry')
        if index % 9 == 0:
            flags += ['--no-stream-dedup', '--no-goal-first']
        a = run(args.baseline, data, flags, args.time_limit + 10)
        b = run(args.exe, data, flags, args.time_limit + 10)
        # Raw counts measure attempted geometry work, deliberately reduced by
        # cached priority bands and increased by the reverse-line bug fix.
        fields = ['status', 'solutions', 'nodes', 'unique_candidates']
        complete = a['status'] != 'TIMEOUT_PARTIAL' and b['status'] != 'TIMEOUT_PARTIAL'
        equal = complete and all(a[k] == b[k] for k in fields)
        row = {'case': index, 'pass': equal, 'baseline': a, 'optimized': b}
        if not equal:
            row['input'] = data.decode()
            row['args'] = flags
        results.append(row)
        print(f"{'PASS' if equal else 'FAIL'} random#{index} {a['status']}/{b['status']} "
              f"nodes={a['nodes']}/{b['nodes']}", flush=True)
    report = {'all_pass': all(x['pass'] for x in results), 'seed': 20261004, 'results': results}
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return 0 if report['all_pass'] else 1

if __name__ == '__main__':
    sys.exit(main())
