#!/usr/bin/env python3
"""Run deterministic regressions and fair fixed-work benchmarks."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import platform
import re
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]

def run(executable, data, args, timeout):
    started = time.perf_counter()
    p = subprocess.run([str(Path(executable).resolve()), *args], input=data,
                       stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout)
    text = p.stdout.decode('utf-8', errors='replace')
    row = {'returncode': p.returncode, 'wall_seconds': time.perf_counter() - started}
    for key, pattern, cast in [
        ('status', r'Result status:\s*(\S+)', str),
        ('seconds', r'Search Time:\s*([\d.eE+-]+)', float),
        ('nodes', r'^Nodes:\s*(\d+)', int),
        ('raw_candidates', r'^Raw candidates:\s*(\d+)', int),
        ('unique_candidates', r'^Unique candidates:\s*(\d+)', int),
        ('solutions', r'^Distinct solutions:\s*(\d+)', int),
    ]:
        m = re.search(pattern, text, re.MULTILINE)
        row[key] = cast(m.group(1)) if m else None
    if row['status'] is None:
        row['error'] = (p.stderr + p.stdout[-1000:]).decode('utf-8', errors='replace')
    return row


def common_args(threads, eps, seconds):
    return [f'--threads={threads}', '--solutions=1', f'--eps={eps}',
            f'--time-limit={seconds}', '--no-pause', '--raw-output']


def regression(args):
    rows = json.loads((ROOT / 'tests/regression_manifest.json').read_text())
    results = []
    for row in rows:
        if args.quick and row['slow']:
            continue
        data = (ROOT / 'tests/cases' / row['input']).read_bytes()
        for threads in args.threads:
            result = run(args.exe, data, common_args(threads, row['eps'], args.time_limit),
                         args.time_limit + 20)
            result.update(input=row['input'], eps=row['eps'], threads=threads,
                          expected=row['expected'])
            result['pass'] = result['status'] == row['expected'] and result['returncode'] == (
                0 if row['expected'] == 'QUOTA_REACHED' else 2)
            results.append(result)
            print(f"{'PASS' if result['pass'] else 'FAIL'} t={threads} {row['eps']} "
                  f"{row['input']} {result['status']} {result['seconds']}s", flush=True)
    return {'kind': 'regression', 'all_pass': all(r['pass'] for r in results), 'tests': results}


def benchmark(args):
    manifest = json.loads((ROOT / 'benchmarks/manifest.json').read_text())
    results = []
    for case in manifest:
        if args.case and case['name'] not in args.case:
            continue
        data = (ROOT / case['input']).read_bytes()
        opts = common_args(args.threads[0], case.get('eps', '1e-11'), args.time_limit)
        opts += case.get('args', [])
        timings = {'baseline': [], 'optimized': []}
        for repeat in range(args.repeat):
            order = [('baseline', args.baseline), ('optimized', args.exe)]
            if repeat % 2:
                order.reverse()
            for label, exe in order:
                result = run(exe, data, opts, args.time_limit + 20)
                timings[label].append(result)
                print(f"{case['name']} {label} #{repeat+1} {result['status']} "
                      f"{result['seconds']}s nodes={result['nodes']}", flush=True)
        a, b = timings['baseline'], timings['optimized']
        valid = all(r['status'] == case['expected'] for r in a+b)
        work_equal = all((r['nodes'], r['raw_candidates'], r['unique_candidates'], r['solutions']) ==
                         (a[0]['nodes'], a[0]['raw_candidates'], a[0]['unique_candidates'], a[0]['solutions'])
                         for r in a+b)
        baseline_median = statistics.median(r['seconds'] for r in a if r['seconds'] is not None)
        optimized_median = statistics.median(r['seconds'] for r in b if r['seconds'] is not None)
        results.append({'name': case['name'], 'valid': valid, 'identical_work_counters': work_equal,
                        'baseline_median_seconds': baseline_median,
                        'optimized_median_seconds': optimized_median,
                        'speedup': baseline_median / optimized_median if valid and optimized_median else None,
                        'runs': timings})
    return {'kind': 'benchmark', 'threads': args.threads[0],
            'all_pass': all(r['valid'] for r in results), 'results': results}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('mode', choices=['regression', 'benchmark'])
    ap.add_argument('--exe', required=True)
    ap.add_argument('--baseline')
    ap.add_argument('--threads', nargs='+', type=int, default=[1])
    ap.add_argument('--time-limit', type=float, default=120)
    ap.add_argument('--repeat', type=int, default=3)
    ap.add_argument('--case', action='append')
    ap.add_argument('--quick', action='store_true')
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    if args.mode == 'benchmark' and not args.baseline:
        ap.error('--baseline is required for benchmark')
    result = regression(args) if args.mode == 'regression' else benchmark(args)
    result.update(platform=platform.platform(), cpu=platform.processor(),
                  executable=str(Path(args.exe).resolve()))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    return 0 if result['all_pass'] else 1

if __name__ == '__main__':
    sys.exit(main())
