#!/usr/bin/env python3
"""CLI validation, progress output and bounded cancellation smoke tests."""
import argparse
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--exe', required=True)
    args = parser.parse_args()
    exe = str(Path(args.exe).resolve())
    count = 0
    for option in ['--help', '--version']:
        p = subprocess.run([exe, option], input=b'', capture_output=True, timeout=5)
        assert p.returncode == 0, (option, p.returncode, p.stderr)
        assert p.stdout, option
        count += 1
    for option in ['--eps=nan', '--eps=inf', '--eps=1e-11junk', '--eps=0',
                   '--time-limit=-1', '--time-limit=1garbage', '--tt-mb=xyz',
                   '--threads=4294967296', '--threads=-1', '--solutions=0',
                   '--progress-interval=0', '--progress-interval=nan', '--unknown']:
        p = subprocess.run([exe, '--no-pause', option], input=b'', capture_output=True, timeout=5)
        assert p.returncode == 1, (option, p.returncode, p.stderr)
        count += 1
    for bad_input in [b'65536\n', b'-1\n']:
        p = subprocess.run([exe, '--no-pause', '--threads=1', '--solutions=1'],
                           input=bad_input, capture_output=True, timeout=5)
        assert p.returncode == 1, (bad_input, p.returncode)
        count += 1
    data = (ROOT / 'benchmarks/cases/opt4_main_t7_lines_prefix7_D_tail3.in').read_bytes()
    for threads in [1, 2, 4]:
        for progress in [True, False]:
            flags = [f'--threads={threads}', '--solutions=1', '--no-pause', '--raw-output',
                     '--time-limit=0.3', '--progress-interval=0.05' if progress else '--no-progress']
            start = time.perf_counter()
            p = subprocess.run([exe, *flags], input=data, capture_output=True, timeout=10)
            elapsed = time.perf_counter() - start
            assert p.returncode == 3 and b'TIMEOUT_PARTIAL' in p.stdout, (threads, p.returncode)
            assert elapsed < 5, (threads, elapsed)
            if progress:
                assert b'ETA' in p.stderr and b'nodes' in p.stderr, p.stderr
            else:
                assert not p.stderr, p.stderr
            count += 1
    print(f'PASS: {count} CLI/progress/cancellation checks')


if __name__ == '__main__':
    main()
