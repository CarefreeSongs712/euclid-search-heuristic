#!/usr/bin/env python3
"""Check v11's command-line contract and distinguish heuristic termination."""
from pathlib import Path
import argparse
import subprocess
import time

P=Path(__file__).resolve().parents[1]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--exe',required=True)
    a=ap.parse_args()
    exe=str(Path(a.exe).resolve())
    def call(args,data=b'',timeout=5):
        return subprocess.run([exe,'--no-pause',*args],input=data,capture_output=True,timeout=timeout)
    count=0
    for option in ['--help','--version']:
        r=call([option]); assert r.returncode==0 and b'v11' in r.stdout
        count+=1
    for option in ['--search=oops','--beam-width=0','--beam-width=4junk','--beam-width=999999999',
                   '--branch-limit=-1','--restarts=-1','--seed=1oops','--tail-seconds=nan',
                   '--tail-candidates=-1','--eps=nan','--time-limit=inf','--solutions=0']:
        r=call([option]); assert r.returncode==1,(option,r.returncode,r.stderr)
        count+=1
    unsolved=b'0\n2\n2 0 0 0 0\n0 0\n1 0\n0 0 1\n3.14 2.71\n'
    for search,code,status in [('heuristic',4,b'HEURISTIC_STOPPED'),('exhaustive',2,b'EXHAUSTED')]:
        start=time.perf_counter()
        r=call([f'--search={search}','--threads=1','--solutions=1','--restarts=1',
                '--progress-interval=3600'],unsolved)
        assert r.returncode==code and status in r.stdout,(code,r.returncode,r.stdout,r.stderr)
        assert time.perf_counter()-start<2
        count+=1
    solved=b'0\n2\n2 0 0 0 0\n0 0\n1 0\n0 0 1\n0 0\n'
    r=call(['--threads=1','--solutions=1'],solved)
    assert r.returncode==0 and b'QUOTA_REACHED' in r.stdout
    count+=1
    # Synthetic rational-grid search for cancellation; no contest T6 workload.
    data=b'6\n3\n8 8\n0 0 0 0 0\n0 0 1\n0.12345678901234567 0.2718281828459\n'
    for threads in [1,2]:
        r=call([f'--threads={threads}','--solutions=1','--time-limit=0.25',
                '--progress-interval=0.02','--seed=37'],data)
        assert r.returncode==3 and b'TIMEOUT_PARTIAL' in r.stdout,(r.returncode,r.stderr)
        assert b'exhaustion ETA=unavailable' in r.stderr and b'root=' not in r.stderr,r.stderr
        assert b'EXHAUSTED' not in r.stdout
        count+=1
    print(f'PASS: {count} v11 CLI/status/progress checks')

if __name__=='__main__':main()
