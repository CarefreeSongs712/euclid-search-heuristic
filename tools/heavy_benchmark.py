#!/usr/bin/env python3
"""Run long searches sequentially, retaining stdout/stderr and resource samples."""
from __future__ import annotations
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


def resources(pid, handle=None):
    if os.name == 'nt':
        from ctypes import wintypes
        class Counters(ctypes.Structure):
            _fields_ = [('cb', wintypes.DWORD), ('PageFaultCount', wintypes.DWORD),
                        ('PeakWorkingSetSize', ctypes.c_size_t), ('WorkingSetSize', ctypes.c_size_t),
                        ('QuotaPeakPagedPoolUsage', ctypes.c_size_t), ('QuotaPagedPoolUsage', ctypes.c_size_t),
                        ('QuotaPeakNonPagedPoolUsage', ctypes.c_size_t), ('QuotaNonPagedPoolUsage', ctypes.c_size_t),
                        ('PagefileUsage', ctypes.c_size_t), ('PeakPagefileUsage', ctypes.c_size_t)]
        c = Counters()
        c.cb = ctypes.sizeof(c)
        ok = ctypes.windll.psapi.GetProcessMemoryInfo(wintypes.HANDLE(handle), ctypes.byref(c), c.cb)
        if not ok: return {}
        return {'rss_bytes': c.WorkingSetSize, 'peak_rss_bytes': c.PeakWorkingSetSize}
    try:
        status = Path(f'/proc/{pid}/status').read_text()
        d = {}
        for key, name in [('VmRSS', 'rss_bytes'), ('VmHWM', 'peak_rss_bytes')]:
            m = re.search(r'^' + key + r':\s*(\d+) kB', status, re.M)
            if m: d[name] = int(m.group(1)) * 1024
        return d
    except (OSError, ValueError):
        return {}


def parse_output(text):
    result = {}
    for key, pat, cast in [('status', r'Result status:\s*(\S+)', str),
        ('seconds', r'Search Time:\s*([\d.eE+-]+)', float),
        ('nodes', r'^Nodes:\s*(\d+)', int),
        ('raw_candidates', r'^Raw candidates:\s*(\d+)', int),
        ('unique_candidates', r'^Unique candidates:\s*(\d+)', int),
        ('solutions', r'^Distinct solutions:\s*(\d+)', int)]:
        m = re.search(pat, text, re.M)
        result[key] = cast(m.group(1)) if m else None
    return result


def run(exe, case, label, repetition, directory, limit_override=None):
    source = ROOT / case['input']
    data = source.read_bytes()
    limit = limit_override or case.get('time_limit', 180)
    threads = case.get('threads', 1)
    command = [str(Path(exe).resolve()), f'--threads={threads}',
               f'--solutions={case.get("solutions", 1)}', f'--eps={case.get("eps", "1e-11")}',
               f'--time-limit={limit}', '--no-pause', '--raw-output', *case.get('args', [])]
    stem = f'{case["name"]}.{label}.r{repetition}'
    out = directory / (stem + '.stdout.log')
    err = directory / (stem + '.stderr.log')
    start = time.perf_counter()
    sampled = []
    external_timeout = False
    with source.open('rb') as stdin, out.open('wb') as stdout, err.open('wb') as stderr:
        proc = subprocess.Popen(command, stdin=stdin, stdout=stdout, stderr=stderr)
        while proc.poll() is None:
            elapsed = time.perf_counter() - start
            r = resources(proc.pid, getattr(proc, '_handle', None))
            if r: sampled.append({'elapsed': elapsed, **r})
            if elapsed > limit + 20:
                external_timeout = True
                proc.terminate()
                try: proc.wait(timeout=5)
                except subprocess.TimeoutExpired: proc.kill(); proc.wait()
                break
            try: proc.wait(timeout=0.5)
            except subprocess.TimeoutExpired: pass
    wall = time.perf_counter() - start
    result = parse_output(out.read_text(encoding='utf-8', errors='replace'))
    result.update(name=case['name'], label=label, repetition=repetition,
        command=command, returncode=proc.returncode, wall_seconds=wall,
        external_timeout=external_timeout, time_limit=limit, threads=threads,
        input_sha256=hashlib.sha256(data).hexdigest(),
        peak_rss_bytes=max((s.get('peak_rss_bytes', 0) for s in sampled), default=None),
        stdout=str(out), stderr=str(err), resources=sampled)
    seconds = result['seconds']
    result['nodes_per_second'] = result['nodes'] / seconds if result['nodes'] and seconds else None
    estimates = []
    progress = err.read_text(encoding='utf-8', errors='replace')
    for line in progress.splitlines():
        e = re.search(r'elapsed=([\d.]+)s', line)
        eta = re.search(r'exhaustion ETA~([\d.]+)s', line)
        if e and eta:
            elapsed, remaining = float(e.group(1)), float(eta.group(1))
            estimates.append({'elapsed': elapsed, 'eta_seconds': remaining,
                              'predicted_total_seconds': elapsed + remaining})
    result['progress_lines'] = progress.count('[progress]')
    result['eta_samples'] = estimates
    if seconds and result['status'] == 'EXHAUSTED':
        for e in estimates:
            e['predicted_total_over_actual'] = e['predicted_total_seconds'] / seconds
    print(f"{case['name']} {label} r{repetition} t={threads} {result['status']} "
          f"{seconds}s nodes={result['nodes']} peakMiB={(result['peak_rss_bytes'] or 0)/1048576:.1f}", flush=True)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--exe', required=True)
    ap.add_argument('--baseline')
    ap.add_argument('--manifest', type=Path, required=True)
    ap.add_argument('--output-dir', type=Path, required=True)
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--case', action='append')
    ap.add_argument('--limit', type=float)
    a = ap.parse_args()
    cases = json.loads(a.manifest.read_text(encoding='utf-8'))
    a.output_dir.mkdir(parents=True, exist_ok=True)
    report = {'platform': platform.platform(), 'cpu': platform.processor(), 'runs': [],
              'executables': {name: {'path': str(Path(exe).resolve()),
                  'sha256': hashlib.sha256(Path(exe).read_bytes()).hexdigest()}
                  for name, exe in [('optimized', a.exe), ('baseline', a.baseline)] if exe}}
    output = a.output_dir / 'results.json'
    for case in cases:
        if a.case and case['name'] not in a.case: continue
        for rep in range(1, a.repeat+1):
            order = [('baseline', a.baseline), ('optimized', a.exe)] if a.baseline else [('optimized', a.exe)]
            if rep % 2 == 0: order.reverse()
            for name, exe in order:
                report['runs'].append(run(exe, case, name, rep, a.output_dir, a.limit))
                output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    return 1 if any(r['external_timeout'] or r['status'] is None for r in report['runs']) else 0

if __name__ == '__main__':
    sys.exit(main())
