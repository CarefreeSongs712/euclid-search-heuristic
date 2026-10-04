#!/usr/bin/env python3
"""Validate v11 on known-solvable E3--6 constructions and honest stop statuses.

Examples:
  python tools/heuristic_validate.py --certificates-only --checker PATH/heuristic_tests
  python tools/heuristic_validate.py --exe PATH/bs_v11 --checker PATH/heuristic_tests
  python tools/heuristic_validate.py --exe PATH/bs_v11 --algorithms heuristic exhaustive \
      --threads 1 4 --seeds 1 17 --repeat 3 --output PATH/results.json
  python tools/heuristic_validate.py --exe PATH/bs_v11 --baseline PATH/v10 --threads 1

Only successful quota=1 runs have time_to_solution_seconds. Search Time includes
engine setup/quota shutdown and is an upper-bound proxy, not an instrumented
first-hit timestamp. Failed/timed-out runs remain censored; their timing is NEVER
reported as a speedup. Node counts are diagnostic only: different algorithms do
not perform equal work. The checker reads original double-precision collector
Graphs in a separate API run, NOT the CLI's lossy 12-digit raw report.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import statistics
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'benchmarks/heuristic_manifest.json'
EXIT_CODES = {'QUOTA_REACHED': 0, 'EXHAUSTED': 2, 'TIMEOUT_PARTIAL': 3, 'HEURISTIC_STOPPED': 4}


def invoke(command, data=b'', timeout=30):
    start = time.perf_counter()
    try:
        proc = subprocess.run(list(map(str, command)), input=data, stdout=subprocess.PIPE,
                              stderr=subprocess.PIPE, timeout=timeout)
        return {'returncode': proc.returncode, 'wall_seconds': time.perf_counter()-start,
                'stdout': proc.stdout.decode('utf-8', errors='replace'),
                'stderr': proc.stderr.decode('utf-8', errors='replace'), 'process_timeout': False}
    except subprocess.TimeoutExpired as exc:
        return {'returncode': None, 'wall_seconds': time.perf_counter()-start,
                'stdout': (exc.stdout or b'').decode('utf-8', errors='replace'),
                'stderr': (exc.stderr or b'').decode('utf-8', errors='replace'), 'process_timeout': True}


def parse_cli(row):
    text = row['stdout']
    for key, pattern, convert in (
        ('status', r'Result status:\s*(\S+)', str),
        ('search_seconds', r'^Search Time:\s*([\d.eE+-]+)', float),
        ('solutions', r'^Distinct solutions:\s*(\d+)', int),
        ('nodes', r'^Nodes:\s*(\d+)', int),
        ('raw_candidates', r'^Raw candidates:\s*(\d+)', int),
    ):
        match = re.search(pattern, text, re.MULTILINE)
        row[key] = convert(match.group(1)) if match else None
    row['returned_e'] = [int(e) for e in re.findall(r'个不同解[（(](\d+)E[）)]', text)]
    row['success'] = row['status'] == 'QUOTA_REACHED' and row['returncode'] == 0
    return row


def settings(defaults, case, args):
    out = dict(defaults)
    for key in ('eps', 'time_budget_seconds', 'beam_width', 'branch_limit', 'restarts',
                'tail_seconds', 'tail_candidates', 'quota'):
        if key in case:
            out[key] = case[key]
    # A global benchmark budget must not erase dedicated tiny-timeout controls.
    if args.time_limit is not None and case['category'] == 'solvable':
        out['time_budget_seconds'] = args.time_limit
    out.setdefault('quota', 1)
    return out


def cli_args(config, threads, seed, algorithm, legacy=False):
    flags = [f'--threads={threads}', f'--eps={config["eps"]}',
             f'--time-limit={config["time_budget_seconds"]}', f'--solutions={config["quota"]}',
             '--no-pause', '--no-progress', '--raw-output']
    if not legacy:
        flags.append(f'--search={algorithm}')
        if algorithm == 'heuristic':
            flags += [f'--beam-width={config["beam_width"]}', f'--branch-limit={config["branch_limit"]}',
                      f'--restarts={config["restarts"]}', f'--seed={seed}',
                      f'--tail-seconds={config["tail_seconds"]}', f'--tail-candidates={config["tail_candidates"]}']
    return flags


def load_manifest(path):
    doc = json.loads(path.read_text(encoding='utf-8'))
    if doc.get('schema_version') != 1 or not doc.get('cases'):
        raise ValueError('invalid/empty heuristic manifest')
    names = set()
    for case in doc['cases']:
        if case['name'] in names:
            raise ValueError('duplicate case name: '+case['name'])
        names.add(case['name'])
        data = (ROOT / case['input']).read_text(encoding='utf-8')
        tokens = data.split()
        case['limit'], case['mode'] = map(int, tokens[:2])
        if case['category'] == 'solvable':
            if not 0 <= case['known_solution_e'] <= case['limit']:
                raise ValueError('invalid known certificate length: '+case['name'])
        if 'certificate' in case:
            if len(case['certificate']) != case['known_solution_e']:
                raise ValueError('certificate E mismatch: '+case['name'])
            for step in case['certificate']:
                if len(step) != 5 or step[0] not in ('line', 'circle') or not all(
                        math.isfinite(float(x)) for x in step[1:]):
                    raise ValueError('invalid certificate step: '+case['name'])
    return doc


def certificate_checks(cases, checker):
    results = []
    for case in cases:
        if 'certificate' not in case:
            continue
        data = ''.join(' '.join(map(str, step))+'\n' for step in case['certificate']).encode('ascii')
        row = invoke([checker, '--certificate', ROOT / case['input']], data, 15)
        row.update(case=case['name'], expected_e=case['known_solution_e'])
        try:
            certificate = json.loads(row['stdout'])
            row['pass'] = row['returncode'] == 0 and certificate['certificate_valid'] and (
                certificate['returned_e'] == case['known_solution_e'])
        except (ValueError, KeyError):
            row['pass'] = False
        results.append(row)
        print(f'{"PASS" if row["pass"] else "FAIL"} certificate {case["name"]}', flush=True)
    return results


def checker_run(checker, case, config, threads, seed):
    command = [checker, '--case', ROOT / case['input'], '--threads', threads, '--seed', seed,
               '--seconds', config['time_budget_seconds'], '--quota', config['quota'],
               '--eps', config['eps']]
    for key in ('beam_width', 'branch_limit', 'restarts', 'tail_seconds', 'tail_candidates'):
        command += ['--'+key.replace('_', '-'), config[key]]
    row = invoke(command, timeout=max(5, config['time_budget_seconds']+4))
    try:
        row['result'] = json.loads(row['stdout'])
        row['pass'] = row['returncode'] == 0 and row['result']['valid']
    except (ValueError, KeyError):
        row['pass'] = False
    row['scope'] = 'separate API run: every SolutionCollector Graph replayed at original precision'
    return row


def validate_result(row, case, config, algorithm):
    errors = []
    status = row['status']
    if status not in EXIT_CODES or row['returncode'] != EXIT_CODES.get(status):
        errors.append('missing/incorrect status or exit code')
    if algorithm == 'heuristic' and status == 'EXHAUSTED':
        errors.append('heuristic falsely claimed EXHAUSTED')
    if row['process_timeout']:
        errors.append('process exceeded budget plus shutdown grace')
    count = row['solutions']
    if count is None or not 0 <= count <= config['quota']:
        errors.append('invalid reported solution count')
    elif len(row['returned_e']) != count:
        errors.append('could not account for every returned construction E')
    if any(e > case['limit'] for e in row['returned_e']):
        errors.append('returned E exceeds input limit')
    if status == 'QUOTA_REACHED' and count != config['quota']:
        errors.append('quota status disagrees with count')
    if status != 'QUOTA_REACHED' and count is not None and count >= config['quota']:
        errors.append('stopped despite reached quota')
    if (row['search_seconds'] is None or not math.isfinite(row['search_seconds'])
            or row['search_seconds'] < 0):
        errors.append('invalid/missing search time')
    elif row['search_seconds'] > config['time_budget_seconds']+3:
        errors.append('search exceeded budget plus grace')
    if case['category'] == 'status':
        expected = case['expected_heuristic' if algorithm == 'heuristic' else 'expected_exhaustive']
        if isinstance(expected, str):
            expected = [expected]
        if status not in expected:
            errors.append('wrong status-control result; expected '+str(expected))
        if count is not None and count < case.get('minimum_solutions', 0):
            errors.append('missing expected partial solution')
    elif case.get('required') and not row['success']:
        errors.append('known-solvable required case was not solved within budget')
    row['time_to_solution_seconds'] = row['search_seconds'] if row['success'] and config['quota'] == 1 else None
    row['time_to_solution_kind'] = 'quota1 search-duration upper-bound proxy' if row['time_to_solution_seconds'] is not None else 'censored/not quota1'
    row['errors'], row['pass'] = errors, not errors


def contracts(exe, defaults):
    cases = []
    config = dict(defaults, quota=1, restarts=1, tail_seconds=0, tail_candidates=0)
    data = (ROOT / 'tests/cases/heuristic_E0_missing.in').read_bytes()
    flags = cli_args(config, 1, 1, 'heuristic')
    for name, passed, payload in (
        ('default_search_is_heuristic', [f for f in flags if not f.startswith('--search=')], data),
        ('old_stdin_quota', [f for f in flags if not f.startswith('--solutions=')], data+b'1\n'),
    ):
        row = parse_cli(invoke([exe, *passed], payload, 10))
        row.update(name=name, **{'pass': row['returncode'] == 4 and row['status'] == 'HEURISTIC_STOPPED'})
        cases.append(row)
    for flag in ('--search=unknown', '--beam-width=0', '--branch-limit=0', '--seed=-1',
                 '--restarts=-1', '--tail-seconds=-1', '--solutions=0'):
        row = invoke([exe, '--no-pause', flag], timeout=5)
        row.update(name='reject '+flag, **{'pass': row['returncode'] == 1})
        cases.append(row)
    # Finite restarts, tail disabled, ample deadline: no wall-clock nondeterminism.
    config.update(restarts=2, time_budget_seconds=15)
    data = (ROOT / 'tests/cases/heuristic_midpoint_E4.in').read_bytes()
    signatures = []
    raw = []
    for _ in range(2):
        row = parse_cli(invoke([exe, *cli_args(config, 1, 17, 'heuristic')], data, 19))
        raw.append(row)
        match = re.search(r'Points \(.*?(?=\n已返回|\nSEARCH STOPPED|\nHeuristic|\nResult status:)', row['stdout'], re.DOTALL)
        signatures.append(match.group(0) if match else None)
    cases.append({'name': 'single_thread_seed_deterministic',
                  'pass': all(r['success'] for r in raw) and signatures[0] is not None and signatures[0] == signatures[1],
                  'runs': raw, 'note': 'C++ unit test additionally compares original bits and deterministic counters.'})
    for row in cases:
        print(f'{"PASS" if row["pass"] else "FAIL"} CLI {row["name"]}', flush=True)
    return cases


def summarize(rows):
    summaries = []
    keys = sorted({(r['algorithm'], r['threads']) for r in rows if r['category'] == 'solvable'})
    for algorithm, threads in keys:
        group = [r for r in rows if r['category'] == 'solvable' and (r['algorithm'], r['threads']) == (algorithm, threads)]
        successes = [r for r in group if r['success']]
        summaries.append({'algorithm': algorithm, 'threads': threads, 'attempts': len(group),
                          'solved': len(successes), 'success_rate': len(successes)/len(group),
                          'median_success_only_seconds': statistics.median(r['time_to_solution_seconds'] for r in successes) if successes else None,
                          'total_time_budget_seconds': sum(r['time_budget_seconds'] for r in group),
                          'note': 'Success-only median has survivorship bias; compare success rate and per-case censored runs, not this number alone.'})
    return summaries


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--exe', type=Path)
    parser.add_argument('--checker', type=Path, help='standalone tests/heuristic_tests.cpp executable')
    parser.add_argument('--baseline', type=Path, help='preserved v10/v10.1 executable (no --search flag)')
    parser.add_argument('--manifest', type=Path, default=MANIFEST)
    parser.add_argument('--algorithms', nargs='+', choices=['heuristic', 'exhaustive'], default=['heuristic'])
    parser.add_argument('--threads', nargs='+', type=int, default=[1, 4])
    parser.add_argument('--seeds', nargs='+', type=int, default=[1])
    parser.add_argument('--repeat', type=int, default=1)
    parser.add_argument('--time-limit', type=float, help='override solvable-case budgets only')
    parser.add_argument('--case', action='append', help='exact manifest case name; repeatable')
    parser.add_argument('--certificates-only', action='store_true')
    parser.add_argument('--skip-contracts', action='store_true')
    parser.add_argument('--output', type=Path, help='explicit JSON result path; no output file written by default')
    args = parser.parse_args()
    if args.repeat < 1 or any(t < 1 for t in args.threads) or any(s < 0 for s in args.seeds):
        parser.error('repeat/threads must be positive and seeds nonnegative')
    if args.time_limit is not None and (not math.isfinite(args.time_limit) or args.time_limit <= 0):
        parser.error('--time-limit must be finite and positive')
    if args.certificates_only and not args.checker:
        parser.error('--certificates-only requires --checker; manifest labels alone are not proof')
    if not args.certificates_only and not args.exe:
        parser.error('--exe is required unless --certificates-only')
    doc = load_manifest(args.manifest)
    selected = [c for c in doc['cases'] if not args.case or c['name'] in args.case]
    if not selected or (args.case and set(args.case)-{c['name'] for c in selected}):
        parser.error('empty selection or unknown --case')
    certificates = certificate_checks(selected, args.checker.resolve()) if args.checker else []
    rows = []
    checks = []
    if not args.certificates_only:
        exe = args.exe.resolve()
        if not args.skip_contracts:
            checks = contracts(exe, doc['defaults'])
        engines = [(a, exe, False) for a in args.algorithms]
        if args.baseline:
            engines.append(('baseline', args.baseline.resolve(), True))
        for case in selected:
            data = (ROOT / case['input']).read_bytes()
            config = settings(doc['defaults'], case, args)
            for threads in args.threads:
                for seed in args.seeds:
                    for repeat in range(args.repeat):
                        # Alternate algorithms to reduce always-first warmup/order bias.
                        order = engines if repeat % 2 == 0 else list(reversed(engines))
                        for algorithm, executable, legacy in order:
                            flags = cli_args(config, threads, seed, algorithm, legacy)
                            row = parse_cli(invoke([executable, *flags], data, max(5, config['time_budget_seconds']+4)))
                            row.update(case=case['name'], category=case['category'], algorithm=algorithm,
                                       threads=threads, seed=seed, repeat=repeat+1, command=[str(executable), *flags],
                                       input=str((ROOT / case['input']).resolve()), input_sha256=hashlib.sha256(data).hexdigest(),
                                       time_budget_seconds=config['time_budget_seconds'], options=config,
                                       known_solution_e=case.get('known_solution_e'))
                            validate_result(row, case, config, algorithm)
                            # A separate API run audits precise collector contents. Do not claim
                            # it certifies the raw CLI text or requires identical timeout paths.
                            if args.checker and algorithm == 'heuristic':
                                audit = checker_run(args.checker.resolve(), case, config, threads, seed)
                                row['collector_audit'] = audit
                                if audit['pass']:
                                    row['collector_audit_success'] = audit['result']['quota_reached']
                                    if case['category'] == 'solvable' and case.get('required') and not audit['result']['quota_reached']:
                                        row['errors'].append('separate full-precision API audit missed required known solution')
                                        row['pass'] = False
                                else:
                                    row['errors'].append('full-precision collector replay failed')
                                    row['pass'] = False
                            rows.append(row)
                            print(f'{"PASS" if row["pass"] else "FAIL"} {algorithm} t={threads} seed={seed} '
                                  f'{case["name"]}: {row["status"]} E={row["returned_e"]} '
                                  f'tts={row["time_to_solution_seconds"]} budget={config["time_budget_seconds"]}', flush=True)
    all_results = certificates+checks+rows
    report = {'kind': 'heuristic_known_solvable_validation', 'all_pass': bool(all_results) and all(r['pass'] for r in all_results),
              'platform': platform.platform(), 'processor': platform.processor(),
              'manifest': str(args.manifest.resolve()), 'certificates': certificates, 'cli_contracts': checks,
              'runs': rows, 'summary': summarize(rows),
              'limitations': ['No node-count equal-work or exhaustion-only speedup claims.',
                              'CLI raw output is not used for geometric certificate replay.',
                              'Failed runs have censored time-to-solution; search duration is a quota1 upper-bound proxy.',
                              'Seed reproducibility is tested only with finite restarts, no tail cutoff, and nonbinding wall budget.']}
    if not args.checker:
        report['limitations'].append('No --checker supplied: original-precision geometry validation was NOT run.')
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps({'all_pass': report['all_pass'], 'summary': report['summary']}, ensure_ascii=False, indent=2))
    return 0 if report['all_pass'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        print('Validation failed: '+str(exc), file=sys.stderr)
        sys.exit(1)
