#!/usr/bin/env python3
"""Recheck certificates produced by revision benchmarks at 100 digits."""
from pathlib import Path
import argparse
import json
import subprocess
import sys

P=Path(__file__).resolve().parents[1]
a=argparse.ArgumentParser(description=__doc__)
a.add_argument('--directory',type=Path,required=True)
a.add_argument('--output',type=Path,required=True)
args=a.parse_args()
rows=json.loads((args.directory/'results.json').read_text())['runs']
checks=[]
for row in rows:
    if row['label']!='v11-r2' or row['status']!='QUOTA_REACHED':continue
    name=f"{row['name']}.{row['label']}.r{row['repetition']}.certificate.json"
    cert=args.directory/name
    r=subprocess.run([sys.executable,str(P/'tools/verify_certificate.py'),'--input',str(P/row['input']),
        '--input-layout','compact','--certificate',str(cert),'--precision','100','--require-original-eps'],capture_output=True)
    try: report=json.loads(r.stdout)
    except json.JSONDecodeError:report={'error':r.stderr.decode(errors='replace')}
    checks.append({'case':row['name'],'repetition':row['repetition'],'pass':r.returncode==0,'verification':report})
    print('PASS' if r.returncode==0 else 'FAIL',row['name'],row['repetition'],report.get('status'),flush=True)
args.output.write_text(json.dumps({'all_pass':all(x['pass'] for x in checks),'checks':checks},indent=2)+'\n')
raise SystemExit(0 if all(x['pass'] for x in checks) else 1)
