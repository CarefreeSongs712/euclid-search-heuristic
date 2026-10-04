#!/usr/bin/env python3
"""Verify every successful r3 result independently, at the original EPS."""
from pathlib import Path
import argparse
import json
import subprocess
import sys

P=Path(__file__).resolve().parents[1]
a=argparse.ArgumentParser(description=__doc__)
a.add_argument('--directory',type=Path,required=True)
a.add_argument('--output',type=Path,required=True)
x=a.parse_args()
rows=json.loads((x.directory/'results.json').read_text())['runs']
checks=[]
for r in rows:
    if r['label']!='v11-r3' or r['status']!='QUOTA_REACHED':continue
    cert=x.directory/f"{r['name']}.v11-r3.r{r['repetition']}.certificate.json"
    p=subprocess.run([sys.executable,str(P/'tools/verify_certificate.py'),'--input',str(P/r['input']),
        '--input-layout','compact','--certificate',str(cert),'--precision','100','--require-original-eps'],capture_output=True)
    try:result=json.loads(p.stdout)
    except json.JSONDecodeError:result={'error':p.stderr.decode(errors='replace')}
    checks.append({'name':r['name'],'repeat':r['repetition'],'pass':p.returncode==0,'result':result})
    print('PASS' if p.returncode==0 else 'FAIL',r['name'],r['repetition'],result.get('status'),flush=True)
x.output.write_text(json.dumps({'all_pass':all(c['pass'] for c in checks),'checks':checks},indent=2)+'\n')
raise SystemExit(0 if all(c['pass'] for c in checks) else 1)
