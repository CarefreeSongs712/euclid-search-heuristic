#!/usr/bin/env python3
"""Independently verify successful certificates from improved-r3 comparisons."""
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
rows=json.loads((x.directory/'results.json').read_text(encoding='utf-8'))['runs']
results=[]
for row in rows:
    if row['label']!='r3.1' or row['status']!='QUOTA_REACHED':continue
    cert=x.directory/f"{row['name']}.r3.1.r{row['repetition']}.certificate.json"
    p=subprocess.run([sys.executable,str(P/'tools/verify_certificate.py'),'--input',str(P/row['input']),
        '--input-layout','compact','--certificate',str(cert),'--precision','100','--require-original-eps'],capture_output=True)
    try:report=json.loads(p.stdout)
    except json.JSONDecodeError:report={'error':p.stderr.decode(errors='replace')}
    results.append({'name':row['name'],'repeat':row['repetition'],'pass':p.returncode==0,'verification':report})
    print('PASS' if p.returncode==0 else 'FAIL',row['name'],row['repetition'],report.get('status'),flush=True)
x.output.write_text(json.dumps({'all_pass':all(r['pass'] for r in results),'checks':results},indent=2)+'\n',encoding='utf-8')
raise SystemExit(int(not all(r['pass'] for r in results)))
