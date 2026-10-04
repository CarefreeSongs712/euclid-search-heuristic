#!/usr/bin/env python3
"""Compare time-to-solution, never heuristic stopping time as exhaustive speed."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics
import subprocess
import time
import re

P=Path(__file__).resolve().parents[1]

def parse(text):
    row={}
    for name,pattern,cast in [('status',r'Result status:\s*(\S+)',str),
          ('seconds',r'Search Time:\s*([\d.eE+-]+)',float),
          ('solutions',r'^Distinct solutions:\s*(\d+)',int),
          ('nodes',r'^Nodes:\s*(\d+)',int)]:
        m=re.search(pattern,text,re.M)
        row[name]=cast(m.group(1)) if m else None
    row['returned_E']=[int(x) for x in re.findall(r'（(\d+)E）',text)]
    return row

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--v10',required=True)
    ap.add_argument('--v11',required=True)
    ap.add_argument('--manifest',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--repeat',type=int,default=3)
    ap.add_argument('--threads',type=int,default=1)
    ap.add_argument('--time-limit',type=float,default=10)
    ap.add_argument('--case',action='append')
    a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=True)
    cases=json.loads(a.manifest.read_text(encoding='utf-8'))
    rows=[]
    for case in cases:
        if a.case and case['name'] not in a.case: continue
        data=(P/case['input']).read_bytes()
        for rep in range(a.repeat):
            order=[('v10',a.v10),('v11',a.v11)]
            if rep%2:order.reverse()
            for label,exe in order:
                cmd=[str(Path(exe).resolve()),f'--threads={a.threads}','--solutions=1',
                     f'--time-limit={case.get("time_limit",a.time_limit)}','--no-pause','--raw-output',
                     '--eps='+case.get('eps','1e-11')]
                cmd+=case.get('args',[])
                if label=='v11':cmd+=[f'--seed={rep+1}',*case.get('heuristic_args',[])]
                start=time.perf_counter()
                r=subprocess.run(cmd,input=data,capture_output=True,timeout=case.get('time_limit',a.time_limit)+20)
                prefix=a.out/f'{case["name"]}.{label}.s{rep+1}'
                prefix.with_suffix(prefix.suffix+'.stdout.log').write_bytes(r.stdout)
                prefix.with_suffix(prefix.suffix+'.stderr.log').write_bytes(r.stderr)
                row=parse(r.stdout.decode('utf-8',errors='replace'))
                row.update(name=case['name'],version=label,seed=rep+1,threads=a.threads,
                           returncode=r.returncode,wall_seconds=time.perf_counter()-start,
                           input_sha256=hashlib.sha256(data).hexdigest(),command=cmd)
                rows.append(row)
                (a.out/'results.json').write_text(json.dumps({'runs':rows},indent=2)+'\n')
                print(case['name'],label,rep+1,row['status'],row['seconds'],row['returned_E'],flush=True)
    return int(any(r['status'] is None or r['returncode'] not in (0,2,3,4) for r in rows))

if __name__=='__main__': raise SystemExit(main())
