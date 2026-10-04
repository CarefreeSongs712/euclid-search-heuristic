#!/usr/bin/env python3
"""Frozen v10/r2/r3 Euclidea-model comparison. Heuristic failure is not exhaustion."""
from pathlib import Path
import argparse
import hashlib
import json
import platform
import re
from heavy_benchmark import run

P=Path(__file__).resolve().parents[1]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--v10',required=True)
    ap.add_argument('--r2',required=True)
    ap.add_argument('--r3',required=True)
    ap.add_argument('--manifest',type=Path,default=P/'benchmarks/euclidea8/manifest.json')
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--case',action='append')
    ap.add_argument('--threads',type=int,default=8)
    ap.add_argument('--seconds',type=float,default=60)
    ap.add_argument('--seed',type=int,default=1)
    ap.add_argument('--resume',action='store_true')
    a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=True)
    output=a.out/'results.json'
    binaries=[('v10',a.v10),('v11-r2',a.r2),('v11-r3',a.r3)]
    identity={name:{'path':str(Path(exe).resolve()),'sha256':hashlib.sha256(Path(exe).read_bytes()).hexdigest()}
              for name,exe in binaries}
    if output.exists():
        if not a.resume:ap.error('Output exists; use --resume only for the identical matrix')
        report=json.loads(output.read_text(encoding='utf-8'))
        if report['binaries']!=identity or report['threads']!=a.threads or report['seconds']!=a.seconds or report['seed']!=a.seed:
            ap.error('Resume parameters or executable hashes changed')
    else:
        report={'kind':'euclidea8_models_frozen_comparison','platform':platform.platform(),
                'processor':platform.processor(),'threads':a.threads,'seconds':a.seconds,'seed':a.seed,
                'binaries':identity,'runs':[],
                'scope':'Analytic representative models, not screenshot pixel coordinates or exact game state. Fixed complete branch. E only, not L macro count.'}
    rows=json.loads(a.manifest.read_text(encoding='utf-8'))
    completed={(r['name'],r['label']) for r in report['runs']}
    for i,case in enumerate(rows):
        if a.case and case['name'] not in a.case:continue
        # Rotate version order to avoid always putting the same version first.
        order=binaries[i%3:]+binaries[:i%3]
        for name,exe in order:
            if (case['name'],name) in completed:continue
            cfg=dict(case);cfg['threads']=a.threads;cfg['time_limit']=a.seconds
            cfg['args']=['--readable-output']
            cert=a.out/f"{case['name']}.{name}.r1.certificate.json"
            if name!='v10':cfg['args'] += [f'--seed={a.seed}','--certificate='+str(cert.resolve())]
            r=run(exe,cfg,name,1,a.out)
            text=Path(r['stdout']).read_text(encoding='utf-8',errors='replace')
            r.update(input=case['input'],title=case['title'],budget_E=case['E'],
                     returned_E=[int(x) for x in re.findall(r'个不同解（(\d+)E）',text)],
                     certificate=str(cert) if cert.exists() else None)
            for field,key in [('Beam successful state visits','beam'),('Helper successful state visits','helper'),
                ('Rendezvous successful state visits','rendezvous'),('Chain join successful state visits','chain_join'),
                ('Point join successful state visits','point_join')]:
                m=re.search(r'^'+field+r':(\d+)',text,re.M)
                r[key]=int(m.group(1)) if m else None
            report['runs'].append(r)
            temp=output.with_suffix('.json.tmp')
            temp.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
            temp.replace(output)
    return int(any(r['external_timeout'] or r['status'] is None or r['returncode'] not in (0,2,3,4) for r in report['runs']))

if __name__=='__main__':raise SystemExit(main())
