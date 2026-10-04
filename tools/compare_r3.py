#!/usr/bin/env python3
"""Seeded r2/r3 comparison with long-budget evidence, certificates and origins."""
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
    ap.add_argument('--r2',required=True)
    ap.add_argument('--r3',required=True)
    ap.add_argument('--v10')
    ap.add_argument('--manifest',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--case',action='append')
    ap.add_argument('--repeat',type=int,default=2)
    ap.add_argument('--seeds',nargs='+',type=int,default=[1,2])
    a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=True)
    if (a.out/'results.json').exists():ap.error('Output exists; choose a new directory')
    cases=json.loads(a.manifest.read_text())
    binaries=[('v11-r2',a.r2),('v11-r3',a.r3)]
    if a.v10:binaries.insert(0,('v10',a.v10))
    report={'platform':platform.platform(),'kind':'r3_long_comparison','runs':[],
            'binaries':{label:{'path':str(Path(exe).resolve()),'sha256':hashlib.sha256(Path(exe).read_bytes()).hexdigest()}
                        for label,exe in binaries}}
    for case in cases:
        if a.case and case['name'] not in a.case:continue
        for repeat in range(a.repeat):
            seed=a.seeds[repeat%len(a.seeds)]
            order=binaries if repeat%2==0 else list(reversed(binaries))
            for label,exe in order:
                cfg=dict(case);cfg['args']=list(case.get('args',[]))
                cert=a.out/f'{case["name"]}.{label}.r{repeat+1}.certificate.json'
                if label!='v10':cfg['args'] += [f'--seed={seed}','--certificate='+str(cert.resolve())]
                if label=='v11-r3':cfg['args']+=case.get('r3_args',[])
                row=run(exe,cfg,label,repeat+1,a.out)
                text=Path(row['stdout']).read_text(encoding='utf-8',errors='replace')
                row.update(input=case['input'],seed=seed if label!='v10' else None,
                           certificate=str(cert) if cert.exists() else None,
                           returned_E=[int(x) for x in re.findall(r'（(\d+)E）',text)])
                for field,key in [('Beam successful state visits','beam'),('Helper successful state visits','helper'),
                    ('Rendezvous successful state visits','rendezvous'),('Chain join successful state visits','chain_join'),
                    ('Point join successful state visits','point_join')]:
                    m=re.search(r'^'+field+r':(\d+)',text,re.M)
                    row[key]=int(m.group(1)) if m else None
                report['runs'].append(row)
                (a.out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    return int(any(r['status'] is None or r['external_timeout'] for r in report['runs']))

if __name__=='__main__':raise SystemExit(main())
