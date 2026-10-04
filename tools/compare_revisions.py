#!/usr/bin/env python3
"""Long, seeded revision comparison; timeout is censored, never a speedup."""
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
    ap.add_argument('--old',required=True)
    ap.add_argument('--new',required=True)
    ap.add_argument('--v10')
    ap.add_argument('--manifest',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--case',action='append')
    ap.add_argument('--repeat',type=int,default=2)
    ap.add_argument('--seeds',nargs='+',type=int,default=[1,2])
    a=ap.parse_args()
    a.out.mkdir(parents=True,exist_ok=True)
    if (a.out/'results.json').exists():ap.error('Output exists; choose new directory')
    cases=json.loads(a.manifest.read_text())
    binaries=[('v11-r1',a.old),('v11-r2',a.new)]
    if a.v10:binaries.insert(0,('v10',a.v10))
    report={'platform':platform.platform(),'kind':'long_revision_comparison','runs':[],
            'binaries':{name:{'path':str(Path(exe).resolve()),'sha256':hashlib.sha256(Path(exe).read_bytes()).hexdigest()}
                        for name,exe in binaries}}
    for case in cases:
        if a.case and case['name'] not in a.case:continue
        for repeat in range(a.repeat):
            seed=a.seeds[repeat%len(a.seeds)]
            order=binaries if repeat%2==0 else list(reversed(binaries))
            for name,exe in order:
                cfg=dict(case);cfg['args']=list(case.get('args',[]))
                certificate=a.out/f'{case["name"]}.{name}.r{repeat+1}.certificate.json'
                if name!='v10':cfg['args'] += [f'--seed={seed}','--certificate='+str(certificate.resolve())]
                row=run(exe,cfg,name,repeat+1,a.out)
                text=Path(row['stdout']).read_text(encoding='utf-8',errors='replace')
                row['returned_E']=[int(x) for x in re.findall(r'（(\d+)E）',text)]
                row['seed']=seed if name!='v10' else None
                row['input']=case['input']
                row['certificate']=str(certificate) if certificate.exists() else None
                for field,key in [('Beam successful state visits','beam'),('Helper successful state visits','helper'),
                                  ('Rendezvous successful state visits','rendezvous'),('Family-diversity discards','family_discards')]:
                    found=re.search(r'^'+field+r':(\d+)',text,re.M)
                    row[key]=int(found.group(1)) if found else None
                report['runs'].append(row)
                (a.out/'results.json').write_text(json.dumps(report,indent=2)+'\n')
    return int(any(r['status'] is None or r['external_timeout'] for r in report['runs']))

if __name__=='__main__':raise SystemExit(main())
