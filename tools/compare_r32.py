#!/usr/bin/env python3
"""Compare frozen r3.1 with r3.2: same models, budgets and worker count."""
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
    ap.add_argument('--resume',action='store_true')
    a=ap.parse_args();a.out.mkdir(parents=True,exist_ok=True)
    binaries=[('r3.1',a.old),('r3.2',a.new)]
    if a.v10:binaries.insert(0,('v10',a.v10))
    hashes={label:{'path':str(Path(exe).resolve()),'sha256':hashlib.sha256(Path(exe).read_bytes()).hexdigest()} for label,exe in binaries}
    output=a.out/'results.json'
    if output.exists():
        if not a.resume:ap.error('existing results; use --resume with identical binaries')
        report=json.loads(output.read_text(encoding='utf-8'))
        if report['binaries']!=hashes:ap.error('binary identity changed')
    else:report={'kind':'r3.2_comparison','platform':platform.platform(),'binaries':hashes,'runs':[]}
    done={(r['name'],r['label'],r['repetition']) for r in report['runs']}
    for case in json.loads(a.manifest.read_text(encoding='utf-8')):
        if a.case and case['name'] not in a.case:continue
        for rep in range(a.repeat):
            seed=a.seeds[rep%len(a.seeds)]
            for label,exe in binaries if rep%2==0 else list(reversed(binaries)):
                if (case['name'],label,rep+1) in done:continue
                cfg=dict(case);cfg['args']=['--readable-output',*case.get('args',[])]
                cert=a.out/f"{case['name']}.{label}.r{rep+1}.certificate.json"
                if label!='v10':cfg['args'] += [f'--seed={seed}','--certificate='+str(cert.resolve())]
                if label=='r3.2':cfg['args']+=case.get('new_args',[])
                r=run(exe,cfg,label,rep+1,a.out)
                text=Path(r['stdout']).read_text(encoding='utf-8',errors='replace')
                r.update(input=case['input'],seed=seed if label!='v10' else None,
                    certificate=str(cert) if cert.exists() else None,
                    returned_E=[int(x) for x in re.findall(r'个不同解（(\d+)E）',text)])
                for field,key in [('Beam successful state visits','beam'),('Helper successful state visits','helper'),
                    ('Coverage successful state visits','coverage'),('Equal-radius probe successful state visits','equal_radius'),
                    ('Prerequisite successful state visits','prerequisite'),('Rendezvous successful state visits','rendezvous'),
                    ('Chain join successful state visits','chain_join'),('Point join successful state visits','point_join')]:
                    m=re.search(r'^'+field+r':(\d+)',text,re.M);r[key]=int(m.group(1)) if m else None
                report['runs'].append(r)
                temp=output.with_suffix('.tmp');temp.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n',encoding='utf-8');temp.replace(output)
    return int(any(r['external_timeout'] or r['status'] is None or r['returncode'] not in (0,2,3,4) for r in report['runs']))

if __name__=='__main__':raise SystemExit(main())
