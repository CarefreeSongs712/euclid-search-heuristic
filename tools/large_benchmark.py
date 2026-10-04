#!/usr/bin/env python3
"""Long v10/v11 comparisons with resource samples and actual v11 certificates."""
from pathlib import Path
import argparse
import hashlib
import json
import platform
import re
import sys
from heavy_benchmark import run

ROOT=Path(__file__).resolve().parents[1]

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--v10',required=True)
    ap.add_argument('--v11',required=True)
    ap.add_argument('--manifest',type=Path,required=True)
    ap.add_argument('--output-dir',type=Path,required=True)
    ap.add_argument('--case',action='append')
    a=ap.parse_args()
    if a.output_dir.exists() and any(a.output_dir.iterdir()):
        ap.error('Output directory must be new or empty; previous evidence is not overwritten')
    a.output_dir.mkdir(parents=True,exist_ok=True)
    cases=json.loads(a.manifest.read_text(encoding='utf-8'))
    report={'kind':'large_v10_v11_comparison','platform':platform.platform(),
            'cpu':platform.processor(),'runs':[],
            'executables':{name:{'path':str(Path(exe).resolve()),'sha256':hashlib.sha256(Path(exe).read_bytes()).hexdigest()}
                           for name,exe in [('v10',a.v10),('v11',a.v11)]},
            'interpretation':'Heuristic timeout/stop is not exhaustion; nodes across engines are not equal work.'}
    for case in cases:
        if a.case and case['name'] not in a.case:continue
        for label,exe in [('v10',a.v10),('v11',a.v11)]:
            if label not in case.get('versions',['v10','v11']):continue
            cfg=dict(case)
            cfg['args']=list(case.get('args',[]))
            certificate=a.output_dir/f'{case["name"]}.{label}.certificate.json'
            if label=='v11':
                cfg['args'] += [f'--seed={case.get("seed",1)}',*case.get('v11_args',[]),
                                '--certificate='+str(certificate.resolve())]
            row=run(exe,cfg,label,1,a.output_dir)
            text=Path(row['stdout']).read_text(encoding='utf-8',errors='replace')
            row['returned_E']=[int(x) for x in re.findall(r'（(\d+)E）',text)]
            for name,key in [('Heuristic restarts','restarts'),('Beam successful state visits','beam_success_visits'),
                             ('Helper successful state visits','helper_success_visits'),
                             ('Candidate paths discarded','candidate_discarded'),('Beam paths discarded','beam_discarded')]:
                m=re.search(r'^'+name+r':\s*(\d+)',text,re.M)
                row[key]=int(m.group(1)) if m else None
            row['certificate']=str(certificate) if certificate.exists() else None
            row['input']=case['input']
            row['seed']=case.get('seed',1) if label=='v11' else None
            row['engine_success']=row['status']=='QUOTA_REACHED' and row['solutions']>0
            report['runs'].append(row)
            (a.output_dir/'results.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    invalid=any(r['external_timeout'] or r['status'] is None or
                (r['label']=='v11' and r['status']=='EXHAUSTED') for r in report['runs'])
    return int(invalid)

if __name__=='__main__':sys.exit(main())
