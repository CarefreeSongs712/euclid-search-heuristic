#!/usr/bin/env python3
"""Independently replay certificates/readable v10 witnesses from Euclidea tests."""
from pathlib import Path
import argparse
import json
import re
import subprocess
import sys

P=Path(__file__).resolve().parents[1]
a=argparse.ArgumentParser(description=__doc__)
a.add_argument('--directory',type=Path,required=True)
a.add_argument('--output',type=Path,required=True)
x=a.parse_args()
rows=json.loads((x.directory/'results.json').read_text(encoding='utf-8'))['runs']
results=[]
for r in rows:
    if r['status']!='QUOTA_REACHED':continue
    stem=f"{r['name']}.{r['label']}.r1"
    certificate=x.directory/(stem+'.certificate.json')
    if r['label']=='v10':
        text=(x.directory/(stem+'.stdout.log')).read_text(encoding='utf-8',errors='replace')
        steps=[]
        for line in text.splitlines():
            if not re.match(r'^第\d+步：',line):continue
            coords=re.findall(r'P\d+=\(([^,()]+),([^,()]+)\)',line)
            if len(coords)<2:raise RuntimeError('Cannot extract two 17-digit defining points: '+line)
            first=[float(q) for q in coords[0]];second=[float(q) for q in coords[1]]
            type_='circle' if '为圆心' in line else 'line'
            if type_=='circle':e=[first[0],first[1],(first[0]-second[0])**2+(first[1]-second[1])**2]
            else:
                aa=second[1]-first[1];bb=first[0]-second[0];cc=first[0]*second[1]-first[1]*second[0]
                eps=float(next(v[6:] for v in r['command'] if v.startswith('--eps=')))
                if abs(bb)>=eps:aa,cc,bb=aa/bb,cc/bb,1.
                elif abs(aa)>=eps:cc,aa,bb=cc/aa,1.,0.
                e=[0. if abs(v)<eps else v for v in [aa,bb,cc]]
            steps.append({'type':type_,'first':first,'second':second,'element':e})
        if len(steps)!=r['returned_E'][0]:raise RuntimeError('Incomplete readable certificate')
        eps=next(v[6:] for v in r['command'] if v.startswith('--eps='))
        certificate=x.directory/(stem+'.extracted_certificate.json')
        certificate.write_text(json.dumps({'version':'v11','source_version':'v10-readable-report-extraction','eps':float(eps),
             'solutions':[{'steps':steps,'E':len(steps)}]},indent=2)+'\n')
    p=subprocess.run([sys.executable,str(P/'tools/verify_certificate.py'),'--input',str(P/r['input']),
        '--input-layout','compact','--certificate',str(certificate),'--precision','100','--require-original-eps'],capture_output=True)
    try:check=json.loads(p.stdout)
    except json.JSONDecodeError:check={'error':p.stderr.decode(errors='replace')}
    results.append({'case':r['name'],'version':r['label'],'pass':p.returncode==0,'certificate':str(certificate),'result':check})
    print('PASS' if p.returncode==0 else 'FAIL',r['name'],r['label'],check.get('status'),flush=True)
x.output.write_text(json.dumps({'all_pass':all(r['pass'] for r in results),'checks':results},indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
raise SystemExit(int(not all(r['pass'] for r in results)))
