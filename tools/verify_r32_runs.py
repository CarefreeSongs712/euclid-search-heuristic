#!/usr/bin/env python3
"""Classify replayed certificates without equating EPS success to exact tangency."""
from pathlib import Path
import argparse
import json
import subprocess
import sys
import mpmath as mp

P=Path(__file__).resolve().parents[1]
mp.mp.dps=120

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--directory',type=Path,required=True)
    ap.add_argument('--out',type=Path,required=True)
    a=ap.parse_args()
    results=[]
    data=json.loads((a.directory/'results.json').read_text(encoding='utf-8'))
    for r in data['runs']:
        if r['label']=='v10' or r['status']!='QUOTA_REACHED':continue
        cert=a.directory/f"{r['name']}.{r['label']}.r{r['repetition']}.certificate.json"
        p=subprocess.run([sys.executable,str(P/'tools/verify_certificate.py'),'--input',str(P/r['input']),
                         '--input-layout','compact','--certificate',str(cert),'--precision','120','--require-original-eps'],capture_output=True)
        try:verification=json.loads(p.stdout)
        except json.JSONDecodeError:verification={'status':'ERROR','error':p.stderr.decode(errors='replace')}
        row={'name':r['name'],'version':r['label'],'repetition':r['repetition'],'eps_replay_pass':p.returncode==0,
             'verification':verification,'strict_numeric_residual_pass':False}
        if p.returncode==0:
            solution=verification['solutions'][0]
            goals=solution['goals']
            residuals=[mp.mpf(g['residual']) for g in goals if g.get('residual') is not None]
            row['max_target_residual']=mp.nstr(max(residuals,default=mp.inf),60)
            # This is a stronger diagnostic, NOT an interval/symbolic proof.
            row['strict_numeric_residual_pass']=bool(residuals and max(residuals)<mp.mpf('1e-60'))
            if 'external_tangent' in r['name']:
                goal=next(g for g in goals if g['type']=='line')
                aa,bb,cc=map(mp.mpf,goal['nearest_value'])
                normal=mp.sqrt(aa*aa+bb*bb)
                gaps=[abs(aa*x+bb*y-cc)/normal-radius for x,y,radius in
                      [(mp.mpf('0'),mp.mpf('0'),mp.mpf('1.3')),(mp.mpf('4.7'),mp.mpf('.4'),mp.mpf('2.1'))]]
                row['tangency_distance_residuals']=[mp.nstr(g,60) for g in gaps]
                row['strict_numeric_residual_pass']=bool(max(map(abs,gaps))<mp.mpf('1e-60'))
                row['classification']='HIGH_PRECISION_TANGENCY' if row['strict_numeric_residual_pass'] else 'EPS_ONLY_NEAR_TANGENCY'
            else:row['classification']='HIGH_PRECISION_TARGETS' if row['strict_numeric_residual_pass'] else 'EPS_ONLY'
        results.append(row)
        print(r['name'],r['label'],r['repetition'],verification.get('status'),row.get('classification'),flush=True)
    a.out.write_text(json.dumps({'all_eps_replays_pass':all(r['eps_replay_pass'] for r in results),
                                'strict_threshold':'1e-60','strict_is_symbolic_proof':False,'checks':results},indent=2)+'\n')
    return int(not all(r['eps_replay_pass'] for r in results))

if __name__=='__main__':raise SystemExit(main())
