#!/usr/bin/env python3
"""Analytic nondegenerate representatives for the two new screenshot tasks."""
from pathlib import Path
import json
import mpmath as mp

P=Path(__file__).resolve().parents[1]
D=P/'benchmarks/r32'
(D/'cases').mkdir(parents=True,exist_ok=True)
mp.mp.dps=110
F=mp.mpf
point=lambda x,y:(F(str(x)),F(str(y)))
add=lambda a,b:(a[0]+b[0],a[1]+b[1])
sub=lambda a,b:(a[0]-b[0],a[1]-b[1])
scale=lambda k,p:(k*p[0],k*p[1])
dot=lambda a,b:a[0]*b[0]+a[1]*b[1]
norm=lambda a:mp.sqrt(dot(a,a))
cross=lambda a,b:a[0]*b[1]-a[1]*b[0]
line=lambda a,b:(b[1]-a[1],a[0]-b[0],a[0]*b[1]-a[1]*b[0])
fmt=lambda x:mp.nstr(x,90)
row=lambda p:' '.join(map(fmt,p))
records=[]

O1=point(0,0);O2=point('4.7','.4');r1=F('1.3');r2=F('2.1')
v=sub(O2,O1);d=norm(v);u=scale(1/d,v);c=(r1-r2)/d
n=add(scale(c,u),scale(mp.sqrt(1-c*c),(-u[1],u[0])))
h=dot(n,O1)+r1
T1=add(O1,scale(r1,n));T2=add(O2,scale(r2,n))
assert d>r1+r2 and n[1]>0
checks={'tangent1':abs(dot(n,O1)-h)-r1,'tangent2':abs(dot(n,O2)-h)-r2,
        'unit_normal':dot(n,n)-1,'T1line':dot(n,T1)-h,'T2line':dot(n,T2)-h}
assert max(map(abs,checks.values()))<F('1e-95')
file=D/'cases/eu10_2_external_tangent.in'
file.write_text('\n'.join(['8','2','2 0 0 0 2',row(O1),row(O2),row((*O1,r1)),row((*O2,r2)),
                          '1 0 0',row((*n,h))])+'\n',encoding='ascii')
records.append({'name':'eu10_2_external_tangent','title':'10.2 两圆外公切线','E':8,
    'input':file.relative_to(P).as_posix(),'checks':{k:fmt(v) for k,v in checks.items()},
    'known':'Two centers and two disjoint circles, no free radius endpoint or center line.',
    'goal':'Upper external common tangent line only. Tangency points are NOT initially known.',
    'centers':[list(map(fmt,O1)),list(map(fmt,O2))],'radii':[fmt(r1),fmt(r2)],
    'target_line':list(map(fmt,(*n,h))),'target_tangencies':[list(map(fmt,T1)),list(map(fmt,T2))]})

A=point('.8','4.3');B=point('-2.7','.1');C=point('4.2','-.2')
vertices=[A,B,C]
for i in range(3):
    assert dot(sub(vertices[(i+1)%3],vertices[i]),sub(vertices[(i+2)%3],vertices[i]))>0
feet=[];checks={}
for i,(V,X,Y) in enumerate([(A,B,C),(B,C,A),(C,A,B)]):
    w=sub(Y,X);t=dot(sub(V,X),w)/dot(w,w);assert 0<t<1
    H=add(X,scale(t,w));feet.append(H)
    checks[f'foot{i}_perp']=dot(sub(V,H),w)
    checks[f'foot{i}_incidence']=cross(sub(H,X),w)
assert max(map(abs,checks.values()))<F('1e-95')
D0,E,G=feet
file=D/'cases/eu9_7_minimum_perimeter.in'
file.write_text('\n'.join(['8','2','3 0 0 3 0',row(A),row(B),row(C),row(A+B),row(B+C),row(C+A),
    '3 0 3',row(line(D0,E)),row(line(E,G)),row(line(G,D0)),row(D0),row(E),row(G)])+'\n',encoding='ascii')
records.append({'name':'eu9_7_minimum_perimeter','title':'9.7 最小周长内接三角形','E':8,
    'input':file.relative_to(P).as_posix(),'checks':{k:fmt(v) for k,v in checks.items()},
    'known':'Three vertices and three FINITE segments; no free altitudes or circumcircle.',
    'goal':'Orthic triangle feet and all three edges. Fagnano theorem applies to this strictly acute scalene triangle.',
    'vertices':[list(map(fmt,p)) for p in vertices],'feet':[list(map(fmt,p)) for p in feet],
    'perimeter':fmt(sum(norm(sub(feet[i],feet[(i+1)%3])) for i in range(3)))})
(D/'model_validation.json').write_text(json.dumps({'precision':110,'scope':'Analytic representative models, not pixel-extracted game state','models':records},indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
manifest=[dict(name=r['name'],input=r['input'],eps='1e-11',threads=8,time_limit=90) for r in records]
(D/'new_cases.json').write_text(json.dumps(manifest,indent=2)+'\n')
print('2 models validated at110 digits, all residuals <1e-95')
