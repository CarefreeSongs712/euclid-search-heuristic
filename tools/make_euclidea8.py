#!/usr/bin/env python3
"""Build analytic, reproducible numeric representatives of eight screenshot tasks."""
from pathlib import Path
import json
import mpmath as m

P=Path(__file__).resolve().parents[1]
D=P/'benchmarks/euclidea8'
(D/'cases').mkdir(parents=True,exist_ok=True)
m.mp.dps=110
F=m.mpf
pt=lambda x,y:(F(str(x)),F(str(y)))
add=lambda a,b:(a[0]+b[0],a[1]+b[1])
sub=lambda a,b:(a[0]-b[0],a[1]-b[1])
mul=lambda k,a:(k*a[0],k*a[1])
dot=lambda a,b:a[0]*b[0]+a[1]*b[1]
norm=lambda a:m.sqrt(dot(a,a))
dist=lambda a,b:norm(sub(a,b))
cross=lambda a,b:a[0]*b[1]-a[1]*b[0]
line=lambda a,b:(b[1]-a[1],a[0]-b[0],a[0]*b[1]-a[1]*b[0])
fmt=lambda x:m.nstr(x,90)
row=lambda a:' '.join(fmt(x) for x in a)
models=[]

def save(name,title,E,points,lines,rays,segments,circles,target_lines,target_circles,target_points,
         checks,notes,branches=None):
    for key,value in checks.items():
        assert abs(value)<F('1e-95'),(name,key,value)
    file=D/'cases'/f'{name}.in'
    records=[str(E),'2',f'{len(points)} {len(lines)} {len(rays)} {len(segments)} {len(circles)}',
             *map(row,points),*map(row,lines),*[row(a+b) for a,b in rays],
             *[row(a+b) for a,b in segments],*map(row,circles),
             f'{len(target_lines)} {len(target_circles)} {len(target_points)}',
             *map(row,target_lines),*map(row,target_circles),*map(row,target_points)]
    file.write_text('\n'.join(records)+'\n',encoding='ascii')
    encode=lambda xs:[[fmt(x) for x in a] for a in xs]
    models.append({'name':name,'title':title,'E':E,'input':file.relative_to(P).as_posix(),
        'points':encode(points),'lines':encode(lines),'rays':[[encode([a])[0],encode([b])[0]] for a,b in rays],
        'segments':[[encode([a])[0],encode([b])[0]] for a,b in segments],'circles':encode(circles),
        'goal_lines':encode(target_lines),'goal_circles':encode(target_circles),'goal_points':encode(target_points),
        'checks':{k:fmt(v) for k,v in checks.items()},'notes':notes,'branches':branches})

# 13.5: MD equals distance of D to the second ray.
B=pt(0,0);A=pt('5.7',0);C=pt('3.1','4.3');M=pt('2.7','1.1')
u,v=C;p,q=M;S=u*u+v*v;alpha=u*u/S
roots=[(p-sign*m.sqrt(p*p-alpha*(p*p+q*q)))/alpha for sign in [1,-1]]
d=roots[0];X=pt(d,0);Y=mul(d*u/S,C)
assert d>0 and d<M[0] and dot(Y,C)>0
save('eu13_5_equal_distance','13.5 定点与角边等距',8,[B,A,C,M],[],[(B,A),(B,C)],[],[],
     [line(M,X),line(X,Y)],[],[X,Y],{'equal_lengths':dist(M,X)-dist(X,Y),'perpendicular':dot(sub(X,Y),C),'E_on_BC':cross(Y,C)},
     ['Initial angle is two rays, not infinite lines. A/C are explicitly selectable defining points in this test model.',
      'Two ray-valid branches exist; the smaller positive d (D left of M) is fixed as the target. MD and DE must be drawn.'],
     {'D_x_roots':[fmt(x) for x in roots]})

# 15.10: scale has no mathematical meaning, but the discrete solver needs a second point.
O=pt(0,0);A=pt(1,0);theta=m.pi/60
save('eu15_10_angle3','15.10 3度角',7,[O,A],[],[(O,A)],[],[],
     [(-m.sin(theta),m.cos(theta),F(0))],[],[],{'angle':m.atan2(m.sin(theta),m.cos(theta))-theta},
     ['A=(1,0) is a declared arbitrary free point on the given ray, needed because the solver cannot sample free points.',
      'Target is the support line of the positive 3 degree ray; no prescribed endpoint is required. E cost, not L macro count.'])

# 14.1: A,X,Z,Y rhombus, X on AB,Y on AC,Z on BC.
A=pt(0,0);B=pt('6.7',0);C=pt('2.4','4.1');a=norm(B);b=norm(C);s=a*b/(a+b)
X=pt(s,0);Y=mul(s/b,C);Z=add(X,Y)
assert 0<s<a and 0<s<b
save('eu14_1_inscribed_rhombus','14.1 三角形内接菱形',8,[A,B,C],[],[],[(A,B),(B,C),(C,A)],[],
     [line(X,Z),line(Y,Z)],[],[X,Y,Z],{'BC_incidence':cross(sub(Z,B),sub(C,B)),
      'side1':dist(A,X)-dist(X,Z),'side2':dist(A,X)-dist(Z,Y),'side3':dist(A,X)-dist(Y,A)},
     ['Only finite triangle sides and vertices are given. AX/AY lie on existing segments; XZ/YZ are the two new target lines.',
      'Generic scalene triangle; no midpoint or equal-side special case.'])

# 14.5: internally tangent outer circle and two externally tangent inner circles.
a=F('1.4');b=F('2.9');R=a+b;O=pt(0,0);O1=pt(-b,0);O2=pt(a,0)
den=a*a+a*b+b*b;rho=a*b*R/den;K=pt(R*R*(a-b)/den,2*rho)
save('eu14_5_apollonius','14.5 共线圆心三相切圆',7,[O,O1,O2],[],[],[],[(O[0],O[1],R),(O1[0],O1[1],a),(O2[0],O2[1],b)],
     [],[(K[0],K[1],rho)],[],{'outer_tangent':norm(K)+rho-R,'inner1_tangent':dist(K,O1)-rho-a,'inner2_tangent':dist(K,O2)-rho-b},
     ['Only three centers and three circles are initially given. Tangency points arise from initial intersections, not extra free coordinates.',
      'Upper tangent circle is fixed; lower reflected circle is also valid. The result circle must actually be drawn.'])

# 15.4: AX=XY=YB within the two given finite segments.
O=pt(0,0);A=pt('4.8',0);B=pt('2.3','3.7');a=norm(A);b=norm(B);co=B[0]/b
qa=1-2*co;qb=-2*(a+b)*(1-co);qc=a*a+b*b-2*a*b*co
rr=[(-qb-sign*m.sqrt(qb*qb-4*qa*qc))/(2*qa) for sign in [1,-1]]
legal=[x for x in rr if 0<x<min(a,b)];assert len(legal)==1
s=legal[0];X=pt(a-s,0);Y=mul(1-s/b,B)
save('eu15_4_three_equal','15.4 三条相等线段',7,[O,A,B],[],[],[(O,A),(O,B)],[],[line(X,Y)],[],[X,Y],
     {'equal1':dist(A,X)-dist(X,Y),'equal2':dist(B,Y)-dist(X,Y),'on_OB':cross(Y,B)},
     ['Finite OA/OB segments. X/Y strictly interior; draw XY. AX and YB are existing subsegments.',
      'Only one positive interior branch is valid.'],{'common_length_roots':[fmt(x) for x in rr]})

# 14.3: incircle touch point on a specified hypotenuse; circle is NOT given.
A=pt(0,0);B=pt('7.1',0);T=pt('2.3',0);c=B[0];t=T[0]
r=(-c+m.sqrt(c*c+4*t*(c-t)))/2;a=c-t+r;b=t+r
C=pt(b*b/c,a*b/c);I=pt(t,r)
save('eu14_3_hypotenuse_touch','14.3 由斜边切点作直角三角形',9,[A,B,T],[],[],[(A,B)],[],
     [line(A,C),line(B,C)],[],[C],{'right_angle':dot(sub(A,C),sub(B,C)),
     'touch_AC':abs(cross(C,I))/b-r,'touch_BC':abs(cross(sub(C,B),sub(I,B)))/a-r,
     'incircle_incenter_x':(b*c+c*C[0])/(a+b+c)-t,'incircle_incenter_y':c*C[1]/(a+b+c)-r},
     ['Only A/B/T and finite AB are given. The grey incircle in the screenshot is explanatory, NOT an initial circle.',
      'Upper right-angle vertex C fixed; its reflection is another solution. Draw both legs.'])

# 13.10: angle bisector theorem with O inside AB.
O=pt(0,0);R=F('3.4');p=F('1.2');q=F('1.63');A=pt(-p,0);B=pt(q,0)
x=(p-q)*R*R/(2*p*q);y=m.sqrt(R*R-x*x);C=pt(x,y)
u=mul(1/dist(C,A),sub(A,C));v=mul(1/dist(C,B),sub(B,C));w=sub(O,C)
assert dot(add(u,v),w)>0
save('eu13_10_billiards','13.10 圆桌台球',6,[O,A,B],[],[],[],[(F(0),F(0),R)],[],[],[C],
     {'on_circle':dot(C,C)-R*R,'bisector_ratio':dist(C,A)/dist(C,B)-p/q,'bisector_direction':cross(add(u,v),w)},
     ['Only O/A/B and centered circle are given, not the diameter carrier. O lies between A/B.',
      'The upper nondegenerate C is the fixed target; drawing CA/CB/OC is not required by the point-only task.',
      'B=1.63 intentionally avoids the R/2 midpoint special case.'])

# 13.3: rotate one circle inversely around A, intersect, then reconstruct Y.
O=pt(0,0);A=pt('3.8','0.4');r=F('1.7');R=F('2.9')
rot=lambda v,sig:(v[0]/2-sig*m.sqrt(3)*v[1]/2,sig*m.sqrt(3)*v[0]/2+v[1]/2)
branches=[]
for sig in [1,-1]:
    center=add(A,rot(sub(O,A),-sig));d=norm(center)
    foot=(r*r-R*R+d*d)/(2*d);height=m.sqrt(r*r-foot*foot)
    for tau in [-1,1]:
        X=add(mul(foot/d,center),mul(tau*height/d,(-center[1],center[0])))
        Y=add(A,rot(sub(X,A),sig))
        branches.append((sig,tau,X,Y))
sig,tau,X,Y=branches[0]
save('eu13_3_concentric_equilateral','13.3 同心圆上的等边三角形',8,[O,A],[],[],[],[(F(0),F(0),r),(F(0),F(0),R)],
     [line(A,X),line(X,Y),line(Y,A)],[],[X,Y],{'inner':norm(X)-r,'outer':norm(Y)-R,
      'equal1':dist(A,X)-dist(X,Y),'equal2':dist(A,X)-dist(A,Y)},
     ['Only center O and external vertex A plus two concentric circles are given. No radius endpoint is initially known.',
      'Four oriented circle-intersection branches exist; one complete branch is fixed. All three triangle sides must be drawn.'],
     [{'rotation_sign':s,'intersection_sign':t,'X':[fmt(x) for x in xx],'Y':[fmt(y) for y in yy]} for s,t,xx,yy in branches])

(D/'model_validation.json').write_text(json.dumps({'precision':110,'scope':'Analytic representative instances, not recovered exact game states',
    'strict_E_tools':'ordinary lines/circles, intersections free, no implicit extensions',
    'fixed_branches':True,'models':models},indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
manifest=[{'name':x['name'],'title':x['title'],'input':x['input'],'E':x['E'],
           'eps':'1e-11','threads':8,'time_limit':60,'solutions':1} for x in models]
(D/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print('Built',len(models),'analytic models, all 110-digit residual checks <1e-95')
