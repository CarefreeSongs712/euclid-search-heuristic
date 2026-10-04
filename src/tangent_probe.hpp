#pragma once
#include "geometry.hpp"
#include "progress.hpp"
#include "probe_validation.hpp"

namespace bs::tangent_detail {
struct Counts {uint64_t circles=0,externalPoints=0,applied=0,callbacks=0,successes=0;};
inline bool Valid(const Element& e) {
    return isfinite(e.a)&&isfinite(e.b)&&isfinite(e.c)&&
        (e.type==Type::Circle?e.c>0.0:e.type==Type::Line&&(e.a!=0.0||e.b!=0.0));
}
inline Candidate Pair(uint32_t a,uint32_t b,bool circle) {
    return a<b?Candidate{a,b,static_cast<uint8_t>(circle?0:2)}:Candidate{b,a,static_cast<uint8_t>(circle?1:2)};
}

// Geometric tangency is only a trigger. First try a paid diameter/doubled-chord
// construction, then a Thales-circle fallback. Every endpoint is obtained from
// actual known intersections; no analytical tangent point or length is inserted.
template<class Complete>
bool Probe(const Graph& parent,int remaining,int tools,ParallelControl& control,SearchStats& stats,
           Counts& counts,chrono::steady_clock::time_point deadline,Complete&& complete,ProgressSlot* progress=nullptr) {
    using Clock=chrono::steady_clock;
    if(control.deadline!=Clock::time_point{})deadline=min(deadline,control.deadline);
    if(control.stop.load(memory_order_acquire)||Clock::now()>=deadline)return false;
    if(parent.initialElementCount>parent.elements.size()||parent.points.size()!=parent.pointBirth.size())
        throw invalid_argument("tangent probe inconsistent parent");
    const size_t depth=ValidateProbeParent(parent);
    if(tools!=2||remaining<1||parent.points.size()>128||parent.elements.size()>64||
       depth+static_cast<size_t>(remaining)>numeric_limits<uint16_t>::max())return false;
    vector<Element> targets;
    for(const Element& e:parent.goalElements)if(e.type==Type::Line&&!parent.HasElement(e))targets.push_back(e);
    if(targets.empty())return false;
    auto find=[](const Graph& g,Point p){for(uint32_t i=0;i<g.points.size();++i)if(SamePoint(g.points[i],p))return i;return NO_BOUND;};
    Graph graph=parent;graph.SetStateHashingEnabled(false);const Mark root=graph.GetMark();
    auto stop=[&](){if(progress)progress->Publish(stats,static_cast<uint16_t>(graph.elements.size()-graph.initialElementCount));return control.stop.load(memory_order_acquire)||Clock::now()>=deadline;};
    auto apply=[&](uint32_t a,uint32_t b,bool circle) {
        if(stop()||a==b||SamePoint(graph.points[a],graph.points[b]))return false;
        const Element e=graph.MakeCandidate(Pair(a,b,circle));++stats.rawCandidates;
        if(!Valid(e))return false;
        if(graph.HasElement(e))return true;
        if(graph.elements.size()-parent.elements.size()>=static_cast<size_t>(remaining))return false;
        if(!ApplyFiniteProbe(graph,e,static_cast<uint16_t>(graph.elements.size()-graph.initialElementCount+1)))return false;
        ++counts.applied;++stats.applied;++stats.nodes;
        stats.maxPoints=max(stats.maxPoints,graph.points.size());stats.maxElements=max(stats.maxElements,graph.elements.size());return true;
    };
    for(const Element& circle:parent.elements) {
        if(stop())return false;
        if(circle.type!=Type::Circle||circle.c<=0.0)continue;
        if(++counts.circles>16)break;
        const uint32_t center=find(parent,{circle.a,circle.b});if(center==NO_BOUND)continue;
        for(const Element& target:targets) {
            const double normal2=Sq(target.a)+Sq(target.b);
            if(normal2<=0.0||!isfinite(normal2))continue;
            const double residual=target.a*circle.a+target.b*circle.b-target.c;
            if(abs(Sq(residual)/normal2-circle.c)>max(EPS*100,1e-10)*(1+circle.c))continue;
            for(uint32_t h=0;h<parent.points.size();++h) {
                if(stop()||counts.externalPoints>=64)return false;
                const Point O=parent.points[center],H=parent.points[h];
                if(!parent.PointOnElement(H,target)||Sq(H.x-O.x)+Sq(H.y-O.y)<=circle.c+EPS)continue;
                ++counts.externalPoints;
                graph.Rollback(root);
                // Doubling-chord construction: O=0,H=-h,P=-r gives H'=h,
                // Q=h-2r; circle(H',Q) meets circle(O,H) at K and HK is
                // tangent to the radius-r circle. All named points below are
                // resolved from actual known intersections before being used.
                const Element axis=Element::FromPoints(O,H,Type::Line);
                const double distance=sqrt(Sq(H.x-O.x)+Sq(H.y-O.y));
                const double radius=sqrt(circle.c);
                const Point predictedP{O.x+(H.x-O.x)*radius/distance,O.y+(H.y-O.y)*radius/distance};
                uint32_t p=find(graph,predictedP);
                if(p==NO_BOUND&&apply(center,h,false))p=find(graph,predictedP);
                if(p!=NO_BOUND&&apply(center,h,true)) {
                    const Element outer=Element::FromPoints(O,H,Type::Circle);
                    const uint32_t opposite=find(graph,{2*O.x-H.x,2*O.y-H.y});
                    if(opposite!=NO_BOUND&&apply(p,h,true)) {
                        const Element transfer=Element::FromPoints(graph.points[p],graph.points[h],Type::Circle);
                        vector<uint32_t> qCandidates;
                        graph.VisitIntersections(axis,transfer,[&](Point q){
                            const uint32_t id=find(graph,q);
                            if(id!=NO_BOUND&&id!=h&&!SamePoint(q,H))qCandidates.push_back(id);
                            return false;
                        });
                        const Mark beforeLastCircle=graph.GetMark();
                        for(uint32_t q:qCandidates) {
                            graph.Rollback(beforeLastCircle);
                            if(stop()||!apply(opposite,q,true))continue;
                            const Element doubled=Element::FromPoints(graph.points[opposite],graph.points[q],Type::Circle);
                            vector<uint32_t> candidates;
                            graph.VisitIntersections(outer,doubled,[&](Point k){auto id=find(graph,k);if(id!=NO_BOUND)candidates.push_back(id);return false;});
                            const Mark ready=graph.GetMark();
                            for(uint32_t k:candidates) {
                                graph.Rollback(ready);
                                if(stop()||!apply(h,k,false))continue;
                                ++counts.callbacks;
                                const int spent=static_cast<int>(graph.elements.size()-parent.elements.size());
                                if(complete(static_cast<const Graph&>(graph),remaining-spent)){++counts.successes;return true;}
                            }
                        }
                    }
                }
                graph.Rollback(root);
                uint32_t middle=find(graph,{(O.x+H.x)*.5,(O.y+H.y)*.5});
                if(middle==NO_BOUND) {
                    if(!apply(center,h,true)||!apply(h,center,true))continue;
                    const Element c1=Element::FromPoints(O,H,Type::Circle),c2=Element::FromPoints(H,O,Type::Circle);
                    vector<uint32_t> cross;
                    graph.VisitIntersections(c1,c2,[&](Point p){auto id=find(graph,p);if(id!=NO_BOUND)cross.push_back(id);return false;});
                    if(cross.size()!=2||!apply(cross[0],cross[1],false))continue;
                    middle=find(graph,{(O.x+H.x)*.5,(O.y+H.y)*.5});
                    if(middle==NO_BOUND) {
                        if(!apply(center,h,false))continue;
                        middle=find(graph,{(O.x+H.x)*.5,(O.y+H.y)*.5});
                    }
                }
                if(middle==NO_BOUND||!apply(middle,center,true))continue;
                const Element thales=Element::FromPoints(graph.points[middle],graph.points[center],Type::Circle);
                vector<uint32_t> touches;
                graph.VisitIntersections(thales,circle,[&](Point p){auto id=find(graph,p);if(id!=NO_BOUND)touches.push_back(id);return false;});
                const Mark base=graph.GetMark();
                for(uint32_t t:touches) {
                    graph.Rollback(base);
                    if(stop()||!apply(h,t,false))continue;
                    ++counts.callbacks;
                    const int spent=static_cast<int>(graph.elements.size()-parent.elements.size());
                    if(complete(static_cast<const Graph&>(graph),remaining-spent)){++counts.successes;return true;}
                }
            }
        }
    }
    return false;
}
} // namespace bs::tangent_detail
