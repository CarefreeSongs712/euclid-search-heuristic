#pragma once
#include "geometry.hpp"
#include "progress.hpp"
#include "probe_validation.hpp"

namespace bs::homothety_detail {
struct Counts {uint64_t pairs=0,applied=0,callbacks=0,successes=0;};
inline bool Valid(const Element& e) {
    return isfinite(e.a)&&isfinite(e.b)&&isfinite(e.c)&&
       (e.type==Type::Circle?e.c>0.0:e.type==Type::Line&&(e.a!=0.0||e.b!=0.0));
}
inline Candidate Pair(uint32_t a,uint32_t b,bool circle) {
    return a<b?Candidate{a,b,static_cast<uint8_t>(circle?0:2)}:Candidate{b,a,static_cast<uint8_t>(circle?1:2)};
}

// A family of paid similarity-center prefixes for known-center circles. The
// two axial-circle constructions yield corresponding equilateral points.
// Nothing is transferred as a free length; every radius uses real endpoints.
template<class Complete>
bool Probe(const Graph& parent,int remaining,int tools,ParallelControl& control,SearchStats& stats,
           Counts& counts,chrono::steady_clock::time_point deadline,Complete&& complete,ProgressSlot* progress=nullptr) {
    using Clock=chrono::steady_clock;
    if(control.deadline!=Clock::time_point{})deadline=min(deadline,control.deadline);
    if(control.stop.load(memory_order_acquire)||Clock::now()>=deadline)return false;
    if(parent.initialElementCount>parent.elements.size()||parent.pointBirth.size()!=parent.points.size())
        throw invalid_argument("homothety probe inconsistent parent");
    const size_t depth=ValidateProbeParent(parent);
    if(tools!=2||remaining<1||parent.points.size()>64||parent.elements.size()>32||
       depth+static_cast<size_t>(remaining)>numeric_limits<uint16_t>::max())return false;
    auto find=[](const Graph& g,Point p){for(uint32_t i=0;i<g.points.size();++i)if(SamePoint(g.points[i],p))return i;return NO_BOUND;};
    struct Circle {Element e;uint32_t center;};
    vector<Circle> circles;
    for(const Element& e:parent.elements)if(e.type==Type::Circle&&circles.size()<8) {
        const uint32_t c=find(parent,{e.a,e.b});if(c!=NO_BOUND&&e.c>0)circles.push_back({e,c});
    }
    if(circles.size()<2)return false;
    Graph graph=parent;graph.SetStateHashingEnabled(false);const Mark root=graph.GetMark();
    auto stop=[&]() {
        if(progress)progress->Publish(stats,static_cast<uint16_t>(graph.elements.size()-graph.initialElementCount));
        return control.stop.load(memory_order_acquire)||Clock::now()>=deadline;
    };
    auto apply=[&](uint32_t a,uint32_t b,bool circle) {
        if(stop()||a==b||SamePoint(graph.points[a],graph.points[b]))return false;
        const Element e=graph.MakeCandidate(Pair(a,b,circle));++stats.rawCandidates;
        if(!Valid(e))return false;
        if(graph.HasElement(e))return true;
        if(graph.elements.size()-parent.elements.size()>=static_cast<size_t>(remaining))return false;
        if(!ApplyFiniteProbe(graph,e,static_cast<uint16_t>(graph.elements.size()-graph.initialElementCount+1)))return false;
        ++stats.applied;++stats.nodes;++counts.applied;
        stats.maxPoints=max(stats.maxPoints,graph.points.size());stats.maxElements=max(stats.maxElements,graph.elements.size());
        return true;
    };
    for(size_t i=0;i<circles.size();++i)for(size_t j=i+1;j<circles.size();++j) {
        if(stop())return false;
        if(++counts.pairs>16)return false;
        graph.Rollback(root);
        const auto first=circles[i],second=circles[j];
        if(!apply(first.center,second.center,false))continue;
        const Element axis=Element::FromPoints(graph.points[first.center],graph.points[second.center],Type::Line);
        vector<uint32_t> p,q;
        graph.VisitIntersections(axis,first.e,[&](Point x){auto id=find(graph,x);if(id!=NO_BOUND)p.push_back(id);return false;});
        graph.VisitIntersections(axis,second.e,[&](Point x){auto id=find(graph,x);if(id!=NO_BOUND)q.push_back(id);return false;});
        const Mark base=graph.GetMark();
        for(uint32_t a:p)for(uint32_t b:q) {
            if(stop())return false;
            graph.Rollback(base);
            if(!apply(a,first.center,true))continue;
            const Element circleA=Element::FromPoints(graph.points[a],graph.points[first.center],Type::Circle);
            if(!apply(b,second.center,true))continue;
            const Element circleB=Element::FromPoints(graph.points[b],graph.points[second.center],Type::Circle);
            vector<uint32_t> aa,bb;
            graph.VisitIntersections(circleA,first.e,[&](Point x){auto id=find(graph,x);if(id!=NO_BOUND)aa.push_back(id);return false;});
            graph.VisitIntersections(circleB,second.e,[&](Point x){auto id=find(graph,x);if(id!=NO_BOUND)bb.push_back(id);return false;});
            const Mark aux=graph.GetMark();
            for(uint32_t x:aa)for(uint32_t y:bb) {
                if(stop())return false;
                graph.Rollback(aux);
                if(!apply(x,y,false))continue;
                ++counts.callbacks;
                const int spent=static_cast<int>(graph.elements.size()-parent.elements.size());
                if(complete(static_cast<const Graph&>(graph),remaining-spent)){++counts.successes;return true;}
            }
        }
    }
    return false;
}
} // namespace bs::homothety_detail
