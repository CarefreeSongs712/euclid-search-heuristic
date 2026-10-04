#pragma once
#include "geometry.hpp"
#include "progress.hpp"
#include "probe_validation.hpp"

namespace bs::diameter_detail {
struct Counts {uint64_t pairs=0,applied=0,callbacks=0,successes=0;};
inline bool Valid(const Element& e) {
    return isfinite(e.a)&&isfinite(e.b)&&isfinite(e.c)&&
        (e.type==Type::Circle?e.c>0.0:e.type==Type::Line&&(e.a!=0.0||e.b!=0.0));
}
inline Candidate Pair(uint32_t a,uint32_t b,bool circle) {
    return a<b?Candidate{a,b,static_cast<uint8_t>(circle?0:2)}:
               Candidate{b,a,static_cast<uint8_t>(circle?1:2)};
}

// Construct a diameter circle using only real known-pair operations: reciprocal
// endpoint circles, their common chord, optional paid endpoint line, and the
// midpoint circle. Analytic midpoint is only a lookup key AFTER actual Apply.
template<class Complete>
bool Probe(const Graph& parent,int remaining,int tools,ParallelControl& control,
           SearchStats& stats,Counts& counts,chrono::steady_clock::time_point deadline,
           Complete&& complete,ProgressSlot* progress=nullptr) {
    using Clock=chrono::steady_clock;
    if(control.deadline!=Clock::time_point{})deadline=min(deadline,control.deadline);
    if(control.stop.load(memory_order_acquire)||Clock::now()>=deadline)return false;
    if(parent.initialElementCount>parent.elements.size()||parent.pointBirth.size()!=parent.points.size())
        throw invalid_argument("diameter probe requires consistent paid parent");
    const size_t depth=ValidateProbeParent(parent);
    if(tools!=2||remaining<1||parent.points.size()<2||parent.points.size()>32||parent.elements.size()>64||
       depth+static_cast<size_t>(remaining)>numeric_limits<uint16_t>::max())return false;
    Graph graph=parent;
    graph.SetStateHashingEnabled(false);
    const Mark root=graph.GetMark();
    auto stop=[&]() {
        if(progress)progress->Publish(stats,static_cast<uint16_t>(graph.elements.size()-graph.initialElementCount));
        return control.stop.load(memory_order_acquire)||Clock::now()>=deadline;
    };
    auto apply=[&](uint32_t a,uint32_t b,bool circle) {
        if(stop()||a==b||a>=graph.points.size()||b>=graph.points.size()||SamePoint(graph.points[a],graph.points[b]))return false;
        ++stats.rawCandidates;
        const Element e=graph.MakeCandidate(Pair(a,b,circle));
        if(!Valid(e))return false;
        if(graph.HasElement(e))return true;
        if(graph.elements.size()-parent.elements.size()>=static_cast<size_t>(remaining))return false;
        if(!ApplyFiniteProbe(graph,e,static_cast<uint16_t>(graph.elements.size()-graph.initialElementCount+1)))return false;
        ++stats.applied;++stats.nodes;++stats.uniqueCandidates;++counts.applied;
        stats.maxPoints=max(stats.maxPoints,graph.points.size());
        stats.maxElements=max(stats.maxElements,graph.elements.size());
        return true;
    };
    auto find=[&](Point p) {
        for(uint32_t i=0;i<graph.points.size();++i)if(SamePoint(p,graph.points[i]))return i;
        return NO_BOUND;
    };
    for(uint32_t a=0;a<parent.points.size();++a)for(uint32_t b=a+1;b<parent.points.size();++b) {
        if(stop())return false;
        if(++counts.pairs>64)return false;
        graph.Rollback(root);
        const Point p=graph.points[a],q=graph.points[b];
        if(SamePoint(p,q))continue;
        const Point predictedMid{(p.x+q.x)*0.5,(p.y+q.y)*0.5};
        uint32_t mid=find(predictedMid);
        if(mid==NO_BOUND) {
            if(!apply(a,b,true)||!apply(b,a,true))continue;
            const Element first=Element::FromPoints(p,q,Type::Circle);
            const Element second=Element::FromPoints(q,p,Type::Circle);
            vector<Point> intersections;
            graph.VisitIntersections(first,second,[&](Point v){intersections.push_back(v);return false;});
            if(intersections.size()!=2)continue;
            const uint32_t x=find(intersections[0]),y=find(intersections[1]);
            if(x==NO_BOUND||y==NO_BOUND||!apply(x,y,false))continue;
            mid=find(predictedMid);
            if(mid==NO_BOUND) {
                if(!apply(a,b,false))continue;
                mid=find(predictedMid);
            }
        }
        if(mid==NO_BOUND||mid==a||mid==b||!apply(mid,a,true))continue;
        if(stop())return false;
        ++counts.callbacks;
        const int spent=static_cast<int>(graph.elements.size()-parent.elements.size());
        if(complete(static_cast<const Graph&>(graph),remaining-spent)){++counts.successes;return true;}
    }
    return false;
}
} // namespace bs::diameter_detail
