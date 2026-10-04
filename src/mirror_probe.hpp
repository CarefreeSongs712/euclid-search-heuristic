#pragma once
#include "geometry.hpp"
#include "progress.hpp"
#include "probe_validation.hpp"

namespace bs::mirror_detail {
struct Counts {uint64_t candidates=0,applied=0,callbacks=0,successes=0;};
inline bool Valid(const Element& e) {
    return isfinite(e.a)&&isfinite(e.b)&&isfinite(e.c)&&
        (e.type==Type::Circle?e.c>0.0:e.type==Type::Line&&(e.a!=0.0||e.b!=0.0));
}
inline Candidate Pair(uint32_t a,uint32_t b,bool circle) {
    return a<b?Candidate{a,b,static_cast<uint8_t>(circle?0:2)}:Candidate{b,a,static_cast<uint8_t>(circle?1:2)};
}

// Candidate reflection is geometric metadata, never an inserted point. Two
// known points lying on an existing carrier center circles through a known P;
// the other actual circle intersection is its mirror across the carrier.
template<class Complete>
bool Probe(const Graph& parent,int remaining,int tools,ParallelControl& control,SearchStats& stats,
           Counts& counts,chrono::steady_clock::time_point deadline,Complete&& complete,ProgressSlot* progress=nullptr) {
    using Clock=chrono::steady_clock;
    if(control.deadline!=Clock::time_point{})deadline=min(deadline,control.deadline);
    if(control.stop.load(memory_order_acquire)||Clock::now()>=deadline)return false;
    if(parent.initialElementCount>parent.elements.size()||parent.pointBirth.size()!=parent.points.size())
        throw invalid_argument("mirror probe requires consistent parent");
    const size_t depth=ValidateProbeParent(parent);
    if((tools!=0&&tools!=2)||remaining<1||parent.points.size()<3||parent.points.size()>128||parent.elements.size()>64||
       depth+static_cast<size_t>(remaining)>numeric_limits<uint16_t>::max())return false;
    Graph graph=parent;graph.SetStateHashingEnabled(false);const Mark root=graph.GetMark();
    auto stop=[&]() {
        if(progress)progress->Publish(stats,static_cast<uint16_t>(graph.elements.size()-graph.initialElementCount));
        return control.stop.load(memory_order_acquire)||Clock::now()>=deadline;
    };
    auto apply=[&](uint32_t a,uint32_t b,bool circle) {
        if(stop()||a==b||SamePoint(graph.points[a],graph.points[b]))return false;
        ++stats.rawCandidates;++counts.candidates;
        const Element e=graph.MakeCandidate(Pair(a,b,circle));
        if(!Valid(e))return false;
        if(graph.HasElement(e))return true;
        if(graph.elements.size()-parent.elements.size()>=static_cast<size_t>(remaining))return false;
        if(!ApplyFiniteProbe(graph,e,static_cast<uint16_t>(graph.elements.size()-graph.initialElementCount+1)))return false;
        ++stats.applied;++stats.nodes;++stats.uniqueCandidates;++counts.applied;
        stats.maxPoints=max(stats.maxPoints,graph.points.size());stats.maxElements=max(stats.maxElements,graph.elements.size());
        return true;
    };
    auto findPoint=[&](Point p){for(uint32_t i=0;i<graph.points.size();++i)if(SamePoint(p,graph.points[i]))return i;return NO_BOUND;};
    size_t axes=0,proposals=0;
    for(const Element& axis:parent.elements) {
        if(stop())return false;
        if(axis.type==Type::Circle)continue;
        if(++axes>24)break;
        const double denom=Sq(axis.a)+Sq(axis.b);if(!isfinite(denom)||denom<=0.0)continue;
        vector<uint32_t> anchors;
        for(uint32_t i=0;i<parent.points.size()&&anchors.size()<16;++i)
            if(parent.PointOnElement(parent.points[i],axis))anchors.push_back(i);
        if(anchors.size()<2)continue;
        vector<uint32_t> selected;
        for(uint32_t i=0;i<parent.points.size();++i) {
            bool useful=false;
            for(Point goal:parent.goalPoints)if(SamePoint(parent.points[i],goal)){useful=true;break;}
            if(!useful)for(const Element& goal:parent.goalElements)if(parent.PointOnElement(parent.points[i],goal)){useful=true;break;}
            if(useful)selected.push_back(i);
        }
        for(uint32_t i=0;i<parent.points.size()&&selected.size()<32;++i)
            if(find(selected.begin(),selected.end(),i)==selected.end())selected.push_back(i);
        if(selected.size()>32)selected.resize(32);
        for(uint32_t p:selected) {
            if(stop())return false;
            const Point point=parent.points[p];
            const double residual=axis.a*point.x+axis.b*point.y-axis.c;
            if(IsZero(residual))continue;
            const Point mirror{point.x-2*axis.a*residual/denom,point.y-2*axis.b*residual/denom};
            if(!isfinite(mirror.x)||!isfinite(mirror.y)||!parent.PointAllowed(mirror)||parent.HasPoint(mirror))continue;
            for(size_t a=0;a<anchors.size();++a)for(size_t b=a+1;b<anchors.size();++b) {
                if(stop()||++proposals>512)return false;
                graph.Rollback(root);
                const uint32_t i=anchors[a],j=anchors[b];
                if(!apply(i,p,true)||!apply(j,p,true))continue;
                uint32_t mirrored=findPoint(mirror);
                if(mirrored==NO_BOUND)continue;
                const Mark circles=graph.GetMark();
                int spent=static_cast<int>(graph.elements.size()-parent.elements.size());
                ++counts.callbacks;
                if(complete(static_cast<const Graph&>(graph),remaining-spent)){++counts.successes;return true;}
                if(tools==2&&remaining>spent&&apply(p,mirrored,false)) {
                    ++counts.callbacks;
                    spent=static_cast<int>(graph.elements.size()-parent.elements.size());
                    if(complete(static_cast<const Graph&>(graph),remaining-spent)){++counts.successes;return true;}
                }
                graph.Rollback(circles);
            }
        }
    }
    return false;
}
} // namespace bs::mirror_detail
