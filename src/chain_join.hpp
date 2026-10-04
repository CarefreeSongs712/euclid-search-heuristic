#pragma once
#include "rendezvous.hpp"

namespace bs::chain_join_detail {

struct Counts {
    uint64_t generated=0,proposals=0,replays=0,successes=0;
    size_t reachablePoints=0;
};
struct ElementBits {
    array<uint64_t,4> key;
    bool operator==(const ElementBits&) const=default;
};
struct ElementHash {
    size_t operator()(const ElementBits& e)const {
        return static_cast<size_t>(SplitMix64(e.key[0]^rotl(e.key[1],17)^rotl(e.key[2],39)^e.key[3]));
    }
};
inline bool Valid(const Element& e) {
    return isfinite(e.a)&&isfinite(e.b)&&isfinite(e.c)&&
        (e.type==Type::Circle?e.c>0.0:e.type==Type::Line&&(e.a!=0.0||e.b!=0.0));
}
inline ElementBits Key(const Element& e) {
    return {{static_cast<uint64_t>(e.type),bit_cast<uint64_t>(e.a),
             bit_cast<uint64_t>(e.b),bit_cast<uint64_t>(e.c)}};
}

// Backward proposal Q--A3--M--A2--H, where H has a one-operation witness.
// Q and M are virtual indexing points only. The accepted construction replays
// witness(H), line(A2,H), line(A3,M_actual), and optionally a final goal line.
inline bool Find(const Graph& parent,int remaining,int tools,SolutionCollector& collector,
                 ParallelControl& control,SearchStats& stats,Counts& counts,
                 chrono::steady_clock::time_point deadline,ProgressSlot* progress=nullptr) {
    using Clock=chrono::steady_clock;
    using namespace rendezvous_detail;
    if(parent.pointBirth.size()!=parent.points.size()||parent.initialElementCount>parent.elements.size())
        throw invalid_argument("chain join received inconsistent parent state");
    const size_t depth=parent.elements.size()-parent.initialElementCount;
    if(depth+static_cast<size_t>(max(0,remaining))>numeric_limits<uint16_t>::max())
        throw invalid_argument("chain join depth exceeds birth range");
    if(tools<0||tools>2)throw invalid_argument("chain join tool mode");
    if(control.stop.load(memory_order_acquire)||Clock::now()>=deadline)return false;
    if(tools==0||remaining<3||parent.points.size()<2||parent.points.size()>128||parent.elements.size()>96)
        return false;
    const uint32_t n=static_cast<uint32_t>(parent.points.size());
    uint64_t polls=0;
    bool cancelled=false;
    auto stop=[&](bool force=false) {
        if(cancelled)return true;
        if(!force && (++polls&255u))return false;
        if(progress)progress->Publish(stats,static_cast<uint16_t>(depth));
        cancelled=control.stop.load(memory_order_acquire)||Clock::now()>=deadline;
        return cancelled;
    };
    vector<Point> targets;
    for(const Point& p:parent.goalPoints) {
        if(stop())return false;
        if(parent.HasPoint(p))continue;
        if(isfinite(p.x)&&isfinite(p.y)&&parent.PointAllowed(p))targets.push_back(p);
        if(targets.size()>=8)break;
    }
    if(targets.empty())return false;
    const size_t cap=min<size_t>(32768,(16*1024*1024)/(sizeof(Direction)*n));
    vector<Reachable> reach;
    reach.reserve(min<size_t>(cap,4096));
    unordered_set<PointBits,PointBitsHash> pointsSeen;
    unordered_set<ElementBits,ElementHash> elementSeen;
    constexpr size_t maxOps=8192;
    for(uint32_t i=0;i<n;++i)for(uint32_t j=i+1;j<n;++j) {
        if(stop())return false;
        for(uint8_t tool=0;tool<3;++tool) {
            if((tools==1&&tool!=2)||(tools==0&&tool==2))continue;
            if(stop())return false;
            ++stats.rawCandidates;++counts.generated;
            const Element e=parent.MakeCandidate({i,j,tool});
            if(!Valid(e)||parent.HasElement(e))continue;
            if(elementSeen.size()>=maxOps||reach.size()>=cap)break;
            if(!elementSeen.insert(Key(e)).second)continue;
            ++stats.uniqueCandidates;
            for(const Element& old:parent.elements) {
                if(stop())return false;
                parent.VisitIntersections(e,old,[&](const Point& p) {
                    if(stop())return true;
                    if(reach.size()>=cap)return true;
                    if(!isfinite(p.x)||!isfinite(p.y)||parent.HasPoint(p))return false;
                    if(pointsSeen.insert({bit_cast<uint64_t>(p.x),bit_cast<uint64_t>(p.y)}).second)
                        reach.push_back({p,e});
                    return false;
                });
            }
        }
    }
    counts.reachablePoints=reach.size();
    if(reach.empty()||stop(true))return false;
    vector<vector<Direction>> index(n);
    for(uint32_t anchor=0;anchor<n;++anchor) {
        if(stop(true))return false;
        auto& row=index[anchor];row.reserve(reach.size());
        for(uint32_t i=0;i<reach.size();++i) {
            if(stop())return false;
            row.push_back({DirectionOf(reach[i].point,parent.points[anchor]),i});
        }
        sort(row.begin(),row.end(),[](const Direction&a,const Direction&b){
            return a.angle<b.angle||(a.angle==b.angle&&a.reachable<b.reachable);
        });
    }
    Graph graph=parent;
    graph.SetStateHashingEnabled(false);
    const Mark root=graph.GetMark();
    auto applied=[&]() {
        ++stats.applied;++stats.nodes;
        stats.maxPoints=max(stats.maxPoints,graph.points.size());
        stats.maxElements=max(stats.maxElements,graph.elements.size());
    };
    auto submit=[&]() {
        if(stop(true)||!graph.GoalsMet())return false;
        if(stop(true))return false;
        ++counts.successes;
        return collector.Submit(graph,&control);
    };
    auto replay=[&](const Reachable& source,uint32_t a2,uint32_t a3,const Element& carrier,const Point& virtualM) {
        if(stop(true))return false;
        ++counts.replays;
        graph.Rollback(root);
        if(!graph.Apply(source.witness,static_cast<uint16_t>(depth+1)))return false;
        applied();
        const uint32_t h=FindPoint(graph,source.point);
        if(h==NO_BOUND||h==a2)return false;
        const Element second=Element::FromPoints(graph.points[a2],graph.points[h],Type::Line);
        if(!Valid(second)||!graph.Apply(second,static_cast<uint16_t>(depth+2)))return false;
        applied();
        // Match the predicted intersection to an ACTUALLY produced known point.
        // If numerical ordering shifts it beyond EPS this proposal is declined.
        const uint32_t m=FindPoint(graph,virtualM);
        if(m==NO_BOUND||m==a3||!graph.PointOnElement(graph.points[m],carrier))return false;
        const Element third=Element::FromPoints(graph.points[a3],graph.points[m],Type::Line);
        if(!Valid(third)||!graph.Apply(third,static_cast<uint16_t>(depth+3)))return false;
        applied();
        if(submit())return true;
        if(remaining<4)return false;
        const Mark three=graph.GetMark();
        for(const Element& goal:graph.goalElements) {
            if(stop())return false;
            if(goal.type!=Type::Line||graph.HasElement(goal))continue;
            vector<uint32_t> hits;
            graph.VisitPointIncidences(goal,[&](uint32_t i){hits.push_back(i);});
            for(size_t i=0;i<hits.size();++i)for(size_t j=i+1;j<hits.size();++j) {
                if(stop())return false;
                ++stats.rawCandidates;
                const Element last=graph.MakeCandidate({hits[i],hits[j],2});
                if(!Valid(last)||!SameElement(last,goal))continue;
                if(graph.Apply(last,static_cast<uint16_t>(depth+4))) {
                    applied();
                    if(submit())return true;
                    graph.Rollback(three);
                }
            }
        }
        return false;
    };
    constexpr double pi=3.1415926535897932384626433832795;
    const double window=min(1e-4,max(1e-8,1000*EPS));
    for(const Point& target:targets)for(uint32_t a3=0;a3<n;++a3) {
        if(stop())return false;
        if(SamePoint(target,parent.points[a3]))continue;
        const Element virtualThird=Element::FromPoints(target,parent.points[a3],Type::Line);
        for(const Element& carrier:parent.elements) {
            if(stop())return false;
            bool found=false;
            parent.VisitIntersections(virtualThird,carrier,[&](const Point& virtualM) {
                if(stop())return true;
                if(!isfinite(virtualM.x)||!isfinite(virtualM.y)||SamePoint(virtualM,target)||
                   parent.HasPoint(virtualM))return false;
                for(uint32_t a2=0;a2<n;++a2) {
                    if(stop())return true;
                    if(a2==a3||SamePoint(parent.points[a2],virtualM))continue;
                    const double direction=DirectionOf(virtualM,parent.points[a2]);
                    const auto& row=index[a2];
                    size_t matches=0;
                    for(int wrap=-1;wrap<=1;++wrap) {
                        const double middle=direction+wrap*pi;
                        auto it=lower_bound(row.begin(),row.end(),middle-window,
                            [](const Direction& d,double a){return d.angle<a;});
                        for(;it!=row.end()&&it->angle<=middle+window&&matches<32;++it) {
                            if(stop())return true;
                            ++matches;++counts.proposals;
                            if(replay(reach[it->reachable],a2,a3,carrier,virtualM)){
                                found=true;return true;
                            }
                        }
                    }
                }
                return false;
            });
            if(found)return true;
        }
    }
    return false;
}

} // namespace bs::chain_join_detail
