#pragma once
#include "solver.hpp"
#include <unordered_set>

namespace bs::rendezvous_detail {

struct Counts { uint64_t proposals=0, replayed=0, successes=0; size_t reachablePoints=0; };
struct PointBits {
    uint64_t x,y;
    bool operator==(const PointBits&) const=default;
};
struct PointBitsHash {
    size_t operator()(const PointBits& x) const {return static_cast<size_t>(SplitMix64(x.x^rotl(x.y,27)));}
};
struct Reachable {Point point;Element witness;};
struct Direction {double angle;uint32_t reachable;};
inline double DirectionOf(Point a,Point b) {
    constexpr double pi=3.1415926535897932384626433832795;
    double v=atan2(a.y-b.y,a.x-b.x);
    if(v<0)v+=pi;
    if(v>=pi)v-=pi;
    return v;
}
inline uint32_t FindPoint(const Graph& g,Point p) {
    for(uint32_t i=0;i<g.points.size();++i) if(SamePoint(g.points[i],p)) return i;
    return NO_BOUND;
}
inline bool SameRaw(const Element& a,const Element& b) {
    return a.type==b.type && bit_cast<uint64_t>(a.a)==bit_cast<uint64_t>(b.a) &&
        bit_cast<uint64_t>(a.b)==bit_cast<uint64_t>(b.b) && bit_cast<uint64_t>(a.c)==bit_cast<uint64_t>(b.c);
}

// A bounded structural proposal search: two independent straightedge strokes,
// one joining a known anchor to their reachable point, then a missing goal line.
// Direction tolerances affect proposals ONLY. Every hit is replayed using real
// known points, ordinary Apply, and GoalsMet; a target coordinate is never free.
inline bool Find(const Graph& initial,int limit,int tools,SolutionCollector& solutions,
                 ParallelControl& global,SearchStats& stats,Counts& counts,
                 chrono::steady_clock::time_point deadline,ProgressSlot* progress=nullptr) {
    if(initial.initialElementCount!=initial.elements.size() || initial.pointBirth.size()!=initial.points.size())
        throw invalid_argument("rendezvous requires a sealed initial graph");
    if(global.stop.load(memory_order_acquire)||chrono::steady_clock::now()>=deadline)return false;
    if(limit<4 || tools==0 || initial.points.size()<2 || initial.points.size()>128 ||
       initial.elements.size()>128 || initial.goalElements.size()>8) return false;
    vector<Element> goals;
    for(const Element& e:initial.goalElements)
        if(e.type==Type::Line && !initial.HasElement(e))goals.push_back(e);
    if(goals.empty())return false;
    uint64_t polls=0;
    auto stopped=[&]() {
        if((++polls&255u)!=0)return false;
        if(progress)progress->Publish(stats,0);
        return global.stop.load(memory_order_acquire)||chrono::steady_clock::now()>=deadline;
    };
    constexpr size_t maxCandidates=8192;
    constexpr size_t indexByteBudget=32*1024*1024;
    const size_t maxReachable=min<size_t>(65536,indexByteBudget/(sizeof(Direction)*initial.points.size()));
    vector<Element> candidates;
    vector<Reachable> reachable;
    candidates.reserve(min(maxCandidates,initial.points.size()*initial.points.size()/2));
    reachable.reserve(16384);
    unordered_set<PointBits,PointBitsHash> pointSeen;
    pointSeen.reserve(16384);
    for(uint32_t i=0;i<initial.points.size();++i)for(uint32_t j=i+1;j<initial.points.size();++j) {
        if(stopped())return false;
        ++stats.rawCandidates;
        const Element e=initial.MakeCandidate({i,j,2});
        if(!isfinite(e.a)||!isfinite(e.b)||!isfinite(e.c)||initial.HasElement(e))continue;
        bool duplicate=false;
        for(const Element& old:candidates)if(SameRaw(old,e)){duplicate=true;break;}
        if(duplicate)continue;
        if(candidates.size()>=maxCandidates)break;
        candidates.push_back(e);
        ++stats.uniqueCandidates;
        if(reachable.size()>=maxReachable)continue;
        for(const Element& old:initial.elements) {
            if(stopped())return false;
            initial.VisitIntersections(e,old,[&](const Point& p) {
                if(reachable.size()>=maxReachable)return true;
                if(!isfinite(p.x)||!isfinite(p.y)||initial.HasPoint(p))return false;
                if(pointSeen.insert({bit_cast<uint64_t>(p.x),bit_cast<uint64_t>(p.y)}).second)
                    reachable.push_back({p,e});
                return false;
            });
        }
    }
    counts.reachablePoints=reachable.size();
    if(reachable.empty())return false;
    vector<vector<Direction>> index(initial.points.size());
    for(size_t i=0;i<index.size();++i) {
        auto& angles=index[i];angles.reserve(reachable.size());
        for(uint32_t j=0;j<reachable.size();++j) {
            if(stopped())return false;
            angles.push_back({DirectionOf(reachable[j].point,initial.points[i]),j});
        }
        sort(angles.begin(),angles.end(),[](const Direction&a,const Direction&b) {
            return a.angle<b.angle || (a.angle==b.angle&&a.reachable<b.reachable);
        });
    }
    Graph graph=initial;
    graph.SetStateHashingEnabled(false);
    const Mark root=graph.GetMark();
    auto replay=[&](const Element& first,const Reachable& second,uint32_t anchor) {
        if(global.stop.load(memory_order_acquire)||chrono::steady_clock::now()>=deadline)return false;
        ++counts.replayed;
        graph.Rollback(root);
        if(!graph.Apply(first,1))return false;
        ++stats.applied;
        if(!graph.Apply(second.witness,2))return false;
        ++stats.applied;
        const uint32_t point=FindPoint(graph,second.point);
        if(point==NO_BOUND||point==anchor)return false;
        const Element third=Element::FromPoints(graph.points[anchor],graph.points[point],Type::Line);
        if(!graph.Apply(third,3))return false;
        ++stats.applied;++stats.nodes;
        stats.maxPoints=max(stats.maxPoints,graph.points.size());
        stats.maxElements=max(stats.maxElements,graph.elements.size());
        if(graph.GoalsMet()) {
            ++counts.successes;
            return solutions.Submit(graph,&global);
        }
        const Mark three=graph.GetMark();
        for(const Element& goal:goals) {
            vector<uint32_t> hits;
            graph.VisitPointIncidences(goal,[&](uint32_t i){hits.push_back(i);});
            for(size_t a=0;a<hits.size();++a)for(size_t b=a+1;b<hits.size();++b) {
                if(stopped())return false;
                ++stats.rawCandidates;
                const Element last=graph.MakeCandidate({hits[a],hits[b],2});
                if(!SameElement(last,goal))continue;
                if(graph.Apply(last,4)) {
                    ++stats.applied;++stats.nodes;
                    if(graph.GoalsMet()) {
                        ++counts.successes;
                        if(solutions.Submit(graph,&global))return true;
                    }
                    graph.Rollback(three);
                }
            }
        }
        return false;
    };
    constexpr double pi=3.1415926535897932384626433832795;
    const double window=min(1e-4,max(1e-8,1000*EPS));
    for(const Element& first:candidates)for(const Element& goal:goals) {
        if(stopped())return false;
        bool solved=false;
        initial.VisitIntersections(first,goal,[&](const Point& target) {
            if(!isfinite(target.x)||!isfinite(target.y)||initial.HasPoint(target))return false;
            for(uint32_t anchor=0;anchor<initial.points.size();++anchor) {
                if(stopped())return true;
                if(SamePoint(target,initial.points[anchor]))continue;
                const double angle=DirectionOf(target,initial.points[anchor]);
                const auto& directions=index[anchor];
                size_t proposals=0;
                for(int wrap=-1;wrap<=1;++wrap) {
                    const double middle=angle+wrap*pi;
                    auto begin=lower_bound(directions.begin(),directions.end(),middle-window,
                        [](const Direction& d,double value){return d.angle<value;});
                    for(auto it=begin;it!=directions.end()&&it->angle<=middle+window&&proposals<64;++it) {
                        if(stopped())return true;
                        const Reachable& point=reachable[it->reachable];
                        if(SameRaw(point.witness,first))continue;
                        ++counts.proposals;++proposals;
                        if(replay(first,point,anchor)){solved=true;return true;}
                    }
                }
            }
            return false;
        });
        if(solved)return true;
        if(global.stop.load(memory_order_acquire)||chrono::steady_clock::now()>=deadline)return false;
    }
    return false;
}

} // namespace bs::rendezvous_detail
