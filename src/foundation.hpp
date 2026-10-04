#pragma once
#include "geometry.hpp"

namespace bs::foundation_detail {

// Every macro is only a schedule of ordinary one-E operations. Validate the
// entire schedule before applying anything; failure leaves graph/paid unchanged.
inline bool ApplyPairScaffold(Graph& graph,uint32_t a,uint32_t b,int remaining,
                              SearchStats& stats,vector<Element>& paid) {
    if(graph.initialElementCount>graph.elements.size()||graph.pointBirth.size()!=graph.points.size())
        throw invalid_argument("scaffold requires a consistent paid parent");
    if(remaining<0||!paid.empty()||a>=graph.points.size()||b>=graph.points.size()||a==b)return false;
    const Point first=graph.points[a],second=graph.points[b];
    if(!isfinite(first.x)||!isfinite(first.y)||!isfinite(second.x)||!isfinite(second.y)||SamePoint(first,second))return false;
    const array<Element,3> sequence{
        Element::FromPoints(first,second,Type::Line),
        Element::FromPoints(first,second,Type::Circle),
        Element::FromPoints(second,first,Type::Circle)};
    vector<Element> additions;
    additions.reserve(3);
    for(const Element& e:sequence) {
        if(!isfinite(e.a)||!isfinite(e.b)||!isfinite(e.c)||
           (e.type==Type::Circle?e.c<=0.0:(e.a==0.0&&e.b==0.0)))return false;
        if(graph.HasElement(e))continue;
        bool duplicate=false;
        for(const Element& old:additions)if(SameElement(old,e)){duplicate=true;break;}
        if(!duplicate)additions.push_back(e);
    }
    const size_t depth=graph.elements.size()-graph.initialElementCount;
    if(additions.size()>static_cast<size_t>(remaining)||depth>numeric_limits<uint16_t>::max()-additions.size())return false;
    for(const Element& e:additions) {
        graph.ApplyKnownNew(e,static_cast<uint16_t>(graph.elements.size()-graph.initialElementCount+1));
        paid.push_back(e);
        ++stats.nodes;++stats.applied;
        stats.maxPoints=max(stats.maxPoints,graph.points.size());
        stats.maxElements=max(stats.maxElements,graph.elements.size());
    }
    return true;
}

} // namespace bs::foundation_detail
