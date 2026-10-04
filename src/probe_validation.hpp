#pragma once
#include "geometry.hpp"

namespace bs {
inline size_t ValidateProbeParent(const Graph& graph) {
    if(graph.initialElementCount>graph.elements.size()||graph.points.size()!=graph.pointBirth.size())
        throw invalid_argument("probe parent count/birth mismatch");
    const size_t depth=graph.elements.size()-graph.initialElementCount;
    if(depth>numeric_limits<uint16_t>::max())throw invalid_argument("probe paid depth overflow");
    for(size_t i=0;i<graph.points.size();++i) {
        if(graph.pointBirth[i]>depth||(i&&graph.pointBirth[i]<graph.pointBirth[i-1]))
            throw invalid_argument("probe parent births are inconsistent");
        if(!isfinite(graph.points[i].x)||!isfinite(graph.points[i].y))
            throw invalid_argument("probe parent contains nonfinite point");
    }
    for(Point p:graph.goalPoints)if(!isfinite(p.x)||!isfinite(p.y))
        throw invalid_argument("probe target point is nonfinite");
    auto finite=[](const Element& e){return isfinite(e.a)&&isfinite(e.b)&&isfinite(e.c);};
    for(const Element& e:graph.elements)if(!finite(e))throw invalid_argument("probe parent element is nonfinite");
    for(const Element& e:graph.goalElements)if(!finite(e))throw invalid_argument("probe target element is nonfinite");
    return depth;
}
inline bool ApplyFiniteProbe(Graph& graph,const Element& e,uint16_t birth) {
    const Mark before=graph.GetMark();
    if(!graph.Apply(e,birth))return false;
    for(size_t i=before.pointCount;i<graph.points.size();++i) {
        const Point& p=graph.points[i];
        if(!isfinite(p.x)||!isfinite(p.y)||!graph.PointAllowed(p)) {
            graph.Rollback(before);
            return false;
        }
    }
    return true;
}
}
