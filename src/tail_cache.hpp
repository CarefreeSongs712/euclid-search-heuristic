#pragma once
#include "geometry.hpp"

namespace bs {

// A cache belongs to one DFS parent, never to a hash of an unordered state.
// Child operations append points, so the parent's ordered prefix and every
// incidence evaluated on it are unchanged until the parent returns.
class TailPrefixCache {
public:
    struct Entry {
        Element line, circle;
        uint32_t first = NO_BOUND, second = NO_BOUND;
        uint32_t circumference = NO_BOUND;
        bool lineValid = false, circleValid = false, duplicateLine = false;
    };
private:
    struct Target {
        Point point;
        vector<Entry> entries;
    };
    size_t prefixCount_ = 0;
    size_t usedTargets_ = 0;
    vector<Target> targets_;
public:
    void Reset(size_t prefixCount) {
        prefixCount_ = prefixCount;
        usedTargets_ = 0;
    }
    size_t PrefixCount() const { return prefixCount_; }

    const vector<Entry>& Get(const Graph& g, const Point& point, int toolType) {
        for (size_t t = 0; t < usedTargets_; ++t) {
            const Point& p = targets_[t].point;
            if (bit_cast<uint64_t>(p.x) == bit_cast<uint64_t>(point.x) &&
                bit_cast<uint64_t>(p.y) == bit_cast<uint64_t>(point.y))
                return targets_[t].entries;
        }
        if (usedTargets_ == targets_.size()) targets_.emplace_back();
        Target& target = targets_[usedTargets_++];
        target.point = point;
        target.entries.clear();
        target.entries.resize(prefixCount_);
        for (uint32_t q = 0; q < prefixCount_; ++q) {
            Entry& entry = target.entries[q];
            const Point& center = g.points[q];
            if (toolType != 0 && !SamePoint(point, center)) {
                entry.line = Element::FromPoints(point, center, Type::Line);
                entry.lineValid = true;
                for (uint32_t old = 0; old < q; ++old) {
                    const Entry& prior = target.entries[old];
                    if (prior.lineValid && !prior.duplicateLine &&
                        SameElement(prior.line, entry.line)) {
                        entry.duplicateLine = true;
                        break;
                    }
                }
                if (!entry.duplicateLine) {
                    for (uint32_t i = 0; i < prefixCount_; ++i) {
                        if (!g.PointOnElement(g.points[i], entry.line)) continue;
                        if (entry.first == NO_BOUND) entry.first = i;
                        else { entry.second = i; break; }
                    }
                }
            }
            if (toolType != 1) {
                const double r2 = Sq(center.x - point.x) + Sq(center.y - point.y);
                if (IsZero(r2)) continue;
                entry.circle = Element::FromCoefficients(center.x, center.y, r2, Type::Circle);
                entry.circleValid = true;
                for (uint32_t i = 0; i < prefixCount_; ++i) {
                    if (i != q && g.PointOnElement(g.points[i], entry.circle)) {
                        entry.circumference = i;
                        break;
                    }
                }
            }
        }
        return target.entries;
    }
};

} // namespace bs
