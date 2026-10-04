#pragma once
#include "geometry.hpp"
#include "progress.hpp"

namespace bs::equal_radius_detail {

struct Counts {
    uint64_t generated = 0, prefixes = 0, replays = 0, successes = 0;
    uint64_t firstCircles = 0, reciprocalCircles = 0, carriers = 0;
    uint64_t proposals = 0, supplies = 0, callbacks = 0, capHits = 0;
    bool budgetLimited = false;
};

// All caps/orderings are HEURISTICS, not equivalences or completeness claims.
inline constexpr size_t MaxRootPoints = 8;
inline constexpr size_t MaxParentPoints = 128;
inline constexpr size_t MaxParentElements = 16;
inline constexpr size_t MaxFirstCircles = 32;
inline constexpr size_t MaxNewCenters = 8;
inline constexpr size_t MaxCarrierLines = 128;
inline constexpr size_t MaxSupplyOperations = 4096;
inline constexpr size_t MaxSupplyProposals = 64;
inline constexpr size_t MaxCallbacks = 32;
inline constexpr size_t MaxGoals = 16;

inline bool LegalFinite(const Element& e) {
    return isfinite(e.a) && isfinite(e.b) && isfinite(e.c) &&
        (e.type == Type::Circle ? e.c > 0.0 :
         e.type == Type::Line && (e.a != 0.0 || e.b != 0.0));
}
inline Candidate KnownPair(uint32_t first, uint32_t second, bool circle) {
    // Keep the same endpoint arithmetic as ordinary i<j candidate generation;
    // this also makes the cold-path bit-exact certificate witness discoverable.
    if (first < second) return {first, second, static_cast<uint8_t>(circle ? 0 : 2)};
    return {second, first, static_cast<uint8_t>(circle ? 1 : 2)};
}

// Bounded equal-radius chain, operating on a legally PAID parent (often the
// sealed root). `remaining` is the additional E budget AFTER that parent.
// Ordinary known-pair operations only:
//   circle(O,B), circle(R,O), line(A,D), circle(S,R), goal-point supply.
// R is really born on the first circle; S is really born on the reciprocal
// circle after the line. The first four operations therefore propagate radius
// using point incidences, never a free compass/radius-transfer operation.
//
// complete(const Graph&, int remaining) receives the ACTUAL five-op state,
// and returns true ONLY when its solution quota has been reached. It must use
// ordinary known-pair operations, check GoalsMet(), and own its tail budget.
// No blanket three-level tail DFS is launched here. A solved four-op state may
// also be handed off. successes counts callback quota acknowledgements, not
// successful collector visits (which are the callback's responsibility).
//
// Deadline covers this entire probe; callbacks get no automatic time extension.
// The caller budgets each callback separately. This code caps its own deadline
// at two seconds and, when global.deadline is set, 10% of the time still left.
// A local timeout/cap is Unknown and NEVER writes global.stop/timedOut/found.
// Existing births/initialElementCount are kept, never resealed or renumbered.
// The input graph is const, and callback graph references live only in the call.
template<class Complete>
bool Probe(const Graph& parent, int remaining, int tools, ParallelControl& global,
           SearchStats& stats, Counts& counts, chrono::steady_clock::time_point deadline,
           Complete&& complete, ProgressSlot* progress = nullptr) {
    using Clock = chrono::steady_clock;
    const auto start = Clock::now();
    if (global.stop.load(memory_order_acquire) || start >= deadline) return false;
    if (tools < 0 || tools > 2) throw invalid_argument("equal-radius probe tool mode");
    if (parent.initialElementCount > parent.elements.size() ||
        parent.pointBirth.size() != parent.points.size())
        throw invalid_argument("equal-radius probe requires a consistent paid parent");
    const size_t depth = parent.elements.size() - parent.initialElementCount;
    if (depth > numeric_limits<uint16_t>::max())
        throw invalid_argument("equal-radius probe parent depth exceeds birth range");
    for (size_t i = 0; i < parent.pointBirth.size(); ++i) {
        if (parent.pointBirth[i] > depth || (i && parent.pointBirth[i] < parent.pointBirth[i - 1]))
            throw invalid_argument("equal-radius probe requires ordered parent births");
    }
    if (tools != 2 || remaining < 6 || parent.points.size() < 2 ||
        parent.points.size() > MaxParentPoints || parent.elements.size() > MaxParentElements ||
        static_cast<size_t>(remaining) > numeric_limits<uint16_t>::max() - depth ||
        (parent.goalPoints.empty() && parent.goalElements.empty())) return false;

    vector<uint32_t> roots;
    for (uint32_t i = 0; i < parent.points.size() && parent.pointBirth[i] == 0; ++i)
        roots.push_back(i);
    if (roots.size() < 2 || roots.size() > MaxRootPoints) return false;
    deadline = min(deadline, start + chrono::seconds(2));
    if (global.deadline != Clock::time_point{}) {
        if (start >= global.deadline) return false;
        deadline = min(deadline, start + (global.deadline - start) / 10);
    }

    Graph graph = parent;
    graph.SetStateHashingEnabled(false);
    graph.Reserve(parent.elements.size() + 5);
    const Mark root = graph.GetMark();
    bool cancelled = false;
    uint64_t ticks = 0;
    size_t callbacks = 0;
    auto cap = [&] { ++counts.capHits; counts.budgetLimited = true; };
    auto stop = [&](bool force = false) {
        if (cancelled) return true;
        if (callbacks >= MaxCallbacks) { cap(); cancelled = true; return true; }
        if (!force && (++ticks & 63u)) return false;
        if (progress) progress->Publish(stats, static_cast<uint16_t>(
            graph.elements.size() - graph.initialElementCount));
        cancelled = global.stop.load(memory_order_acquire) || Clock::now() >= deadline;
        if (cancelled && !global.stop.load(memory_order_acquire)) counts.budgetLimited = true;
        return cancelled;
    };
    auto make = [&](Candidate candidate) {
        ++stats.rawCandidates; ++counts.generated;
        return graph.MakeCandidate(candidate);
    };
    auto apply = [&](Candidate candidate) {
        if (stop(true) || candidate.i >= graph.points.size() || candidate.j >= graph.points.size() ||
            candidate.i == candidate.j || SamePoint(graph.points[candidate.i], graph.points[candidate.j]))
            return false;
        const Element e = make(candidate);
        if (!LegalFinite(e)) return false;
        if (graph.HasElement(e)) { ++stats.existingCandidates; return false; }
        if (!graph.Apply(e, static_cast<uint16_t>(graph.elements.size() - graph.initialElementCount + 1)))
            return false;
        ++stats.uniqueCandidates; ++stats.applied; ++stats.nodes;
        stats.maxPoints = max(stats.maxPoints, graph.points.size());
        stats.maxElements = max(stats.maxElements, graph.elements.size());
        return true;
    };
    auto handoff = [&] {
        if (stop(true)) return false;
        if (callbacks >= MaxCallbacks) { cap(); return false; }
        ++callbacks; ++counts.callbacks;
        const int spent = static_cast<int>(graph.elements.size() - parent.elements.size());
        const bool quota = complete(static_cast<const Graph&>(graph), remaining - spent);
        if (quota) { ++counts.successes; return true; }
        stop(true);
        return false;
    };

    // Missing explicit goals are stronger proposals than incidental points on
    // a target element. The latter also supports element-only input problems.
    vector<Point> targetPoints;
    vector<Element> targetElements;
    for (const Point& p : parent.goalPoints) {
        if (stop()) return false;
        if (!isfinite(p.x) || !isfinite(p.y) || !parent.PointAllowed(p) || parent.HasPoint(p)) continue;
        if (targetPoints.size() >= MaxGoals) { cap(); break; }
        targetPoints.push_back(p);
    }
    for (const Element& e : parent.goalElements) {
        if (stop()) return false;
        if (parent.HasElement(e)) continue;
        if (targetElements.size() >= MaxGoals) { cap(); break; }
        targetElements.push_back(e);
        if (e.type == Type::Circle && !parent.HasPoint({e.a, e.b}) && targetPoints.size() < MaxGoals)
            targetPoints.push_back({e.a, e.b});
    }
    if ((targetPoints.empty() && targetElements.empty()) || stop(true)) return false;

    struct Supply { Candidate candidate; int priority; };
    auto supplyGoalPoint = [&](size_t newBegin) {
        const Mark four = graph.GetMark();
        if (stop(true)) return false;
        if (graph.GoalsMet()) return handoff();
        vector<Point> missing;
        vector<Element> missingElements;
        for (Point p : targetPoints) if (!graph.HasPoint(p)) missing.push_back(p);
        for (Element e : targetElements) if (!graph.HasElement(e)) missingElements.push_back(e);
        auto priority = [&](Point p) {
            if (!isfinite(p.x) || !isfinite(p.y) || !graph.PointAllowed(p)) return 0;
            for (Point goal : missing) if (SamePoint(p, goal)) return 2;
            for (const Element& goal : missingElements) if (graph.PointOnElement(p, goal)) return 1;
            return 0;
        };
        vector<Supply> proposals;
        vector<Candidate> candidates;
        candidates.reserve(MaxSupplyOperations);
        // Prioritize a newly generated Q joined to any root anchor. The full
        // ordinary known-pair pass below remains available within its cap.
        bool candidateCap = false;
        for (uint32_t q = static_cast<uint32_t>(newBegin); q < graph.points.size() && !candidateCap; ++q)
            for (uint32_t anchor : roots) {
                if (stop()) return false;
                if (candidates.size() >= MaxSupplyOperations) { cap(); candidateCap = true; break; }
                candidates.push_back(KnownPair(q, anchor, false));
            }
        bool full = candidateCap;
        for (uint32_t i = 0; i < graph.points.size() && !full; ++i)
            for (uint32_t j = i + 1; j < graph.points.size() && !full; ++j) {
                if (stop()) return false;
                for (uint8_t tool = 0; tool < 3; ++tool) {
                    if (candidates.size() >= MaxSupplyOperations) { cap(); full = true; break; }
                    // Preferred new-point/root lines were already included.
                    if (tool == 2 && j >= newBegin && graph.pointBirth[i] == 0) continue;
                    candidates.push_back({i, j, tool});
                }
            }
        for (Candidate candidate : candidates) {
            if (stop()) return false;
            if (SamePoint(graph.points[candidate.i], graph.points[candidate.j])) continue;
            const Element e = make(candidate);
            if (!LegalFinite(e)) continue;
            if (graph.HasElement(e)) { ++stats.existingCandidates; continue; }
            bool pointDirected = false;
            for (Point p : missing) if (graph.PointOnElement(p, e)) { pointDirected = true; break; }
            if (!pointDirected && missingElements.empty()) continue;
            int score = 0;
            for (const Element& old : graph.elements) {
                if (stop()) return false;
                graph.VisitIntersections(e, old, [&](Point p) {
                    if (stop()) return true;
                    if (!graph.HasPoint(p)) score = max(score, priority(p));
                    return score == 2;
                });
                if (score == 2) break;
            }
            if (score == 0) continue;
            ++counts.proposals;
            proposals.push_back({candidate, score});
            if (proposals.size() >= MaxSupplyProposals) { cap(); break; }
        }
        // Stable priority preserves ordinary pair/real-new-point order.
        stable_sort(proposals.begin(), proposals.end(), [](const Supply& a, const Supply& b) {
            return a.priority > b.priority;
        });
        for (const Supply& proposal : proposals) {
            if (stop(true) || callbacks >= MaxCallbacks) return false;
            graph.Rollback(four);
            ++counts.replays;
            if (!apply(proposal.candidate)) continue;
            bool supplied = false;
            for (size_t i = four.pointCount; i < graph.points.size(); ++i)
                if (priority(graph.points[i]) != 0) { supplied = true; break; }
            // A preview is not evidence: only actual Apply-born points qualify.
            if (supplied) {
                ++counts.supplies;
                if (handoff()) return true;
            }
        }
        graph.Rollback(four);
        return false;
    };

    size_t firstCount = 0;
    for (uint32_t center : roots) for (uint32_t through : roots) {
        if (stop(true) || callbacks >= MaxCallbacks) return false;
        if (center == through) continue;
        if (firstCount++ >= MaxFirstCircles) { cap(); return false; }
        graph.Rollback(root);
        if (!apply(KnownPair(center, through, true))) continue;
        ++counts.firstCircles;
        const Element first = graph.elements.back();
        const Mark one = graph.GetMark();
        size_t rCount = 0;
        for (uint32_t r = static_cast<uint32_t>(root.pointCount); r < one.pointCount; ++r) {
            if (stop(true) || callbacks >= MaxCallbacks) return false;
            graph.Rollback(one);
            if (!graph.PointOnElement(graph.points[r], first)) continue;
            if (rCount++ >= MaxNewCenters) { cap(); break; }
            if (!apply(KnownPair(r, center, true))) continue;
            ++counts.reciprocalCircles;
            const Element second = graph.elements.back();
            const Mark two = graph.GetMark();
            vector<Candidate> carriers;
            // Root-root carriers first, then other real known-root pairs.
            for (size_t i = 0; i < roots.size(); ++i)
                for (size_t j = i + 1; j < roots.size(); ++j)
                    carriers.push_back(KnownPair(roots[i], roots[j], false));
            for (uint32_t q = 0; q < two.pointCount && carriers.size() < MaxCarrierLines; ++q) {
                if (graph.pointBirth[q] == 0) continue;
                for (uint32_t anchor : roots) {
                    if (carriers.size() >= MaxCarrierLines) { cap(); break; }
                    carriers.push_back(KnownPair(q, anchor, false));
                }
            }
            for (Candidate carrier : carriers) {
                if (stop(true) || callbacks >= MaxCallbacks) return false;
                graph.Rollback(two);
                if (!apply(carrier)) continue;
                ++counts.carriers;
                const Mark three = graph.GetMark();
                size_t sCount = 0;
                for (uint32_t s = static_cast<uint32_t>(two.pointCount); s < three.pointCount; ++s) {
                    if (stop(true) || callbacks >= MaxCallbacks) return false;
                    graph.Rollback(three);
                    if (!graph.PointOnElement(graph.points[s], second)) continue;
                    if (sCount++ >= MaxNewCenters) { cap(); break; }
                    if (!apply(KnownPair(s, r, true))) continue;
                    ++counts.prefixes;
                    if (supplyGoalPoint(three.pointCount)) return true;
                }
            }
        }
    }
    return false; // Unknown, including callback solutions short of the quota.
}

} // namespace bs::equal_radius_detail
