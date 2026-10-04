#pragma once
#include "solver.hpp"

namespace bs::goal_finish_detail {

struct Counts {
    uint64_t candidates = 0, applied = 0, successes = 0;
    uint64_t goalCandidates = 0, auxiliaryCandidates = 0, replays = 0;
    uint64_t goalApplied = 0, auxiliaryApplied = 0, previews = 0;
    uint64_t capHits = 0, gated = 0, timeouts = 0, cancellations = 0;
    bool budgetLimited = false;
};

inline constexpr int MaxRemaining = 6;
inline constexpr size_t MaxGoals = 8;
inline constexpr size_t MaxParentPoints = 128;
inline constexpr size_t MaxParentElements = 96;
inline constexpr uint64_t MaxAuxiliaryCandidates = 4096;
inline constexpr uint64_t MaxGoalCandidates = 100000;

inline bool LegalFinite(const Element& e) {
    return isfinite(e.a) && isfinite(e.b) && isfinite(e.c) &&
        (e.type == Type::Circle ? e.c > 0.0 :
         e.type == Type::Line && (e.a != 0.0 || e.b != 0.0));
}
inline Candidate KnownPair(uint32_t first, uint32_t second, bool circle) {
    if (first < second) return {first, second, static_cast<uint8_t>(circle ? 0 : 2)};
    return {second, first, static_cast<uint8_t>(circle ? 1 : 2)};
}

// HEURISTIC caps/gates: false is Unknown, never a reason to prune the caller.
// Try goal-only DFS, then ONE parent-known-pair operation followed by goal-only
// DFS. All verified goal representations are tried until the shared cap/deadline.
// True means collector quota reached; successes counts submitted visits (including
// duplicates), not unique solutions. applied includes actual preview Apply calls.
// Preserve the paid parent's initialElementCount/births; never reseal or mutate it.
inline bool Find(const Graph& parent, int remaining, int tools,
                 SolutionCollector& collector, ParallelControl& global,
                 SearchStats& stats, Counts& counts,
                 chrono::steady_clock::time_point deadline, ProgressSlot* progress = nullptr) {
    using Clock = chrono::steady_clock;
    if (global.stop.load(memory_order_acquire)) { ++counts.cancellations; return false; }
    if (global.deadline != Clock::time_point{}) deadline = min(deadline, global.deadline);
    if (Clock::now() >= deadline) { ++counts.timeouts; counts.budgetLimited = true; return false; }
    if (tools < 0 || tools > 2 || remaining < 0)
        throw invalid_argument("goal finish tool mode/remaining budget");
    if (parent.initialElementCount > parent.elements.size() ||
        parent.pointBirth.size() != parent.points.size())
        throw invalid_argument("goal finish requires a consistent paid parent");
    const size_t depth = parent.elements.size() - parent.initialElementCount;
    if (depth > numeric_limits<uint16_t>::max() ||
        static_cast<size_t>(remaining) > numeric_limits<uint16_t>::max() - depth)
        throw invalid_argument("goal finish paid depth plus remaining exceeds birth range");
    if (remaining > MaxRemaining || parent.points.size() > MaxParentPoints ||
        parent.elements.size() > MaxParentElements || parent.goalElements.size() > MaxGoals ||
        parent.goalPoints.size() > MaxGoals - parent.goalElements.size() ||
        (parent.goalElements.empty() && parent.goalPoints.empty())) {
        ++counts.gated;
        return false;
    }
    for (size_t i = 0; i < parent.pointBirth.size(); ++i)
        if (parent.pointBirth[i] > depth || (i && parent.pointBirth[i] < parent.pointBirth[i - 1]))
            throw invalid_argument("goal finish requires ordered parent point births");

    Graph graph = parent;
    graph.SetStateHashingEnabled(false);
    graph.elements.reserve(parent.elements.size() + static_cast<size_t>(remaining));
    graph.points.reserve(parent.points.size() + 2 * static_cast<size_t>(remaining) *
                         (parent.elements.size() + static_cast<size_t>(remaining)));
    graph.pointBirth.reserve(graph.points.capacity());
    const Mark root = graph.GetMark();
    bool cancelled = false;
    uint64_t ticks = 0, goalOperations = 0, auxiliaryOperations = 0;
    auto stop = [&](bool force = false) {
        if (cancelled) return true;
        if (!force && (++ticks & 63u)) return false;
        if (progress) progress->Publish(stats, static_cast<uint16_t>(
            graph.elements.size() - graph.initialElementCount));
        if (global.stop.load(memory_order_acquire)) {
            ++counts.cancellations;
            cancelled = true;
        } else if (Clock::now() >= deadline) {
            ++counts.timeouts;
            counts.budgetLimited = true;
            cancelled = true;
        }
        return cancelled;
    };
    auto cap = [&] { ++counts.capHits; counts.budgetLimited = true; };
    auto make = [&](Candidate candidate) -> optional<Element> {
        if (stop() || candidate.i >= graph.points.size() || candidate.j >= graph.points.size() ||
            candidate.i == candidate.j) return nullopt;
        const Point p = graph.points[candidate.i], q = graph.points[candidate.j];
        if (!isfinite(p.x) || !isfinite(p.y) || !isfinite(q.x) || !isfinite(q.y) ||
            !graph.PointAllowed(p) || !graph.PointAllowed(q) || SamePoint(p, q)) return nullopt;
        ++counts.candidates; ++stats.rawCandidates;
        const Element e = graph.MakeCandidate(candidate);
        if (!LegalFinite(e)) return nullopt;
        return e;
    };
    auto apply = [&](const Element& e, bool preview, bool goal) {
        if (stop(true)) return false;
        if (graph.HasElement(e)) { ++stats.existingCandidates; return false; }
        const Mark before = graph.GetMark();
        if (!graph.Apply(e, static_cast<uint16_t>(graph.elements.size() - graph.initialElementCount + 1)))
            return false;
        ++counts.applied; ++stats.uniqueCandidates; ++stats.applied; ++stats.nodes;
        if (preview) ++counts.previews;
        else if (goal) ++counts.goalApplied;
        else ++counts.auxiliaryApplied;
        stats.maxPoints = max(stats.maxPoints, graph.points.size());
        stats.maxElements = max(stats.maxElements, graph.elements.size());
        for (size_t i = before.pointCount; i < graph.points.size(); ++i) {
            const Point p = graph.points[i];
            if (stop() || !isfinite(p.x) || !isfinite(p.y) || !graph.PointAllowed(p)) {
                graph.Rollback(before);
                return false;
            }
        }
        return true;
    };
    auto goalOnly = [&](auto&& self, int left) -> bool {
        if (stop(true)) return false;
        if (graph.GoalsMet()) {
            if (stop(true)) return false;
            return collector.Submit(graph, &global, &counts.successes);
        }
        if (left == 0) return false;
        const Mark before = graph.GetMark();
        auto attempt = [&](Candidate candidate, const Element& goal) {
            if (stop()) return false;
            if (goalOperations >= MaxGoalCandidates) { cap(); cancelled = true; return false; }
            ++goalOperations; ++counts.goalCandidates;
            const auto e = make(candidate);
            if (!e || !SameElement(*e, goal) || !apply(*e, false, true)) return false;
            const bool quota = self(self, left - 1);
            graph.Rollback(before);
            return quota;
        };
        // Incidence is only a proposal filter; never Apply the goal's coefficients.
        for (const Element& goal : graph.goalElements) {
            if (stop()) return false;
            if (!LegalFinite(goal) || graph.HasElement(goal)) continue;
            if ((goal.type == Type::Line && tools == 0) ||
                (goal.type == Type::Circle && tools == 1)) continue;
            vector<uint32_t> rim, centers;
            for (uint32_t i = 0; i < graph.points.size(); ++i) {
                if (stop()) return false;
                if (graph.PointOnElement(graph.points[i], goal)) rim.push_back(i);
                if (goal.type == Type::Circle && SamePoint(graph.points[i], {goal.a, goal.b}))
                    centers.push_back(i);
            }
            if (goal.type == Type::Line) {
                for (size_t i = 0; i < rim.size(); ++i)
                    for (size_t j = i + 1; j < rim.size(); ++j) {
                        if (stop()) return false;
                        if (attempt(KnownPair(rim[i], rim[j], false), goal)) return true;
                    }
            } else {
                for (uint32_t center : centers) for (uint32_t through : rim) {
                    if (stop()) return false;
                    if (center != through && attempt(KnownPair(center, through, true), goal)) return true;
                }
            }
        }
        return false;
    };
    if (goalOnly(goalOnly, remaining)) return true;
    if (remaining == 0 || stop(true)) return false;

    // Rank only actual Apply states. Incidence/readiness scores are ordering
    // hints, never evidence of a solution or a reason to discard a candidate.
    auto score = [&] {
        int value = 0;
        for (Point p : graph.goalPoints) {
            if (stop()) return value;
            if (graph.HasPoint(p)) value += 1024;
        }
        for (const Element& goal : graph.goalElements) {
            if (stop()) return value;
            if (graph.HasElement(goal)) { value += 1024; continue; }
            if (!LegalFinite(goal) || (goal.type == Type::Line && tools == 0) ||
                (goal.type == Type::Circle && tools == 1)) continue;
            int hits = 0;
            bool center = false;
            for (Point p : graph.points) {
                if (stop()) return value;
                if (graph.PointOnElement(p, goal)) ++hits;
                if (goal.type == Type::Circle && SamePoint(p, {goal.a, goal.b})) center = true;
            }
            value += min(hits, 8);
            if (goal.type == Type::Circle) value += center ? 16 : 0;
            if (goal.type == Type::Line ? hits >= 2 : center && hits >= 1) value += 128;
        }
        return value;
    };
    struct Proposal { Candidate candidate; int priority; };
    vector<Proposal> proposals;
    proposals.reserve(static_cast<size_t>(MaxAuxiliaryCandidates));
    bool full = false;
    for (uint32_t i = 0; i < root.pointCount && !full; ++i)
        for (uint32_t j = i + 1; j < root.pointCount && !full; ++j)
            for (uint8_t tool = tools == 1 ? 2 : 0; tool < (tools == 0 ? 2 : 3); ++tool) {
                if (stop()) return false;
                if (auxiliaryOperations >= MaxAuxiliaryCandidates) { cap(); full = true; break; }
                ++auxiliaryOperations; ++counts.auxiliaryCandidates;
                const Candidate candidate{i, j, tool};
                const auto e = make(candidate);
                if (!e || !apply(*e, true, false)) continue;
                const int priority = score();
                graph.Rollback(root);
                if (stop()) return false;
                proposals.push_back({candidate, priority});
            }
    if (stop(true)) return false;
    struct CancelledSort {};
    try {
        stable_sort(proposals.begin(), proposals.end(), [&](const Proposal& a, const Proposal& b) {
            if (stop()) throw CancelledSort{};
            return a.priority > b.priority;
        });
    } catch (const CancelledSort&) { return false; }
    for (const Proposal& proposal : proposals) {
        if (stop(true)) return false;
        ++counts.replays;
        const auto e = make(proposal.candidate);
        if (!e || !apply(*e, false, false)) continue;
        const bool quota = goalOnly(goalOnly, remaining - 1);
        graph.Rollback(root);
        if (quota) return true;
    }
    return false;
}

} // namespace bs::goal_finish_detail
