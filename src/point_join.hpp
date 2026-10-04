#pragma once
#include "solver.hpp"
#include <unordered_set>

namespace bs::point_join_detail {

struct Counts {
    uint64_t generated = 0, proposals = 0, replays = 0, successes = 0;
    size_t points = 0; // Peak indexed points, including known (zero-cost) points.
};

inline constexpr size_t MaxRootPoints = 128;
inline constexpr size_t MaxRootElements = 64;
inline constexpr size_t MaxCandidates = 16384;
inline constexpr size_t MaxReachable = 65536;
inline constexpr double Pi = 3.1415926535897932384626433832795;

struct PointBits {
    uint64_t x, y;
    bool operator==(const PointBits&) const = default;
};
struct PointBitsHash {
    size_t operator()(const PointBits& p) const noexcept {
        return static_cast<size_t>(SplitMix64(p.x ^ rotl(p.y, 27)));
    }
};
struct ElementBits {
    uint64_t a, b, c;
    Type type;
    bool operator==(const ElementBits&) const = default;
};
struct ElementBitsHash {
    size_t operator()(const ElementBits& e) const noexcept {
        return static_cast<size_t>(SplitMix64(e.a ^ rotl(e.b, 21) ^ rotl(e.c, 42) ^
            static_cast<uint64_t>(e.type)));
    }
};
struct Reachable {
    Point point;
    uint32_t witness = NO_BOUND; // Index of a real parent-known Candidate; none for known points.
};
struct Direction {
    double angle;
    uint32_t reachable;
};

inline bool LegalFinite(const Element& e) {
    return isfinite(e.a) && isfinite(e.b) && isfinite(e.c) &&
        (e.type == Type::Circle ? e.c > 0.0 :
         e.type == Type::Line && (!IsZero(e.a) || !IsZero(e.b)));
}
inline uint32_t FindPoint(const Graph& graph, Point p) {
    for (uint32_t i = 0; i < graph.points.size(); ++i)
        if (SamePoint(graph.points[i], p)) return i;
    return NO_BOUND;
}
inline double DirectionOf(Point point, Point goal) {
    double angle = atan2(point.y - goal.y, point.x - goal.x);
    if (angle < 0.0) angle += Pi;
    if (angle >= Pi) angle -= Pi;
    return angle;
}

// Bounded HEURISTIC, not a reachability/unsolvability proof. Enumerate ordinary
// parent-known-pair operations and their intersections with current elements.
// Raw-bit point dedup keeps only one witness (alternate witnesses can be lost).
// Candidate/point/match caps and the angular proposal window are also incomplete.
// Goal coordinates are index keys ONLY: no predicted point/element is inserted.
//
// parent is an already legally paid prefix, NOT a resealed initial graph.
// depth must equal its paid E; preserve initialElementCount and all old births.
// True means collector quota reached, not merely one successful visit. A local
// deadline or external stop discards unfinished work and never sets timedOut.
inline bool Find(const Graph& parent, int remaining, int depth, int toolType,
                 SolutionCollector& collector, ParallelControl& global,
                 SearchStats& stats, Counts& counts,
                 chrono::steady_clock::time_point deadline, ProgressSlot* progress = nullptr) {
    if (global.stop.load(memory_order_acquire) || chrono::steady_clock::now() >= deadline)
        return false;
    if (remaining < 1 || toolType == 0 || toolType < 0 || toolType > 2 ||
        parent.points.size() < 2 || parent.points.size() > MaxRootPoints ||
        parent.elements.size() > MaxRootElements || parent.goalPoints.empty()) return false;
    const int budget = min(remaining, 3);
    if (parent.initialElementCount > parent.elements.size() ||
        parent.pointBirth.size() != parent.points.size() || depth < 0 ||
        static_cast<size_t>(depth) != parent.elements.size() - parent.initialElementCount)
        throw invalid_argument("point join requires a consistent paid parent prefix/depth");
    if (depth > numeric_limits<uint16_t>::max() - budget) return false;
    for (size_t i = 0; i < parent.pointBirth.size(); ++i)
        if (parent.pointBirth[i] > depth || (i && parent.pointBirth[i] < parent.pointBirth[i - 1]))
            throw invalid_argument("point join requires ordered parent point births");

    bool cancelled = false;
    uint64_t ticks = 0;
    uint16_t activeDepth = static_cast<uint16_t>(depth);
    auto stopNow = [&] {
        cancelled = cancelled || global.stop.load(memory_order_acquire) ||
                    chrono::steady_clock::now() >= deadline;
        if (progress) progress->Publish(stats, activeDepth);
        return cancelled;
    };
    auto poll = [&] { return cancelled || ((++ticks & 255u) == 0 && stopNow()); };
    optional<Point> goal;
    for (Point p : parent.goalPoints) {
        if (poll()) return false;
        if (isfinite(p.x) && isfinite(p.y) && !parent.HasPoint(p)) { goal = p; break; }
    }
    if (!goal || !parent.PointAllowed(*goal) || stopNow()) return false;

    vector<Candidate> candidates;
    vector<Reachable> reachable;
    vector<uint32_t> directWitnesses;
    unordered_set<PointBits, PointBitsHash> pointSeen;
    unordered_set<ElementBits, ElementBitsHash> candidateSeen;
    candidates.reserve(min(MaxCandidates, parent.points.size() * (parent.points.size() - 1) *
                           (toolType == 2 ? 3 : 1) / 2));
    reachable.reserve(4096);
    pointSeen.reserve(4096);
    candidateSeen.reserve(candidates.capacity());
    for (Point p : parent.points) {
        if (poll()) return false;
        if (!isfinite(p.x) || !isfinite(p.y)) continue;
        pointSeen.insert({bit_cast<uint64_t>(p.x), bit_cast<uint64_t>(p.y)});
        reachable.push_back({p, NO_BOUND});
    }
    // With one E left only the final line is affordable, so known points suffice.
    if (budget > 1) {
        bool full = false;
        for (uint32_t i = 0; i < parent.points.size() && !full; ++i)
            for (uint32_t j = i + 1; j < parent.points.size() && !full; ++j) {
                if (poll()) return false;
                if (SamePoint(parent.points[i], parent.points[j])) continue;
                for (uint8_t tool = toolType == 1 ? 2 : 0; tool <= 2; ++tool) {
                    if (poll()) return false;
                    if (candidates.size() >= MaxCandidates || reachable.size() >= MaxReachable) {
                        full = true; break;
                    }
                    ++stats.rawCandidates;
                    const Candidate candidate{i, j, tool};
                    const Element e = parent.MakeCandidate(candidate);
                    if (!LegalFinite(e)) continue;
                    if (parent.HasElement(e)) { ++stats.existingCandidates; continue; }
                    if (!candidateSeen.insert({bit_cast<uint64_t>(e.a), bit_cast<uint64_t>(e.b),
                                               bit_cast<uint64_t>(e.c), e.type}).second) {
                        ++stats.duplicateCandidates; continue;
                    }
                    const uint32_t witness = static_cast<uint32_t>(candidates.size());
                    candidates.push_back(candidate);
                    ++counts.generated; ++stats.uniqueCandidates;
                    bool createsGoal = false;
                    for (const Element& old : parent.elements) {
                        if (poll()) return false;
                        if (reachable.size() >= MaxReachable) break;
                        parent.VisitIntersections(e, old, [&](Point p) {
                            if (poll() || reachable.size() >= MaxReachable) return true;
                            if (!isfinite(p.x) || !isfinite(p.y) || !parent.PointAllowed(p) || parent.HasPoint(p))
                                return false;
                            if (SamePoint(p, *goal)) createsGoal = true;
                            if (pointSeen.insert({bit_cast<uint64_t>(p.x), bit_cast<uint64_t>(p.y)}).second)
                                reachable.push_back({p, witness});
                            return false;
                        });
                    }
                    if (createsGoal) directWitnesses.push_back(witness);
                }
            }
    }
    counts.points = max(counts.points, reachable.size());
    if (stopNow()) return false;
    vector<Direction> index;
    index.reserve(reachable.size());
    for (uint32_t i = 0; i < reachable.size(); ++i) {
        if (poll()) return false;
        if (SamePoint(reachable[i].point, *goal)) continue; // No direction from a point to itself.
        const double angle = DirectionOf(reachable[i].point, *goal);
        if (isfinite(angle)) index.push_back({angle, i});
    }
    // At most MaxReachable items; keep the comparator order fixed even on stop.
    // Throw only a local cancellation tag so sort can be interrupted/discarded.
    struct CancelledSort {};
    try {
        sort(index.begin(), index.end(), [&](const Direction& a, const Direction& b) {
            if (poll()) throw CancelledSort{};
            return a.angle < b.angle || (a.angle == b.angle && a.reachable < b.reachable);
        });
    } catch (const CancelledSort&) { return false; }
    if (stopNow()) return false;

    // The sole working graph snapshot is shared by all replays via rollback.
    Graph graph = parent;
    graph.SetStateHashingEnabled(false);
    graph.elements.reserve(parent.elements.size() + static_cast<size_t>(budget));
    graph.points.reserve(parent.points.size() + 6 * parent.elements.size() + 6);
    graph.pointBirth.reserve(graph.points.capacity());
    const Mark root = graph.GetMark();
    auto replay = [&](uint32_t first, uint32_t second, const Reachable* a, const Reachable* b) {
        if (stopNow()) return false;
        ++counts.replays;
        graph.Rollback(root);
        int paid = 0;
        activeDepth = static_cast<uint16_t>(depth);
        auto recordApply = [&] {
            ++paid; ++stats.applied; ++stats.nodes;
            activeDepth = static_cast<uint16_t>(depth + paid);
            stats.maxPoints = max(stats.maxPoints, graph.points.size());
            stats.maxElements = max(stats.maxElements, graph.elements.size());
        };
        // 0: not solved; 1: submitted, quota not full; 2: quota; -1: cancelled.
        auto submit = [&] {
            if (stopNow()) return -1;
            if (!graph.GoalsMet()) return 0;
            if (stopNow()) return -1;
            ++counts.successes;
            return collector.Submit(graph, &global) ? 2 : 1;
        };
        for (uint32_t witness : {first, second}) {
            if (witness == NO_BOUND) continue;
            if (stopNow()) return false;
            // Regenerate from REAL known points, not stored predicted coefficients.
            const Element e = graph.MakeCandidate(candidates[witness]);
            if (graph.HasElement(e)) continue; // Same witness costs once; endpoint lookup still must succeed.
            if (paid >= budget || !graph.Apply(e, static_cast<uint16_t>(depth + paid + 1))) return false;
            recordApply();
            const int status = submit();
            if (status != 0) return status == 2;
        }
        if (!a || !b || paid >= budget || stopNow()) return false;
        const uint32_t ia = FindPoint(graph, a->point), ib = FindPoint(graph, b->point);
        if (ia == NO_BOUND || ib == NO_BOUND || ia == ib || SamePoint(graph.points[ia], graph.points[ib]))
            return false;
        ++stats.rawCandidates;
        const Element last = graph.MakeCandidate({ia, ib, 2});
        if (!LegalFinite(last) || !graph.Apply(last, static_cast<uint16_t>(depth + paid + 1))) return false;
        ++stats.uniqueCandidates;
        recordApply();
        return submit() == 2;
    };
    for (uint32_t witness : directWitnesses) {
        if (stopNow()) return false;
        ++counts.proposals;
        if (replay(witness, NO_BOUND, nullptr, nullptr)) return true;
    }

    const double window = min(1e-4, max(1e-8, 1000 * EPS));
    constexpr size_t maxPartners = 256; // Explicit heuristic cap for dense angular clusters.
    constexpr uint64_t maxProposals = 65536;
    uint64_t proposals = 0;
    auto propose = [&](uint32_t ai, uint32_t bi) {
        const Reachable& a = reachable[ai];
        const Reachable& b = reachable[bi];
        const int cost = 1 + (a.witness != NO_BOUND) +
            (b.witness != NO_BOUND && b.witness != a.witness);
        if (cost > budget || SamePoint(a.point, b.point)) return false;
        // This is only an incomplete proposal prefilter. The actual last line
        // is regenerated from the replay's known point representatives below.
        const Element predicted = Element::FromPoints(a.point, b.point, Type::Line);
        if (!LegalFinite(predicted) || parent.HasElement(predicted)) return false;
        ++proposals; ++counts.proposals;
        if (replay(a.witness, b.witness, &a, &b)) return true;
        // Independent operations can still have order-dependent EPS representatives.
        if (a.witness != NO_BOUND && b.witness != NO_BOUND && a.witness != b.witness)
            return replay(b.witness, a.witness, &a, &b);
        return false;
    };
    for (size_t i = 0; i < index.size(); ++i) {
        if (poll() || proposals >= maxProposals) return false;
        size_t partners = 0;
        for (size_t j = i + 1; j < index.size() && index[j].angle - index[i].angle <= window; ++j) {
            if (poll() || proposals >= maxProposals) return false;
            if (partners++ >= maxPartners) break;
            if (propose(index[i].reachable, index[j].reachable)) return true;
        }
        // Modulo-pi directions straddling 0/pi are neighbors too. Each unordered
        // pair is proposed once, with no doubled index allocation.
        if (index[i].angle + window >= Pi) {
            for (size_t j = 0; j < i && index[j].angle + Pi - index[i].angle <= window; ++j) {
                if (poll() || proposals >= maxProposals) return false;
                if (partners++ >= maxPartners) break;
                if (propose(index[i].reachable, index[j].reachable)) return true;
            }
        }
    }
    return false; // Unknown, including successful visits short of the quota.
}

// Convenience interface for callers whose paid depth is encoded in the graph.
inline bool Find(const Graph& parent, int remaining, int toolType,
                 SolutionCollector& collector, ParallelControl& global,
                 SearchStats& stats, Counts& counts,
                 chrono::steady_clock::time_point deadline, ProgressSlot* progress = nullptr) {
    if (global.stop.load(memory_order_acquire) || chrono::steady_clock::now() >= deadline) return false;
    if (parent.initialElementCount > parent.elements.size() ||
        parent.elements.size() - parent.initialElementCount > static_cast<size_t>(numeric_limits<int>::max()))
        throw invalid_argument("point join requires a consistent paid parent prefix");
    return Find(parent, remaining, static_cast<int>(parent.elements.size() - parent.initialElementCount),
                toolType, collector, global, stats, counts, deadline, progress);
}

} // namespace bs::point_join_detail
