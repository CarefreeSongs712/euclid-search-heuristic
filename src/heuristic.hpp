#pragma once
#include "solver.hpp"
#include "family_pool.hpp"
#include "novelty_pool.hpp"
#include "rendezvous.hpp"
#include "landmarks.hpp"
#include "chain_join.hpp"
#include "point_join.hpp"
#include "foundation.hpp"
#include "equal_radius_probe.hpp"
#include "goal_finish.hpp"
#include "diameter_probe.hpp"
#include "mirror_probe.hpp"
#include "homothety_probe.hpp"
#include "tangent_probe.hpp"

namespace bs {

struct HeuristicOptions {
    size_t beamWidth = 64;
    size_t branchLimit = 96;
    // Per worker, INCLUDING the first start. Zero means keep restarting until
    // the shared deadline, unless a complete pass had no heuristic truncation.
    size_t restarts = 0;
    uint64_t seed = 1;
    uint32_t threads = 1;
    double tailSeconds = 0.02;
    size_t tailCandidates = 4;
    bool adaptive = true;
    bool landmarks = false;
    bool structural = true;
    bool prerequisites = true;
    int coverageThreads = -1; // -1 auto, 0 off; includes the prefix producer
};

struct HeuristicMetrics {
    uint64_t restarts = 0, layers = 0, expanded = 0, generated = 0, evaluated = 0;
    uint64_t beamDiscarded = 0, candidateDiscarded = 0, tailCalls = 0;
    // Verified successful state visits by source; collector may deduplicate
    // them. These are deliberately not claimed to be distinct solution counts.
    uint64_t beamSolutions = 0, helperSolutions = 0;
    uint64_t familyMerged = 0;
    uint64_t rendezvousProposals = 0, rendezvousReplays = 0, rendezvousSolutions = 0;
    uint64_t chainCalls=0, chainProposals=0, chainReplays=0, chainSolutions=0;
    uint64_t pointJoinCalls=0, pointJoinProposals=0, pointJoinReplays=0, pointJoinSolutions=0;
    uint64_t coverageTasks=0, coverageSolutions=0;
    uint64_t radiusProbeCalls=0,radiusProbePrefixes=0,radiusProbeCompletions=0,radiusProbeSolutions=0;
    uint64_t prerequisiteCalls=0,prerequisiteApplied=0,prerequisiteSolutions=0,goalFinishCalls=0;
    size_t peakBeam = 0;
    bool budgetLimited = false;
};

struct HeuristicResult {
    bool timedOut = false;
    bool quotaReached = false;
    HeuristicMetrics metrics;
};

// This is a bounded, incomplete FINDER, never an exhaustion proof. In
// particular, neither a completed restart nor an unsuccessful legacy tail
// helper proves anything about the existence of a construction. The caller
// must report HEURISTIC_STOPPED rather than EXHAUSTED, and must not estimate
// exhaustion from progress slots (all root fractions remain unknown).
namespace heuristic_detail {

using Clock = chrono::steady_clock;

inline void MergeMetrics(HeuristicMetrics& a, const HeuristicMetrics& b) {
    a.restarts += b.restarts;
    a.layers += b.layers;
    a.expanded += b.expanded;
    a.generated += b.generated;
    a.evaluated += b.evaluated;
    a.beamDiscarded += b.beamDiscarded;
    a.candidateDiscarded += b.candidateDiscarded;
    a.tailCalls += b.tailCalls;
    a.beamSolutions += b.beamSolutions;
    a.helperSolutions += b.helperSolutions;
    a.familyMerged += b.familyMerged;
    a.rendezvousProposals += b.rendezvousProposals;
    a.rendezvousReplays += b.rendezvousReplays;
    a.rendezvousSolutions += b.rendezvousSolutions;
    a.chainCalls+=b.chainCalls;a.chainProposals+=b.chainProposals;
    a.chainReplays+=b.chainReplays;a.chainSolutions+=b.chainSolutions;
    a.pointJoinCalls+=b.pointJoinCalls;a.pointJoinProposals+=b.pointJoinProposals;
    a.pointJoinReplays+=b.pointJoinReplays;a.pointJoinSolutions+=b.pointJoinSolutions;
    a.coverageTasks+=b.coverageTasks;a.coverageSolutions+=b.coverageSolutions;
    a.radiusProbeCalls+=b.radiusProbeCalls;a.radiusProbePrefixes+=b.radiusProbePrefixes;
    a.radiusProbeCompletions+=b.radiusProbeCompletions;a.radiusProbeSolutions+=b.radiusProbeSolutions;
    a.prerequisiteCalls+=b.prerequisiteCalls;a.prerequisiteApplied+=b.prerequisiteApplied;
    a.prerequisiteSolutions+=b.prerequisiteSolutions;a.goalFinishCalls+=b.goalFinishCalls;
    a.peakBeam = max(a.peakBeam, b.peakBeam); // peak PER WORKER, not a sum
    a.budgetLimited = a.budgetLimited || b.budgetLimited;
}

struct Random {
    uint64_t state;
    uint64_t Next() { state += 0x9e3779b97f4a7c15ULL; return SplitMix64(state); }
    double Unit() { return static_cast<double>(Next() >> 11) * 0x1.0p-53; }
};

// Raw bits, NOT Bits()/CleanZero, EPS buckets, or an unordered state hash.
// Collisions overwrite an entry and merely miss a duplicate; they never drop
// a different candidate. Fixed memory even for enormous point-pair streams.
class CandidateSeen {
    struct Entry {
        uint64_t a = 0, b = 0, c = 0;
        Type type = Type::Line;
        bool valid = false;
    };
    vector<Entry> entries_;
public:
    CandidateSeen() : entries_(4096) {}
    void Reset() { fill(entries_.begin(), entries_.end(), Entry{}); }
    bool Duplicate(const Element& e) {
        const uint64_t a = bit_cast<uint64_t>(e.a);
        const uint64_t b = bit_cast<uint64_t>(e.b);
        const uint64_t c = bit_cast<uint64_t>(e.c);
        const uint64_t hash = SplitMix64(a ^ rotl(b, 21) ^ rotl(c, 42) ^
                                       static_cast<uint8_t>(e.type));
        Entry& old = entries_[static_cast<size_t>(hash) & (entries_.size() - 1)];
        if (old.valid && old.a == a && old.b == b && old.c == c && old.type == e.type)
            return true;
        old = {a, b, c, e.type, true};
        return false;
    }
};

struct Ranked {
    double score = 0.0;
    uint64_t serial = 0;
    uint64_t diversity = 0;
};
struct RankedElement : Ranked { Element element; double bridge = 0.0; };
struct Child : Ranked { size_t parent = 0; Element element; };
struct BeamEntry : Ranked { vector<Element> prefix; };

// Two DISJOINT bounded heaps: top-scoring entries and a uniform random-rank
// reservoir of everything else. Demoted elite entries compete for reservoir
// space too. No O(P^2) candidate storage and no O(beam*branch) child storage.
// The reservoir reserves 1/4..1/2 of each layer for a different search route.
template<class T>
class DiversePool {
    size_t eliteLimit_, diverseLimit_;
    vector<T> elite_, diverse_;
    uint64_t& discarded_;
    bool& limited_;
    struct ScoreBetter {
        bool operator()(const T& a, const T& b) const {
            return a.score > b.score || (a.score == b.score && a.serial < b.serial);
        }
    };
    struct RandomBetter {
        bool operator()(const T& a, const T& b) const {
            return a.diversity > b.diversity ||
                   (a.diversity == b.diversity && a.serial < b.serial);
        }
    };
    void Drop() { ++discarded_; limited_ = true; }
    void OfferDiverse(T value) {
        if (diverse_.size() < diverseLimit_) {
            diverse_.push_back(std::move(value));
            push_heap(diverse_.begin(), diverse_.end(), RandomBetter{});
        } else if (diverseLimit_ && RandomBetter{}(value, diverse_.front())) {
            pop_heap(diverse_.begin(), diverse_.end(), RandomBetter{});
            diverse_.back() = std::move(value);
            push_heap(diverse_.begin(), diverse_.end(), RandomBetter{});
            Drop();
        } else Drop();
    }
public:
    DiversePool(size_t capacity, size_t diverseCount,
                uint64_t& discarded, bool& limited)
        : eliteLimit_(capacity - diverseCount), diverseLimit_(diverseCount),
          discarded_(discarded), limited_(limited) {
        elite_.reserve(eliteLimit_);
        diverse_.reserve(diverseLimit_);
    }
    DiversePool(const DiversePool&) = delete;
    DiversePool& operator=(const DiversePool&) = delete;
    // Entries still owned when cancellation unwinds are resource discards too.
    ~DiversePool() {
        if (!elite_.empty() || !diverse_.empty()) {
            discarded_ += static_cast<uint64_t>(elite_.size() + diverse_.size());
            limited_ = true;
        }
    }
    void Offer(T value) {
        if (elite_.size() < eliteLimit_) {
            elite_.push_back(std::move(value));
            push_heap(elite_.begin(), elite_.end(), ScoreBetter{});
        } else if (eliteLimit_ && ScoreBetter{}(value, elite_.front())) {
            pop_heap(elite_.begin(), elite_.end(), ScoreBetter{});
            swap(value, elite_.back());
            push_heap(elite_.begin(), elite_.end(), ScoreBetter{});
            OfferDiverse(std::move(value));
        } else OfferDiverse(std::move(value));
    }
    vector<T> Take() {
        vector<T> out;
        out.reserve(elite_.size() + diverse_.size());
        for (T& v : elite_) out.push_back(std::move(v));
        for (T& v : diverse_) out.push_back(std::move(v));
        elite_.clear();
        diverse_.clear();
        sort(out.begin(), out.end(), ScoreBetter{});
        return out;
    }
};

struct ProgressTask {
    ProgressSlot* slot;
    const SearchStats& stats;
    ProgressTask(ProgressSlot* s, const SearchStats& st) : slot(s), stats(st) {
        if (slot) slot->BeginTask(stats);
    }
    ~ProgressTask() { if (slot) slot->EndTask(stats); }
};

// On cancellation, record the already selected work that was not consumed.
struct PendingWork {
    size_t count;
    uint64_t& discarded;
    bool& limited;
    ~PendingWork() {
        if (count) { discarded += static_cast<uint64_t>(count); limited = true; }
    }
};

struct Cancelled {};
struct Output { SearchStats stats; HeuristicMetrics metrics; exception_ptr error; };

struct Weights {
    double goal = 10000.0;
    double support = 350.0;
    double prerequisite = 400.0;
    double direction = 35.0;
    double proximity = 8.0;
    double growth = 10.0;
};

struct PointTarget { Point point; unsigned support = 0; double importance = 1.0; };
struct Context {
    vector<PointTarget> targets;
    vector<Element> missingElements;
    vector<Element> bridgeGuides; // score-only, NEVER used as operations
};

class Worker {
    const Graph& initial_;
    const int limit_, toolType_;
    const HeuristicOptions& options_;
    SolutionCollector& collector_;
    ParallelControl& control_;
    SearchStats& stats_;
    HeuristicMetrics& metrics_;
    ProgressSlot* slot_;
    const uint32_t workerId_;
    Graph graph_;
    Mark root_;
    CandidateSeen seen_;
    Random random_{0};
    Weights weights_;
    uint64_t polls_ = 0, serial_ = 0;
    uint16_t depth_ = 0;
    size_t diversityDivisor_ = 4;
    bool bridgeStyle_ = false;
    bool adaptiveStyle_ = false;
    bool noveltyStyle_ = false;
    BackwardLandmarks landmarks_;
    bool landmarkStyle_ = false;
    optional<Graph> tailGraph_;
    set<vector<RawElementKey>> structureSeen_;
    set<vector<RawElementKey>> goalFinishSeen_;
    double structuralSeconds_ = 0.0;
    Solver tailSolver_;

    void CheckNow() {
        if (control_.stop.load(memory_order_acquire)) throw Cancelled{};
        if (Clock::now() >= control_.deadline) {
            if (!control_.found.load(memory_order_acquire)) {
                control_.timedOut.store(true, memory_order_release);
                control_.stop.store(true, memory_order_release);
            }
            throw Cancelled{};
        }
    }
    void Poll() {
        ++polls_;
        if ((polls_ & 255u) == 0) CheckNow();
        if ((polls_ & 1023u) == 0 && slot_) slot_->Publish(stats_, depth_);
    }
    void ObserveState() {
        stats_.maxPoints = max(stats_.maxPoints, graph_.points.size());
        stats_.maxElements = max(stats_.maxElements, graph_.elements.size());
    }
    bool Submit(const Graph& g, bool helper = false) {
        // A scoring predicate is NEVER a success predicate.
        if (!g.GoalsMet()) return false;
        if (helper) ++metrics_.helperSolutions;
        else ++metrics_.beamSolutions;
        if (!collector_.Submit(g, &control_)) return false;
        // Submit may also return true if somebody already filled the quota.
        control_.found.store(true, memory_order_release);
        control_.stop.store(true, memory_order_release);
        return true;
    }
    size_t DiverseCount(size_t capacity) const {
        return capacity > 1 ? max<size_t>(1, capacity / diversityDivisor_) : 0;
    }
    static double Smooth(double distance) {
        return isfinite(distance) && distance >= 0.0 ? 1.0 / (1.0 + distance) : 0.0;
    }
    static double PointNear(const Point& a, const Point& b) {
        const double scale = 1.0 + abs(b.x) + abs(b.y);
        return Smooth((abs(a.x - b.x) + abs(a.y - b.y)) / scale);
    }
    static double CurveNear(const Point& p, const Element& e) {
        if (e.type == Type::Circle) {
            const double d = Sq(p.x - e.a) + Sq(p.y - e.b);
            return Smooth(abs(d - e.c) / (1.0 + abs(e.c)));
        }
        const double scale = 1.0 + abs(e.a) + abs(e.b);
        return Smooth(abs(e.a * p.x + e.b * p.y - e.c) / scale);
    }
    static double DirectionScore(const Element& a, const Element& b) {
        if (a.type == Type::Circle || b.type == Type::Circle) return 0.0;
        // Parallel OR perpendicular carriers can supply useful intersections.
        const double na = hypot(a.a, a.b), nb = hypot(b.a, b.b);
        if (!(na > 0.0) || !(nb > 0.0) || !isfinite(na) || !isfinite(nb)) return 0.0;
        const double cosine = abs((a.a / na) * (b.a / nb) + (a.b / na) * (b.b / nb));
        return max(cosine * cosine, (1.0 - cosine) * (1.0 - cosine));
    }
    bool HasPoint(const Point& point) {
        // Poll even inside feature scans: large grids must remain cancellable.
        for (const Point& p : graph_.points) {
            Poll();
            if (SamePoint(p, point)) return true;
        }
        return false;
    }
    bool HasElement(const Element& element) {
        for (const Element& e : graph_.elements) {
            Poll();
            if (graph_.SameStoredElement(e, element)) return true;
        }
        return false;
    }
    unsigned Supports(const Point& point) {
        unsigned count = 0;
        Element first;
        for (const Element& e : graph_.elements) {
            Poll();
            if (!graph_.PointOnElement(point, e)) continue;
            Element carrier = e;
            if (carrier.type != Type::Circle) carrier.type = Type::Line;
            // A ray/segment and its supporting infinite line are ONE feature
            // support. This is only a score, never a necessary-condition gate.
            if (count && SameElement(first, carrier)) continue;
            first = carrier;
            if (++count == 2) break;
        }
        return count;
    }
    Context MakeContext() {
        Context context;
        for (const Point& p : graph_.goalPoints) {
            Poll();
            if (!HasPoint(p)) context.targets.push_back({p, Supports(p), 1.0});
        }
        for (const Element& e : graph_.goalElements) {
            Poll();
            if (HasElement(e)) continue;
            context.missingElements.push_back(e);
            if (e.type == Type::Circle) {
                const Point center{e.a, e.b};
                if (!HasPoint(center)) context.targets.push_back({center, Supports(center), 0.7});
            }
        }
        if (bridgeStyle_ && toolType_ != 0 && limit_ - depth_ <= 3) {
            // Targets may define scoring carriers but never construction
            // inputs. The final operation must still be reconstructed from
            // known-point pairs by SelectCandidates and verified by GoalsMet.
            auto guide = [&](const Element& e) {
                if (graph_.HasElement(e)) return;
                for (const Element& old : context.bridgeGuides) {
                    Poll();
                    if (SameElement(e, old)) return; // score-only representative
                }
                context.bridgeGuides.push_back(e);
            };
            for (const Element& e : context.missingElements) {
                Poll();
                if (e.type == Type::Line) guide(e);
            }
            // A missing goal line already supplies the most informative
            // carrier; only point-only searches need inferred directions.
            if (context.bridgeGuides.empty()) for (const PointTarget& target : context.targets) {
                const size_t n = graph_.points.size();
                for (size_t k = 0; k < min<size_t>(64, n); ++k) {
                    Poll();
                    const Point& p = graph_.points[k * n / min<size_t>(64, n)];
                    if (!SamePoint(p, target.point))
                        guide(Element::FromPoints(p, target.point, Type::Line));
                }
            }
        }
        return context;
    }
    double BridgeScore(const Element& candidate, const Context& context) {
        if (context.bridgeGuides.empty() || limit_ - depth_ > 3) return 0.0;
        // Bounded geometric lookahead, used by only one portfolio style.
        // r is an intersection which this candidate could really create; k is
        // merely a score-only intersection with a target carrier. A known
        // center O with |Or|^2 == |Ok|^2 suggests the legal second circle O,r
        // will unlock a point on that carrier. No r/k is inserted for free.
        vector<Point> born;
        born.reserve(32);
        for (const Element& old : graph_.elements) {
            Poll();
            graph_.VisitIntersections(candidate, old, [&](const Point& p) {
                Poll();
                if (born.size() < 32 && isfinite(p.x) && isfinite(p.y) && !HasPoint(p))
                    born.push_back(p);
                return born.size() >= 32;
            });
            if (born.size() >= 32) break; // feature cap, not search pruning
        }
        if (born.empty()) return 0.0;
        for (const Element& guide : context.bridgeGuides) {
            Poll();
            for (const Point& r : born) {
                Poll();
                if (graph_.PointOnElement(r, guide)) return weights_.prerequisite * 12.0;
            }
        }
        if (limit_ - depth_ != 3 || toolType_ == 1 || candidate.type != Type::Circle) return 0.0;
        bool bridge = false;
        for (const Element& guide : context.bridgeGuides) {
            Poll();
            graph_.VisitIntersections(candidate, guide, [&](const Point& k) {
                Poll();
                if (!isfinite(k.x) || !isfinite(k.y) || HasPoint(k)) return false;
                const size_t n = graph_.points.size();
                for (size_t index = 0; index < min<size_t>(64, n); ++index) {
                    Poll();
                    const Point& center = graph_.points[index * n / min<size_t>(64, n)];
                    const double radius = Sq(k.x - center.x) + Sq(k.y - center.y);
                    for (const Point& r : born) {
                        Poll();
                        if (SamePoint(k, r) || !IsZero(Sq(r.x - center.x) + Sq(r.y - center.y) - radius))
                            continue;
                        const Element second = Element::FromPoints(center, r, Type::Circle);
                        if (SameElement(second, candidate) || graph_.HasElement(second)) continue;
                        // Equal radius residual is not enough near tangency or
                        // EPS boundaries. Confirm the actual legacy arithmetic
                        // really creates a new point on the intended carrier.
                        graph_.VisitIntersections(second, candidate, [&](const Point& actual) {
                            Poll();
                            if (!graph_.PointOnElement(actual, guide) || HasPoint(actual)) return false;
                            for (const Point& firstBorn : born)
                                if (SamePoint(actual, firstBorn)) return false;
                            bridge = true;
                            return true;
                        });
                        if (bridge) return true;
                    }
                }
                return false;
            });
            if (bridge) return weights_.prerequisite * 8.0;
        }
        return 0.0;
    }
    double CheapScore(const Element& e, const Candidate& c, const Context& context) {
        double value = 0.0, hits = 0.0;
        for (const PointTarget& target : context.targets) {
            Poll();
            if (graph_.PointOnElement(target.point, e)) {
                hits += target.importance;
                value += weights_.support * target.importance * (1.0 + target.support);
            } else value += weights_.proximity * target.importance * CurveNear(target.point, e);
        }
        value += weights_.support * min(8.0, hits * hits) * 0.25;
        for (const Element& goal : context.missingElements) {
            Poll();
            if (SameElement(e, goal)) value += weights_.goal;
            if (goal.type == Type::Circle) {
                if (e.type == Type::Circle) {
                    if (SamePoint({e.a, e.b}, {goal.a, goal.b})) value += weights_.prerequisite;
                    value += weights_.direction * Smooth(abs(e.c - goal.c) / (1.0 + abs(goal.c)));
                }
            } else {
                value += weights_.direction * DirectionScore(e, goal);
                // A new carrier through an already useful goal-line point can
                // expose a second point. Endpoints are always KNOWN points.
                if (graph_.PointOnElement(graph_.points[c.i], goal) ||
                    graph_.PointOnElement(graph_.points[c.j], goal))
                    value += weights_.direction;
            }
        }
        if (depth_ && (graph_.pointBirth[c.i] == depth_ || graph_.pointBirth[c.j] == depth_))
            value += weights_.growth;
        return isfinite(value) ? value : 0.0;
    }
    double WholeScore() {
        double value = 0.0;
        for (const Point& goal : graph_.goalPoints) {
            Poll();
            if (HasPoint(goal)) { value += weights_.goal; continue; }
            value += weights_.support * Supports(goal);
            double closeness = 0.0;
            for (const Point& p : graph_.points) {
                Poll();
                closeness = max(closeness, PointNear(p, goal));
            }
            value += weights_.proximity * closeness;
        }
        for (const Element& goal : graph_.goalElements) {
            Poll();
            if (HasElement(goal)) { value += weights_.goal; continue; }
            unsigned hits = 0;
            double closeness = 0.0;
            for (const Point& p : graph_.points) {
                Poll();
                if (hits < 3 && graph_.PointOnElement(p, goal)) ++hits;
                closeness = max(closeness, CurveNear(p, goal));
            }
            if (goal.type == Type::Circle) {
                const Point center{goal.a, goal.b};
                const bool centerKnown = HasPoint(center);
                value += weights_.prerequisite * (centerKnown ? 1.5 : 0.0);
                value += weights_.prerequisite * (hits ? 0.75 : 0.0);
                if (centerKnown && hits) value += weights_.prerequisite;
                if (!centerKnown) value += 0.5 * weights_.support * Supports(center);
            } else {
                // Incidence is only a FEATURE. Even two EPS-incident points
                // need not construct the target line, and are never accepted
                // as a direct operation without ordinary pair generation.
                value += weights_.prerequisite * static_cast<double>(min(2u, hits));
                if (hits > 2) value += weights_.direction;
            }
            value += weights_.proximity * closeness;
        }
        // Reward known points on carriers joining target points. These virtual
        // guides are SCORE-ONLY; they are not inserted in any graph or prefix.
        // Cap feature work, not real candidate generation, at 16 guides.
        size_t guides = 0;
        for (size_t i = 0; i < graph_.goalPoints.size() && guides < 16; ++i) {
            for (size_t j = i + 1; j < graph_.goalPoints.size() && guides < 16; ++j) {
                Poll();
                if (SamePoint(graph_.goalPoints[i], graph_.goalPoints[j])) continue;
                ++guides;
                const Element guide = Element::FromPoints(graph_.goalPoints[i], graph_.goalPoints[j], Type::Line);
                unsigned hits = 0;
                for (const Point& p : graph_.points) {
                    Poll();
                    if (graph_.PointOnElement(p, guide) && ++hits == 3) break;
                }
                value += weights_.direction * static_cast<double>(hits);
            }
        }
        const size_t added = graph_.points.size() - initial_.points.size();
        value += weights_.growth * log1p(static_cast<double>(added));
        return isfinite(value) ? value : 0.0;
    }
    double PrerequisitePreview(const Element& element, const Context& context) {
        if (!adaptiveStyle_ || context.missingElements.empty()) return 0.0;
        double score = 0.0;
        for (const Element& old : graph_.elements) {
            Poll();
            graph_.VisitIntersections(element, old, [&](const Point& point) {
                Poll();
                if (!isfinite(point.x) || !isfinite(point.y)) return false;
                double proposed=score;
                for (const auto& target : context.targets) {
                    if (SamePoint(point,target.point)) proposed=max(proposed,weights_.goal*0.5);
                }
                for (const auto& goal : context.missingElements) {
                    if (graph_.PointOnElement(point,goal)) proposed=max(proposed,weights_.prerequisite*3.0);
                    if (goal.type==Type::Circle && SamePoint(point,{goal.a,goal.b}))
                        proposed=max(proposed,weights_.prerequisite*4.0);
                }
                if(proposed>score&&!graph_.HasPoint(point))score=proposed;
                return false;
            });
        }
        return score;
    }
    vector<RankedElement> SelectCandidates() {
        const Context context = MakeContext();
        DiversePool<RankedElement> pool(options_.branchLimit, DiverseCount(options_.branchLimit),
                                       metrics_.candidateDiscarded, metrics_.budgetLimited);
        seen_.Reset();
        if (graph_.points.size() > numeric_limits<uint32_t>::max())
            throw length_error("heuristic point indices exceed uint32_t");
        const uint32_t n = static_cast<uint32_t>(graph_.points.size());
        auto emit = [&](uint32_t i, uint32_t j, uint8_t tool) {
            Poll();
            ++stats_.rawCandidates;
            ++metrics_.generated;
            const Candidate c{i, j, tool};
            const Element e = graph_.MakeCandidate(c);
            if (!isfinite(e.a) || !isfinite(e.b) || !isfinite(e.c) ||
                (e.type == Type::Line ? (e.a == 0.0 && e.b == 0.0) : e.c <= 0.0)) {
                ++metrics_.candidateDiscarded;
                metrics_.budgetLimited = true;
                return;
            }
            if (graph_.HasElement(e)) { ++stats_.existingCandidates; return; }
            if (seen_.Duplicate(e)) { ++stats_.duplicateCandidates; return; }
            ++stats_.uniqueCandidates;
            RankedElement ranked;
            ranked.element = e;
            ranked.serial = serial_++;
            ranked.diversity = random_.Next();
            ranked.bridge = BridgeScore(e, context) + PrerequisitePreview(e, context);
            if (landmarkStyle_) ranked.bridge += weights_.prerequisite *
                landmarks_.ScoreCandidate(graph_,e,graph_.points.size(),[&]{Poll();});
            ranked.score = CheapScore(e, c, context) + ranked.bridge;
            pool.Offer(std::move(ranked));
        };
        for (uint32_t i = 0; i < n; ++i) {
            Poll();
            for (uint32_t j = i + 1; j < n; ++j) {
                Poll();
                if (toolType_ != 1) { emit(i, j, 0); emit(i, j, 1); }
                if (toolType_ != 0) emit(i, j, 2);
            }
        }
        return pool.Take();
    }
    void Replay(const vector<Element>& prefix) {
        graph_.Rollback(root_);
        uint16_t birth = 0;
        for (const Element& e : prefix) {
            CheckNow();
            if (!graph_.Apply(e, ++birth))
                throw logic_error("heuristic prefix replay changed its ordered state");
        }
        // Replay is reconstruction, excluded from search nodes/applied counts,
        // just like Solver::SearchPrefixTask. No unordered state fingerprint.
    }
    void TailHelper(const vector<Element>& prefix, double sliceSeconds = -1.0) {
        if (options_.tailCandidates == 0 || options_.tailSeconds == 0.0) return;
        CheckNow();
        if (!tailGraph_) tailGraph_.emplace(initial_);
        tailGraph_->Rollback(root_);
        ParallelControl local;
        const auto now = Clock::now();
        const double seconds = min(sliceSeconds >= 0.0 ? sliceSeconds : options_.tailSeconds,
            max(0.0, chrono::duration<double>(control_.deadline - now).count()));
        local.deadline = min(control_.deadline, now + chrono::duration_cast<Clock::duration>(
            chrono::duration<double>(seconds)));
        SearchStats tailStats;
        ++metrics_.tailCalls;
        // The one-off warm portfolio member preserves the familiar v10
        // symmetry order. Ordinary rem<=2 helpers remain symmetry=false.
        Solver warmSolver(toolType_, true, true, true, 0, 0, seconds);
        warmSolver.SetExternalStop(&control_.stop);
        Solver& helper = sliceSeconds >= 0.0 ? warmSolver : tailSolver_;
        const bool found = helper.SearchPrefixTask(*tailGraph_, limit_, PrefixTask{prefix},
                                                  tailStats, &local);
        MergeSearchStats(stats_, tailStats);
        if (local.timedOut.load(memory_order_relaxed)) metrics_.budgetLimited = true;
        // Deliberately no SetSolutionCollector and no SetProgress on the helper.
        // A local timeout cannot poison global stop/timedOut. Legacy lower
        // bounds are NOT trusted: failure never removes a beam entry.
        if (found && tailGraph_->GoalsMet()) Submit(*tailGraph_, true);
        CheckNow();
        if (slot_) slot_->Publish(stats_, depth_);
    }
    void GoalLockedCompletion(const vector<Element>& prefix,int remaining,size_t rank) {
        if(!options_.prerequisites||!options_.structural||!options_.adaptive||initial_.goalElements.empty()||
           remaining<1||remaining>6||rank>=2||structuralSeconds_>=6.0||goalFinishSeen_.size()>=128)return;
        if(options_.threads>1&&workerId_%4!=0)return;
        vector<RawElementKey> key;for(const Element& e:prefix)key.push_back(RawKey(e));
        if(!goalFinishSeen_.insert(std::move(key)).second)return;
        const auto now=Clock::now();
        const double left=max(0.0,chrono::duration<double>(control_.deadline-now).count());
        const double budget=min({0.025,left*0.01,6.0-structuralSeconds_});
        if(budget<0.001)return;
        FinishPrerequisite(graph_,remaining,now+chrono::duration_cast<Clock::duration>(chrono::duration<double>(budget)));
        structuralSeconds_+=chrono::duration<double>(Clock::now()-now).count();
        CheckNow();
    }
    void StructuralCompletion(const vector<Element>& prefix,int remaining,size_t rank) {
        if(!options_.structural||!options_.adaptive||initial_.goalPoints.empty())return;
        if((remaining!=4&&remaining!=3)||toolType_==0||rank>=4)return;
        if(options_.threads>1 && workerId_%4!=0)return;
        if(structureSeen_.size()>=128||structuralSeconds_>=6.0)return;
        vector<RawElementKey> key;key.reserve(prefix.size());
        for(const Element& e:prefix)key.push_back(RawKey(e));
        if(!structureSeen_.insert(std::move(key)).second)return;
        CheckNow();
        const auto now=Clock::now();
        const double left=max(0.0,chrono::duration<double>(control_.deadline-now).count());
        const double budget=min({0.6,left*0.05,6.0-structuralSeconds_});
        if(budget<0.005)return;
        const auto deadline=min(control_.deadline,now+chrono::duration_cast<Clock::duration>(chrono::duration<double>(budget)));
        if(remaining==3) {
            point_join_detail::Counts counts;
            ++metrics_.pointJoinCalls;
            point_join_detail::Find(graph_,remaining,toolType_,collector_,control_,stats_,counts,deadline,slot_);
            metrics_.pointJoinProposals+=counts.proposals;metrics_.pointJoinReplays+=counts.replays;
            metrics_.pointJoinSolutions+=counts.successes;
        } else {
            chain_join_detail::Counts counts;
            ++metrics_.chainCalls;
            chain_join_detail::Find(graph_,remaining,toolType_,collector_,control_,stats_,counts,deadline,slot_);
            metrics_.chainProposals+=counts.proposals;metrics_.chainReplays+=counts.replays;
            metrics_.chainSolutions+=counts.successes;
        }
        structuralSeconds_+=chrono::duration<double>(Clock::now()-now).count();
        CheckNow();
    }
    void FoundationCompletion() {
        if(!options_.structural||!options_.adaptive||workerId_!=0||toolType_!=2||limit_<4||
           initial_.goalPoints.empty()||initial_.points.size()>16)return;
        CheckNow();
        ProgressTask progress(slot_,stats_);
        const auto start=Clock::now();
        const double seconds=min(3.0,max(0.0,chrono::duration<double>(control_.deadline-start).count())*0.1);
        const auto deadline=min(control_.deadline,start+chrono::duration_cast<Clock::duration>(chrono::duration<double>(seconds)));
        size_t pairs=0;
        for(uint32_t i=0;i<initial_.points.size();++i)for(uint32_t j=i+1;j<initial_.points.size();++j) {
            CheckNow();
            if(++pairs>24||Clock::now()>=deadline){graph_.Rollback(root_);return;}
            graph_.Rollback(root_);
            vector<Element> paid;
            if(!foundation_detail::ApplyPairScaffold(graph_,i,j,limit_-1,stats_,paid))continue;
            if(Submit(graph_))throw Cancelled{};
            const int remaining=limit_-static_cast<int>(paid.size());
            if(remaining<1)continue;
            point_join_detail::Counts counts;
            ++metrics_.pointJoinCalls;
            point_join_detail::Find(graph_,min(remaining,3),toolType_,collector_,control_,stats_,counts,deadline,slot_);
            metrics_.pointJoinProposals+=counts.proposals;metrics_.pointJoinReplays+=counts.replays;
            metrics_.pointJoinSolutions+=counts.successes;
            CheckNow();
        }
        graph_.Rollback(root_);
    }
    void EqualRadiusCompletion() {
        if(!options_.structural||!options_.adaptive||workerId_!=0||toolType_!=2||
           limit_<6||initial_.points.size()>8)return;
        CheckNow();
        ProgressTask progress(slot_,stats_);
        const auto now=Clock::now();
        const double seconds=min(2.0,max(0.0,chrono::duration<double>(control_.deadline-now).count())*0.1);
        const auto deadline=min(control_.deadline,now+chrono::duration_cast<Clock::duration>(chrono::duration<double>(seconds)));
        equal_radius_detail::Counts counts;
        ++metrics_.radiusProbeCalls;
        auto complete=[&](const Graph& prefix,int remaining) {
            CheckNow();
            if(prefix.GoalsMet()) {
                ++metrics_.radiusProbeSolutions;
                return collector_.Submit(prefix,&control_);
            }
            if(remaining<1||options_.tailCandidates==0||options_.tailSeconds==0.0||Clock::now()>=deadline)return false;
            Graph tail=initial_;
            PrefixTask task{vector<Element>(prefix.elements.begin()+initial_.elements.size(),prefix.elements.end())};
            ParallelControl local;
            local.deadline=min(deadline,Clock::now()+chrono::milliseconds(30));
            SearchStats work;
            ++metrics_.radiusProbeCompletions;
            const bool found=tailSolver_.SearchPrefixTask(tail,limit_,task,work,&local);
            MergeSearchStats(stats_,work);
            if(found&&tail.GoalsMet()&&!control_.stop.load(memory_order_acquire)&&Clock::now()<deadline) {
                ++metrics_.radiusProbeSolutions;
                return collector_.Submit(tail,&control_);
            }
            return false;
        };
        equal_radius_detail::Probe(initial_,limit_,toolType_,control_,stats_,counts,deadline,complete,slot_);
        metrics_.radiusProbePrefixes+=counts.prefixes;
        if(counts.budgetLimited)metrics_.budgetLimited=true;
        CheckNow();
    }
    bool FinishPrerequisite(const Graph& parent,int remaining,Clock::time_point deadline) {
        CheckNow();
        if(parent.GoalsMet()) {
            ++metrics_.prerequisiteSolutions;
            return collector_.Submit(parent,&control_);
        }
        if(remaining<1||Clock::now()>=deadline)return false;
        goal_finish_detail::Counts counts;
        ++metrics_.goalFinishCalls;
        const auto slice=min(deadline,Clock::now()+chrono::milliseconds(40));
        const bool quota=goal_finish_detail::Find(parent,min(remaining,6),toolType_,collector_,control_,stats_,counts,slice,slot_);
        metrics_.prerequisiteSolutions+=counts.successes;
        if(counts.budgetLimited)metrics_.budgetLimited=true;
        return quota;
    }
    void PrerequisiteCompletion() {
        if(!options_.prerequisites||!options_.structural||!options_.adaptive||workerId_!=0||
           toolType_!=2||initial_.goalElements.empty())return;
        CheckNow();
        ProgressTask task(slot_,stats_);
        const auto start=Clock::now();
        const double seconds=min(2.0,max(0.0,chrono::duration<double>(control_.deadline-start).count())*0.1);
        const auto deadline=min(control_.deadline,start+chrono::duration_cast<Clock::duration>(chrono::duration<double>(seconds)));
        auto direct=[&](const Graph& parent,int left) {return FinishPrerequisite(parent,left,deadline);};
        diameter_detail::Counts diameters;
        ++metrics_.prerequisiteCalls;
        diameter_detail::Probe(initial_,limit_,toolType_,control_,stats_,diameters,deadline,direct,slot_);
        metrics_.prerequisiteApplied+=diameters.applied;
        CheckNow();
        if(Clock::now()>=deadline)return;
        auto circleFamily=[&](const Graph& parent,int left) {
            CheckNow();
            tangent_detail::Counts tangents;
            tangent_detail::Probe(parent,left,toolType_,control_,stats_,tangents,deadline,direct,slot_);
            metrics_.prerequisiteApplied+=tangents.applied;
            CheckNow();
            if(Clock::now()>=deadline)return false;
            return direct(parent,left);
        };
        homothety_detail::Counts similarities;
        homothety_detail::Probe(initial_,limit_,toolType_,control_,stats_,similarities,deadline,circleFamily,slot_);
        metrics_.prerequisiteApplied+=similarities.applied;
        CheckNow();
    }
    void ConfigurePortfolio(size_t restart) {
        random_.state = SplitMix64(options_.seed ^ SplitMix64(workerId_) ^
                                  SplitMix64(static_cast<uint64_t>(restart) + 0xd1b54a32d192ed03ULL));
        weights_ = Weights{};
        auto varied = [&] { return 0.55 + 1.35 * random_.Unit(); };
        weights_.support *= varied();
        weights_.prerequisite *= varied();
        weights_.direction *= varied();
        weights_.proximity *= varied();
        weights_.growth *= varied();
        const size_t style = (static_cast<size_t>(workerId_) + restart) % 4;
        if (style == 1) { weights_.support *= 2.0; weights_.growth *= 0.5; }
        if (style == 2) { weights_.prerequisite *= 2.0; weights_.proximity = 0.0; }
        if (style == 3) { weights_.direction *= 2.0; weights_.growth *= 3.0; }
        diversityDivisor_ = style == 3 ? 2 : 4;
        bridgeStyle_ = style == 3;
        landmarkStyle_ = options_.landmarks || (options_.adaptive && !initial_.goalElements.empty() &&
            (options_.threads==1 ? restart%4==0 : workerId_%4==1));
        adaptiveStyle_ = options_.adaptive && !initial_.goalElements.empty() &&
            (options_.threads == 1 ? restart % 2 == 0 : workerId_ % 2 == 0);
        noveltyStyle_ = adaptiveStyle_ && (options_.threads==1 ? restart%4==0 : workerId_%4==0);
        if (adaptiveStyle_) {
            // Growth without new goal structure is a plateau, not progress.
            weights_.growth = restart % 3 == 0 ? 0.0 : weights_.growth * 0.2;
            weights_.proximity = 0.0;
        }
    }
    void OneRestart() {
        vector<BeamEntry> beam(1);
        metrics_.peakBeam = max(metrics_.peakBeam, size_t{1});
        ++stats_.nodes;
        for (int depth = 0; depth < limit_ && !beam.empty(); ++depth) {
            depth_ = static_cast<uint16_t>(depth);
            CheckNow();
            ++metrics_.layers;
            const int remaining = limit_ - depth;
            DiversePool<Child> next(options_.beamWidth, DiverseCount(options_.beamWidth),
                                    metrics_.beamDiscarded, metrics_.budgetLimited);
            FamilyPool<Child> families(adaptiveStyle_ && !noveltyStyle_ ? options_.beamWidth * 4 : 0,
                metrics_.beamDiscarded, metrics_.familyMerged, metrics_.budgetLimited);
            NoveltyPool<Child> novelty(noveltyStyle_ ? options_.beamWidth : 0,
                metrics_.beamDiscarded,metrics_.familyMerged,metrics_.budgetLimited);
            PendingWork pendingParents{beam.size(), metrics_.beamDiscarded, metrics_.budgetLimited};
            for (size_t parent = 0; parent < beam.size(); ++parent) {
                CheckNow();
                --pendingParents.count;
                if (remaining <= 2 && parent < options_.tailCandidates)
                    TailHelper(beam[parent].prefix);
                Replay(beam[parent].prefix);
                GoalLockedCompletion(beam[parent].prefix,remaining,parent);
                StructuralCompletion(beam[parent].prefix,remaining,parent);
                ObserveState();
                ++metrics_.expanded;
                auto candidates = SelectCandidates();
                PendingWork pendingCandidates{candidates.size(), metrics_.candidateDiscarded,
                                              metrics_.budgetLimited};
                for (const RankedElement& candidate : candidates) {
                    Poll();
                    --pendingCandidates.count;
                    const Mark mark = graph_.GetMark();
                    graph_.ApplyKnownNew(candidate.element, static_cast<uint16_t>(depth + 1));
                    ++stats_.nodes;
                    ++stats_.applied;
                    ++metrics_.evaluated;
                    ObserveState();
                    if (Submit(graph_)) throw Cancelled{};
                    if (remaining > 1) {
                        Child child;
                        child.parent = parent;
                        child.element = candidate.element;
                        child.serial = serial_++;
                        child.diversity = random_.Next();
                        child.score = WholeScore() + candidate.bridge;
                        if (landmarkStyle_) child.score += weights_.prerequisite *
                            landmarks_.ScoreState(graph_,initial_.points.size(),[&]{Poll();});
                        if(noveltyStyle_) novelty.Offer(std::move(child),
                            ConstructionFamily(beam[parent].prefix,candidate.element));
                        else if (adaptiveStyle_) families.Offer(std::move(child),
                            ConstructionFamily(beam[parent].prefix,candidate.element));
                        else next.Offer(std::move(child));
                    }
                    graph_.Rollback(mark);
                }
                if (slot_) slot_->Publish(stats_, depth_);
            }
            auto survivors = noveltyStyle_ ? novelty.Take() : adaptiveStyle_ ? families.Take() : next.Take();
            if (adaptiveStyle_ && survivors.size() > options_.beamWidth) {
                vector<Child> diversified;
                diversified.reserve(options_.beamWidth);
                vector<size_t> parentCounts(beam.size());
                const size_t quota = max<size_t>(2, options_.beamWidth / 8);
                vector<uint8_t> picked(survivors.size(),0);
                const size_t elite = options_.beamWidth - DiverseCount(options_.beamWidth);
                for (size_t i=0;i<survivors.size() && diversified.size()<elite;++i) {
                    Poll();
                    if (parentCounts[survivors[i].parent] >= quota) continue;
                    ++parentCounts[survivors[i].parent];
                    picked[i]=1;
                    diversified.push_back(survivors[i]);
                }
                vector<size_t> randomOrder;
                for (size_t i=0;i<survivors.size();++i) if (!picked[i]) randomOrder.push_back(i);
                sort(randomOrder.begin(),randomOrder.end(),[&](size_t a,size_t b) {
                    return survivors[a].diversity > survivors[b].diversity;
                });
                for(size_t i:randomOrder) {
                    if(diversified.size()>=options_.beamWidth) break;
                    diversified.push_back(survivors[i]);
                }
                metrics_.beamDiscarded += survivors.size()-diversified.size();
                metrics_.budgetLimited = true;
                sort(diversified.begin(),diversified.end(),[](const Child& a,const Child& b) {
                    return a.score>b.score || (a.score==b.score && a.serial<b.serial);
                });
                survivors=std::move(diversified);
            }
            PendingWork pendingSurvivors{survivors.size(), metrics_.beamDiscarded,
                                         metrics_.budgetLimited};
            vector<BeamEntry> nextBeam;
            nextBeam.reserve(survivors.size());
            for (const Child& child : survivors) {
                CheckNow();
                BeamEntry entry;
                static_cast<Ranked&>(entry) = child;
                entry.prefix = beam[child.parent].prefix;
                entry.prefix.push_back(child.element);
                nextBeam.push_back(std::move(entry));
            }
            beam = std::move(nextBeam);
            pendingSurvivors.count = 0;
            metrics_.peakBeam = max(metrics_.peakBeam, beam.size());
        }
    }
public:
    Worker(const Graph& initial, int limit, int toolType, const HeuristicOptions& options,
           SolutionCollector& collector, ParallelControl& control, Output& out,
           ProgressSlot* slot, uint32_t id)
        : initial_(initial), limit_(limit), toolType_(toolType), options_(options),
          collector_(collector), control_(control), stats_(out.stats), metrics_(out.metrics),
          slot_(slot), workerId_(id), graph_(initial), root_(initial.GetMark()),
          tailSolver_(toolType, false, true, true, 0, 0, options.tailSeconds) {
        graph_.SetStateHashingEnabled(false);
        tailSolver_.SetExternalStop(&control_.stop);
    }
    void Run() {
        try {
            FoundationCompletion();
            EqualRadiusCompletion();
            PrerequisiteCompletion();
            if (options_.landmarks || (options_.adaptive && !initial_.goalElements.empty()))
                landmarks_.Build(initial_,toolType_,[&]{Poll();});
            if (options_.adaptive && workerId_ == 0 && limit_ >= 4) {
                ProgressTask task(slot_, stats_);
                const auto now = Clock::now();
                const double seconds = min(12.0, max(0.0,
                    chrono::duration<double>(control_.deadline-now).count())*0.25);
                rendezvous_detail::Counts counters;
                rendezvous_detail::Find(initial_,limit_,toolType_,collector_,control_,stats_,counters,
                    min(control_.deadline,now+chrono::duration_cast<Clock::duration>(chrono::duration<double>(seconds))),slot_);
                metrics_.rendezvousProposals+=counters.proposals;
                metrics_.rendezvousReplays+=counters.replayed;
                metrics_.rendezvousSolutions+=counters.successes;
                CheckNow();
            }
            // One explicit shallow portfolio fallback, not repeated short DFS
            // slices from root. Other workers remain genuine beam workers.
            // Never more than 25% of the remaining total time (or two seconds),
            // never enabled when the user disabled tail helpers. Failures do
            // not affect beam coverage; helper hits have separate attribution.
            if (workerId_ == 0 && limit_ <= 3 && options_.tailCandidates && options_.tailSeconds > 0.0) {
                ProgressTask task(slot_, stats_);
                const double left = max(0.0, chrono::duration<double>(control_.deadline - Clock::now()).count());
                TailHelper({}, min(2.0, left * 0.25));
            }
            for (size_t restart = 0;; ++restart) {
                CheckNow();
                if (options_.adaptive && options_.coverageThreads == 0 && options_.restarts == 0 && workerId_ == 0 && options_.threads > 1 &&
                    limit_ >= 4 && restart > 0 && options_.tailCandidates && options_.tailSeconds > 0.0) {
                    // One continuing DFS worker is insurance against ranking
                    // plateaus. Other workers keep exploring heuristic lanes.
                    if (!tailGraph_) tailGraph_.emplace(initial_);
                    tailGraph_->Rollback(root_);
                    Solver insurance(toolType_,true,true,true,0,0,1.0);
                    insurance.SetExternalStop(&control_.stop);
                    insurance.SetSolutionCollector(&collector_);
                    insurance.SetSuccessfulVisits(&metrics_.helperSolutions);
                    insurance.SetProgress(slot_);
                    ++metrics_.tailCalls;
                    insurance.SearchPrefixTask(*tailGraph_,limit_,PrefixTask{},stats_,&control_);
                    CheckNow();
                    return;
                }
                ConfigurePortfolio(restart);
                ++metrics_.restarts;
                ProgressTask task(slot_, stats_);
                const uint64_t oldCandidates = metrics_.candidateDiscarded;
                const uint64_t oldBeam = metrics_.beamDiscarded;
                OneRestart();
                CheckNow();
                // No new information can be obtained by another differently
                // weighted pass if all actual candidates/states fitted. This
                // is STILL only HEURISTIC_STOPPED, not a mathematical proof.
                if (metrics_.candidateDiscarded == oldCandidates && metrics_.beamDiscarded == oldBeam)
                    break;
                if (options_.restarts && restart + 1 >= options_.restarts) {
                    metrics_.budgetLimited = true;
                    break;
                }
                if (restart == numeric_limits<size_t>::max() - 1)
                    throw overflow_error("heuristic restart counter overflow");
            }
        } catch (const Cancelled&) {
            if (!control_.found.load(memory_order_acquire)) metrics_.budgetLimited = true;
        }
    }
};

} // namespace heuristic_detail

// SearchStats count actual candidate scans / newly applied child states plus
// legacy helper work, NOT prefix reconstruction. Metrics.generated counts raw
// pair/tool candidates in the beam engine; evaluated counts its Apply calls.
// Existing/bitwise-duplicate skips are in SearchStats; capacity/numeric/cancel
// losses are in *Discarded. Unknown, unscanned suffixes at timeout are NOT
// invented counts. Stats are accumulated only after all workers have joined.
// Reproducible order for seed + one thread + fixed restart cap, provided the
// deadline/helper slices do not cut a pass short. Timing cutoffs are inherently
// machine-dependent. Multiple workers share only collector/control/progress.
inline HeuristicResult RunHeuristic(const Graph& initial, int limit, int toolType,
        const HeuristicOptions& options, SolutionCollector& collector,
        ParallelControl& control, SearchStats& stats,
        ProgressSlot* slots = nullptr, size_t slotCount = 0) {
    if (limit < 0 || limit > numeric_limits<uint16_t>::max())
        throw invalid_argument("heuristic limit must fit uint16_t");
    if (toolType < 0 || toolType > 2)
        throw invalid_argument("heuristic tool type must be 0, 1 or 2");
    if (!options.beamWidth || !options.branchLimit || !options.threads)
        throw invalid_argument("heuristic beam width, branch limit and threads must be positive");
    if (!isfinite(options.tailSeconds) || options.tailSeconds < 0.0)
        throw invalid_argument("heuristic tail seconds must be finite and nonnegative");
    if (initial.pointBirth.size() != initial.points.size() ||
        initial.initialElementCount != initial.elements.size())
        throw invalid_argument("heuristic initial graph has inconsistent point births/elements");
    if(options.coverageThreads < -1 || options.coverageThreads > static_cast<int64_t>(options.threads) ||
       options.coverageThreads == 1 || (options.coverageThreads > 0 && options.coverageThreads >= static_cast<int64_t>(options.threads)))
        throw invalid_argument("coverage requires 0, auto, or 2..threads-1 slots");
    HeuristicResult result;
    if (!control.stop.load(memory_order_acquire) && initial.GoalsMet()) {
        ++stats.nodes;
        stats.maxPoints = max(stats.maxPoints, initial.points.size());
        stats.maxElements = max(stats.maxElements, initial.elements.size());
        if (collector.Submit(initial, &control)) {
            control.found.store(true, memory_order_release);
            control.stop.store(true, memory_order_release);
        }
    }
    if (limit == 0 || control.stop.load(memory_order_acquire)) {
        result.quotaReached = control.found.load(memory_order_acquire);
        result.timedOut = !result.quotaReached && control.timedOut.load(memory_order_acquire);
        return result;
    }
    if(options.coverageThreads < -1 || options.coverageThreads > static_cast<int64_t>(options.threads))
        throw invalid_argument("coverage thread count exceeds total threads");
    const bool coverageAllowed = options.adaptive && options.restarts == 0 &&
        options.tailCandidates > 0 && options.tailSeconds > 0.0 && limit >= 4 && options.threads >= 4;
    const uint32_t coverage = coverageAllowed ? (options.coverageThreads < 0
        ? (limit >= 8 ? max<uint32_t>(2,options.threads/4) : max<uint32_t>(2,options.threads/2))
        : static_cast<uint32_t>(options.coverageThreads)) : 0;
    if(coverage == 1 || coverage >= options.threads)
        throw invalid_argument("coverage requires a producer plus consumer and at least one heuristic worker");
    const uint32_t beamWorkers = options.threads - coverage;
    HeuristicOptions workerOptions = options;
    workerOptions.threads = beamWorkers;
    workerOptions.coverageThreads = static_cast<int>(coverage);
    optional<PrefixTaskQueue> queue;
    if(coverage) queue.emplace(control,max<size_t>(16,coverage * 8));
    vector<heuristic_detail::Output> outputs(options.threads);
    vector<thread> threads;
    threads.reserve(options.threads - 1);
    auto run = [&](uint32_t id) {
        try {
            ProgressSlot* slot = slots && id < slotCount ? &slots[id] : nullptr;
            if(id < beamWorkers) {
                heuristic_detail::Worker worker(initial, limit, toolType, workerOptions, collector, control,
                    outputs[id], slot, id);
                worker.Run();
            } else {
                Solver dfs(toolType,true,true,true,2048,0,1.0);
                dfs.SetExternalStop(&control.stop);
                dfs.SetSolutionCollector(&collector);
                dfs.SetSuccessfulVisits(&outputs[id].metrics.coverageSolutions);
                dfs.SetProgress(slot);
                Graph graph = initial;
                const Mark root = graph.GetMark();
                if(id == beamWorkers) {
                    function<bool(const Graph&)> sink=[&](const Graph& prefix) {
                        return queue->Push(prefix.elements,prefix.initialElementCount);
                    };
                    const uint16_t split=static_cast<uint16_t>(min(2,max(1,limit-2)));
                    dfs.ProduceFrontierTasks(graph,limit,outputs[id].stats,&control,sink,split);
                    queue->Close();
                } else {
                    PrefixTask task;
                    while(queue->Pop(task)) {
                        graph.Rollback(root);
                        ++outputs[id].metrics.coverageTasks;
                        dfs.SearchPrefixTask(graph,limit,task,outputs[id].stats,&control);
                        if(control.stop.load(memory_order_acquire)) {queue->Close();break;}
                    }
                }
            }
        } catch (...) {
            outputs[id].error = current_exception();
            control.stop.store(true, memory_order_release);
            if(queue)queue->Close();
        }
    };
    exception_ptr launchError;
    try {
        for (uint32_t i = 1; i < options.threads; ++i) threads.emplace_back(run, i);
        run(0);
    } catch (...) {
        launchError = current_exception();
        control.stop.store(true, memory_order_release);
        if(queue)queue->Close();
    }
    // No detached work, including failed worker construction/thread launch.
    for (thread& worker : threads) worker.join();
    for (const auto& out : outputs) {
        MergeSearchStats(stats, out.stats);
        heuristic_detail::MergeMetrics(result.metrics, out.metrics);
    }
    if (launchError) rethrow_exception(launchError);
    for (const auto& out : outputs) if (out.error) rethrow_exception(out.error);
    result.quotaReached = control.found.load(memory_order_acquire);
    result.timedOut = !result.quotaReached && control.timedOut.load(memory_order_acquire);
    return result;
}

} // namespace bs
