#pragma once
#include "geometry.hpp"
#include "progress.hpp"
#include "tail_cache.hpp"

namespace bs {
// search remains complete.
class BoundedElementSet {
    struct Slot {
        Element value{};
        uint32_t generation = 0;
    };

    vector<Slot> slots_;
    size_t mask_ = 0;
    size_t maxEntries_ = 0;
    size_t used_ = 0;
    uint32_t generation_ = 1;

    optional<size_t> FindBucket(const ElementBucket& key, const Element& e) const {
        if (slots_.empty()) return nullopt;
        size_t pos = ElementBucketHash{}(key) & mask_;
        for (size_t probe = 0; probe < slots_.size(); ++probe) {
            const Slot& slot = slots_[pos];
            if (slot.generation != generation_) return nullopt;
            // Any geometrically equal element encountered in this probe chain is
            // already a valid duplicate. Re-quantizing slot.value here only to
            // re-check its home bucket is redundant and very expensive.
            if (SameElement(slot.value, e)) return pos;
            pos = (pos + 1) & mask_;
        }
        return nullopt;
    }

public:
    enum class InsertResult { NewTracked, Duplicate, NewUntracked };

    void Configure(size_t maxEntries) {
        maxEntries_ = maxEntries;
        slots_.clear();
        mask_ = 0;
        used_ = 0;
        generation_ = 1;
    }

    void AllocateOnFirstInsert() {
        if (maxEntries_ == 0 || !slots_.empty()) return;
        size_t capacity = 1;
        while (capacity < maxEntries_ * 2) capacity <<= 1;
        slots_.assign(capacity, Slot{});
        mask_ = capacity - 1;
    }

    void BeginNode() {
        used_ = 0;
        if (slots_.empty()) return;
        ++generation_;
        if (generation_ == 0) {
            for (Slot& slot : slots_) slot.generation = 0;
            generation_ = 1;
        }
    }

    InsertResult Insert(const Element& e) {
        if (maxEntries_ == 0) return InsertResult::NewUntracked;
        AllocateOnFirstInsert();
        const ElementBucket b = BucketOf(e);
        size_t pos = ElementBucketHash{}(b) & mask_;
        for (size_t probe = 0; probe < slots_.size(); ++probe) {
            Slot& slot = slots_[pos];
            if (slot.generation != generation_) {
                if (used_ >= maxEntries_) return InsertResult::NewUntracked;
                slot.value = e;
                slot.generation = generation_;
                ++used_;
                return InsertResult::NewTracked;
            }
            if (SameElement(slot.value, e)) return InsertResult::Duplicate;
            pos = (pos + 1) & mask_;
        }
        return InsertResult::NewUntracked;
    }
};

// In enumeration mode, a completed subtree is memoized only after all of its
// solutions have been submitted. The table stays private and remains disabled
// for symmetry-pruned searches and for parallel root-queue workers.
class TranspositionTable {
    struct Entry {
        uint64_t h1 = 0;
        uint64_t h2 = 0;
        uint32_t pointCount = 0;
        uint32_t elementCount = 0;
        uint16_t remaining = 0;
        uint8_t valid = 0;
    };

    vector<Entry> entries_;
    size_t mask_ = 0;

    size_t Index(uint64_t h1, uint64_t h2) const {
        return static_cast<size_t>(SplitMix64(h1 ^ (h2 << 1))) & mask_;
    }

public:
    void Configure(size_t bytes) {
        if (bytes < sizeof(Entry) * 1024) {
            entries_.clear();
            mask_ = 0;
            return;
        }
        size_t count = 1;
        const size_t maxCount = bytes / sizeof(Entry);
        while ((count << 1) <= maxCount) count <<= 1;
        entries_.assign(count, Entry{});
        mask_ = count - 1;
    }

    bool Enabled() const { return !entries_.empty(); }
    size_t Bytes() const { return entries_.size() * sizeof(Entry); }

    bool WasFailed(uint64_t h1, uint64_t h2, uint32_t pointCount,
                   uint32_t elementCount, uint16_t remaining) const {
        if (entries_.empty()) return false;
        const Entry& e = entries_[Index(h1, h2)];
        return e.valid && e.h1 == h1 && e.h2 == h2 &&
               e.pointCount == pointCount && e.elementCount == elementCount &&
               e.remaining >= remaining;
    }

    void StoreFailed(uint64_t h1, uint64_t h2, uint32_t pointCount,
                     uint32_t elementCount, uint16_t remaining) {
        if (entries_.empty()) return;
        Entry& e = entries_[Index(h1, h2)];
        if (e.valid && e.h1 == h1 && e.h2 == h2 &&
            e.pointCount == pointCount && e.elementCount == elementCount) {
            e.remaining = max(e.remaining, remaining);
            return;
        }
        e.h1 = h1;
        e.h2 = h2;
        e.pointCount = pointCount;
        e.elementCount = elementCount;
        e.remaining = remaining;
        e.valid = 1;
    }
};


// Cold-path result collection. No signatures, allocations, or mutex operations
// are performed until a fully verified goal state is actually reached.
class SolutionCollector {
public:
    struct Entry {
        Graph graph;
        vector<Element> newElements;
        size_t circles = 0;
    };
private:
    size_t requested_;
    mutable mutex mutex_;
    vector<Entry> entries_;
    uint64_t successfulVisits_ = 0;
    uint64_t duplicateVisits_ = 0;

    // A perfect matching, rather than sorting EPS-near coefficients, avoids
    // dependence on their input order and avoids quantization-boundary misses.
    // SameElement intentionally retains the searcher's existing EPS semantics.
    static bool SameUnorderedSet(const vector<Element>& a,
                                 const vector<Element>& b) {
        if (a.size() != b.size()) return false;
        vector<int> matched(b.size(), -1);
        vector<uint8_t> seen(b.size());
        auto augment = [&](auto&& self, size_t i) -> bool {
            for (size_t j = 0; j < b.size(); ++j) {
                if (seen[j] || !SameElement(a[i], b[j])) continue;
                seen[j] = 1;
                if (matched[j] < 0 || self(self, static_cast<size_t>(matched[j]))) {
                    matched[j] = static_cast<int>(i);
                    return true;
                }
            }
            return false;
        };
        for (size_t i = 0; i < a.size(); ++i) {
            fill(seen.begin(), seen.end(), 0);
            if (!augment(augment, i)) return false;
        }
        return true;
    }

public:
    explicit SolutionCollector(size_t requested) : requested_(requested) {}

    // Returns true ONLY when the requested number has been collected. Before
    // that, the caller must backtrack and keep enumerating. All parallel workers
    // and the root producer share this collector.
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((noinline))
#elif defined(_MSC_VER)
    __declspec(noinline)
#endif
    bool Submit(const Graph& g, ParallelControl* control, uint64_t* localVisits = nullptr) {
        lock_guard<mutex> guard(mutex_);
        if (entries_.size() >= requested_) return true;
        ++successfulVisits_;
        if (localVisits) ++*localVisits;
        vector<Element> elements(g.elements.begin() + g.initialElementCount,
                                 g.elements.end());
        const size_t circles = static_cast<size_t>(count_if(
            elements.begin(), elements.end(),
            [](const Element& e) { return e.type == Type::Circle; }));
        for (const Entry& old : entries_) {
            if (old.circles == circles && SameUnorderedSet(elements, old.newElements)) {
                ++duplicateVisits_;
                return false;
            }
        }
        // Never reserve requested_ entries in advance: a no-solution search
        // allocates no result storage even for a very large requested count.
        entries_.push_back(Entry{g, std::move(elements), circles});
        if (entries_.size() < requested_) return false;
        if (control) {
            // ParallelControl::found means QUOTA REACHED, not first hit.
            control->found.store(true, memory_order_release);
            control->stop.store(true, memory_order_release);
        }
        return true;
    }
    // Read these only after every worker has been joined.
    const vector<Entry>& Entries() const { return entries_; }
    size_t Count() const { return entries_.size(); }
    uint64_t SuccessfulVisits() const { return successfulVisits_; }
    uint64_t DuplicateVisits() const { return duplicateVisits_; }
};

class ExactGridLineCache {
    struct Slot {
        uint64_t a = 0, b = 0, c = 0;
        uint32_t generation = 0;
    };
    vector<Slot> slots_;
    uint32_t generation_ = 0;
public:
    void BeginNode() {
        if (++generation_ == 0) {
            for (Slot& s : slots_) s.generation = 0;
            generation_ = 1;
        }
    }
    bool Insert(const Element& e) {
        if (slots_.empty()) slots_.resize(1024);
        const uint64_t a = Bits(e.a), b = Bits(e.b), c = Bits(e.c);
        const size_t h = static_cast<size_t>(SplitMix64(a ^ rotl(b, 21) ^ rotl(c, 42))) & 1023;
        Slot& s = slots_[h];
        if (s.generation == generation_ && s.a == a && s.b == b && s.c == c)
            return false;
        s = {a, b, c, generation_};
        return true;
    }
};

class Solver {
    struct PreApplyContext {
        int missingGoalElements = 0;
        vector<uint8_t> missingDistinctGoal;
        vector<Point> requiredPoints;
        vector<uint8_t> goalPointKnown;
        vector<uint8_t> existingSupport;
        vector<uint32_t> missingGoalSupports;
        vector<uint32_t> missingGoals;
        vector<uint32_t> missingGoalPoints;
        // Only used when one missing target element has two operations left.
        int previewGoal = -1;
        int knownLineHits = 0;
        Point knownLinePoint{};
        bool knownCenter = false;
        bool knownCircumference = false;
    };

    struct PreparedCandidate {
        Candidate candidate;
        Element element;
        uint8_t priority = 0;
    };

    struct NodeScratch {
        PreApplyContext preApply;
        vector<PreparedCandidate> candidates;
        unordered_map<ElementBucket, uint32_t, ElementBucketHash> candidateBuckets;
        vector<Element> deferred;
        vector<uint32_t> incidences;
        vector<uint32_t> forcedTailPoints;
        vector<size_t> missingGoals;
        vector<Point> requiredPoints;
        vector<Element> triedLines;
        TailPrefixCache tailPrefix;
    };
    // Sized before DFS, never while references into a parent node are live.
    // Worker prefix tasks retain capacities across independent subtrees.
    vector<NodeScratch> scratch_;
    TailPrefixCache* activeTailPrefix_ = nullptr;
    uint16_t activeTailDepth_ = 0;
    struct TailPrefixScope {
        Solver& solver;
        TailPrefixCache* previous;
        uint16_t previousDepth;
        TailPrefixScope(Solver& owner, Graph& graph, int remaining, uint16_t depth)
            : solver(owner), previous(owner.activeTailPrefix_), previousDepth(owner.activeTailDepth_) {
#ifndef BS_DISABLE_TAIL_CACHE
            if (remaining == 2 && !graph.goalPoints.empty() && graph.goalPoints.size() <= 16 &&
                graph.points.size() >= 8 && graph.points.size() <= 1024) {
                auto& cache = solver.scratch_[depth].tailPrefix;
                cache.Reset(graph.points.size());
                solver.activeTailPrefix_ = &cache;
                solver.activeTailDepth_ = static_cast<uint16_t>(depth + 1);
            }
#else
            (void)graph; (void)remaining; (void)depth;
#endif
        }
        ~TailPrefixScope() {
            solver.activeTailPrefix_ = previous;
            solver.activeTailDepth_ = previousDepth;
        }
    };
    static constexpr size_t DEFERRED_CANDIDATE_LIMIT = 4096;
    friend struct SolverTestAccess;

    int toolType_ = 2;
    bool symmetry_ = true;
    bool goalFirst_ = true;
    bool lowMemory_ = false;
    size_t streamDedupEntries_ = 0;
    size_t ttBytes_ = 0;
    double timeLimitSeconds_ = DEFAULT_TIME_LIMIT_SECONDS;
    chrono::steady_clock::time_point deadline_{};
    bool timedOut_ = false;
    uint32_t timeoutPollCounter_ = 0;
    vector<BoundedElementSet> streamSeen_;
    vector<ExactGridLineCache> gridSeen_;
    BoundedElementSet oneStepSeen_;
    TranspositionTable tt_;
    ParallelControl* parallelControl_ = nullptr;
    const atomic<bool>* externalStop_ = nullptr;
    SolutionCollector* solutions_ = nullptr;
    uint64_t* successfulVisits_ = nullptr;
    ProgressSlot* progress_ = nullptr;
    const SearchStats* activeStats_ = nullptr;
    uint16_t progressDepth_ = 0;
    struct ProgressScope {
        Solver& solver;
        const SearchStats& stats;
        ProgressScope(Solver& owner, const SearchStats& current) : solver(owner), stats(current) {
            solver.activeStats_ = &stats;
            if (solver.progress_) solver.progress_->BeginTask(stats);
        }
        ~ProgressScope() {
            if (solver.progress_) solver.progress_->EndTask(stats);
            solver.activeStats_ = nullptr;
        }
    };
    // Producer-only handoff at a configurable DFS depth. Workers keep this
    // null and run the unchanged DFS/tail solvers below their claimed prefix.
    function<bool(const Graph&)>* frontierTaskSink_ = nullptr;
    uint16_t frontierDepth_ = 3;
    bool frontierStopped_ = false;
    bool frontierProbeMode_ = false;

    bool CheckTimeout() {
        if (frontierStopped_ || timedOut_) return true;
        const uint32_t poll = ++timeoutPollCounter_;
        if ((poll & 63u) != 0) return false;
        if (externalStop_ && externalStop_->load(memory_order_acquire)) {
            frontierStopped_ = true;
            return true;
        }
        if (parallelControl_ && parallelControl_->stop.load(memory_order_relaxed)) {
            timedOut_ = parallelControl_->timedOut.load(memory_order_relaxed);
            return true;
        }
        if ((poll & 1023u) != 0) return false;
        if (progress_ && activeStats_) progress_->Publish(*activeStats_, progressDepth_);
        if (chrono::steady_clock::now() >= deadline_) {
            timedOut_ = true;
            if (parallelControl_) {
                if (!parallelControl_->found.load(memory_order_acquire)) {
                    parallelControl_->timedOut.store(true, memory_order_release);
                    parallelControl_->stop.store(true, memory_order_release);
                }
            }
            return true;
        }
        return false;
    }

    const vector<PreparedCandidate>& GenerateUniqueCandidates(
            const Graph& g, uint16_t depth, const PreApplyContext& preCtx,
            SearchStats& stats) {
        auto& result = scratch_[depth].candidates;
        auto& seen = scratch_[depth].candidateBuckets;
        result.clear();
        seen.clear();

        const size_t n = g.points.size();
        const size_t rough = n > 1 ? (n * (n - 1) / 2) * (toolType_ == 2 ? 3 : (toolType_ == 0 ? 2 : 1)) : 0;
        const size_t capacity = min<size_t>(rough, 1'000'000);
        if (result.capacity() < capacity) result.reserve(capacity);
        if (seen.bucket_count() < capacity * 2 + 1) seen.reserve(capacity * 2 + 1);

        auto add = [&](uint32_t i, uint32_t j, uint8_t tool) {
            if (CheckTimeout()) return;
            ++stats.rawCandidates;
            Candidate cand{i, j, tool};
            Element e = g.MakeCandidate(cand);
            if (g.HasElement(e)) {
                ++stats.existingCandidates;
                return;
            }
            const ElementBucket b = BucketOf(e);
            for (int da = -1; da <= 1; ++da) {
                for (int db = -1; db <= 1; ++db) {
                    for (int dc = -1; dc <= 1; ++dc) {
                        ElementBucket q{b.a + da, b.b + db, b.c + dc, b.type};
                        auto it = seen.find(q);
                        if (it != seen.end() && SameElement(result[it->second].element, e)) {
                            ++stats.duplicateCandidates;
                            return;
                        }
                    }
                }
            }
            const uint32_t id = static_cast<uint32_t>(result.size());
            result.push_back({cand, e, static_cast<uint8_t>(
                goalFirst_ ? GoalPriority(g, e, preCtx) : 0)});
            seen.emplace(b, id);
            ++stats.uniqueCandidates;
        };

        for (uint32_t i = 0; i < n; ++i) {
            if (CheckTimeout()) return result;
            for (uint32_t j = i + 1; j < n; ++j) {
                if (CheckTimeout()) return result;
                if (toolType_ == 0 || toolType_ == 2) {
                    add(i, j, 0);
                    if (timedOut_) return result;
                    add(i, j, 1);
                    if (timedOut_) return result;
                }
                if (toolType_ == 1 || toolType_ == 2) {
                    add(i, j, 2);
                    if (timedOut_) return result;
                }
            }
        }

        if (CheckTimeout()) return result;
        if (goalFirst_) {
            // Three priority bands, in this order:
            //   1) directly constructible missing target lines/circles;
            //   2) candidates directed at missing target points;
            //   3) ordinary auxiliary candidates.
            // The partition only changes search order and does not remove candidates.
            size_t exactEnd = 0;
            for (size_t read = 0; read < result.size(); ++read) {
                if (CheckTimeout()) return result;
                if (result[read].priority == 2) {
                    if (exactEnd != read) swap(result[exactEnd], result[read]);
                    ++exactEnd;
                }
            }

            if (!preCtx.missingGoalPoints.empty()) {
                size_t directedEnd = exactEnd;
                for (size_t read = exactEnd; read < result.size(); ++read) {
                    if (CheckTimeout()) return result;
                    if (result[read].priority == 1) {
                        if (directedEnd != read) swap(result[directedEnd], result[read]);
                        ++directedEnd;
                    }
                }
            }
        }
        return result;
    }

    bool SymmetryPruned(const Graph& g, const Candidate& cand, const Element& e,
                        uint16_t depth, const optional<OperationKey>& previous,
                        SearchStats& stats) const {
        if (!symmetry_ || !previous) return false;
        const bool dependsOnPrevious = depth > 0 &&
            (g.pointBirth[cand.i] == depth || g.pointBirth[cand.j] == depth);
        if (!dependsOnPrevious && OperationKey::From(e) < *previous) {
            ++stats.symmetryPruned;
            return true;
        }
        return false;
    }

    const PreApplyContext& BuildPreApplyContext(
            const Graph& g, int remaining, uint16_t depth) {
        PreApplyContext& ctx = scratch_[depth].preApply;
        ctx.missingGoalElements = 0;
        ctx.previewGoal = -1;
        ctx.knownLineHits = 0;
        ctx.knownLinePoint = {};
        ctx.knownCenter = false;
        ctx.knownCircumference = false;
        ctx.missingGoals.clear();
        ctx.missingGoalPoints.clear();
        ctx.missingDistinctGoal.assign(g.goalElements.size(), 0);
        for (size_t i = 0; i < g.goalElements.size(); ++i) {
            const Element& goal = g.goalElements[i];
            if (g.HasElement(goal)) continue;
            // Priority tests must retain every missing goal, including EPS-near
            // goals omitted by the existing distinct-goal lower bound.
            ctx.missingGoals.push_back(static_cast<uint32_t>(i));
            bool duplicate = false;
            for (size_t j = 0; j < i; ++j) {
                if (SameElement(g.goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) {
                ctx.missingDistinctGoal[i] = 1;
                ++ctx.missingGoalElements;
            }
        }

        ctx.requiredPoints = g.goalPoints;
        for (size_t gi = 0; gi < g.goalElements.size(); ++gi) {
            if (!ctx.missingDistinctGoal[gi]) continue;
            const Element& goal = g.goalElements[gi];
            if (goal.type != Type::Circle) continue;
            const Point center{goal.a, goal.b};
            bool duplicate = false;
            for (const Point& old : ctx.requiredPoints) {
                if (SamePoint(old, center)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) ctx.requiredPoints.push_back(center);
        }

        const size_t np = ctx.requiredPoints.size();
        ctx.goalPointKnown.resize(np);
        ctx.existingSupport.resize(np);
        ctx.missingGoalSupports.resize(np);
        for (size_t pi = 0; pi < np; ++pi) {
            const Point& p = ctx.requiredPoints[pi];
            const bool known = g.HasPoint(p);
            ctx.goalPointKnown[pi] = known ? 1 : 0;
            if (known) continue;
            if (pi < g.goalPoints.size())
                ctx.missingGoalPoints.push_back(static_cast<uint32_t>(pi));
            ctx.existingSupport[pi] = static_cast<uint8_t>(min(2, g.ExistingSupportCount(p)));
            int forced = 0;
            for (size_t gi = 0; gi < g.goalElements.size(); ++gi) {
                if (ctx.missingDistinctGoal[gi] && g.PointOnElement(p, g.goalElements[gi]))
                    ++forced;
            }
            ctx.missingGoalSupports[pi] = static_cast<uint32_t>(forced);
        }
        if (remaining == 2 && ctx.missingGoalElements == 1) {
            for (size_t gi = 0; gi < g.goalElements.size(); ++gi) {
                if (!ctx.missingDistinctGoal[gi]) continue;
                const Element& goal = g.goalElements[gi];
                if (goal.type == Type::Line) {
                    for (const Point& p : g.points) {
                        if (!g.PointOnElement(p, goal)) continue;
                        ctx.knownLinePoint = p;
                        if (++ctx.knownLineHits >= 2) break;
                    }
                    if (ctx.knownLineHits < 2)
                        ctx.previewGoal = static_cast<int>(gi);
                } else if (goal.type == Type::Circle) {
                    ctx.knownCenter = g.HasPoint({goal.a, goal.b});
                    for (const Point& p : g.points) {
                        if (g.PointOnElement(p, goal)) {
                            ctx.knownCircumference = true;
                            break;
                        }
                    }
                    if (!ctx.knownCenter || !ctx.knownCircumference)
                        ctx.previewGoal = static_cast<int>(gi);
                }
                break;
            }
        }
        return ctx;
    }

    int GoalPriority(const Graph& g, const Element& candidate,
                     const PreApplyContext& ctx) const {
        // The parent graph is restored after every subtree. Cache only goal
        // membership, not approximate equivalence classes or new predicates.
        for (uint32_t gi : ctx.missingGoals)
            if (SameElement(g.goalElements[gi], candidate)) return 2;
        for (uint32_t pi : ctx.missingGoalPoints)
            if (g.PointOnElement(g.goalPoints[pi], candidate)) return 1;
        return 0;
    }

    bool CandidateViolatesNextJointLowerBoundFast(
        const Graph& g, const Element& candidate, int remainingAfter,
        const PreApplyContext& ctx) const {
        if (remainingAfter < 0) return true;

        bool completesMissingGoal = false;
        for (size_t gi = 0; gi < g.goalElements.size(); ++gi) {
            if (ctx.missingDistinctGoal[gi] &&
                SameElement(g.goalElements[gi], candidate)) {
                completesMissingGoal = true;
                break;
            }
        }
        const int missingAfter = ctx.missingGoalElements - (completesMissingGoal ? 1 : 0);
        if (missingAfter > remainingAfter) return true;
        if (ctx.requiredPoints.empty()) return false;

        const int auxiliaryBudget = remainingAfter - missingAfter;
        int requiredAuxiliary = 0;
        for (size_t pi = 0; pi < ctx.requiredPoints.size(); ++pi) {
            if (ctx.goalPointKnown[pi]) continue;
            const bool passes = g.PointOnElement(ctx.requiredPoints[pi], candidate);
            const int existingBefore = ctx.existingSupport[pi];
            if (passes && existingBefore >= 1) continue;

            const int existingAfter = min(2, existingBefore + (passes ? 1 : 0));
            int forcedAfter = ctx.missingGoalSupports[pi];
            if (completesMissingGoal && passes && forcedAfter > 0) --forcedAfter;
            const int need = max(0, 2 - existingAfter - forcedAfter);
            requiredAuxiliary = max(requiredAuxiliary, need);
            if (requiredAuxiliary > auxiliaryBudget) return true;
        }
        return false;
    }

    // A conservative preview of the SAME numerical intersections that Apply
    // would insert. It never constructs an element from the target coordinates.
    // If the auxiliary element cannot supply necessary prerequisites for the
    // last target, skip point deduplication, storage, DFS, and rollback entirely.
    bool CandidateCanSupplyLastElementPrerequisites(
        const Graph& g, const Element& candidate, const PreApplyContext& ctx) const {
        if (ctx.previewGoal < 0) return true;
        const Element& goal = g.goalElements[ctx.previewGoal];
        if (SameElement(candidate, goal)) return true; // goal finished now
        if (goal.type == Type::Line) {
            int rawHits = 0;
            const int needed = 2 - ctx.knownLineHits;
            auto visit = [&](const Point& p) {
                if (!g.PointOnElement(p, goal)) return false;
                if (ctx.knownLineHits == 1 && SamePoint(p, ctx.knownLinePoint))
                    return false;
                // When there are zero old hits, count raw events, not unique
                // points. This intentionally admits false positives, avoiding
                // extra assumptions about non-transitive EPS deduplication.
                return ++rawHits >= needed;
            };
            for (const Element& old : g.elements)
                if (g.VisitIntersections(candidate, old, visit)) return true;
            return false;
        }
        bool center = ctx.knownCenter;
        bool circumference = ctx.knownCircumference;
        const Point centerPoint{goal.a, goal.b};
        auto visit = [&](const Point& p) {
            if (!center && SamePoint(p, centerPoint)) center = true;
            if (!circumference && g.PointOnElement(p, goal)) circumference = true;
            return center && circumference;
        };
        for (const Element& old : g.elements)
            if (g.VisitIntersections(candidate, old, visit)) return true;
        return false;
    }

    bool TryCandidate(Graph& g, const Element& e, int remaining,
                      uint16_t depth, SearchStats& stats,
                      const PreApplyContext& preCtx,
                      bool previewAlreadyPassed = false) {
        if (CheckTimeout()) return false;
        if (CandidateViolatesNextJointLowerBoundFast(g, e, remaining - 1, preCtx)) {
            ++stats.preApplyLowerBoundPruned;
            return false;
        }
        if (!previewAlreadyPassed && preCtx.previewGoal >= 0) {
            ++stats.prerequisitePreviewTested;
            if (!CandidateCanSupplyLastElementPrerequisites(g, e, preCtx)) {
                ++stats.prerequisitePreviewPruned;
                return false;
            }
        }
        const OperationKey key = OperationKey::From(e);
        const Mark mark = g.GetMark();
        g.ApplyKnownNew(e, static_cast<uint16_t>(depth + 1));
        ++stats.applied;
        if (DFS(g, remaining - 1, static_cast<uint16_t>(depth + 1), key, stats)) return true;
        g.Rollback(mark);
        return false;
    }

    bool FastCandidatePassesPoint(const Graph& g, const Candidate& cand,
                                  const Point& p) const {
        const Point& pi = g.points[cand.i];
        const Point& pj = g.points[cand.j];
        if (cand.tool == 0) {
            return IsZero(Sq(pi.x - pj.x) + Sq(pi.y - pj.y) -
                          Sq(pi.x - p.x) - Sq(pi.y - p.y));
        }
        if (cand.tool == 1) {
            return IsZero(Sq(pi.x - pj.x) + Sq(pi.y - pj.y) -
                          Sq(pj.x - p.x) - Sq(pj.y - p.y));
        }
        double a = pj.y - pi.y;
        double b = pi.x - pj.x;
        double c = pi.x * pj.y - pi.y * pj.x;
        if (!IsZero(b))
            return IsZero((a * p.x + b * p.y - c) / b);
        if (!IsZero(a))
            return IsZero((a * p.x + b * p.y - c) / a);
        return false;
    }

    bool FastCandidatePassesForcedTailPoints(const Graph& g, const Candidate& cand,
                                              const vector<uint32_t>& forced) const {
        for (uint32_t gi : forced) {
            if (!FastCandidatePassesPoint(g, cand, g.goalPoints[gi])) return false;
        }
        return true;
    }

    bool TailCandidateAllowed(const Graph& g, const Element& e,
                              const vector<uint32_t>& forcedTailPoints,
                              SearchStats& stats) const {
        if (!g.CandidatePassesForcedTailPoints(e, forcedTailPoints)) {
            ++stats.forcedPointTailPruned;
            return false;
        }
        return true;
    }

    // Find one symmetry-compatible point-pair representation that constructs an
    // exact missing target element.  Different representations of the same
    // geometric element lead to the same state, so after one compatible
    // representation is found there is no reason to try another.
    optional<Candidate> FindGoalConstructionCandidate(
        const Graph& g, const Element& goal, uint16_t depth,
        const optional<OperationKey>& previous,
        const vector<uint32_t>& forcedTailPoints, SearchStats& stats) {
        const uint32_t n = static_cast<uint32_t>(g.points.size());

        if (goal.type == Type::Line) {
            if (toolType_ == 0) return nullopt;
            // Enumerate every incidence-point pair.  A single EPS-near first
            // pair may describe a different line, so each pair must prove the
            // actual goal line before being accepted.
            auto& hits = scratch_[depth].incidences;
            hits.clear();
            g.VisitPointIncidences(goal, [&](uint32_t i) { hits.push_back(i); });
            for (size_t a = 0; a < hits.size(); ++a) {
                for (size_t b = a + 1; b < hits.size(); ++b) {
                    if (CheckTimeout()) return nullopt;
                    ++stats.rawCandidates;
                    Candidate cand{hits[a], hits[b], 2};
                    Element e = g.MakeCandidate(cand);
                    if (!SameElement(e, goal)) continue;
                    if (!g.CandidatePassesForcedTailPoints(e, forcedTailPoints)) {
                        ++stats.forcedPointTailPruned;
                        continue;
                    }
                    if (SymmetryPruned(g, cand, e, depth, previous, stats)) continue;
                    ++stats.uniqueCandidates;
                    return cand;
                }
            }
            return nullopt;
        }

        if (goal.type == Type::Circle) {
            if (toolType_ == 1 || IsZero(goal.c)) return nullopt;
            int centerId = -1;
            const Point center{goal.a, goal.b};
            for (uint32_t i = 0; i < n; ++i) {
                if (CheckTimeout()) return nullopt;
                if (SamePoint(g.points[i], center)) {
                    centerId = static_cast<int>(i);
                    break;
                }
            }
            if (centerId < 0) return nullopt;

            for (uint32_t p = 0; p < n; ++p) {
                if (CheckTimeout()) return nullopt;
                if (static_cast<int>(p) == centerId || !g.PointOnElement(g.points[p], goal))
                    continue;
                ++stats.rawCandidates;
                const uint32_t i = min<uint32_t>(static_cast<uint32_t>(centerId), p);
                const uint32_t j = max<uint32_t>(static_cast<uint32_t>(centerId), p);
                const uint8_t tool = (i == static_cast<uint32_t>(centerId)) ? 0 : 1;
                Candidate cand{i, j, tool};
                Element e = g.MakeCandidate(cand);
                if (!SameElement(e, goal)) continue;
                if (!g.CandidatePassesForcedTailPoints(e, forcedTailPoints)) {
                    ++stats.forcedPointTailPruned;
                    return nullopt;
                }
                if (SymmetryPruned(g, cand, e, depth, previous, stats)) continue;
                ++stats.uniqueCandidates;
                return cand;
            }
        }
        return nullopt;
    }

    // Safe combination #1.  When r == k (remaining operations equal distinct
    // missing target line/circle elements), every future E must itself be a
    // missing target element.  Search only those elements; never enumerate
    // auxiliary lines/circles in this tail.
    bool SearchForcedGoalTail(Graph& g, int remaining, uint16_t depth,
                              const optional<OperationKey>& previous,
                              const vector<uint32_t>& forcedTailPoints,
                              SearchStats& stats) {
        ++stats.forcedGoalTailNodes;

        auto& missing = scratch_[depth].missingGoals;
        missing.clear();
        missing.reserve(g.goalElements.size());
        for (size_t gi = 0; gi < g.goalElements.size(); ++gi) {
            const Element& goal = g.goalElements[gi];
            if (g.HasElement(goal)) continue;
            bool duplicate = false;
            for (size_t j = 0; j < gi; ++j) {
                if (SameElement(g.goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) missing.push_back(gi);
        }
        sort(missing.begin(), missing.end(), [&](size_t a, size_t b) {
            return OperationKey::From(g.goalElements[a]) <
                   OperationKey::From(g.goalElements[b]);
        });

        for (size_t gi : missing) {
            if (CheckTimeout()) return false;
            const Element& goal = g.goalElements[gi];
            auto cand = FindGoalConstructionCandidate(
                g, goal, depth, previous, forcedTailPoints, stats);
            if (!cand) continue;
            const Element e = g.MakeCandidate(*cand);
            const OperationKey key = OperationKey::From(e);
            const Mark mark = g.GetMark();
            g.ApplyKnownNew(e, static_cast<uint16_t>(depth + 1));
            ++stats.applied;
            if (DFS(g, remaining - 1, static_cast<uint16_t>(depth + 1), key, stats))
                return true;
            g.Rollback(mark);
        }
        return false;
    }

    // Build the distinct geometric target points that every candidate in this
    // tail is forced to pass through. Goal-point lists may contain duplicates,
    // so normalize them here before reverse generation.
    void CollectUniqueForcedTailPoints(const Graph& g,
                                       const vector<uint32_t>& forcedTailPoints,
                                       vector<Point>& required) const {
        required.clear();
        required.reserve(forcedTailPoints.size());
        for (uint32_t gi : forcedTailPoints) {
            const Point& p = g.goalPoints[gi];
            bool duplicate = false;
            for (const Point& q : required) {
                if (SamePoint(p, q)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) required.push_back(p);
        }
    }

    // Find one symmetry-compatible known-point representation of an already
    // specified line.  The line geometry is fixed by reverse reasoning; point
    // pairs are searched only to prove that the straightedge can draw it now.
    optional<Candidate> FindReverseLineRepresentation(
        const Graph& g, const Element& line, uint16_t depth,
        const optional<OperationKey>& previous, SearchStats& stats,
        const TailPrefixCache::Entry* cached = nullptr, size_t prefixCount = 0) {
        if (toolType_ == 0) return nullopt;

        uint32_t first = cached ? cached->first : NO_BOUND;
        uint32_t second = cached ? cached->second : NO_BOUND;
        uint32_t newborn = NO_BOUND;
        auto incidence = [&](uint32_t i) {
            if (first == NO_BOUND) first = i;
            else if (second == NO_BOUND) second = i;
            if (depth > 0 && g.pointBirth[i] == depth) newborn = i;
        };
        g.VisitPointIncidences(line, incidence, cached ? static_cast<uint32_t>(prefixCount) : 0);
        if (second == NO_BOUND) return nullopt;

        auto make = [&](uint32_t a, uint32_t b) -> optional<Candidate> {
            if (a == b) return nullopt;
            if (a > b) swap(a, b);
            Candidate cand{a, b, 2};
            ++stats.rawCandidates;
            Element e = g.MakeCandidate(cand);
            if (!SameElement(e, line)) return nullopt;
            if (SymmetryPruned(g, cand, e, depth, previous, stats)) return nullopt;
            return cand;
        };

        if (auto cand = make(first, second)) return cand;

        // If the canonical pair was rejected only by the adjacent-operation
        // partial-order rule, any representation using a point born at the
        // current depth depends on the previous operation and is admissible.
        if (newborn != NO_BOUND) {
            const uint32_t other = (first == newborn) ? second : first;
            if (auto cand = make(newborn, other)) return cand;
        }
        // EPS incidence does not prove that the first pair constructs this
        // line. Preserve the old fast-path order, then verify every other pair
        // just like the v9 target-line construction fix. Never infer equality
        // transitively through the requested line or an earlier failed pair.
        auto& hits = scratch_[depth].incidences;
        hits.clear();
        g.VisitPointIncidences(line, [&](uint32_t i) { hits.push_back(i); });
        for (size_t a = 0; a < hits.size(); ++a) {
            for (size_t b = a + 1; b < hits.size(); ++b) {
                if (a == 0 && b == 1) continue;
                if (newborn != NO_BOUND) {
                    const uint32_t other = first == newborn ? second : first;
                    if (hits[a] == min(newborn, other) &&
                        hits[b] == max(newborn, other)) continue;
                }
                if (CheckTimeout()) return nullopt;
                if (auto cand = make(hits[a], hits[b])) return cand;
            }
        }
        return nullopt;
    }

    // Same as above for a specified circle. The center is fixed and must be a
    // known point; we only search for one known circumference point that gives a
    // symmetry-compatible center->circumference compass operation.
    optional<Candidate> FindReverseCircleRepresentation(
        const Graph& g, uint32_t centerId, const Element& circle, uint16_t depth,
        const optional<OperationKey>& previous, SearchStats& stats,
        const TailPrefixCache::Entry* cached = nullptr, size_t prefixCount = 0) const {
        if (toolType_ == 1 || IsZero(circle.c) || centerId >= g.points.size())
            return nullopt;

        uint32_t first = cached ? cached->circumference : NO_BOUND;
        uint32_t newborn = NO_BOUND;
        auto incidence = [&](uint32_t q) {
            if (q == centerId) return;
            if (first == NO_BOUND) first = q;
            if (depth > 0 && g.pointBirth[q] == depth) newborn = q;
        };
        g.VisitPointIncidences(circle, incidence, cached ? static_cast<uint32_t>(prefixCount) : 0);
        if (first == NO_BOUND) return nullopt;

        auto make = [&](uint32_t q) -> optional<Candidate> {
            const uint32_t i = min(centerId, q);
            const uint32_t j = max(centerId, q);
            const uint8_t tool = (i == centerId) ? 0 : 1;
            Candidate cand{i, j, tool};
            ++stats.rawCandidates;
            Element e = g.MakeCandidate(cand);
            if (!SameElement(e, circle)) return nullopt;
            if (SymmetryPruned(g, cand, e, depth, previous, stats)) return nullopt;
            return cand;
        };

        if (auto cand = make(first)) return cand;

        // If the center itself was born at this depth, the first representation
        // would already have depended on the previous operation. Otherwise try a
        // newly born circumference point, if one exists.
        if (depth > 0 && g.pointBirth[centerId] != depth && newborn != NO_BOUND &&
            newborn != first) {
            if (auto cand = make(newborn)) return cand;
        }
        return nullopt;
    }

    // Reverse-generate every currently constructible line/circle that passes
    // through all forced target points. This replaces the former O(n^2)*tools
    // point-pair stream in the final one/two layers.
    //
    // Completeness:
    //   line   -> if >=2 distinct forced points, its geometry is unique; if one
    //             forced point P, every feasible line is P joined to at least
    //             one known point, and we enumerate those distinct geometries.
    //   circle -> its center must be a known point. For each known center we
    //             test whether all forced points have the same radius and then
    //             whether a known circumference point exists.
    template <class AcceptCandidate>
    bool SearchReverseForcedPointCandidates(
        Graph& g, uint16_t depth, const optional<OperationKey>& previous,
        const vector<uint32_t>& forcedTailPoints, BoundedElementSet& seen,
        SearchStats& stats, bool countStreamDuplicates, bool oneStepTail,
        AcceptCandidate&& acceptCandidate) {
        auto& required = scratch_[depth].requiredPoints;
        CollectUniqueForcedTailPoints(g, forcedTailPoints, required);
        if (required.empty()) return false;

        const vector<TailPrefixCache::Entry>* prefixEntries = nullptr;
        size_t prefixCount = 0;
        if (oneStepTail && activeTailPrefix_ && depth == activeTailDepth_) {
            prefixCount = activeTailPrefix_->PrefixCount();
            prefixEntries = &activeTailPrefix_->Get(g, required[0], toolType_);
        }
        seen.BeginNode();

        auto emit = [&](const Candidate& cand, const Element& e) -> bool {
            if (CheckTimeout()) return false;
            if (!g.CandidatePassesForcedTailPoints(e, forcedTailPoints)) {
                ++stats.forcedPointTailPruned;
                return false;
            }
            if (g.HasElement(e)) {
                ++stats.existingCandidates;
                return false;
            }
            const auto d = seen.Insert(e);
            if (d == BoundedElementSet::InsertResult::Duplicate) {
                ++stats.duplicateCandidates;
                if (countStreamDuplicates) ++stats.streamDuplicates;
                return false;
            }
            if (d == BoundedElementSet::InsertResult::NewUntracked)
                ++stats.streamDedupOverflow;

            ++stats.uniqueCandidates;
            if (oneStepTail) ++stats.tailOneCandidates;
            return acceptCandidate(cand, e);
        };

        // ----- Straightedge candidates -----
        if (toolType_ != 0) {
            if (required.size() >= 2) {
                Element line = Element::FromPoints(required[0], required[1], Type::Line);
                bool allOnLine = true;
                for (size_t k = 2; k < required.size(); ++k) {
                    if (!g.PointOnElement(required[k], line)) {
                        allOnLine = false;
                        break;
                    }
                }
                if (allOnLine) {
                    if (auto cand = FindReverseLineRepresentation(
                            g, line, depth, previous, stats)) {
                        const Element e = g.MakeCandidate(*cand);
                        if (emit(*cand, e)) return true;
                        if (timedOut_) return false;
                    }
                }
            } else {
                // With exactly one forced point P, a feasible new straightedge
                // line must be the line P-Q for some currently known point Q,
                // and at least two known points must lie on that same line.
                const Point& p = required[0];
                auto& triedLines = scratch_[depth].triedLines;
                triedLines.clear();
                triedLines.reserve(g.points.size());
                for (uint32_t q = 0; q < g.points.size(); ++q) {
                    if (CheckTimeout()) return false;
                    const TailPrefixCache::Entry* cached = prefixEntries && q < prefixCount
                        ? &(*prefixEntries)[q] : nullptr;
                    Element line;
                    if (cached) {
                        if (!cached->lineValid || cached->duplicateLine) continue;
                        line = cached->line;
                    } else {
                        if (SamePoint(p, g.points[q])) continue;
                        line = Element::FromPoints(p, g.points[q], Type::Line);
                        bool duplicate = false;
                        for (const Element& old : triedLines) {
                            if (SameElement(old, line)) { duplicate = true; break; }
                        }
                        if (duplicate) continue;
                    }
                    triedLines.push_back(line);

                    if (auto cand = FindReverseLineRepresentation(
                            g, line, depth, previous, stats, cached, prefixCount)) {
                        const Element e = g.MakeCandidate(*cand);
                        if (emit(*cand, e)) return true;
                        if (timedOut_) return false;
                    }
                }
            }
        }

        // ----- Compass candidates -----
        if (toolType_ != 1) {
            const Point& p0 = required[0];
            for (uint32_t centerId = 0; centerId < g.points.size(); ++centerId) {
                if (CheckTimeout()) return false;
                const Point& center = g.points[centerId];
                const TailPrefixCache::Entry* cached = prefixEntries && centerId < prefixCount
                    ? &(*prefixEntries)[centerId] : nullptr;
                if (cached && !cached->circleValid) continue;
                const double r2 = cached ? cached->circle.c
                    : Sq(center.x - p0.x) + Sq(center.y - p0.y);
                if (IsZero(r2)) continue; // no nonzero compass circle can realize this

                bool allOnCircle = true;
                for (size_t k = 1; k < required.size(); ++k) {
                    const double d2 = Sq(center.x - required[k].x) +
                                      Sq(center.y - required[k].y);
                    if (!IsZero(d2 - r2)) {
                        allOnCircle = false;
                        break;
                    }
                }
                if (!allOnCircle) continue;

                Element circle = Element::FromCoefficients(
                    center.x, center.y, r2, Type::Circle);
                if (auto cand = FindReverseCircleRepresentation(
                        g, centerId, circle, depth, previous, stats, cached, prefixCount)) {
                    const Element e = g.MakeCandidate(*cand);
                    if (emit(*cand, e)) return true;
                    if (timedOut_) return false;
                }
            }
        }
        return false;
    }

    // Exact one-step point completion by reverse generation. At this point the
    // joint lower bound guarantees that every still-missing target point has one
    // existing support, hence the sole remaining element must pass through all
    // of them. We generate only such constructible elements.
    bool SearchOneStepPointTail(Graph& g, uint16_t depth,
                                const optional<OperationKey>& previous,
                                const vector<uint32_t>& forcedTailPoints,
                                SearchStats& stats) {
        BoundedElementSet& seen = oneStepSeen_;
        auto accept = [&](const Candidate&, const Element& e) -> bool {
            const Mark mark = g.GetMark();
            g.ApplyKnownNew(e, static_cast<uint16_t>(depth + 1));
            ++stats.applied;
            if (g.GoalsMet() && (!solutions_ || solutions_->Submit(g, parallelControl_, successfulVisits_)))
                return true;
            g.Rollback(mark);
            return false;
        };
        return SearchReverseForcedPointCandidates(
            g, depth, previous, forcedTailPoints, seen, stats,
            false, true, accept);
    }

    // Specialized r=2 reverse tail. It is called only when at least one missing
    // target point has zero existing supports, so both of the final two elements
    // must pass every point in forcedTailPoints. Reverse-generate only currently
    // constructible first elements satisfying that condition; DFS then enters
    // the exact one-step reverse solver above.
    bool StreamForcedPointTailCandidates(
        Graph& g, int remaining, uint16_t depth,
        const optional<OperationKey>& previous,
        const vector<uint32_t>& forcedTailPoints, SearchStats& stats) {
        BoundedElementSet& seen = streamSeen_[depth];
        // Defer the context until a reverse candidate actually survives.
        // Before then the graph is unchanged; no search branch is removed.
        const PreApplyContext* preCtx = nullptr;
        auto accept = [&](const Candidate&, const Element& e) -> bool {
            if (!preCtx) preCtx = &BuildPreApplyContext(g, remaining, depth);
            return TryCandidate(g, e, remaining, depth, stats, *preCtx);
        };
        return SearchReverseForcedPointCandidates(
            g, depth, previous, forcedTailPoints, seen, stats,
            true, false, accept);
    }

    bool TryDirectMissingGoalElements(Graph& g, int remaining, uint16_t depth,
                                      const optional<OperationKey>& previous,
                                      SearchStats& stats,
                                      const PreApplyContext& preCtx) {
        // Goal-priority band 2 does not need a full point-pair scan.  A target
        // line is drawable iff two known points lie on it; a target circle is
        // drawable iff its center and one circumference point are known.  The
        // existing helper finds one symmetry-compatible representation of the
        // exact geometric element, and all representations lead to the same
        // next graph state.
        const vector<uint32_t> noForcedPoints;
        for (size_t gi = 0; gi < g.goalElements.size(); ++gi) {
            if (!preCtx.missingDistinctGoal[gi]) continue;
            const Element& goal = g.goalElements[gi];

            auto cand = FindGoalConstructionCandidate(
                g, goal, depth, previous, noForcedPoints, stats);
            if (!cand) continue;
            const Element e = g.MakeCandidate(*cand);
            if (TryCandidate(g, e, remaining, depth, stats, preCtx))
                return true;
            if (timedOut_) return false;
        }
        return false;
    }

    bool StreamCandidates(Graph& g, int remaining, uint16_t depth,
                          const optional<OperationKey>& previous, SearchStats& stats) {
        const uint32_t n = static_cast<uint32_t>(g.points.size());
        const bool trackRoot = progress_ && depth == 0 && !frontierTaskSink_;
        const double pairCount = static_cast<double>(n) * (n > 0 ? n - 1 : 0) / 2.0;
        auto pairOrdinal = [n](uint32_t i, uint32_t j) {
            return static_cast<double>(i) * (2.0 * n - i - 1) / 2.0 + (j - i - 1);
        };
        if (trackRoot) progress_->SetRootFraction(0.0);
        BoundedElementSet& seen = streamSeen_[depth];
        seen.BeginNode();
        ExactGridLineCache* exactSeen = nullptr;
        if (g.gridMode && g.gridFast) {
            exactSeen = &gridSeen_[depth];
            exactSeen->BeginNode();
        }
        const PreApplyContext& preCtx = BuildPreApplyContext(g, remaining, depth);

        const bool useGoalBands = goalFirst_ && remaining > 1;
        if (useGoalBands) {
            if (TryDirectMissingGoalElements(g, remaining, depth, previous, stats, preCtx)) return true;
            if (timedOut_) return false;
        }
        const bool twoBands = useGoalBands && !preCtx.missingGoalPoints.empty();
        auto& deferred = scratch_[depth].deferred;
        deferred.clear();
        optional<Candidate> resume;

        auto accept = [&](const Element& e) -> bool {
            if (CheckTimeout()) return false;
            if (g.HasElement(e)) {
                ++stats.existingCandidates;
                return false;
            }
            if (exactSeen && !exactSeen->Insert(e)) {
                ++stats.gridExactDuplicates;
                return false;
            }
            if (preCtx.previewGoal >= 0) {
                ++stats.prerequisitePreviewTested;
                if (!CandidateCanSupplyLastElementPrerequisites(g, e, preCtx)) {
                    ++stats.prerequisitePreviewPruned;
                    return false;
                }
            }
            // Keep dedup at acceptance time, in the original priority-band
            // order. Inserting deferred candidates early would change the EPS
            // representative and is unsound because equality is not transitive.
            if (preCtx.previewGoal < 0) {
                const auto dedup = seen.Insert(e);
                if (dedup == BoundedElementSet::InsertResult::Duplicate) {
                    ++stats.duplicateCandidates;
                    ++stats.streamDuplicates;
                    return false;
                }
                if (dedup == BoundedElementSet::InsertResult::NewUntracked)
                    ++stats.streamDedupOverflow;
            }
            ++stats.uniqueCandidates;
            return TryCandidate(g, e, remaining, depth, stats, preCtx, true);
        };

        // Cache only the auxiliary band, with a strict per-depth memory cap.
        // Once full, remember the first uncached operation and rescan exactly
        // that suffix after consuming the cache. This preserves every candidate
        // and its order without a second MakeCandidate/GoalPriority in the
        // common small/medium nodes, or unbounded O(P^2) storage in large ones.
        auto scan = [&](bool suffix, Candidate start) -> bool {
            for (uint32_t i = start.i; i < n; ++i) {
                if (CheckTimeout()) return false;
                const uint32_t firstJ = i == start.i ? max(i + 1, start.j) : i + 1;
                for (uint32_t j = firstJ; j < n; ++j) {
                    if (CheckTimeout()) return false;
                    auto consider = [&](uint8_t tool) -> bool {
                        if (i == start.i && j == start.j && tool < start.tool) return false;
                        if (CheckTimeout()) return false;
                        ++stats.rawCandidates;
                        const Candidate cand{i, j, tool};
                        const Element e = g.MakeCandidate(cand);
                        if (SymmetryPruned(g, cand, e, depth, previous, stats)) return false;
                        const int priority = (useGoalBands || remaining == 1)
                            ? GoalPriority(g, e, preCtx) : 0;
                        if (remaining == 1 && priority == 0) {
                            ++stats.finalStepPruned;
                            return false;
                        }
                        if (useGoalBands) {
                            if (priority == 2) return false; // direct-goal band above
                            if (twoBands && !suffix && priority == 0) {
                                if (!resume) {
                                    if (deferred.size() < DEFERRED_CANDIDATE_LIMIT)
                                        deferred.push_back(e);
                                    else
                                        resume = cand;
                                }
                                return false;
                            }
                            if (suffix && priority != 0) return false;
                        }
                        return accept(e);
                    };
                    if (toolType_ == 0 || toolType_ == 2) {
                        if (consider(0)) return true;
                        if (timedOut_) return false;
                        if (consider(1)) return true;
                        if (timedOut_) return false;
                    }
                    if (toolType_ == 1 || toolType_ == 2) {
                        if (consider(2)) return true;
                        if (timedOut_) return false;
                    }
                    if (trackRoot && pairCount > 0.0) {
                        const double done = pairOrdinal(i, j) + 1.0;
                        if (!twoBands) progress_->SetRootFraction(done / pairCount);
                        else if (!suffix) progress_->SetRootFraction(0.5 * done / pairCount);
                        else {
                            const double startPair = pairOrdinal(start.i, start.j);
                            progress_->SetRootFraction(0.75 + 0.25 *
                                (done - startPair) / max(1.0, pairCount - startPair));
                        }
                    }
                }
            }
            return false;
        };
        if (scan(false, Candidate{0, 1, 0})) return true;
        if (CheckTimeout()) return false;
        size_t deferredDone = 0;
        for (const Element& e : deferred) {
            if (accept(e)) return true;
            if (CheckTimeout()) return false;
            if (trackRoot) progress_->SetRootFraction(0.5 + (resume ? 0.25 : 0.5) *
                static_cast<double>(++deferredDone) / deferred.size());
        }
        if (resume && scan(true, *resume)) return true;
        if (trackRoot && !timedOut_) progress_->SetRootFraction(1.0);
        return false;
    }


#if defined(__GNUC__) || defined(__clang__)
    __attribute__((noinline))
#elif defined(_MSC_VER)
    __declspec(noinline)
#endif
    bool EnumerateSatisfied(Graph& g, int remaining, uint16_t depth,
                            const optional<OperationKey>& previous,
                            SearchStats& stats) {
        // Frontier-width probes must traverse through already-satisfied states
        // without storing solutions: such states can still have distinct
        // extensions that belong to the probed frontier. Ordinary searches keep
        // the original solution-submission behavior unchanged.
        if (!frontierProbeMode_) {
            if (!solutions_) return true; // compatibility for internal single-goal callers
            if (solutions_->Submit(g, parallelControl_, successfulVisits_)) return true;
        }
        if (remaining <= 0) return false;
        const uint32_t n = static_cast<uint32_t>(g.points.size());
        BoundedElementSet& seen = streamSeen_[depth];
        seen.BeginNode();
        for (uint32_t i = 0; i < n; ++i) {
            for (uint32_t j = i + 1; j < n; ++j) {
                auto consider = [&](uint8_t tool) -> bool {
                    if (CheckTimeout()) return false;
                    ++stats.rawCandidates;
                    const Candidate cand{i, j, tool};
                    const Element e = g.MakeCandidate(cand);
                    if (SymmetryPruned(g, cand, e, depth, previous, stats)) return false;
                    if (g.HasElement(e)) {
                        ++stats.existingCandidates;
                        return false;
                    }
                    if (seen.Insert(e) == BoundedElementSet::InsertResult::Duplicate) {
                        ++stats.duplicateCandidates;
                        return false;
                    }
                    ++stats.uniqueCandidates;
                    const Mark mark = g.GetMark();
                    g.ApplyKnownNew(e, static_cast<uint16_t>(depth + 1));
                    ++stats.applied;
                    ++stats.nodes;
                    stats.maxPoints = max(stats.maxPoints, g.points.size());
                    stats.maxElements = max(stats.maxElements, g.elements.size());
                    if (frontierTaskSink_ && depth + 1 >= frontierDepth_) {
                        // Already-satisfied states may have additional distinct
                        // extensions. Partition those at the same frontier, so
                        // parallel enumeration does not lose extra solutions.
                        if (!(*frontierTaskSink_)(g)) frontierStopped_ = true;
                    } else if (EnumerateSatisfied(g, remaining - 1,
                            static_cast<uint16_t>(depth + 1),
                            OperationKey::From(e), stats)) return true;
                    g.Rollback(mark);
                    return false;
                };
                if (toolType_ != 1) {
                    if (consider(0)) return true;
                    if (CheckTimeout()) return false;
                    if (consider(1)) return true;
                    if (CheckTimeout()) return false;
                }
                if (toolType_ != 0) {
                    if (consider(2)) return true;
                    if (CheckTimeout()) return false;
                }
            }
        }
        return false;
    }

    bool DFS(Graph& g, int remaining, uint16_t depth,
             const optional<OperationKey>& previous, SearchStats& stats) {
        progressDepth_ = depth;
        if (CheckTimeout()) return false;
        ++stats.nodes;
        stats.maxPoints = max(stats.maxPoints, g.points.size());
        stats.maxElements = max(stats.maxElements, g.elements.size());

        if (frontierTaskSink_ && depth >= frontierDepth_) {
            // Ownership of this entire subtree transfers to exactly one worker,
            // including goal collection and enumeration of satisfied extensions.
            // The caller rolls back; the worker restores this exact operation
            // prefix (not merely its unordered set of geometric elements).
            if (!(*frontierTaskSink_)(g)) frontierStopped_ = true;
            return false;
        }

        if (g.GoalsMet()) return EnumerateSatisfied(g, remaining, depth, previous, stats);

        const uint16_t rem16 = static_cast<uint16_t>(max(0, remaining));
        if (tt_.WasFailed(g.StateHash1(), g.StateHash2(),
                          static_cast<uint32_t>(g.points.size()),
                          static_cast<uint32_t>(g.elements.size()), rem16)) {
            ++stats.transpositionPruned;
            return false;
        }

        if (remaining == 0) {
            tt_.StoreFailed(g.StateHash1(), g.StateHash2(),
                            static_cast<uint32_t>(g.points.size()),
                            static_cast<uint32_t>(g.elements.size()), rem16);
            return false;
        }

        const int missingGoalElements = g.MissingDistinctGoalElementCount();
        if (missingGoalElements > remaining) {
            ++stats.goalElementLowerBoundPruned;
            tt_.StoreFailed(g.StateHash1(), g.StateHash2(),
                            static_cast<uint32_t>(g.points.size()),
                            static_cast<uint32_t>(g.elements.size()), rem16);
            return false;
        }

        // Safe combination #1: after reserving k operations for all distinct
        // missing target elements, the remaining auxiliary budget must still be
        // sufficient to supply two incidences for every missing target point.
        const int auxiliaryBudget = remaining - missingGoalElements;
        if (g.RequiredAuxiliaryStepsForGoalPoints() > auxiliaryBudget) {
            ++stats.jointPointLowerBoundPruned;
            tt_.StoreFailed(g.StateHash1(), g.StateHash2(),
                            static_cast<uint32_t>(g.points.size()),
                            static_cast<uint32_t>(g.elements.size()), rem16);
            return false;
        }

        // Safe combination #2.  If k==r, every remaining E must be a missing
        // target element.  Keep the original v6 one-step reachability test as a
        // very cheap fast path: line/circle-only searches hit this node millions
        // of times, so avoiding the more general tail routine when no target is
        // drawable is important for throughput.
        if (missingGoalElements == remaining && missingGoalElements > 0) {
            bool anyDrawable = true;
            if (remaining == 1)
                anyDrawable = g.MissingGoalElementsDrawableInOneStep(toolType_);
            else
                anyDrawable = g.AnyMissingGoalElementDrawableNow(toolType_);
            if (!anyDrawable) {
                ++stats.exactReachabilityPruned;
                tt_.StoreFailed(g.StateHash1(), g.StateHash2(),
                                static_cast<uint32_t>(g.points.size()),
                                static_cast<uint32_t>(g.elements.size()), rem16);
                return false;
            }
        }

        // Safe combination #3: precompute target points that force every one of
        // the final one/two constructions to pass through them.  Candidate
        // checks below become O(number of forced points), with no element scan.
        TailPrefixScope tailPrefixScope(*this, g, remaining, depth);
        auto& forcedTailPoints = scratch_[depth].forcedTailPoints;
        forcedTailPoints.clear();
        if (remaining <= 2 && !g.goalPoints.empty())
            g.CollectForcedTailPointIndices(remaining, forcedTailPoints);

        if (missingGoalElements == remaining && missingGoalElements > 0) {
            const bool found = SearchForcedGoalTail(
                g, remaining, depth, previous, forcedTailPoints, stats);
            if (!found && !timedOut_) {
                tt_.StoreFailed(g.StateHash1(), g.StateHash2(),
                                static_cast<uint32_t>(g.points.size()),
                                static_cast<uint32_t>(g.elements.size()), rem16);
            }
            return found;
        }

        // Safe combination #4: with one operation left and no missing target
        // element, solve the point-only tail directly instead of entering the
        // generic candidate/DFS machinery.
        if (remaining == 1) {
            const bool found = SearchOneStepPointTail(
                g, depth, previous, forcedTailPoints, stats);
            if (!found && !timedOut_) {
                tt_.StoreFailed(g.StateHash1(), g.StateHash2(),
                                static_cast<uint32_t>(g.points.size()),
                                static_cast<uint32_t>(g.elements.size()), rem16);
            }
            return found;
        }

        bool found = false;
        if (lowMemory_) {
            if (remaining == 2 && !forcedTailPoints.empty())
                found = StreamForcedPointTailCandidates(
                    g, remaining, depth, previous, forcedTailPoints, stats);
            else
                found = StreamCandidates(g, remaining, depth, previous, stats);
        } else {
            const PreApplyContext& preCtx = BuildPreApplyContext(g, remaining, depth);
            const auto& candidates = GenerateUniqueCandidates(g, depth, preCtx, stats);
            if (timedOut_) return false;
            size_t candidateIndex = 0;
            for (const PreparedCandidate& prepared : candidates) {
                if (progress_ && depth == 0 && !frontierTaskSink_)
                    progress_->SetRootFraction(static_cast<double>(candidateIndex++) / candidates.size());
                if (CheckTimeout()) return false;
                const Candidate& cand = prepared.candidate;
                const Element& e = prepared.element;
                if (SymmetryPruned(g, cand, e, depth, previous, stats)) continue;
                if (!forcedTailPoints.empty() && !TailCandidateAllowed(g, e, forcedTailPoints, stats)) continue;
                if (remaining == 1 && !g.IsGoalDirected(e)) {
                    ++stats.finalStepPruned;
                    continue;
                }
                // GenerateUniqueCandidates already proved that e is new in this node.
                if (TryCandidate(g, e, remaining, depth, stats, preCtx)) {
                    found = true;
                    break;
                }
            }
        }

        if (!found && !timedOut_) {
            tt_.StoreFailed(g.StateHash1(), g.StateHash2(),
                            static_cast<uint32_t>(g.points.size()),
                            static_cast<uint32_t>(g.elements.size()), rem16);
        }
        return found;
    }

public:
    void SetExternalStop(const atomic<bool>* stop) { externalStop_ = stop; }
    void SetProgress(ProgressSlot* progress) { progress_ = progress; }
    void SetSuccessfulVisits(uint64_t* visits) { successfulVisits_ = visits; }
    void SetSolutionCollector(SolutionCollector* solutions) { solutions_ = solutions; }

    Solver(int toolType, bool symmetry, bool goalFirst, bool lowMemory,
           size_t streamDedupEntries, size_t ttBytes, double timeLimitSeconds)
        : toolType_(toolType), symmetry_(symmetry), goalFirst_(goalFirst),
          lowMemory_(lowMemory), streamDedupEntries_(streamDedupEntries),
          ttBytes_(ttBytes), timeLimitSeconds_(timeLimitSeconds) {}

    // Traverse in ordinary DFS / goal-priority order down to the frontier.
    // Queue backpressure bounds look-ahead and memory; workers start as soon
    // as the first prefix is available, without a full-frontier prepass.
    bool ProduceFrontierTasks(Graph& g, int limit, SearchStats& stats,
                          ParallelControl* control,
                          function<bool(const Graph&)>& sink, uint16_t splitDepth) {
        ProgressScope progressScope(*this, stats);
        scratch_.resize(static_cast<size_t>(limit) + 1);
        timedOut_ = false;
        timeoutPollCounter_ = 1023u;
        parallelControl_ = control;
        frontierTaskSink_ = &sink;
        frontierDepth_ = splitDepth;
        frontierStopped_ = false;
        frontierProbeMode_ = false;
        deadline_ = control ? control->deadline
                            : chrono::steady_clock::now() + chrono::duration_cast<chrono::steady_clock::duration>(
                                  chrono::duration<double>(timeLimitSeconds_));
        streamSeen_.clear();
        streamSeen_.resize(static_cast<size_t>(limit) + 1);
        if (g.gridMode && g.gridFast) gridSeen_.resize(static_cast<size_t>(limit) + 1);
        if (lowMemory_) {
            for (BoundedElementSet& set : streamSeen_) set.Configure(streamDedupEntries_);
        }
        if (!g.goalPoints.empty())
            oneStepSeen_.Configure(max<size_t>(streamDedupEntries_, 4096));
        tt_.Configure(0);
        g.SetStateHashingEnabled(false);
        const bool found = DFS(g, limit, 0, nullopt, stats);
        frontierTaskSink_ = nullptr;
        return found;
    }

    struct FrontierProbeResult {
        size_t tasks = 0;
        bool thresholdReached = false;
        bool timedOut = false;
        SearchStats stats;
    };

    // Count a frontier only until threshold tasks have been seen. This is a
    // planning pass, not a search pass: it never submits solutions and never
    // stores Graph snapshots. If a frontier is narrower than threshold the DFS
    // exhausts that frontier exactly; if it is wider, traversal stops at the
    // threshold so millions of unnecessary prefixes are not generated.
    FrontierProbeResult ProbeFrontierTasks(
            Graph& g, int limit, uint16_t splitDepth, size_t threshold,
            chrono::steady_clock::time_point probeDeadline) {
        FrontierProbeResult result;
        scratch_.resize(static_cast<size_t>(limit) + 1);
        timedOut_ = false;
        timeoutPollCounter_ = 1023u;
        parallelControl_ = nullptr;
        frontierDepth_ = splitDepth;
        frontierStopped_ = false;
        frontierProbeMode_ = true;
        deadline_ = probeDeadline;
        streamSeen_.clear();
        streamSeen_.resize(static_cast<size_t>(limit) + 1);
        gridSeen_.clear();
        if (g.gridMode && g.gridFast) gridSeen_.resize(static_cast<size_t>(limit) + 1);
        if (lowMemory_) {
            for (BoundedElementSet& set : streamSeen_) set.Configure(streamDedupEntries_);
        }
        if (!g.goalPoints.empty())
            oneStepSeen_.Configure(max<size_t>(streamDedupEntries_, 4096));
        tt_.Configure(0);
        g.SetStateHashingEnabled(false);

        function<bool(const Graph&)> sink = [&](const Graph&) {
            ++result.tasks;
            if (result.tasks >= threshold) {
                result.thresholdReached = true;
                return false;
            }
            return true;
        };
        frontierTaskSink_ = &sink;
        DFS(g, limit, 0, nullopt, result.stats);
        frontierTaskSink_ = nullptr;
        frontierProbeMode_ = false;
        result.timedOut = timedOut_;
        return result;
    }

    // Search an exclusively claimed prefix. Replay reconstructs point births
    // and the previous-operation symmetry key exactly. Reuse scratch storage;
    // each per-node candidate set is reset by its existing BeginNode() call.
    bool SearchPrefixTask(Graph& g, int limit, const PrefixTask& task,
                          SearchStats& stats, ParallelControl* control) {
        ProgressScope progressScope(*this, stats);
        scratch_.resize(static_cast<size_t>(limit) + 1);
        timedOut_ = false;
        timeoutPollCounter_ = 1023u;
        parallelControl_ = control;
        frontierTaskSink_ = nullptr;
        frontierStopped_ = false;
        frontierProbeMode_ = false;
        deadline_ = control ? control->deadline
                            : chrono::steady_clock::now() + chrono::duration_cast<chrono::steady_clock::duration>(
                                  chrono::duration<double>(timeLimitSeconds_));
        if (streamSeen_.size() != static_cast<size_t>(limit) + 1) {
            streamSeen_.resize(static_cast<size_t>(limit) + 1);
            if (g.gridMode && g.gridFast) gridSeen_.resize(static_cast<size_t>(limit) + 1);
            if (lowMemory_)
                for (BoundedElementSet& set : streamSeen_) set.Configure(streamDedupEntries_);
            if (!g.goalPoints.empty())
                oneStepSeen_.Configure(max<size_t>(streamDedupEntries_, 4096));
        }
        // Preserve the original conservative policy: parallel tasks never
        // share/reuse a failed-state transposition table.
        tt_.Configure(0);
        g.SetStateHashingEnabled(false);

        optional<OperationKey> previous;
        uint16_t depth = 0;
        for (const Element& e : task.prefix) {
            if (CheckTimeout()) return false;
            if (g.HasElement(e)) return false; // defensive: a valid replayed prefix never hits this
            g.ApplyKnownNew(e, static_cast<uint16_t>(depth + 1));
            previous = OperationKey::From(e);
            ++depth;
        }
        const int remaining = max(0, limit - static_cast<int>(depth));
        return DFS(g, remaining, depth, previous, stats);
    }

    bool Search(Graph& g, int limit, SearchStats& stats) {
        ProgressScope progressScope(*this, stats);
        scratch_.resize(static_cast<size_t>(limit) + 1);
        timedOut_ = false;
        timeoutPollCounter_ = 1023u;
        parallelControl_ = nullptr;
        frontierTaskSink_ = nullptr;
        frontierStopped_ = false;
        deadline_ = chrono::steady_clock::now() + chrono::duration_cast<chrono::steady_clock::duration>(
                                  chrono::duration<double>(timeLimitSeconds_));
        streamSeen_.resize(static_cast<size_t>(limit) + 1);
        if (g.gridMode && g.gridFast) gridSeen_.resize(static_cast<size_t>(limit) + 1);
        if (lowMemory_) {
            for (BoundedElementSet& set : streamSeen_) set.Configure(streamDedupEntries_);
        }
        if (!g.goalPoints.empty())
            oneStepSeen_.Configure(max<size_t>(streamDedupEntries_, 4096));
        // The ordinary single-thread path retains its original TT policy.
        tt_.Configure(symmetry_ ? 0 : ttBytes_);
        g.SetStateHashingEnabled(tt_.Enabled());
        return DFS(g, limit, 0, nullopt, stats);
    }

    size_t TranspositionBytes() const { return tt_.Bytes(); }
    bool TimedOut() const { return timedOut_; }
    double TimeLimitSeconds() const { return timeLimitSeconds_; }

};

} // namespace bs
