#include <algorithm>
#include <atomic>
#include <array>
#include <cassert>
#include <bit>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <exception>
#include <functional>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <charconv>
#include <sstream>
#include <string>
#include <tuple>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif
#if defined(__SSE2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#define BS_HAS_SSE2 1
#include <emmintrin.h>
#else
#define BS_HAS_SSE2 0
#endif

using namespace std;

namespace {
double EPS = 1e-11;
constexpr uint32_t NO_BOUND = numeric_limits<uint32_t>::max();
constexpr double DEFAULT_TIME_LIMIT_SECONDS = 30.0;

inline bool IsZero(double v) { return std::abs(v) < EPS; }
inline double Sq(double v) { return v * v; }
inline double CleanZero(double v) { return IsZero(v) ? 0.0 : v; }

enum class Type : uint8_t { Circle = 0, Line = 1, Ray = 2, Segment = 3 };

struct Point {
    double x = 0.0;
    double y = 0.0;
};

struct Bound {
    Point p1;
    Point p2;
};

struct Element {
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    uint32_t bound = NO_BOUND;
    Type type = Type::Line;

    static void NormalizeOriginal(double& a, double& b, double& c) {
        if (!IsZero(b)) {
            a /= b;
            c /= b;
            b = 1.0;
        } else if (!IsZero(a)) {
            c /= a;
            a = 1.0;
            b = 0.0;
        }
        a = CleanZero(a);
        b = CleanZero(b);
        c = CleanZero(c);
    }

    static Element FromCoefficients(double aa, double bb, double cc, Type t,
                                    uint32_t boundIndex = NO_BOUND) {
        Element e;
        e.a = aa;
        e.b = bb;
        e.c = cc;
        e.type = t;
        e.bound = boundIndex;
        if (t == Type::Line) NormalizeOriginal(e.a, e.b, e.c);
        return e;
    }

    static Element FromPoints(const Point& p1, const Point& p2, Type t,
                              uint32_t boundIndex = NO_BOUND) {
        Element e;
        e.type = t;
        e.bound = boundIndex;
        if (t == Type::Circle) {
            e.a = p1.x;
            e.b = p1.y;
            e.c = Sq(p1.x - p2.x) + Sq(p1.y - p2.y);
        } else {
            e.a = p2.y - p1.y;
            e.b = p1.x - p2.x;
            e.c = p1.x * p2.y - p1.y * p2.x;
            NormalizeOriginal(e.a, e.b, e.c);
        }
        e.a = CleanZero(e.a);
        e.b = CleanZero(e.b);
        e.c = CleanZero(e.c);
        return e;
    }
};

inline bool SamePoint(const Point& p, const Point& q) {
    return IsZero(p.x - q.x) && IsZero(p.y - q.y);
}

inline bool SameElement(const Element& e, const Element& f) {
    return e.type == f.type && IsZero(e.a - f.a) && IsZero(e.b - f.b) &&
           IsZero(e.c - f.c);
}

inline int64_t Quantize(double v) {
    long double q = floor(static_cast<long double>(v) / EPS);
    constexpr long double lo = static_cast<long double>(numeric_limits<int64_t>::min() + 4);
    constexpr long double hi = static_cast<long double>(numeric_limits<int64_t>::max() - 4);
    if (q < lo) return numeric_limits<int64_t>::min() + 4;
    if (q > hi) return numeric_limits<int64_t>::max() - 4;
    return static_cast<int64_t>(q);
}

inline size_t Mix(size_t h, uint64_t v) {
    v += 0x9e3779b97f4a7c15ULL;
    v = (v ^ (v >> 30)) * 0xbf58476d1ce4e5b9ULL;
    v = (v ^ (v >> 27)) * 0x94d049bb133111ebULL;
    v ^= v >> 31;
    return h ^ (static_cast<size_t>(v) + 0x9e3779b9U + (h << 6) + (h >> 2));
}

inline uint64_t SplitMix64(uint64_t v) {
    v += 0x9e3779b97f4a7c15ULL;
    v = (v ^ (v >> 30)) * 0xbf58476d1ce4e5b9ULL;
    v = (v ^ (v >> 27)) * 0x94d049bb133111ebULL;
    return v ^ (v >> 31);
}

inline uint64_t Bits(double v) {
    return bit_cast<uint64_t>(CleanZero(v));
}

inline uint64_t HashPointRaw(const Point& p, uint64_t seed) {
    uint64_t h = SplitMix64(seed ^ Bits(p.x));
    return SplitMix64(h ^ Bits(p.y));
}

inline uint64_t HashElementRaw(const Element& e, uint64_t seed) {
    uint64_t h = SplitMix64(seed ^ static_cast<uint8_t>(e.type));
    h = SplitMix64(h ^ Bits(e.a));
    h = SplitMix64(h ^ Bits(e.b));
    h = SplitMix64(h ^ Bits(e.c));
    // Rays/segments with the same supporting line can still be different
    // geometric objects because their bounded ranges differ.  The bound index
    // is stable for the lifetime of a search and distinguishes those initial
    // bounded objects in the optional transposition-table state hash.
    return SplitMix64(h ^ static_cast<uint64_t>(e.bound));
}

struct PointBucket {
    int64_t x, y;
    bool operator==(const PointBucket&) const = default;
};
struct PointBucketHash {
    size_t operator()(const PointBucket& k) const noexcept {
        size_t h = 0;
        h = Mix(h, static_cast<uint64_t>(k.x));
        return Mix(h, static_cast<uint64_t>(k.y));
    }
};

struct ElementBucket {
    int64_t a, b, c;
    uint8_t type;
    bool operator==(const ElementBucket&) const = default;
};
struct ElementBucketHash {
    size_t operator()(const ElementBucket& k) const noexcept {
        size_t h = Mix(0, static_cast<uint64_t>(k.a));
        h = Mix(h, static_cast<uint64_t>(k.b));
        h = Mix(h, static_cast<uint64_t>(k.c));
        return Mix(h, k.type);
    }
};

PointBucket BucketOf(const Point& p) { return {Quantize(p.x), Quantize(p.y)}; }
ElementBucket BucketOf(const Element& e) {
    return {Quantize(e.a), Quantize(e.b), Quantize(e.c), static_cast<uint8_t>(e.type)};
}

struct OperationKey {
    Type type;
    double a, b, c;

    static OperationKey From(const Element& e) {
        return {e.type, CleanZero(e.a), CleanZero(e.b), CleanZero(e.c)};
    }
    bool operator<(const OperationKey& o) const {
        return tuple(static_cast<uint8_t>(type), a, b, c) <
               tuple(static_cast<uint8_t>(o.type), o.a, o.b, o.c);
    }
};

struct Candidate {
    uint32_t i = 0;
    uint32_t j = 0;
    uint8_t tool = 0; // 0: circle i->j, 1: circle j->i, 2: line ij
};

struct Mark {
    size_t pointCount = 0;
    size_t elementCount = 0;
};

struct SearchStats {
    uint64_t nodes = 0;
    uint64_t rawCandidates = 0;
    uint64_t uniqueCandidates = 0;
    uint64_t duplicateCandidates = 0;
    uint64_t existingCandidates = 0;
    uint64_t symmetryPruned = 0;
    uint64_t applied = 0;
    uint64_t lowerBoundPruned = 0;
    uint64_t transpositionPruned = 0;
    uint64_t streamDuplicates = 0;
    uint64_t streamDedupOverflow = 0;
    uint64_t finalStepPruned = 0;
    uint64_t exactReachabilityPruned = 0;
    uint64_t goalElementLowerBoundPruned = 0;
    uint64_t jointPointLowerBoundPruned = 0;
    uint64_t forcedGoalTailNodes = 0;
    uint64_t forcedPointTailPruned = 0;
    uint64_t tailOneCandidates = 0;
    uint64_t preApplyLowerBoundPruned = 0;
    uint64_t prerequisitePreviewTested = 0;
    uint64_t prerequisitePreviewPruned = 0;
    uint64_t gridExactDuplicates = 0;
    size_t maxPoints = 0;
    size_t maxElements = 0;
};

void MergeSearchStats(SearchStats& dst, const SearchStats& src) {
    dst.nodes += src.nodes;
    dst.rawCandidates += src.rawCandidates;
    dst.uniqueCandidates += src.uniqueCandidates;
    dst.duplicateCandidates += src.duplicateCandidates;
    dst.existingCandidates += src.existingCandidates;
    dst.symmetryPruned += src.symmetryPruned;
    dst.applied += src.applied;
    dst.lowerBoundPruned += src.lowerBoundPruned;
    dst.transpositionPruned += src.transpositionPruned;
    dst.streamDuplicates += src.streamDuplicates;
    dst.streamDedupOverflow += src.streamDedupOverflow;
    dst.finalStepPruned += src.finalStepPruned;
    dst.exactReachabilityPruned += src.exactReachabilityPruned;
    dst.goalElementLowerBoundPruned += src.goalElementLowerBoundPruned;
    dst.jointPointLowerBoundPruned += src.jointPointLowerBoundPruned;
    dst.forcedGoalTailNodes += src.forcedGoalTailNodes;
    dst.forcedPointTailPruned += src.forcedPointTailPruned;
    dst.tailOneCandidates += src.tailOneCandidates;
    dst.preApplyLowerBoundPruned += src.preApplyLowerBoundPruned;
    dst.prerequisitePreviewTested += src.prerequisitePreviewTested;
    dst.prerequisitePreviewPruned += src.prerequisitePreviewPruned;
    dst.gridExactDuplicates += src.gridExactDuplicates;
    dst.maxPoints = max(dst.maxPoints, src.maxPoints);
    dst.maxElements = max(dst.maxElements, src.maxElements);
}

struct ParallelControl {
    atomic<bool> stop{false};
    atomic<bool> found{false};
    atomic<bool> timedOut{false};
    chrono::steady_clock::time_point deadline{};
};

struct PrefixTask {
    vector<Element> prefix;
};


class PrefixTaskQueue {
    ParallelControl& control_;
    const size_t capacity_;
    mutable mutex mutex_;
    condition_variable notEmpty_, notFull_;
    deque<PrefixTask> tasks_;
    bool closed_ = false;
    size_t produced_ = 0, claimed_ = 0, peak_ = 0;

    void PublishTimeout() {
        if (!control_.found.load(memory_order_acquire)) {
            control_.timedOut.store(true, memory_order_release);
            control_.stop.store(true, memory_order_release);
        }
        notEmpty_.notify_all();
        notFull_.notify_all();
    }
public:
    struct Stats { size_t produced, claimed, peak; };
    PrefixTaskQueue(ParallelControl& control, size_t capacity)
        : control_(control), capacity_(max<size_t>(1, capacity)) {}

    bool Push(const vector<Element>& elements, size_t begin) {
        unique_lock<mutex> lock(mutex_);
        const auto ready = [&] {
            return closed_ || control_.stop.load(memory_order_acquire) ||
                   tasks_.size() < capacity_;
        };
        if (!notFull_.wait_until(lock, control_.deadline, ready)) {
            PublishTimeout();
            return false;
        }
        if (closed_ || control_.stop.load(memory_order_acquire)) return false;
        if (chrono::steady_clock::now() >= control_.deadline) {
            PublishTimeout();
            return false;
        }
        tasks_.push_back(PrefixTask{vector<Element>(elements.begin() + begin, elements.end())});
        ++produced_;
        peak_ = max(peak_, tasks_.size());
        lock.unlock();
        notEmpty_.notify_one();
        return true;
    }

    bool Pop(PrefixTask& task) {
        unique_lock<mutex> lock(mutex_);
        const auto ready = [&] {
            return closed_ || control_.stop.load(memory_order_acquire) ||
                   !tasks_.empty();
        };
        if (!notEmpty_.wait_until(lock, control_.deadline, ready)) {
            PublishTimeout();
            return false;
        }
        if (control_.stop.load(memory_order_acquire) || tasks_.empty()) return false;
        if (chrono::steady_clock::now() >= control_.deadline) {
            PublishTimeout();
            return false;
        }
        task = std::move(tasks_.front());
        tasks_.pop_front();
        ++claimed_;
        lock.unlock();
        notFull_.notify_one();
        return true;
    }

    void Close() {
        {
            lock_guard<mutex> lock(mutex_);
            closed_ = true;
        }
        notEmpty_.notify_all();
        notFull_.notify_all();
    }

    Stats GetStats() const {
        lock_guard<mutex> lock(mutex_);
        return {produced_, claimed_, peak_};
    }
};

class Graph {
public:
    // Small-search optimization: points/elements are deduplicated by direct
    // linear scans. For the intended shallow construction searches, these
    // sets stay small, avoiding unordered_map bucket hashing and rollback cost.
    vector<Point> points;
    vector<uint16_t> pointBirth;
    vector<Element> elements;
    vector<Bound> bounds;
    vector<Point> goalPoints;
    vector<Element> goalElements;
    size_t initialElementCount = 0;

    // Mode 3: the allowed point domain is a CLOSED rectangle. No EPS-expanded
    // exterior strip and no implicit clipping of an outside point onto an edge.
    // Curves keep their supporting equations; only in-domain intersections can
    // enter the known-point set or be used by any reverse/preview search.
    bool gridMode = false;
    bool gridFast = true;
    bool gridLinesReady = false;
    int gridM = 0, gridN = 0;

    bool PointAllowed(const Point& p) const {
        return !gridMode || (isfinite(p.x) && isfinite(p.y) &&
            p.x >= 0.0 && p.x <= gridM && p.y >= 0.0 && p.y <= gridN);
    }

    bool IsAutomaticGridLine(const Element& e) const {
        if (!gridMode || !gridLinesReady || EPS >= 0.5 || e.type != Type::Line) return false;
        if (e.a == 1.0 && e.b == 0.0)
            return e.c >= 0.0 && e.c <= gridM && static_cast<int>(e.c) == e.c;
        if (e.a == 0.0 && e.b == 1.0)
            return e.c >= 0.0 && e.c <= gridN && static_cast<int>(e.c) == e.c;
        return false;
    }

private:
    uint64_t stateHash1_ = 0x243f6a8885a308d3ULL;
    uint64_t stateHash2_ = 0x13198a2e03707344ULL;
    bool stateHashEnabled_ = false;

    void TogglePointHash(const Point& p) {
        if (!stateHashEnabled_) return;
        stateHash1_ ^= HashPointRaw(p, 0xa4093822299f31d0ULL);
        stateHash2_ ^= HashPointRaw(p, 0x082efa98ec4e6c89ULL);
    }

    void ToggleElementHash(const Element& e) {
        if (!stateHashEnabled_) return;
        stateHash1_ ^= HashElementRaw(e, 0x452821e638d01377ULL);
        stateHash2_ ^= HashElementRaw(e, 0xbe5466cf34e90c6cULL);
    }

    bool IsInRange(const Element& e, const Point& p) const {
        if (e.type == Type::Line) return true;
        if (e.type == Type::Ray || e.type == Type::Segment) {
            if (e.bound == NO_BOUND || e.bound >= bounds.size()) return false;
            const Bound& bd = bounds[e.bound];
            if (e.type == Type::Ray) {
                const double dotx = (bd.p1.x - p.x) * (bd.p1.x - bd.p2.x);
                const double doty = (bd.p1.y - p.y) * (bd.p1.y - bd.p2.y);
                return dotx + doty > -EPS;
            }
            const double dotx = (bd.p1.x - p.x) * (bd.p2.x - p.x);
            const double doty = (bd.p1.y - p.y) * (bd.p2.y - p.y);
            return dotx + doty < EPS;
        }
        return false;
    }


    template <class Visitor>
    bool VisitLineCircle(const Element& line, const Element& circle,
                         Visitor&& visitor) const {
        const double denom = Sq(line.a) + Sq(line.b);
        if (IsZero(denom)) return false;
        const double dist = line.a * circle.a + line.b * circle.b - line.c;
        const double delta = denom * circle.c - Sq(dist);
        if (IsZero(delta)) {
            Point p{circle.a - line.a * dist / denom,
                    circle.b - line.b * dist / denom};
            if (IsInRange(line, p) && visitor(p)) return true;
        } else if (delta > EPS) {
            const double root = sqrt(delta);
            Point p1{circle.a - (line.a * dist + line.b * root) / denom,
                     circle.b - (line.b * dist - line.a * root) / denom};
            Point p2{circle.a - (line.a * dist - line.b * root) / denom,
                     circle.b - (line.b * dist + line.a * root) / denom};
            if (IsInRange(line, p1) && visitor(p1)) return true;
            if (IsInRange(line, p2) && visitor(p2)) return true;
        }
        return false;
    }

public:
    void Reserve(size_t maxElements) {
        elements.reserve(maxElements);
        const size_t pointHint = 2 * maxElements * maxElements + 16;
        points.reserve(gridMode ? min<size_t>(pointHint, 1'000'000) : pointHint);
        pointBirth.reserve(points.capacity());
        bounds.reserve(maxElements);
    }

    bool HasPoint(const Point& p) const {
        if (gridMode && gridFast && gridLinesReady && EPS < 0.5 && PointAllowed(p) &&
            p.x == static_cast<int>(p.x) && p.y == static_cast<int>(p.y))
            return true; // every integer grid vertex is already initially known
        for (const Point& old : points) {
            if (SamePoint(old, p)) return true;
        }
        return false;
    }

    bool SameStoredElement(const Element& e, const Element& f) const {
        if (!SameElement(e, f)) return false;
        if (e.type != Type::Ray && e.type != Type::Segment) return true;

        if (e.bound == NO_BOUND || f.bound == NO_BOUND ||
            e.bound >= bounds.size() || f.bound >= bounds.size()) {
            return false;
        }
        const Bound& be = bounds[e.bound];
        const Bound& bf = bounds[f.bound];

        if (e.type == Type::Segment) {
            // Segment orientation is irrelevant: [A,B] == [B,A].
            return (SamePoint(be.p1, bf.p1) && SamePoint(be.p2, bf.p2)) ||
                   (SamePoint(be.p1, bf.p2) && SamePoint(be.p2, bf.p1));
        }

        // A ray is determined by its start point and direction.  The second
        // defining point may be anywhere farther along the same ray, so do not
        // require it to be identical. SameElement above has already checked
        // that both rays lie on the same supporting line.
        if (!SamePoint(be.p1, bf.p1)) return false;
        const double ex = be.p2.x - be.p1.x;
        const double ey = be.p2.y - be.p1.y;
        const double fx = bf.p2.x - bf.p1.x;
        const double fy = bf.p2.y - bf.p1.y;
        return ex * fx + ey * fy > EPS;
    }

    bool HasElement(const Element& e) const {
        if (gridFast && IsAutomaticGridLine(e)) return true;
        for (const Element& old : elements) {
            if (SameStoredElement(old, e)) return true;
        }
        return false;
    }

    bool AddPoint(const Point& p, uint16_t birth) {
        if (!PointAllowed(p)) return false;
        if (gridMode && gridFast && gridLinesReady && EPS < 0.5 &&
            p.x == static_cast<int>(p.x) && p.y == static_cast<int>(p.y))
            return false; // exact existing grid vertex: no linear point scan
        for (const Point& old : points) {
            if (SamePoint(old, p)) return false;
        }
        points.push_back(p);
        pointBirth.push_back(birth);
        TogglePointHash(p);
        return true;
    }

    bool PointOnElement(const Point& p, const Element& e) const {
        if (e.type == Type::Circle) {
            return IsZero(Sq(p.x - e.a) + Sq(p.y - e.b) - e.c);
        }
        if (!IsZero(e.a * p.x + e.b * p.y - e.c)) return false;
        return IsInRange(e, p);
    }

    // Same per-point arithmetic and comparison as PointOnElement; visit
    // matches in their original index order.
    template <class Visitor>
    void VisitPointIncidences(const Element& e, Visitor&& visit) const {
        uint32_t i = 0;
#if BS_HAS_SSE2
        if (e.type == Type::Line || e.type == Type::Circle) {
            const __m128d aa = _mm_set1_pd(e.a), bb = _mm_set1_pd(e.b);
            const __m128d cc = _mm_set1_pd(e.c), eps = _mm_set1_pd(EPS);
            const __m128d sign = _mm_set1_pd(-0.0);
            for (; i + 1 < points.size(); i += 2) {
                const __m128d p0 = _mm_loadu_pd(&points[i].x);
                const __m128d p1 = _mm_loadu_pd(&points[i+1].x);
                const __m128d x = _mm_unpacklo_pd(p0, p1);
                const __m128d y = _mm_unpackhi_pd(p0, p1);
                __m128d residual;
                if (e.type == Type::Circle) {
                    const __m128d dx = _mm_sub_pd(x, aa);
                    const __m128d dy = _mm_sub_pd(y, bb);
                    residual = _mm_sub_pd(_mm_add_pd(_mm_mul_pd(dx, dx),
                                                    _mm_mul_pd(dy, dy)), cc);
                } else {
                    residual = _mm_sub_pd(_mm_add_pd(_mm_mul_pd(aa, x),
                                                    _mm_mul_pd(bb, y)), cc);
                }
                const int hits = _mm_movemask_pd(_mm_cmplt_pd(
                    _mm_andnot_pd(sign, residual), eps));
                if (hits & 1) visit(i);
                if (hits & 2) visit(i + 1);
            }
        }
#endif
        for (; i < points.size(); ++i)
            if (PointOnElement(points[i], e)) visit(i);
    }

    template <class Visitor>
    bool VisitUnclippedIntersections(const Element& e1, const Element& e2,
                            Visitor&& visitor) const {
        if (e1.type == Type::Circle) {
            if (e2.type == Type::Circle) {
                double a = 2.0 * (e1.a - e2.a);
                double b = 2.0 * (e1.b - e2.b);
                if (IsZero(a) && IsZero(b)) return false;
                double c = Sq(e1.a) - Sq(e2.a) + Sq(e1.b) - Sq(e2.b) - e1.c + e2.c;
                Element radical = Element::FromCoefficients(a, b, c, Type::Line);
                return VisitLineCircle(radical, e1, visitor);
            }
            return VisitLineCircle(e2, e1, visitor);
        }
        if (e2.type == Type::Circle)
            return VisitLineCircle(e1, e2, visitor);
        const double det = e1.a * e2.b - e1.b * e2.a;
        if (IsZero(det)) return false;
        Point p{(e1.c * e2.b - e1.b * e2.c) / det,
                (e1.a * e2.c - e1.c * e2.a) / det};
        return IsInRange(e1, p) && IsInRange(e2, p) && visitor(p);
    }

    template <class Visitor>
    bool VisitIntersections(const Element& e1, const Element& e2,
                            Visitor&& visitor) const {
        // Central filter also covers circle-circle radical-axis intersections,
        // prerequisite previews, and direct Graph users, not just AddPoint.
        if (!gridMode) return VisitUnclippedIntersections(e1, e2, visitor);
        auto inside = [&](const Point& p) {
            return PointAllowed(p) && visitor(p);
        };
        return VisitUnclippedIntersections(e1, e2, inside);
    }

    void Intersect(const Element& e1, const Element& e2, uint16_t birth) {
        VisitIntersections(e1, e2, [&](const Point& p) {
            AddPoint(p, birth);
            return false;
        });
    }

    Mark GetMark() const { return {points.size(), elements.size()}; }

    void ApplyKnownNew(const Element& e, uint16_t birth) {
        for (const Element& old : elements) Intersect(e, old, birth);
        elements.push_back(e);
        ToggleElementHash(e);
    }

    bool Apply(const Element& e, uint16_t birth) {
        if (HasElement(e)) return false;
        ApplyKnownNew(e, birth);
        return true;
    }

    void SetStateHashingEnabled(bool enabled) {
        stateHashEnabled_ = enabled;
        stateHash1_ = 0x243f6a8885a308d3ULL;
        stateHash2_ = 0x13198a2e03707344ULL;
        if (!enabled) return;
        for (const Point& p : points) TogglePointHash(p);
        for (const Element& e : elements) ToggleElementHash(e);
    }

    uint64_t StateHash1() const { return stateHash1_; }
    uint64_t StateHash2() const { return stateHash2_; }

    void Rollback(const Mark& mark) {
        while (elements.size() > mark.elementCount) {
            const uint32_t id = static_cast<uint32_t>(elements.size() - 1);
            ToggleElementHash(elements[id]);
            elements.pop_back();
        }
        while (points.size() > mark.pointCount) {
            const uint32_t id = static_cast<uint32_t>(points.size() - 1);
            TogglePointHash(points[id]);
            points.pop_back();
            pointBirth.pop_back();
        }
    }

    bool AddInitialBounded(const Point& p1, const Point& p2, Type t) {
        const uint32_t bi = static_cast<uint32_t>(bounds.size());
        bounds.push_back({p1, p2});
        Element e = Element::FromPoints(p1, p2, t, bi);
        if (!Apply(e, 0)) {
            bounds.pop_back();
            return false;
        }
        return true;
    }

    bool AddInitial(const Element& e) { return Apply(e, 0); }

    void AddAutomaticGridLines() {
        if (!gridMode || gridLinesReady) return;
        // Before user elements are added, all auto-grid intersections are known
        // exactly: one lattice point for each (integer x, integer y). Generate
        // them in the SAME row-major order as sequential horizontal/vertical
        // intersections, avoiding a quadratic scan of earlier lattice points.
        if (gridFast && EPS < 0.5 && elements.empty()) {
            for (int x = 0; x <= gridM; ++x) {
                const Element e = Element::FromCoefficients(1, 0, x, Type::Line);
                elements.push_back(e); ToggleElementHash(e);
            }
            for (int y = 0; y <= gridN; ++y) {
                const Element e = Element::FromCoefficients(0, 1, y, Type::Line);
                elements.push_back(e); ToggleElementHash(e);
            }
            const size_t explicitCount = points.size();
            for (int y = 0; y <= gridN; ++y) {
                for (int x = 0; x <= gridM; ++x) {
                    const Point p{static_cast<double>(x), static_cast<double>(y)};
                    bool duplicate = false;
                    for (size_t i = 0; i < explicitCount; ++i) {
                        if (SamePoint(points[i], p)) { duplicate = true; break; }
                    }
                    if (!duplicate) {
                        points.push_back(p); pointBirth.push_back(0); TogglePointHash(p);
                    }
                }
            }
        } else {
            for (int x = 0; x <= gridM; ++x)
                AddInitial(Element::FromCoefficients(1, 0, x, Type::Line));
            for (int y = 0; y <= gridN; ++y)
                AddInitial(Element::FromCoefficients(0, 1, y, Type::Line));
        }
        gridLinesReady = true;
    }


    bool GoalsMet() const {
        for (const Point& p : goalPoints) if (!HasPoint(p)) return false;
        for (const Element& e : goalElements) if (!HasElement(e)) return false;
        return true;
    }

    // Candidate priority used by goal-first search:
    //   2 = the candidate itself is a still-missing target line/circle;
    //   1 = it is not a target element, but it passes through a missing target point;
    //   0 = ordinary auxiliary construction.
    // This does not prune any candidate; it only changes search order.
    int GoalPriority(const Element& candidate) const {
        // Test the cheap geometric predicate first.  Almost every candidate is
        // unrelated to a particular goal, so avoid the linear HasElement /
        // HasPoint scan unless the candidate can actually satisfy that goal.
        for (const Element& e : goalElements) {
            if (SameElement(e, candidate) && !HasElement(e)) return 2;
        }
        for (const Point& p : goalPoints) {
            if (PointOnElement(p, candidate) && !HasPoint(p)) return 1;
        }
        return 0;
    }

    bool IsGoalDirected(const Element& candidate) const {
        return GoalPriority(candidate) > 0;
    }

    bool HasMissingGoalPoints() const {
        for (const Point& p : goalPoints) {
            if (!HasPoint(p)) return true;
        }
        return false;
    }

    int MissingPointLowerBound() const {
        int lower = 0;
        for (const Point& p : goalPoints) {
            if (HasPoint(p)) continue;
            int incidence = 0;
            for (const Element& e : elements) {
                if (PointOnElement(p, e)) {
                    ++incidence;
                    if (incidence >= 1) break;
                }
            }
            lower = max(lower, incidence ? 1 : 2);
        }
        return lower;
    }

    // Each construction adds at most one new element. Therefore the number
    // of distinct missing line/circle goals is a strict lower bound on the
    // number of remaining constructions.
    int MissingDistinctGoalElementCount() const {
        int missing = 0;
        for (size_t i = 0; i < goalElements.size(); ++i) {
            const Element& goal = goalElements[i];
            if (HasElement(goal)) continue;

            bool duplicate = false;
            for (size_t j = 0; j < i; ++j) {
                if (SameElement(goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) ++missing;
        }
        return missing;
    }

    // Count how many currently existing elements pass through a still-missing
    // target point.  In a consistent state a missing target point normally has
    // at most one such support, because the intersection of two existing
    // elements would already have been inserted as a known point. Still
    // count up to two to keep the test conservative under floating-point EPS.
    int ExistingSupportCount(const Point& p) const {
        int count = 0;
        for (const Element& e : elements) {
            if (PointOnElement(p, e) && ++count >= 2) break;
        }
        return count;
    }

    // Number of distinct still-missing target line/circle elements incident on
    // p.  These elements must be constructed anyway, so they can optimistically
    // supply incidence needed to create p without consuming an auxiliary E.
    int MissingDistinctGoalElementsThroughPoint(const Point& p) const {
        int count = 0;
        for (size_t i = 0; i < goalElements.size(); ++i) {
            const Element& goal = goalElements[i];
            if (HasElement(goal) || !PointOnElement(p, goal)) continue;

            bool duplicate = false;
            for (size_t j = 0; j < i; ++j) {
                if (SameElement(goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) ++count;
        }
        return count;
    }

    // Joint strict lower bound for target points after reserving one step for
    // every distinct missing target line/circle.  Let k be that number and
    // s=remaining-k the number of steps that could possibly be auxiliary.
    // A missing target point needs two incident elements in total. Existing
    // supports and still-missing target elements through the point can cover
    // part of that requirement; anything left must come from auxiliary steps.
    int RequiredAuxiliaryStepsForGoalPoints() const {
        int lower = 0;
        auto includeRequiredPoint = [&](const Point& p) {
            if (HasPoint(p)) return;
            const int existing = min(2, ExistingSupportCount(p));
            const int forcedTargets = MissingDistinctGoalElementsThroughPoint(p);
            const int need = max(0, 2 - existing - forcedTargets);
            lower = max(lower, need);
        };

        for (const Point& p : goalPoints) includeRequiredPoint(p);

        // A missing target circle has an implicit prerequisite: its center must
        // already be a known point before the circle can be constructed by this
        // program's center->circumference compass operation.  Treat that center
        // as an additional required point for the joint lower bound.  The goal
        // circle itself does not pass through its center (for nonzero radius),
        // so reserving the circle step cannot falsely help create the center.
        for (size_t i = 0; i < goalElements.size(); ++i) {
            const Element& goal = goalElements[i];
            if (goal.type != Type::Circle || HasElement(goal)) continue;
            bool duplicate = false;
            for (size_t j = 0; j < i; ++j) {
                if (SameElement(goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) includeRequiredPoint({goal.a, goal.b});
        }
        return lower;
    }

    // Collect missing target points for which every remaining construction is
    // forced to pass through the point.  Computing this once per DFS node avoids
    // rescanning all existing elements for every candidate in the last two layers.
    void CollectForcedTailPointIndices(int remaining, vector<uint32_t>& out) const {
        out.clear();
        if (remaining <= 0 || remaining > 2) return;
        for (uint32_t i = 0; i < goalPoints.size(); ++i) {
            const Point& p = goalPoints[i];
            if (HasPoint(p)) continue;
            const int existing = min(2, ExistingSupportCount(p));
            const int need = max(0, 2 - existing);
            // One step left: the sole new element must pass every still-missing
            // target point.  Duplicate coincident supports of the same carrier
            // must not drop the point from the forced set.
            if (remaining == 1 || need == remaining) out.push_back(i);
        }
    }

    bool CandidatePassesForcedTailPoints(const Element& candidate,
                                         const vector<uint32_t>& forced) const {
        for (uint32_t gi : forced) {
            if (!PointOnElement(goalPoints[gi], candidate)) return false;
        }
        return true;
    }

    // True if at least one distinct missing target line/circle can be drawn now
    // from the currently known points using the selected tool.  This is used
    // when remaining == number of missing target elements: the next step must
    // be one of those targets, so if none is directly drawable the node is dead.
    bool AnyMissingGoalElementDrawableNow(int toolType) const {
        for (size_t gi = 0; gi < goalElements.size(); ++gi) {
            const Element& goal = goalElements[gi];
            if (HasElement(goal)) continue;
            bool duplicate = false;
            for (size_t j = 0; j < gi; ++j) {
                if (SameElement(goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;

            if (goal.type == Type::Line) {
                if (toolType == 0) continue;
                int hits = 0;
                for (const Point& p : points) {
                    if (PointOnElement(p, goal) && ++hits >= 2) return true;
                }
            } else if (goal.type == Type::Circle) {
                if (toolType == 1 || IsZero(goal.c)) continue;
                int center = -1;
                for (size_t i = 0; i < points.size(); ++i) {
                    if (SamePoint(points[i], {goal.a, goal.b})) {
                        center = static_cast<int>(i);
                        break;
                    }
                }
                if (center < 0) continue;
                for (size_t i = 0; i < points.size(); ++i) {
                    if (static_cast<int>(i) != center && PointOnElement(points[i], goal))
                        return true;
                }
            }
        }
        return false;
    }

    // Strict necessary conditions for completing all still-missing element
    // goals in one construction.  If one distinct goal element is missing,
    // that element must already be directly constructible from known points:
    //   line   -> two known points on the line;
    //   circle -> known center plus a known point on the circumference.
    // More than one distinct missing element can never be completed by one E.
    bool MissingGoalElementsDrawableInOneStep(int toolType) const {
        const Element* missingGoal = nullptr;
        for (size_t i = 0; i < goalElements.size(); ++i) {
            const Element& goal = goalElements[i];
            if (HasElement(goal)) continue;

            bool duplicate = false;
            for (size_t j = 0; j < i; ++j) {
                if (SameElement(goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;
            if (missingGoal != nullptr) return false;
            missingGoal = &goal;
        }

        if (missingGoal == nullptr) return true;
        const Element& goal = *missingGoal;

        if (goal.type == Type::Line) {
            if (toolType == 0) return false; // compass only
            int onLine = 0;
            for (const Point& p : points) {
                if (PointOnElement(p, goal) && ++onLine >= 2) return true;
            }
            return false;
        }

        if (goal.type == Type::Circle) {
            if (toolType == 1 || IsZero(goal.c)) return false; // straightedge only / zero-radius impossible
            const Point center{goal.a, goal.b};
            int centerId = -1;
            for (size_t i = 0; i < points.size(); ++i) {
                if (SamePoint(points[i], center)) {
                    centerId = static_cast<int>(i);
                    break;
                }
            }
            if (centerId < 0) return false;
            for (size_t i = 0; i < points.size(); ++i) {
                if (static_cast<int>(i) != centerId && PointOnElement(points[i], goal)) return true;
            }
            return false;
        }

        return false;
    }

    Element MakeCandidate(const Candidate& c) const {
        const Point& pi = points[c.i];
        const Point& pj = points[c.j];
        if (c.tool == 0) return Element::FromPoints(pi, pj, Type::Circle);
        if (c.tool == 1) return Element::FromPoints(pj, pi, Type::Circle);
        return Element::FromPoints(pi, pj, Type::Line);
    }

    void PrintSolution() const {
        cout << "Solution found!\n";
        cout << "Points (" << points.size() << "):\n";
        cout << setprecision(12);
        for (const Point& p : points) cout << '(' << CleanZero(p.x) << ',' << CleanZero(p.y) << ")\n";
        cout << "Elements (" << elements.size() << "):\n";
        for (const Element& e : elements) {
            if (e.type == Type::Circle) {
                cout << "(x-" << CleanZero(e.a) << ")^2+(y-" << CleanZero(e.b)
                     << ")^2=" << sqrt(max(0.0, e.c)) << "^2\n";
            } else {
                cout << CleanZero(e.a) << "x+" << CleanZero(e.b) << "y="
                     << CleanZero(e.c) << " [" << static_cast<int>(e.type) << "]\n";
            }
        }
    }
};

// Exact, bounded per-depth candidate deduplication. If the table reaches its
// load limit, new candidates are left untracked rather than discarded, so the
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
        // Same-bucket deduplication is intentionally used in the streaming hot
        // path.  EPS-equal elements straddling a quantization boundary may be
        // searched more than once, but they are never incorrectly discarded.
        if (FindBucket(b, e)) return InsertResult::Duplicate;
        if (used_ >= maxEntries_) return InsertResult::NewUntracked;

        size_t pos = ElementBucketHash{}(b) & mask_;
        for (size_t probe = 0; probe < slots_.size(); ++probe) {
            Slot& slot = slots_[pos];
            if (slot.generation != generation_) {
                slot.value = e;
                slot.generation = generation_;
                ++used_;
                return InsertResult::NewTracked;
            }
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
    bool Submit(const Graph& g, ParallelControl* control) {
        lock_guard<mutex> guard(mutex_);
        if (entries_.size() >= requested_) return true;
        ++successfulVisits_;
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
        // Only used when one missing target element has two operations left.
        int previewGoal = -1;
        int knownLineHits = 0;
        Point knownLinePoint{};
        bool knownCenter = false;
        bool knownCircumference = false;
    };

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
    SolutionCollector* solutions_ = nullptr;
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
        if (parallelControl_ && parallelControl_->stop.load(memory_order_relaxed)) {
            timedOut_ = parallelControl_->timedOut.load(memory_order_relaxed);
            return true;
        }
        if ((poll & 1023u) != 0) return false;
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

    vector<Candidate> GenerateUniqueCandidates(const Graph& g, SearchStats& stats) {
        vector<Candidate> result;
        vector<Element> representatives;
        unordered_map<ElementBucket, uint32_t, ElementBucketHash> seen;

        const size_t n = g.points.size();
        const size_t rough = n > 1 ? (n * (n - 1) / 2) * (toolType_ == 2 ? 3 : (toolType_ == 0 ? 2 : 1)) : 0;
        result.reserve(min<size_t>(rough, 1'000'000));
        representatives.reserve(result.capacity());
        seen.reserve(result.capacity() * 2 + 1);

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
                        if (it != seen.end() && SameElement(representatives[it->second], e)) {
                            ++stats.duplicateCandidates;
                            return;
                        }
                    }
                }
            }
            const uint32_t id = static_cast<uint32_t>(result.size());
            result.push_back(cand);
            representatives.push_back(e);
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
                if (g.GoalPriority(g.MakeCandidate(result[read])) == 2) {
                    if (exactEnd != read) swap(result[exactEnd], result[read]);
                    ++exactEnd;
                }
            }

            if (g.HasMissingGoalPoints()) {
                size_t directedEnd = exactEnd;
                for (size_t read = exactEnd; read < result.size(); ++read) {
                    if (CheckTimeout()) return result;
                    if (g.GoalPriority(g.MakeCandidate(result[read])) == 1) {
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

    PreApplyContext BuildPreApplyContext(const Graph& g, int remaining) const {
        PreApplyContext ctx;
        ctx.missingDistinctGoal.assign(g.goalElements.size(), 0);
        for (size_t i = 0; i < g.goalElements.size(); ++i) {
            const Element& goal = g.goalElements[i];
            if (g.HasElement(goal)) continue;
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
        const vector<uint32_t>& forcedTailPoints, SearchStats& stats) const {
        const uint32_t n = static_cast<uint32_t>(g.points.size());

        if (goal.type == Type::Line) {
            if (toolType_ == 0) return nullopt;
            // Enumerate every incidence-point pair.  A single EPS-near first
            // pair may describe a different line, so each pair must prove the
            // actual goal line before being accepted.
            vector<uint32_t> hits;
            for (uint32_t i = 0; i < n; ++i) {
                if (g.PointOnElement(g.points[i], goal)) hits.push_back(i);
            }
            for (size_t a = 0; a < hits.size(); ++a) {
                for (size_t b = a + 1; b < hits.size(); ++b) {
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
                if (SamePoint(g.points[i], center)) {
                    centerId = static_cast<int>(i);
                    break;
                }
            }
            if (centerId < 0) return nullopt;

            for (uint32_t p = 0; p < n; ++p) {
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

        vector<size_t> missing;
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
        const optional<OperationKey>& previous, SearchStats& stats) const {
        if (toolType_ == 0) return nullopt;

        uint32_t first = NO_BOUND, second = NO_BOUND, newborn = NO_BOUND;
        g.VisitPointIncidences(line, [&](uint32_t i) {
            if (first == NO_BOUND) first = i;
            else if (second == NO_BOUND) second = i;
            if (depth > 0 && g.pointBirth[i] == depth) newborn = i;
        });
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
        return nullopt;
    }

    // Same as above for a specified circle. The center is fixed and must be a
    // known point; we only search for one known circumference point that gives a
    // symmetry-compatible center->circumference compass operation.
    optional<Candidate> FindReverseCircleRepresentation(
        const Graph& g, uint32_t centerId, const Element& circle, uint16_t depth,
        const optional<OperationKey>& previous, SearchStats& stats) const {
        if (toolType_ == 1 || IsZero(circle.c) || centerId >= g.points.size())
            return nullopt;

        uint32_t first = NO_BOUND, newborn = NO_BOUND;
        g.VisitPointIncidences(circle, [&](uint32_t q) {
            if (q == centerId) return;
            if (first == NO_BOUND) first = q;
            if (depth > 0 && g.pointBirth[q] == depth) newborn = q;
        });
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
        vector<Point> required;
        CollectUniqueForcedTailPoints(g, forcedTailPoints, required);
        if (required.empty()) return false;

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
                vector<Element> triedLines;
                triedLines.reserve(g.points.size());
                for (uint32_t q = 0; q < g.points.size(); ++q) {
                    if (CheckTimeout()) return false;
                    if (SamePoint(p, g.points[q])) continue;
                    Element line = Element::FromPoints(p, g.points[q], Type::Line);

                    bool duplicate = false;
                    for (const Element& old : triedLines) {
                        if (SameElement(old, line)) {
                            duplicate = true;
                            break;
                        }
                    }
                    if (duplicate) continue;
                    triedLines.push_back(line);

                    if (auto cand = FindReverseLineRepresentation(
                            g, line, depth, previous, stats)) {
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
                const double r2 = Sq(center.x - p0.x) + Sq(center.y - p0.y);
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
                        g, centerId, circle, depth, previous, stats)) {
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
            if (g.GoalsMet() && (!solutions_ || solutions_->Submit(g, parallelControl_)))
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
        optional<PreApplyContext> preCtx;
        auto accept = [&](const Candidate&, const Element& e) -> bool {
            if (!preCtx) preCtx.emplace(BuildPreApplyContext(g, remaining));
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
        BoundedElementSet& seen = streamSeen_[depth];
        seen.BeginNode();
        ExactGridLineCache* exactSeen = nullptr;
        if (g.gridMode && g.gridFast) {
            exactSeen = &gridSeen_[depth];
            exactSeen->BeginNode();
        }
        const PreApplyContext preCtx = BuildPreApplyContext(g, remaining);

        // With goal-first enabled, try directly drawable missing target
        // line/circle elements without enumerating all point pairs.  The generic
        // stream then needs only the target-point-directed and auxiliary bands.
        const bool useGoalBands = goalFirst_ && remaining > 1;
        if (useGoalBands) {
            if (TryDirectMissingGoalElements(g, remaining, depth, previous, stats, preCtx)) return true;
            if (timedOut_) return false;
        }
        const bool missingGoalPoints = useGoalBands && g.HasMissingGoalPoints();
        const int passes = useGoalBands ? (missingGoalPoints ? 2 : 1) : 1;
        for (int pass = 0; pass < passes; ++pass) {
            if (CheckTimeout()) return false;
            for (uint32_t i = 0; i < n; ++i) {
                if (CheckTimeout()) return false;
                for (uint32_t j = i + 1; j < n; ++j) {
                    if (CheckTimeout()) return false;
                    auto consider = [&](uint8_t tool) -> bool {
                        if (CheckTimeout()) return false;
                        ++stats.rawCandidates;
                        Candidate cand{i, j, tool};
                        Element e = g.MakeCandidate(cand);

                        // Do the zero-memory partial-order check before the more
                        // expensive goal and element-index tests.
                        if (SymmetryPruned(g, cand, e, depth, previous, stats)) return false;
                        const int goalPriority = g.GoalPriority(e);
                        // With one operation left, a non-goal-directed element
                        // cannot possibly complete any still-missing goal.
                        if (remaining == 1 && goalPriority == 0) {
                            ++stats.finalStepPruned;
                            return false;
                        }
                        if (useGoalBands) {
                            // Priority-2 exact target elements were already
                            // handled directly above.  Do not rediscover them by
                            // a full point-pair scan.
                            const int wantedPriority = missingGoalPoints
                                ? (pass == 0 ? 1 : 0)
                                : 0;
                            if (goalPriority != wantedPriority) return false;
                        }
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

                        // EPS equality is not transitive. Moving the preview
                        // before dedup must not install a different surviving
                        // representative that could hide another tail candidate.
                        // Therefore every preview-passing representation is
                        // tested in this final auxiliary layer. Earlier layers
                        // retain the original bounded deduplication unchanged.
                        if (preCtx.previewGoal < 0) {
                            const auto dedup = seen.Insert(e);
                            if (dedup == BoundedElementSet::InsertResult::Duplicate) {
                                ++stats.duplicateCandidates;
                                ++stats.streamDuplicates;
                                return false;
                            }
                            if (dedup == BoundedElementSet::InsertResult::NewUntracked) {
                                ++stats.streamDedupOverflow;
                            }
                        }

                        ++stats.uniqueCandidates;
                        return TryCandidate(g, e, remaining, depth, stats, preCtx, true);
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
                }
            }
        }
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
            if (solutions_->Submit(g, parallelControl_)) return true;
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
        vector<uint32_t> forcedTailPoints;
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
            vector<Candidate> candidates = GenerateUniqueCandidates(g, stats);
            if (timedOut_) return false;
            const PreApplyContext preCtx = BuildPreApplyContext(g, remaining);
            for (const Candidate& cand : candidates) {
                if (CheckTimeout()) return false;
                Element e = g.MakeCandidate(cand);
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

// Reporting only: invoked after the search clock has stopped and workers joined.
// No fields, provenance strings, or callbacks are added to the live search state.
namespace readable_report {
#if defined(__GNUC__) || defined(__clang__)
#define READABLE_ONLY __attribute__((noinline, cold))
#elif defined(_MSC_VER)
#define READABLE_ONLY __declspec(noinline)
#else
#define READABLE_ONLY
#endif
constexpr size_t NONE = numeric_limits<size_t>::max();

struct Origin {
    size_t element = NONE;
    size_t other = NONE;
};
struct Definition {
    size_t first = NONE;
    size_t second = NONE;
};
struct Trace {
    vector<Origin> origins;
    vector<Definition> definitions;
    vector<size_t> pointGoals;
    vector<size_t> elementGoals;
    vector<size_t> pointNumbers;
    vector<string> elementNames;
};

READABLE_ONLY bool ExactNumber(double a, double b) {
    return bit_cast<uint64_t>(a) == bit_cast<uint64_t>(b);
}
READABLE_ONLY bool ExactPoint(const Point& a, const Point& b) {
    return ExactNumber(a.x, b.x) && ExactNumber(a.y, b.y);
}
READABLE_ONLY bool ExactElement(const Element& a, const Element& b) {
    return a.type == b.type && a.bound == b.bound &&
           ExactNumber(a.a, b.a) && ExactNumber(a.b, b.b) && ExactNumber(a.c, b.c);
}

// Report-private numerical helpers. They deliberately do not call Graph's
// mutating or candidate routines, so adding a reporter does not add callers
// that change their interprocedural optimization or return-value specialization.
struct RPoint { double x, y; };
struct RElement { double a, b, c; Type type; };
READABLE_ONLY bool RZero(double v) { return std::abs(v) < EPS; }
READABLE_ONLY double RClean(double v) { return RZero(v) ? 0.0 : v; }
READABLE_ONLY bool RSameElement(const RElement& a, const Element& b) {
    return a.type == b.type && RZero(a.a - b.a) &&
        RZero(a.b - b.b) && RZero(a.c - b.c);
}
READABLE_ONLY RElement RNormalize(RElement e) {
    if (!RZero(e.b)) {
        e.a /= e.b; e.c /= e.b; e.b = 1.0;
    } else if (!RZero(e.a)) {
        e.c /= e.a; e.a = 1.0; e.b = 0.0;
    }
    e.a = RClean(e.a); e.b = RClean(e.b); e.c = RClean(e.c);
    return e;
}
READABLE_ONLY RElement RMake(const Point& p, const Point& q, Type type) {
    RElement e{0, 0, 0, type};
    if (type == Type::Circle) {
        e.a = p.x; e.b = p.y;
        const double dx = p.x - q.x, dy = p.y - q.y;
        e.c = dx * dx + dy * dy;
    } else {
        e.a = q.y - p.y; e.b = p.x - q.x;
        e.c = p.x * q.y - p.y * q.x;
        e = RNormalize(e);
    }
    e.a = RClean(e.a); e.b = RClean(e.b); e.c = RClean(e.c);
    return e;
}
READABLE_ONLY bool RExactElement(const RElement& a, const Element& b) {
    return a.type == b.type && ExactNumber(a.a, b.a) &&
        ExactNumber(a.b, b.b) && ExactNumber(a.c, b.c);
}
READABLE_ONLY bool RSamePoint(const Point& p, const Point& q) {
    return RZero(p.x - q.x) && RZero(p.y - q.y);
}
READABLE_ONLY bool RInRange(const Graph& g, size_t ei, const RPoint& p) {
    if (!g.PointAllowed({p.x, p.y})) return false;
    const Element& e = g.elements[ei];
    if (e.type == Type::Line) return true;
    if (e.type != Type::Ray && e.type != Type::Segment) return false;
    if (e.bound == NO_BOUND || e.bound >= g.bounds.size()) return false;
    const Bound& b = g.bounds[e.bound];
    if (e.type == Type::Ray) {
        const double dx = (b.p1.x - p.x) * (b.p1.x - b.p2.x);
        const double dy = (b.p1.y - p.y) * (b.p1.y - b.p2.y);
        return dx + dy > -EPS;
    }
    const double dx = (b.p1.x - p.x) * (b.p2.x - p.x);
    const double dy = (b.p1.y - p.y) * (b.p2.y - p.y);
    return dx + dy < EPS;
}
READABLE_ONLY bool RRootMatches(const RPoint& p, const Point& goal) {
    return RZero(p.x - goal.x) && RZero(p.y - goal.y);
}
READABLE_ONLY bool RLineCircleContains(const Graph& g, size_t lineIndex,
        const RElement& l, const RElement& c, const Point& goal) {
    const double denom = l.a * l.a + l.b * l.b;
    if (RZero(denom)) return false;
    const double dist = l.a * c.a + l.b * c.b - l.c;
    const double delta = denom * c.c - dist * dist;
    auto match = [&](const RPoint& p) {
        // NONE means the infinite radical axis of two circles.
        return g.PointAllowed({p.x, p.y}) &&
            (lineIndex == NONE || RInRange(g, lineIndex, p)) && RRootMatches(p, goal);
    };
    if (RZero(delta)) {
        return match({c.a - l.a * dist / denom, c.b - l.b * dist / denom});
    } else if (delta > EPS) {
        const double root = sqrt(delta);
        return match({c.a - (l.a * dist + l.b * root) / denom,
                      c.b - (l.b * dist - l.a * root) / denom}) ||
               match({c.a - (l.a * dist - l.b * root) / denom,
                      c.b - (l.b * dist + l.a * root) / denom});
    }
    return false;
}
READABLE_ONLY bool RIntersectionContains(const Graph& g, size_t first,
                                        size_t second, const Point& goal) {
    const Element& x = g.elements[first];
    const Element& y = g.elements[second];
    const RElement a{x.a, x.b, x.c, x.type}, b{y.a, y.b, y.c, y.type};
    if (a.type == Type::Circle) {
        if (b.type == Type::Circle) {
            const double aa = 2.0 * (a.a - b.a), bb = 2.0 * (a.b - b.b);
            if (RZero(aa) && RZero(bb)) return false;
            const double cc = a.a * a.a - b.a * b.a + a.b * a.b - b.b * b.b - a.c + b.c;
            return RLineCircleContains(g, NONE, RNormalize({aa, bb, cc, Type::Line}), a, goal);
        }
        return RLineCircleContains(g, second, b, a, goal);
    }
    if (b.type == Type::Circle) return RLineCircleContains(g, first, a, b, goal);
    const double det = a.a * b.b - a.b * b.a;
    if (RZero(det)) return false;
    const RPoint p{(a.c * b.b - a.b * b.c) / det,
                   (a.a * b.c - a.c * b.a) / det};
    return RInRange(g, first, p) && RInRange(g, second, p) && RRootMatches(p, goal);
}

READABLE_ONLY Definition FindDefinition(const Graph& g, size_t known, const Element& e) {
    if (e.type == Type::Circle) {
        for (size_t i = 0; i < known; ++i) {
            if (!RSamePoint(g.points[i], {e.a, e.b})) continue;
            for (size_t j = 0; j < known; ++j) {
                if (i != j && RSameElement(RMake(g.points[i], g.points[j], Type::Circle), e))
                    return {i, j};
            }
        }
    } else if (e.type == Type::Line) {
        for (size_t i = 0; i < known; ++i)
            for (size_t j = i + 1; j < known; ++j)
                if (RSameElement(RMake(g.points[i], g.points[j], Type::Line), e)) return {i, j};
    }
    return {NONE, NONE};
}

READABLE_ONLY Trace Reconstruct(const Graph& solved, size_t givenPointCount) {
    if (givenPointCount > solved.points.size() ||
        solved.initialElementCount > solved.elements.size() ||
        solved.pointBirth.size() != solved.points.size())
        throw runtime_error("invalid initial counts");
    Trace t;
    t.origins.resize(solved.points.size());
    t.definitions.resize(solved.elements.size());
    t.elementNames.reserve(solved.elements.size());
    array<size_t, 4> counts{};
    const array<char, 4> prefixes{'C', 'L', 'R', 'S'};
    for (const Element& e : solved.elements) {
        const size_t k = static_cast<size_t>(e.type);
        t.elementNames.push_back(string(1, prefixes.at(k)) + to_string(++counts.at(k)));
    }
    // pointBirth is already stored by the solver. Recover each point's exact
    // parent intersection and each element's already-available defining pair.
    for (size_t pi = 0; pi < solved.points.size(); ++pi) {
        if (pi && solved.pointBirth[pi] < solved.pointBirth[pi - 1])
            throw runtime_error("point births are out of order");
        if (pi < givenPointCount) {
            if (solved.pointBirth[pi] != 0) throw runtime_error("invalid given point birth");
            continue;
        }
        const size_t birth = solved.pointBirth[pi];
        const size_t begin = birth ? solved.initialElementCount + birth - 1 : 0;
        const size_t end = birth ? begin + 1 : solved.initialElementCount;
        if (end > solved.elements.size()) throw runtime_error("invalid point birth");
        bool found = false;
        for (size_t ei = begin; ei < end && !found; ++ei)
            for (size_t old = 0; old < ei; ++old)
                if (RIntersectionContains(solved, ei, old, solved.points[pi])) {
                    t.origins[pi] = {ei, old}; found = true; break;
                }
        if (!found) {
            const size_t available = birth
                ? min(solved.elements.size(), solved.initialElementCount + birth)
                : solved.initialElementCount;
            for (size_t a = 0; a < available && !found; ++a)
                for (size_t b = 0; b < a; ++b)
                    if (RIntersectionContains(solved, a, b, solved.points[pi])) {
                        t.origins[pi] = {a, b}; found = true; break;
                    }
        }
    }
    size_t known = 0;
    for (size_t ei = solved.initialElementCount; ei < solved.elements.size(); ++ei) {
        const size_t step = ei - solved.initialElementCount + 1;
        while (known < solved.points.size() && solved.pointBirth[known] < step) ++known;
        t.definitions[ei] = FindDefinition(solved, known, solved.elements[ei]);
    }

    vector<bool> needed(solved.points.size(), false);
    for (size_t i = 0; i < givenPointCount; ++i) needed[i] = true;
    for (size_t ei = solved.initialElementCount; ei < t.definitions.size(); ++ei) {
        if (t.definitions[ei].first != NONE) needed.at(t.definitions[ei].first) = true;
        if (t.definitions[ei].second != NONE) needed.at(t.definitions[ei].second) = true;
    }
    for (const Point& goal : solved.goalPoints) {
        size_t pi = 0;
        while (pi < solved.points.size() && !RSamePoint(solved.points[pi], goal)) ++pi;
        if (pi == solved.points.size()) throw runtime_error("missing point goal");
        needed[pi] = true;
        t.pointGoals.push_back(pi);
    }
    for (const Element& goal : solved.goalElements) {
        size_t ei = 0;
        while (ei < solved.elements.size() && !(solved.elements[ei].type == goal.type &&
                RZero(solved.elements[ei].a - goal.a) && RZero(solved.elements[ei].b - goal.b) &&
                RZero(solved.elements[ei].c - goal.c))) ++ei;
        if (ei == solved.elements.size()) throw runtime_error("missing element goal");
        t.elementGoals.push_back(ei);
    }
    t.pointNumbers.resize(solved.points.size());
    size_t number = 0;
    for (size_t pi = 0; pi < needed.size(); ++pi)
        if (needed[pi]) t.pointNumbers[pi] = ++number;
    return t;
}

READABLE_ONLY void WriteNumber(ostream& out, double value) {
    char buffer[64];
    const auto result = to_chars(buffer, buffer + sizeof(buffer), CleanZero(value));
    if (result.ec == errc{}) out.write(buffer, result.ptr - buffer);
    else out << setprecision(17) << CleanZero(value);
}
READABLE_ONLY void WritePoint(ostream& out, const Point& p) {
    out << '('; WriteNumber(out, p.x); out << ','; WriteNumber(out, p.y); out << ')';
}
READABLE_ONLY void WriteNamedPoint(ostream& out, const string& name, const Point& p) {
    out << name << '='; WritePoint(out, p);
}
READABLE_ONLY void WriteLineEquation(ostream& out, const Element& e) {
    out << '('; WriteNumber(out, e.a); out << ")x+(";
    WriteNumber(out, e.b); out << ")y="; WriteNumber(out, e.c);
}
READABLE_ONLY void WriteCircleEquation(ostream& out, const Element& e) {
    out << "(x-("; WriteNumber(out, e.a); out << "))^2+(y-(";
    WriteNumber(out, e.b); out << "))^2="; WriteNumber(out, e.c);
}

READABLE_ONLY string Format(const Graph& solved, size_t givenPointCount, const Trace& t) {
    ostringstream out;
    out << defaultfloat << setprecision(17);
    auto pointName = [&](size_t pi) { return "P" + to_string(t.pointNumbers.at(pi)); };
    auto writePointRef = [&](size_t pi) {
        WriteNamedPoint(out, pointName(pi), solved.points.at(pi));
    };
    auto writeEquation = [&](size_t ei) {
        const Element& e = solved.elements.at(ei);
        if (e.type == Type::Circle) WriteCircleEquation(out, e);
        else WriteLineEquation(out, e);
    };

    out << "Solution found!\n";
    out << "作图步骤（" << solved.elements.size() - solved.initialElementCount << "E）：\n";
    out << "已知对象：\n";
    for (size_t pi = 0; pi < givenPointCount; ++pi) {
        out << "  "; writePointRef(pi); out << "。\n";
    }
    for (size_t ei = 0; ei < solved.initialElementCount; ++ei) {
        const Element& e = solved.elements[ei];
        out << "  已知";
        if (e.type == Type::Circle) {
            out << "圆" << t.elementNames[ei] << "；方程：";
            WriteCircleEquation(out, e);
        } else if (e.type == Type::Line) {
            out << "直线" << t.elementNames[ei] << "；方程：";
            WriteLineEquation(out, e);
        } else {
            const Bound& bd = solved.bounds.at(e.bound);
            out << (e.type == Type::Ray ? "射线" : "线段") << t.elementNames[ei] << "：";
            if (e.type == Type::Ray) out << "起点";
            else out << "端点";
            WritePoint(out, bd.p1);
            out << (e.type == Type::Ray ? "，经过" : "和");
            WritePoint(out, bd.p2);
            out << "；所在直线方程：";
            WriteLineEquation(out, e);
        }
        out << "。\n";
    }

    auto emitIntersection = [&](size_t pi) {
        const Origin& o = t.origins[pi];
        if (o.element == NONE || o.other == NONE) {
            out << "  得到"; writePointRef(pi);
            out << "";
            return;
        }
        const size_t a = min(o.element, o.other), b = max(o.element, o.other);
        out << "  取" << t.elementNames.at(a) << ',' << t.elementNames.at(b)
            << "交点";
        writePointRef(pi);
        out << "。\n";
    };

    bool initialHeading = false;
    for (size_t pi = givenPointCount; pi < solved.points.size(); ++pi) {
        if (!t.pointNumbers[pi]) continue;
        if (t.origins[pi].element == NONE || t.origins[pi].element >= solved.initialElementCount) continue;
        if (!initialHeading) { out << "已知元素的交点：\n"; initialHeading = true; }
        emitIntersection(pi);
    }
    out << '\n';

    for (size_t ei = solved.initialElementCount; ei < solved.elements.size(); ++ei) {
        const Definition& d = t.definitions[ei];
        const Element& e = solved.elements[ei];
        out << "第" << ei - solved.initialElementCount + 1 << "步：";
        if (d.first != NONE && d.second != NONE) {
            if (e.type == Type::Circle) {
                out << "以"; writePointRef(d.first); out << "为圆心，";
                writePointRef(d.second); out << "为圆周点作圆" << t.elementNames[ei];
            } else {
                out << "过"; writePointRef(d.first); out << "，";
                writePointRef(d.second); out << "作直线" << t.elementNames[ei];
            }
        } else {
            out << (e.type == Type::Circle ? "作圆" : "作直线") << t.elementNames[ei]
                << "";
        }
        out << "；方程：";
        if (e.type == Type::Circle) WriteCircleEquation(out, e);
        else WriteLineEquation(out, e);
        out << "。\n";

        const size_t currentStep = ei - solved.initialElementCount + 1;
        for (size_t pi = givenPointCount; pi < solved.points.size(); ++pi) {
            if (!t.pointNumbers[pi]) continue;
            if (solved.pointBirth[pi] == currentStep) emitIntersection(pi);
        }
    }

    if (solved.gridMode) {
        out << "\n网格校验：全部 " << solved.points.size()
            << " 个已知点均限制在 [0," << solved.gridM << "]×[0," << solved.gridN
            << "] 内（含边界）\n";
    }
    out << "\n目标对应：\n";
    size_t lineGoal = 0, circleGoal = 0;
    for (size_t gi = 0; gi < solved.goalElements.size(); ++gi) {
        const bool circle = solved.goalElements[gi].type == Type::Circle;
        const size_t ei = t.elementGoals[gi];
        out << "  " << (circle ? "目标圆GC" : "目标直线GL")
            << (circle ? ++circleGoal : ++lineGoal) << '='
            << t.elementNames[ei] << "；方程：";
        if (circle) WriteCircleEquation(out, solved.elements[ei]);
        else WriteLineEquation(out, solved.elements[ei]);
        out << "。\n";
    }
    for (size_t gi = 0; gi < t.pointGoals.size(); ++gi) {
        const size_t pi = t.pointGoals[gi];
        out << "  目标点GP" << gi + 1 << '=';
        writePointRef(pi);
        out << "。\n";
    }
    if (solved.elements.size() == solved.initialElementCount)
        out << "目标已由给定对象满足，无须新增作图元素。\n";
    return out.str();
}

#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline, cold))
#endif
void Print(const Graph& solved, size_t givenPointCount) {
    // Re-evaluate numerical definitions/parent intersections only after success.
    // Build and validate the entire report before writing any steps. Failure is
    // a reporting failure only.
    try {
        const Trace trace = Reconstruct(solved, givenPointCount);
        cout << Format(solved, givenPointCount, trace);
    } catch (const exception& ex) {
        for (size_t pi = 0; pi < solved.points.size(); ++pi) {
            cout << "  P" << pi + 1 << '='; WritePoint(cout, solved.points[pi]); cout << "。\n";
        }
        for (size_t ei = 0; ei < solved.elements.size(); ++ei) {
            const Element& e = solved.elements[ei];
            cout << "  " << (e.type == Type::Circle ? "圆C" : e.type == Type::Line ? "直线L" : e.type == Type::Ray ? "射线R" : "线段S")
                 << ei + 1 << "；" << (e.type == Type::Circle ? "方程：" : "所在直线方程：");
            if (e.type == Type::Circle) WriteCircleEquation(cout, e);
            else WriteLineEquation(cout, e);
            cout << "。\n";
        }
    }
}
#undef READABLE_ONLY
} // namespace readable_report


} // namespace

static string DetectOperatingSystem() {
#ifdef _WIN32
    return "Windows";
#elif defined(__linux__)
    ifstream osRelease("/etc/os-release");
    string line;
    while (getline(osRelease, line)) {
        if (line == "ID=ubuntu" || line == "ID=\"ubuntu\"") return "Ubuntu";
    }
    return "Linux";
#else
    return "Unknown OS";
#endif
}

#ifdef __linux__
static uint32_t CountCpuList(const string& text) {
    uint32_t count = 0;
    size_t pos = 0;
    while (pos < text.size()) {
        while (pos < text.size() && (text[pos] == ' ' || text[pos] == '\n' || text[pos] == ',')) ++pos;
        if (pos >= text.size()) break;
        char* endp = nullptr;
        const unsigned long first = strtoul(text.c_str() + pos, &endp, 10);
        if (endp == text.c_str() + pos) break;
        pos = static_cast<size_t>(endp - text.c_str());
        unsigned long last = first;
        if (pos < text.size() && text[pos] == '-') {
            ++pos;
            last = strtoul(text.c_str() + pos, &endp, 10);
            if (endp == text.c_str() + pos) break;
            pos = static_cast<size_t>(endp - text.c_str());
        }
        if (last >= first) count += static_cast<uint32_t>(last - first + 1);
        while (pos < text.size() && text[pos] != ',') ++pos;
        if (pos < text.size()) ++pos;
    }
    return count;
}
#endif

static uint32_t DetectAvailableThreadLimit(uint32_t hardwareThreads) {
    uint32_t available = max<uint32_t>(1, hardwareThreads);
#ifdef __linux__
    {
        ifstream f("/sys/fs/cgroup/cpu.max");
        string quotaText;
        uint64_t period = 0;
        if (f >> quotaText >> period && quotaText != "max" && period > 0) {
            const uint64_t quota = strtoull(quotaText.c_str(), nullptr, 10);
            const uint32_t quotaThreads = static_cast<uint32_t>(max<uint64_t>(1, (quota + period - 1) / period));
            available = min(available, quotaThreads);
        }
    }
    {
        ifstream f("/sys/fs/cgroup/cpuset.cpus.effective");
        string cpus;
        if (getline(f, cpus)) {
            const uint32_t n = CountCpuList(cpus);
            if (n > 0) available = min(available, n);
        }
    }
#endif
    return max<uint32_t>(1, available);
}

static int ExitWithPause(int code, bool pauseOnExit) {
#ifdef _WIN32
    if (pauseOnExit) {
        cout << "\nPress Enter to close this window..." << flush;
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cin.get();
    }
#else
    (void)pauseOnExit;
#endif
    return code;
}

static bool ParseSolutionCount(const string& text, size_t& value) {
    if (text.empty()) return false;
    uint32_t count = 0;
    const auto result = from_chars(text.data(), text.data() + text.size(), count);
    if (result.ec != errc{} || result.ptr != text.data() + text.size() || count == 0)
        return false;
    value = count;
    return true;
}

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
    SetConsoleCP(CP_UTF8);
#endif
    ios::sync_with_stdio(false);
    cin.tie(nullptr);

#ifdef _WIN32
    bool pauseOnExit = true;
#else
    bool pauseOnExit = false;
#endif

    // Defaults: symmetry pruning ON, exact-target-first goal ordering ON, low-memory streaming ON, 30-second limit.
    bool symmetry = true;
    bool goalFirst = true;
    bool lowMemory = true;
    size_t ttMB = 8;
    size_t streamDedupEntries = 2048;
    double timeLimitSeconds = DEFAULT_TIME_LIMIT_SECONDS;
    uint16_t requestedTaskDepth = 0; // 0 = adaptive default from thread count
    uint32_t requestedThreads = 0; // 0 = ask interactively
    bool readableOutput = true;
    bool gridFast = true;
    size_t requestedSolutions = 0; // 0 = ask after all goal data

    for (int i = 1; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "--symmetry") symmetry = true;
        else if (arg == "--no-symmetry") symmetry = false;
        else if (arg == "--no-goal-first") goalFirst = false;
        else if (arg == "--low-memory") lowMemory = true;
        else if (arg == "--dedup-candidates") lowMemory = false;
        else if (arg == "--no-tt") ttMB = 0;
        else if (arg.rfind("--tt-mb=", 0) == 0) ttMB = stoull(arg.substr(8));
        else if (arg == "--no-stream-dedup") {
            streamDedupEntries = 0;
        }
        else if (arg.rfind("--stream-dedup=", 0) == 0) {
            streamDedupEntries = stoull(arg.substr(15));
        }
        else if (arg.rfind("--eps=", 0) == 0)
            EPS = stod(arg.substr(6));
        else if (arg.rfind("--time-limit=", 0) == 0)
            timeLimitSeconds = stod(arg.substr(13));
        else if (arg.rfind("--task-depth=", 0) == 0) {
            const string text = arg.substr(13);
            if (text == "auto") {
                requestedTaskDepth = 0;
            } else {
                uint32_t d = 0;
                const auto parsed = from_chars(text.data(), text.data() + text.size(), d);
                if (parsed.ec != errc{} || parsed.ptr != text.data() + text.size() ||
                    d < 1 || d > 65535) {
                    cerr << "Task depth must be 'auto' or an integer in 1..65535.\n";
                    return ExitWithPause(1, pauseOnExit);
                }
                requestedTaskDepth = static_cast<uint16_t>(d);
            }
        }
        else if (arg.rfind("--threads=", 0) == 0)
            requestedThreads = static_cast<uint32_t>(stoul(arg.substr(10)));
        else if (arg.rfind("--solutions=", 0) == 0) {
            if (!ParseSolutionCount(arg.substr(12), requestedSolutions)) {
                cerr << "Solution count must be an integer in 1..4294967295.\n";
                return ExitWithPause(1, pauseOnExit);
            }
        }
        else if (arg == "--no-grid-fast") gridFast = false;
        else if (arg == "--grid-fast") gridFast = true;
        else if (arg == "--raw-output") readableOutput = false;
        else if (arg == "--readable-output") readableOutput = true;
        else if (arg == "--pause") pauseOnExit = true;
        else if (arg == "--no-pause") pauseOnExit = false;
        else {
            cerr << "Unknown command-line option: " << arg << "\n";
            return ExitWithPause(1, pauseOnExit);
        }
    }

    if (!(EPS > 0.0) || !isfinite(EPS)) {
        cerr << "EPS must be a positive finite number.\n";
        return ExitWithPause(1, pauseOnExit);
    }
    if (!(timeLimitSeconds > 0.0) || !isfinite(timeLimitSeconds)) {
        cerr << "Time limit must be a positive finite number of seconds.\n";
        return ExitWithPause(1, pauseOnExit);
    }

    int limit = 0;
    int toolType = 2;
    int selectedMode = 2;
    array<int, 6> initial{};
    int goalLines = 0, goalCircles = 0, goalPoints = 0;
    Graph graph;
    const string operatingSystem = DetectOperatingSystem();
    uint32_t hardwareThreads = thread::hardware_concurrency();
    if (hardwareThreads == 0) hardwareThreads = 1;
    const uint32_t availableThreads = DetectAvailableThreadLimit(hardwareThreads);

    cout << "============================================================\n";
    cout << "brute_search v8.1\n";
    cout << "Geometry EPS: " << scientific << setprecision(3) << EPS << defaultfloat << "\n";
    cout << "Search time limit: " << timeLimitSeconds << " seconds\n";
    cout << "Detected system: " << operatingSystem << "\n";
    cout << "Hardware logical processors: " << hardwareThreads << "\n";
    cout << "Process-available / recommended maximum threads: "
         << availableThreads << "\n";
#ifdef _WIN32
    if (pauseOnExit)
        cout << "";
#endif
    cout << "============================================================\n\n";

    cout << "[Step 1/6] Maximum number of NEW construction elements (E).\n";
    cout << "  E counts only elements constructed by the solver; given lines/circles do NOT count.\n";
    cout << "  Example: enter 7 to search for a construction using at most 7 new lines/circles.\n";
    cout << "Enter E limit: " << flush;
    if (!(cin >> limit) || limit < 0) {
        cerr << "Invalid E limit. Please enter a non-negative integer.\n";
        return ExitWithPause(1, pauseOnExit);
    }

    uint32_t threads = requestedThreads;
    if (threads == 0) {
        cout << "\n[Step 2/6] Select parallel worker threads.\n";
        cout << "  Detected " << operatingSystem << ": " << hardwareThreads
             << " hardware logical processor(s), " << availableThreads
             << " available to this process.\n";
        cout << "  Recommended maximum: " << availableThreads
             << " thread(s).  Use 1 for deterministic single-thread mode.\n";
        cout << "Enter worker threads [1-" << availableThreads << "]: " << flush;
        if (!(cin >> threads)) {
            cerr << "Invalid thread count.\n";
            return ExitWithPause(1, pauseOnExit);
        }
    }
    if (threads < 1 || threads > availableThreads) {
        cerr << "Invalid thread count. Please enter 1.." << availableThreads
             << " for this " << operatingSystem << " system.\n";
        return ExitWithPause(1, pauseOnExit);
    }

    cout << "\n[Step 3/6] Select allowed construction tools.\n";
    cout << "  0 = compass only      (construct circles from two known points)\n";
    cout << "  1 = straightedge only (construct lines through two known points)\n";
    cout << "  2 = both compass and straightedge\n";
    cout << "  3 = grid straightedge only\n";
    cout << "Enter tool type [0/1/2/3]: " << flush;
    if (!(cin >> selectedMode) || selectedMode < 0 || selectedMode > 3) {
        cerr << "Invalid tool type. Please enter 0, 1, 2, or 3.\n";
        return ExitWithPause(1, pauseOnExit);
    }
    toolType = selectedMode == 3 ? 1 : selectedMode;
    if (selectedMode == 3) {
        cout << "网格左下角=(0,0)，右上角=(m,n)。请输入非负整数 m n：" << flush;
        long long m, n;
        if (!(cin >> m >> n) || m < 0 || n < 0 || m > 100000 || n > 100000 ||
            (m + 1) * (n + 1) > 1000000) {
            cerr << "Invalid grid: m,n must be nonnegative integers <=100000, "
                    "with at most 1000000 grid vertices.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.gridMode = true;
        graph.gridFast = gridFast;
        graph.gridM = static_cast<int>(m);
        graph.gridN = static_cast<int>(n);
        cout << "自动添加 x=0..m、y=0..n 的网格线及其网格内交点，不计入E。\n";
    }

    cout << "\n[Step 4/6] Enter counts of GIVEN objects in this exact order:\n";
    cout << "  P L R S C\n";
    cout << "  P = points, L = infinite lines, R = rays, S = segments, C = circles.\n";
    cout << "  Example: '2 1 0 0 0' means 2 points and 1 line are initially given.\n";
    cout << "Enter P L R S C: " << flush;
    if (!(cin >> initial[1] >> initial[2] >> initial[3] >> initial[4] >> initial[5])) {
        cerr << "Invalid initial-object counts.\n";
        return ExitWithPause(1, pauseOnExit);
    }
    for (int k = 1; k <= 5; ++k) {
        if (initial[k] < 0) {
            cerr << "Initial-object counts must be non-negative.\n";
            return ExitWithPause(1, pauseOnExit);
        }
    }

    const size_t totalElementCapacity = static_cast<size_t>(
        limit + initial[2] + initial[3] + initial[4] + initial[5] + 4) +
        (graph.gridMode ? static_cast<size_t>(graph.gridM) + graph.gridN + 2 : 0);
    graph.Reserve(totalElementCapacity);

    for (int i = 0; i < initial[1]; ++i) {
        double x, y;
        cout << "\nGiven point P" << i + 1 << ": enter x y.\n";
        cout << "  Example: '0 0' means P" << i + 1 << "=(0,0).\n";
        cout << "P" << i + 1 << " = " << flush;
        if (!(cin >> x >> y)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        if (!graph.PointAllowed({x, y})) {
            cerr << "Given point is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.AddPoint({x, y}, 0);
    }

    const size_t givenPointCount = graph.points.size();
    if (graph.gridMode) graph.AddAutomaticGridLines();

    for (int i = 0; i < initial[2]; ++i) {
        double a, b, c;
        cout << "\nGiven line L" << i + 1 << ": enter a b c for a*x + b*y = c.\n";
        cout << "  Examples: y=0 -> '0 1 0'; x=2 -> '1 0 2'.\n";
        cout << "L" << i + 1 << " (a b c) = " << flush;
        if (!(cin >> a >> b >> c)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        graph.AddInitial(Element::FromCoefficients(a, b, c, Type::Line));
    }

    for (int i = 0; i < initial[3]; ++i) {
        double x1, y1, x2, y2;
        cout << "\nGiven ray R" << i + 1 << ": enter x1 y1 x2 y2.\n";
        cout << "  The ray starts at (x1,y1) and passes through (x2,y2).\n";
        cout << "R" << i + 1 << " = " << flush;
        if (!(cin >> x1 >> y1 >> x2 >> y2)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        if (!graph.PointAllowed({x1, y1}) || !graph.PointAllowed({x2, y2})) {
            cerr << "Given ray/segment defining point is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.AddInitialBounded({x1, y1}, {x2, y2}, Type::Ray);
    }

    for (int i = 0; i < initial[4]; ++i) {
        double x1, y1, x2, y2;
        cout << "\nGiven segment S" << i + 1 << ": enter x1 y1 x2 y2 for its two endpoints.\n";
        cout << "S" << i + 1 << " = " << flush;
        if (!(cin >> x1 >> y1 >> x2 >> y2)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        if (!graph.PointAllowed({x1, y1}) || !graph.PointAllowed({x2, y2})) {
            cerr << "Given ray/segment defining point is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.AddInitialBounded({x1, y1}, {x2, y2}, Type::Segment);
    }

    for (int i = 0; i < initial[5]; ++i) {
        double a, b, r;
        cout << "\nGiven circle C" << i + 1 << ": enter center_x center_y radius.\n";
        cout << "  This represents (x-center_x)^2 + (y-center_y)^2 = radius^2.\n";
        cout << "C" << i + 1 << " = " << flush;
        if (!(cin >> a >> b >> r) || r < 0.0) {
            cerr << "Invalid circle. Radius must be non-negative.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        if (!graph.PointAllowed({a, b})) {
            cerr << "Given circle center is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.AddInitial(Element::FromCoefficients(a, b, Sq(r), Type::Circle));
    }
    graph.initialElementCount = graph.elements.size();

    cout << "\n[Step 5/6] Enter ALL construction goals simultaneously.\n";
    cout << "  Input three integers in this order: number_of_target_lines number_of_target_circles number_of_target_points\n";
    cout << "  Example: '2 1 3' means the final construction must contain 2 specified lines,\n";
    cout << "           1 specified circle, and 3 specified points at the same time.\n";
    cout << "Enter target counts L C P: " << flush;
    if (!(cin >> goalLines >> goalCircles >> goalPoints) ||
        goalLines < 0 || goalCircles < 0 || goalPoints < 0) {
        cerr << "Invalid target counts. All three counts must be non-negative integers.\n";
        return ExitWithPause(1, pauseOnExit);
    }
    if (goalLines == 0 && goalCircles == 0 && goalPoints == 0) {
        cerr << "At least one target line, circle, or point is required.\n";
        return ExitWithPause(1, pauseOnExit);
    }

    for (int i = 0; i < goalLines; ++i) {
        double a, b, c;
        cout << "\nTarget line GL" << i + 1 << ": enter a b c for a*x + b*y = c.\n";
        cout << "GL" << i + 1 << " (a b c) = " << flush;
        if (!(cin >> a >> b >> c)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        graph.goalElements.push_back(Element::FromCoefficients(a, b, c, Type::Line));
    }

    for (int i = 0; i < goalCircles; ++i) {
        double a, b, r;
        cout << "\nTarget circle GC" << i + 1 << ": enter center_x center_y radius.\n";
        cout << "GC" << i + 1 << " = " << flush;
        if (!(cin >> a >> b >> r) || r < 0.0) {
            cerr << "Invalid target circle. Radius must be non-negative.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        if (!graph.PointAllowed({a, b})) {
            cerr << "Target circle center is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.goalElements.push_back(Element::FromCoefficients(a, b, Sq(r), Type::Circle));
    }

    for (int i = 0; i < goalPoints; ++i) {
        double x, y;
        cout << "\nTarget point GP" << i + 1 << ": enter x y.\n";
        cout << "GP" << i + 1 << " = " << flush;
        if (!(cin >> x >> y)) { cerr << "Invalid or incomplete numeric input.\n"; return ExitWithPause(1, pauseOnExit); }
        if (!graph.PointAllowed({x, y})) {
            cerr << "Target point is outside the grid rectangle.\n";
            return ExitWithPause(1, pauseOnExit);
        }
        graph.goalPoints.push_back({x, y});
    }

    if (requestedSolutions == 0) {
        cout << "\n[Step 6/6] 需要几个不同的解？\n";
        cout << "请输入所需解数：" << flush;
        string countText;
        if (!(cin >> countText) || !ParseSolutionCount(countText, requestedSolutions)) {
            cerr << "Solution count must be an integer in 1..4294967295.\n";
            return ExitWithPause(1, pauseOnExit);
        }
    }

    cout << "\n============================================================\n";
    cout << "Search configuration\n";
    cout << "  E limit: " << limit << "\n";
    cout << "  Requested distinct solutions: " << requestedSolutions << "\n";
    cout << "  Tools: " << (selectedMode == 3 ? "grid straightedge only (mode 3)" : toolType == 0 ? "compass only" : toolType == 1 ? "straightedge only" : "compass + straightedge") << "\n";
    if (graph.gridMode) {
        cout << "  Grid rectangle: [0," << graph.gridM << "] x [0," << graph.gridN << "] (closed)\n";
        cout << "  Automatic grid lines: " << graph.gridM + graph.gridN + 2 << " (free)\n";
        cout << "  Initial in-grid points: " << graph.points.size() << "\n";
    }
    cout << "  Goals: " << goalLines << " line(s), " << goalCircles << " circle(s), " << goalPoints << " point(s)\n";
    const bool parallelCanSplit = threads > 1 && limit > 1;
    const size_t effectiveStreamDedupEntries = streamDedupEntries;
    constexpr size_t AUTO_FRONTIER_TASKS_PER_WORKER = 8;
    const size_t autoFrontierTarget = max<size_t>(
        static_cast<size_t>(threads) * AUTO_FRONTIER_TASKS_PER_WORKER, 8);
    const uint16_t maxSplitDepth = static_cast<uint16_t>(max(1, limit - 2));

    // Manual depth is exact. Auto mode probes successive depths and stops at
    // the first frontier with enough independent tasks. Each probe is capped at
    // autoFrontierTarget, so a huge frontier is sampled only up to the amount
    // actually useful for load balancing. Planning shares the same global time
    // budget as the real search and is additionally capped at 10% (max 3 s).
    uint16_t splitDepth = static_cast<uint16_t>(
        min<int>(requestedTaskDepth ? requestedTaskDepth : 1, maxSplitDepth));
    vector<pair<uint16_t, size_t>> frontierProbeCounts;
    vector<bool> frontierProbeThreshold;
    bool frontierProbeTimedOut = false;
    double frontierPlanningSeconds = 0.0;

    const auto searchStartTime = chrono::steady_clock::now();
    const auto globalDeadline = searchStartTime + chrono::duration_cast<chrono::steady_clock::duration>(
        chrono::duration<double>(timeLimitSeconds));

    if (parallelCanSplit && requestedTaskDepth == 0) {
        const double planBudgetSeconds = min(3.0, max(0.05, timeLimitSeconds * 0.10));
        const auto planDeadline = min(globalDeadline, searchStartTime +
            chrono::duration_cast<chrono::steady_clock::duration>(
                chrono::duration<double>(planBudgetSeconds)));
        for (uint16_t d = 1; d <= maxSplitDepth; ++d) {
            Graph probeGraph = graph;
            Solver probe(toolType, symmetry, goalFirst, lowMemory,
                         effectiveStreamDedupEntries, 0, timeLimitSeconds);
            auto r = probe.ProbeFrontierTasks(
                probeGraph, limit, d, autoFrontierTarget, planDeadline);
            frontierProbeCounts.push_back({d, r.tasks});
            frontierProbeThreshold.push_back(r.thresholdReached);
            splitDepth = d;
            if (r.thresholdReached) break;
            if (r.timedOut || chrono::steady_clock::now() >= planDeadline) {
                frontierProbeTimedOut = true;
                break;
            }
        }
        frontierPlanningSeconds = chrono::duration<double>(
            chrono::steady_clock::now() - searchStartTime).count();
    }

    // Keep enough look-ahead to feed all workers without materializing a huge
    // fraction of a wide frontier. Manual and automatic depths use the same FIFO.
    const size_t queueFactor = 8;
    const size_t prefixQueueCapacity = parallelCanSplit
        ? max<size_t>(8, static_cast<size_t>(threads) * queueFactor) : 1;
    cout << "  Geometry EPS: " << scientific << setprecision(3) << EPS << defaultfloat << "\n";
    cout << "  Time limit: " << timeLimitSeconds << " seconds\n";
    cout << "  Worker threads: " << threads << " / " << availableThreads
         << " process-available (" << hardwareThreads << " hardware)\n";
    cout << "============================================================\n";
    cout << "Searching...\n" << flush;

    SearchStats stats;
    SolutionCollector solutions(requestedSolutions);
    bool found = false; // during search: requested quota reached

    bool timedOut = false;
    size_t totalTTBytes = 0;

    // Workers share only the bounded prefix FIFO, cancellation flags and
    // solution collector. Graphs and candidate tables stay private and are
    // reused across tasks. The main thread is the lightweight producer.
    const auto startTime = searchStartTime;
    if (threads == 1 || limit <= 1) {
        Solver solver(toolType, symmetry, goalFirst, lowMemory,
                      effectiveStreamDedupEntries, ttMB * 1024ULL * 1024ULL,
                      timeLimitSeconds);
        solver.SetSolutionCollector(&solutions);
        found = solver.Search(graph, limit, stats);
        timedOut = solver.TimedOut();
        totalTTBytes = solver.TranspositionBytes();
    } else {
        ParallelControl control;
        control.deadline = globalDeadline;

        PrefixTaskQueue queue(control, prefixQueueCapacity);
        vector<SearchStats> workerStats(threads);
        vector<size_t> completedTasks(threads, 0);
        vector<exception_ptr> workerErrors(threads);
        vector<thread> workers;
        workers.reserve(threads);
        exception_ptr producerError;
        SearchStats producerStats;
        bool producerFound = false;

        try {
            // Start consumers before producing the DFS frontier. Each Pop()
            // transfers exclusive ownership of a subtree. Graph rollback and
            // retained Solver tables avoid allocation churn at task boundaries.
            for (uint32_t wi = 0; wi < threads; ++wi) {
                workers.emplace_back([&, wi]() {
                    try {
                        PrefixTask task;
                        Graph localGraph = graph;
                        const Mark initialMark = localGraph.GetMark();
                        Solver localSolver(toolType, symmetry, goalFirst, lowMemory,
                                           effectiveStreamDedupEntries, 0, timeLimitSeconds);
                        localSolver.SetSolutionCollector(&solutions);
                        while (queue.Pop(task)) {
                            localGraph.Rollback(initialMark);
                            SearchStats taskStats;
                            const bool localFound = localSolver.SearchPrefixTask(
                                localGraph, limit, task, taskStats, &control);
                            MergeSearchStats(workerStats[wi], taskStats);
                            if (!localFound && !control.stop.load(memory_order_acquire))
                                ++completedTasks[wi];
                            if (localFound || control.stop.load(memory_order_acquire)) {
                                queue.Close();
                                break;
                            }
                        }
                    } catch (...) {
                        workerErrors[wi] = current_exception();
                        control.stop.store(true, memory_order_release);
                        queue.Close();
                    }
                });
            }

            Graph producerGraph = graph;
            Solver producer(toolType, symmetry, goalFirst, lowMemory,
                            effectiveStreamDedupEntries, 0, timeLimitSeconds);
            producer.SetSolutionCollector(&solutions);
            function<bool(const Graph&)> sink = [&](const Graph& g) {
                return queue.Push(g.elements, g.initialElementCount);
            };
            producerFound = producer.ProduceFrontierTasks(
                producerGraph, limit, producerStats, &control, sink, splitDepth);
            if (producerFound) {
                control.found.store(true, memory_order_release);
                control.stop.store(true, memory_order_release);
            }
        } catch (...) {
            producerError = current_exception();
            control.stop.store(true, memory_order_release);
        }
        // Normal producer exhaustion closes admission but lets workers drain.
        // A quota, timeout, or exception has already set stop, so it cancels.
        queue.Close();
        for (thread& worker : workers) worker.join();

        // Never label an allocation/thread/worker failure as an exhausted search.
        if (!producerError) {
            for (const exception_ptr& error : workerErrors)
                if (error) { producerError = error; break; }
        }
        if (producerError) {
            try { rethrow_exception(producerError); }
            catch (const exception& ex) { cerr << "Parallel ordered-frontier search failed: " << ex.what() << "\n"; }
            catch (...) { cerr << "Parallel ordered-frontier search failed: unknown error\n"; }
            return ExitWithPause(1, pauseOnExit);
        }

        found = producerFound || control.found.load(memory_order_acquire);
        timedOut = control.timedOut.load(memory_order_acquire) && !found;
        MergeSearchStats(stats, producerStats);
        size_t completed = 0;
        for (uint32_t wi = 0; wi < threads; ++wi) {
            MergeSearchStats(stats, workerStats[wi]);
            completed += completedTasks[wi];
        }
        totalTTBytes = 0;
    }
    const double seconds = chrono::duration<double>(chrono::steady_clock::now() - startTime).count();

    // Only now, after joins, read and format the stored results.
    const size_t resultCount = solutions.Count();
    const bool quotaReached = resultCount >= requestedSolutions;
    found = resultCount != 0;
    if (quotaReached) timedOut = false;
    cout << "\n============================================================\n";
    double solutionOutputSeconds = 0.0;
    if (found) {
        const auto outputStart = chrono::steady_clock::now();
        for (size_t i = 0; i < resultCount; ++i) {
            const Graph& solved = solutions.Entries()[i].graph;
            cout << "\n========== 第 " << i + 1 << " / " << resultCount
                 << " 个不同解（" << solved.elements.size() - solved.initialElementCount
                 << "E） ==========\n";
            if (readableOutput) readable_report::Print(solved, givenPointCount);
            else solved.PrintSolution();
        }
        cout.flush();
        solutionOutputSeconds = chrono::duration<double>(
            chrono::steady_clock::now() - outputStart).count();
    }
    if (quotaReached) {
        cout << "\n已返回所需的 " << requestedSolutions << " 个不同解。\n";
        cout << "Result status: QUOTA_REACHED\n";
    } else if (timedOut) {
        cout << "\nSEARCH STOPPED: time limit reached (" << timeLimitSeconds << " seconds).\n";
        cout << "已找到并输出 " << resultCount << " / " << requestedSolutions
             << " 个不同解\n";
        cout << "Result status: TIMEOUT_PARTIAL\n";
    } else {
        if (!found) cout << "No solution found within the specified E limit.\n";
        cout << "The search completed without timing out.\n";
        cout << "Result status: EXHAUSTED\n";
    }
    cout << "Requested solutions:" << requestedSolutions << "\n";
    cout << "Distinct solutions:" << resultCount << "\n";
    cout << "Successful state visits:" << solutions.SuccessfulVisits() << "\n";
    cout << fixed << setprecision(6);
    cout << "Search Time:" << seconds << "\n";
    if (found) cout << "Solution output time:" << setprecision(9)
                    << solutionOutputSeconds << setprecision(6) << "\n";
    cout << "Worker threads:" << threads << "\n";
    cout << "Nodes:" << stats.nodes << "\n";
    cout << "Raw candidates:" << stats.rawCandidates << "\n";
    cout << "Unique candidates:" << stats.uniqueCandidates << "\n";
    cout << "============================================================\n";

    int exitCode = 2;
    if (timedOut) exitCode = 3; // also for a partial list: it is not exhausted
    else if (found) exitCode = 0;

    return ExitWithPause(exitCode, pauseOnExit);
}
