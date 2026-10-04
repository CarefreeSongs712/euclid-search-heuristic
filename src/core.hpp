#pragma once
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

namespace bs {
inline double EPS = 1e-11;
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

inline PointBucket BucketOf(const Point& p) { return {Quantize(p.x), Quantize(p.y)}; }
inline ElementBucket BucketOf(const Element& e) {
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

inline void MergeSearchStats(SearchStats& dst, const SearchStats& src) {
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

    bool WaitReady(unique_lock<mutex>& lock,condition_variable& cv,
                   const function<bool()>& ready) {
        while(!ready()) {
            const auto now=chrono::steady_clock::now();
            if(now>=control_.deadline)return false;
            const auto tick=min(control_.deadline,now+chrono::milliseconds(20));
            cv.wait_until(lock,tick,ready);
        }
        return true;
    }
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
        if (!WaitReady(lock,notFull_,ready)) {
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
        if (!WaitReady(lock,notEmpty_,ready)) {
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


} // namespace bs
