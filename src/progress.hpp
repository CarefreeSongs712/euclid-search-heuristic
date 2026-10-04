#pragma once
#include "core.hpp"

namespace bs {

// Exactly one solver thread owns a slot. The sampler reads ONLY the atomics;
// it must never inspect the owner's SearchStats or the non-atomic base below.
// Publish at coarse polling boundaries (e.g. every 1024 polls), not per node.
class alignas(64) ProgressSlot {
public:
    struct Snapshot {
        uint64_t nodes = 0, rawCandidates = 0, uniqueCandidates = 0;
        uint16_t depth = 0;
        double rootFraction = -1.0;
        bool active = false;
    };
private:
    struct alignas(64) Published {
        atomic<uint64_t> nodes{0}, rawCandidates{0}, uniqueCandidates{0};
        atomic<uint16_t> depth{0};
        atomic<double> rootFraction{-1.0};
        atomic<bool> active{false};
    } published_;
    // Keep owner-only writes off the cache line sampled by the reporter.
    struct alignas(64) Owner {
        uint64_t baseNodes = 0, baseRaw = 0, baseUnique = 0;
        uint64_t startNodes = 0, startRaw = 0, startUnique = 0;
        bool active = false;
    } owner_;

    static uint64_t Delta(uint64_t now, uint64_t start) noexcept {
        return now >= start ? now - start : 0;
    }
public:
    ProgressSlot() = default;
    ProgressSlot(const ProgressSlot&) = delete;
    ProgressSlot& operator=(const ProgressSlot&) = delete;

    // Call after task stats have been initialized, before any task work.
    // Both fresh per-task stats and monotonically cumulative stats are valid.
    void BeginTask(const SearchStats& stats) noexcept {
        if (owner_.active) return;
        owner_.startNodes = stats.nodes;
        owner_.startRaw = stats.rawCandidates;
        owner_.startUnique = stats.uniqueCandidates;
        owner_.active = true;
        published_.rootFraction.store(-1.0, memory_order_relaxed);
        published_.depth.store(0, memory_order_relaxed);
        published_.active.store(true, memory_order_relaxed);
        Publish(stats);
    }

    void Publish(const SearchStats& stats, uint16_t depth = 0) noexcept {
        if (!owner_.active) return;
        published_.nodes.store(owner_.baseNodes + Delta(stats.nodes, owner_.startNodes),
                               memory_order_relaxed);
        published_.rawCandidates.store(owner_.baseRaw + Delta(stats.rawCandidates, owner_.startRaw),
                                       memory_order_relaxed);
        published_.uniqueCandidates.store(owner_.baseUnique + Delta(stats.uniqueCandidates, owner_.startUnique),
                                          memory_order_relaxed);
        published_.depth.store(depth, memory_order_relaxed);
    }

    // Idempotent so cleanup after an early return cannot add a task twice.
    // Pair with BeginTask using a solver-local scope guard, including exceptions.
    void EndTask(const SearchStats& stats) noexcept {
        if (!owner_.active) return;
        Publish(stats);
        owner_.baseNodes += Delta(stats.nodes, owner_.startNodes);
        owner_.baseRaw += Delta(stats.rawCandidates, owner_.startRaw);
        owner_.baseUnique += Delta(stats.uniqueCandidates, owner_.startUnique);
        owner_.active = false;
        published_.active.store(false, memory_order_relaxed);
    }

    // Completed root work only. Negative/non-finite values mean unknown.
    void SetRootFraction(double fraction) noexcept {
        published_.rootFraction.store(isfinite(fraction) && fraction >= 0.0
            ? min(1.0, fraction) : -1.0, memory_order_relaxed);
    }

    Snapshot Read() const noexcept {
        // Individually monotonic, approximate snapshots are sufficient for
        // telemetry; no seqlock/spin or synchronization with hot-path stats.
        return {published_.nodes.load(memory_order_relaxed),
                published_.rawCandidates.load(memory_order_relaxed),
                published_.uniqueCandidates.load(memory_order_relaxed),
                published_.depth.load(memory_order_relaxed),
                published_.rootFraction.load(memory_order_relaxed),
                published_.active.load(memory_order_relaxed)};
    }
};
static_assert(alignof(ProgressSlot) >= 64);
static_assert(sizeof(ProgressSlot) % 64 == 0);

enum class ProgressPhase { Planning, Searching, Draining, Stopping };

struct ProgressState {
    const int eLimit;
    const uint32_t workerThreads;
    const bool parallel;
    bool heuristic = false;
    atomic<ProgressPhase> phase{ProgressPhase::Planning};
    atomic<uint64_t> producedTasks{0}, completedTasks{0};
    // True only after NATURAL producer exhaustion; cancellation does not make
    // the admitted task count a complete frontier or permit an exhaustion ETA.
    atomic<bool> producerDone{false};
    atomic<double> searchStartOffset{0.0};

    ProgressState(int limit, uint32_t workers, bool isParallel)
        : eLimit(limit), workerThreads(workers), parallel(isParallel) {}
};

class ProgressReporter {
    using Clock = chrono::steady_clock;
    ProgressState& state_;
    const ProgressSlot* slots_;
    const size_t slotCount_;
    const Clock::time_point start_;
    const double budgetSeconds_;
    const chrono::duration<double> interval_;
    mutex mutex_;
    condition_variable wake_;
    bool stopped_ = false;
    thread sampler_;
    exception_ptr error_;
    uint64_t lastNodes_ = 0;
    Clock::time_point lastSample_;

    static const char* PhaseName(ProgressPhase phase) noexcept {
        switch (phase) {
        case ProgressPhase::Planning: return "frontier-planning";
        case ProgressPhase::Searching: return "searching";
        case ProgressPhase::Draining: return "draining-prefixes";
        case ProgressPhase::Stopping: return "stopping";
        }
        return "unknown";
    }

    void Sample() {
        const auto now = Clock::now();
        const double elapsed = chrono::duration<double>(now - start_).count();
        const double sampleSeconds = chrono::duration<double>(now - lastSample_).count();
        uint64_t nodes = 0, raw = 0, unique = 0;
        uint32_t activeWorkers = 0;
        uint16_t depth = 0;
        double rootFraction = -1.0;
        for (size_t i = 0; i < slotCount_; ++i) {
            const auto s = slots_[i].Read();
            nodes += s.nodes;
            raw += s.rawCandidates;
            unique += s.uniqueCandidates;
            depth = max(depth, s.depth);
            if (i == 0) rootFraction = s.rootFraction;
            if ((!state_.parallel || i != 0) && s.active) ++activeWorkers;
        }
        const double rate = sampleSeconds > 0.0 && nodes >= lastNodes_
            ? static_cast<double>(nodes - lastNodes_) / sampleSeconds : 0.0;
        lastSample_ = now;
        lastNodes_ = nodes;
        const auto phase = state_.phase.load(memory_order_acquire);
        const bool producerDone = state_.producerDone.load(memory_order_acquire);
        const uint64_t produced = state_.producedTasks.load(memory_order_relaxed);
        const uint64_t completed = state_.completedTasks.load(memory_order_relaxed);
        const double searchElapsed = max(0.0, elapsed - state_.searchStartOffset.load(memory_order_relaxed));

        ostringstream line;
        line << fixed << setprecision(1)
             << "[progress] E<=" << state_.eLimit
             << " phase=" << PhaseName(phase)
             << " elapsed=" << elapsed << "s"
             << " nodes=" << nodes << " rate=" << rate << "/s"
             << " candidates(raw/unique)=" << raw << '/' << unique
             << " threads=" << state_.workerThreads << " active=" << activeWorkers
             << " depth~=" << depth;
        double fraction = rootFraction;
        bool enoughSamples = searchElapsed >= 2.0;
        const char* unknownReason = "warming up / insufficient completed root work";
        if (state_.heuristic) {
            line << " engine=heuristic coverage=unknown";
        } else if (state_.parallel) {
            line << " prefixes=" << completed << '/';
            if (producerDone) line << produced;
            else line << "? (admitted=" << produced << ", producer unfinished)";
            fraction = producerDone && produced > 0
                ? static_cast<double>(completed) / static_cast<double>(produced) : -1.0;
            enoughSamples = enoughSamples && completed >= max<uint64_t>(2, state_.workerThreads);
            unknownReason = producerDone ? "insufficient completed prefixes" : "producer unfinished; total work unknown";
        } else {
            line << " prefixes=n/a root=";
            if (rootFraction >= 0.0) line << (100.0 * rootFraction) << '%';
            else line << "unknown";
        }
        if (state_.heuristic) {
            line << " exhaustion ETA=unavailable (heuristic search discards paths)";
        } else if (phase == ProgressPhase::Planning) {
            line << " exhaustion ETA=unknown (frontier planning)";
        } else if (phase == ProgressPhase::Stopping) {
            line << " exhaustion ETA=unknown (search stopping)";
        } else if (enoughSamples && fraction >= 0.01 && fraction < 1.0) {
            const double eta = searchElapsed * (1.0 - fraction) / fraction;
            if (isfinite(eta))
                line << " exhaustion ETA~" << eta << "s (rough/workload-biased estimate, "
                     << (state_.parallel ? "completed prefixes" : "root fraction") << ')';
            else line << " exhaustion ETA=unknown (estimate out of range)";
        } else {
            line << " exhaustion ETA=unknown (" << unknownReason << ')';
        }
        line << " budget-left=" << max(0.0, budgetSeconds_ - elapsed)
             << "s (timeout budget, not exhaustion ETA)\n";
        // A single stderr insertion avoids changing stdout's format flags and
        // preserves the existing machine-readable result stream unchanged.
        cerr << line.str() << flush;
    }

    void Run() noexcept {
        try {
            unique_lock<mutex> lock(mutex_);
            while (!wake_.wait_for(lock, interval_, [&] { return stopped_; })) {
                lock.unlock();
                Sample();
                lock.lock();
            }
        } catch (...) {
            // Never terminate the process from a telemetry thread. The main
            // thread rethrows after joining, including allocation failures.
            error_ = current_exception();
        }
    }
public:
    ProgressReporter(ProgressState& state, const ProgressSlot* slots, size_t count,
                     Clock::time_point start, double budgetSeconds,
                     double intervalSeconds, bool enabled)
        : state_(state), slots_(slots), slotCount_(count), start_(start),
          budgetSeconds_(budgetSeconds), interval_(intervalSeconds), lastSample_(start) {
        if (enabled) sampler_ = thread([this] { Run(); });
    }
    ProgressReporter(const ProgressReporter&) = delete;
    ProgressReporter& operator=(const ProgressReporter&) = delete;
    ~ProgressReporter() { Stop(); }

    void Stop() {
        {
            lock_guard<mutex> lock(mutex_);
            stopped_ = true;
        }
        wake_.notify_all();
        if (sampler_.joinable()) sampler_.join();
    }
    void Finish() {
        Stop();
        if (error_) rethrow_exception(error_);
    }
};

} // namespace bs
