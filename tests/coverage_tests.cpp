// Standalone C++20 test: -O2 -pthread -fno-fast-math -ffp-contract=off.
// Only tiny synthetic cases; no benchmark inputs or application main required.
#include "../src/heuristic.hpp"

using namespace bs;
namespace {
using Clock = chrono::steady_clock;

void Require(bool ok, const string& why) {
    if (!ok) throw runtime_error(why);
}
double Seconds(Clock::time_point start) {
    return chrono::duration<double>(Clock::now() - start).count();
}

// A broken queue/join must fail this short executable, not hang the test runner.
// The normal destructor wakes and joins this sleeping watchdog immediately.
class Watchdog {
    mutex mutex_;
    condition_variable wake_;
    bool done_ = false;
    thread thread_;
public:
    Watchdog() : thread_([this] {
        unique_lock<mutex> lock(mutex_);
        if (!wake_.wait_for(lock, chrono::milliseconds(4500), [&] { return done_; })) {
            cerr << "FAIL coverage tests: 4.5s watchdog (queue/worker join stalled)\n" << flush;
            std::_Exit(2);
        }
    }) {}
    ~Watchdog() {
        { lock_guard<mutex> lock(mutex_); done_ = true; }
        wake_.notify_all();
        thread_.join();
    }
};

bool Exact(double a, double b) { return bit_cast<uint64_t>(a) == bit_cast<uint64_t>(b); }
bool Exact(const Element& a, const Element& b) {
    return a.type == b.type && a.bound == b.bound &&
        Exact(a.a,b.a) && Exact(a.b,b.b) && Exact(a.c,b.c);
}

// Independent ordinary-tool coefficient regeneration. Do not use MakeCandidate,
// FromPoints, reverse lookup, target incidence, or returned points as witnesses.
Element Definition(Point p, Point q, Type type) {
    Require(!SamePoint(p,q), "operation needs two distinct known points");
    Element e;
    e.type = type;
    if (type == Type::Circle) {
        e.a = p.x; e.b = p.y;
        e.c = (p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y);
    } else {
        Require(type == Type::Line, "new bounded elements are not ordinary tools");
        e.a = q.y-p.y; e.b = p.x-q.x; e.c = p.x*q.y-p.y*q.x;
        if (abs(e.b) >= EPS) { e.a /= e.b; e.c /= e.b; e.b = 1; }
        else if (abs(e.a) >= EPS) { e.c /= e.a; e.a = 1; e.b = 0; }
    }
    if (abs(e.a) < EPS) e.a = 0;
    if (abs(e.b) < EPS) e.b = 0;
    if (abs(e.c) < EPS) e.c = 0;
    return e;
}
bool KnownPair(const Graph& graph, const Element& e) {
    for (size_t i = 0; i < graph.points.size(); ++i)
        for (size_t j = 0; j < graph.points.size(); ++j)
            if (i != j && !SamePoint(graph.points[i],graph.points[j]) &&
                Exact(Definition(graph.points[i],graph.points[j],e.type),e)) return true;
    return false;
}
string Fingerprint(const Graph& g) {
    ostringstream out;
    out << g.initialElementCount << ',' << g.gridMode << ',' << g.gridFast << ','
        << g.gridLinesReady << ',' << g.gridM << ',' << g.gridN << ';';
    auto point = [&](Point p) { out << bit_cast<uint64_t>(p.x) << ',' << bit_cast<uint64_t>(p.y) << ';'; };
    auto element = [&](const Element& e) {
        out << int(e.type) << ',' << e.bound << ',' << bit_cast<uint64_t>(e.a) << ','
            << bit_cast<uint64_t>(e.b) << ',' << bit_cast<uint64_t>(e.c) << ';';
    };
    for (Point p : g.points) point(p);
    out << '/'; for (uint16_t birth : g.pointBirth) out << birth << ',';
    out << '/'; for (const Element& e : g.elements) element(e);
    out << '/'; for (const Bound& b : g.bounds) { point(b.p1); point(b.p2); }
    out << '/'; for (Point p : g.goalPoints) point(p);
    out << '/'; for (const Element& e : g.goalElements) element(e);
    return out.str();
}
void Seal(Graph& g) { g.initialElementCount = g.elements.size(); }

void Replay(const Graph& initial, const Graph& result, int limit, int tools) {
    Require(initial.initialElementCount == initial.elements.size(), "test root is not sealed");
    Require(result.initialElementCount == initial.elements.size(), "returned graph resealed paid prefix");
    Require(result.elements.size() >= initial.elements.size(), "lost initial elements");
    const size_t paid = result.elements.size() - initial.elements.size();
    Require(paid <= static_cast<size_t>(limit), "actual paid E exceeds limit");
    for (size_t i = 0; i < initial.elements.size(); ++i)
        Require(Exact(initial.elements[i],result.elements[i]), "initial element changed");
    Graph replay = initial; // never copy result points, births, goals, or bounds
    for (size_t step = 0; step < paid; ++step) {
        const Element& e = result.elements[initial.elements.size()+step];
        Require(e.type == Type::Line || e.type == Type::Circle, "illegal new tool");
        Require(e.bound == NO_BOUND, "fabricated operation bound");
        Require(tools != 0 || e.type == Type::Circle, "line in compass-only mode");
        Require(tools != 1 || e.type == Type::Line, "circle in straightedge-only mode");
        Require(KnownPair(replay,e), "no actual known-pair witness at paid step " + to_string(step+1));
        Require(replay.Apply(e,static_cast<uint16_t>(step+1)), "duplicate counted as paid E");
    }
    Require(replay.GoalsMet() && result.GoalsMet(), "collector/replay misses ORIGINAL goals");
    Require(Fingerprint(replay) == Fingerprint(result), "ordered replay differs in points/births/goals/domain");
    for (Point p : replay.points)
        Require(isfinite(p.x) && isfinite(p.y) && replay.PointAllowed(p), "nonfinite/out-of-domain point");
}
void VerifyEntries(const Graph& initial, int limit, int tools, const SolutionCollector& collector) {
    for (const auto& entry : collector.Entries()) {
        Replay(initial,entry.graph,limit,tools);
        Require(entry.newElements.size() == entry.graph.elements.size()-initial.elements.size(), "collector paid count");
        for (size_t i = 0; i < entry.newElements.size(); ++i)
            Require(Exact(entry.newElements[i],entry.graph.elements[initial.elements.size()+i]), "collector suffix mismatch");
        Require(entry.circles == static_cast<size_t>(count_if(entry.newElements.begin(),entry.newElements.end(),
            [](const Element& e) { return e.type == Type::Circle; })), "collector circle count");
    }
}
uint64_t SourceVisits(const HeuristicMetrics& m) {
    return m.beamSolutions + m.helperSolutions + m.rendezvousSolutions +
        m.chainSolutions + m.pointJoinSolutions + m.coverageSolutions + m.radiusProbeSolutions;
}

Graph LineGoal() {
    Graph g;
    g.AddPoint({0,0},0); g.AddPoint({2,0},0);
    // A given ray is not its infinite carrier: the goal line still costs 1E.
    g.AddInitialBounded({0,0},{2,0},Type::Ray);
    g.goalElements = {Definition(g.points[0],g.points[1],Type::Line)};
    Seal(g);
    return g;
}
Graph PointGoal() {
    Graph g;
    g.AddPoint({0,0},0); g.AddPoint({2,0},0);
    g.AddInitial(Element::FromCoefficients(1,0,1,Type::Line));
    g.goalPoints = {{1,0}};
    Seal(g);
    return g;
}
Graph TriangleGoals() {
    Graph g;
    for (Point p : {Point{0,0},Point{2,0},Point{0,2}}) g.AddPoint(p,0);
    for (size_t i = 0; i < g.points.size(); ++i)
        for (size_t j = i+1; j < g.points.size(); ++j)
            g.goalElements.push_back(Definition(g.points[i],g.points[j],Type::Line));
    Seal(g);
    return g; // only three drawable lines; exactly one unordered solution
}
Graph MixedGoals() {
    Graph g;
    for (Point p : {Point{0,0},Point{2,0},Point{0,2},Point{2,2}}) g.AddPoint(p,0);
    g.goalElements = {Definition(g.points[0],g.points[1],Type::Line),
                      Definition(g.points[2],g.points[3],Type::Line)};
    g.goalPoints = {{1,1}}; // two diagonals plus the two horizontal goals, 4E
    Seal(g);
    return g;
}
Graph CancelGoal() {
    Graph g;
    g.gridMode = true; g.gridM = g.gridN = 2;
    g.AddAutomaticGridLines();
    g.goalPoints = {{3.125,3.375}}; // outside closed domain; no accidental quota
    Seal(g);
    return g;
}
HeuristicOptions Options(uint32_t threads = 4) {
    HeuristicOptions o;
    o.threads = threads; o.beamWidth = 8; o.branchLimit = 16; o.seed = 17;
    o.structural = false; // focus on coverage/beam, not unrelated structure helpers
    return o; // retain default auto coverage, adaptive, unlimited restarts and tails
}

enum class Cancel { None, Preset, Async };
struct Run {
    HeuristicResult result;
    SearchStats stats;
    vector<SolutionCollector::Entry> entries;
    uint64_t successful = 0, duplicates = 0;
    bool stopped = false, found = false, timedOut = false;
    double seconds = 0;
};
Run Search(const Graph& initial, int limit, int tools, const HeuristicOptions& options,
           size_t quota = 1, double seconds = 1.0, Cancel cancel = Cancel::None) {
    const string before = Fingerprint(initial);
    const auto h1 = initial.StateHash1(), h2 = initial.StateHash2();
    SolutionCollector collector(quota);
    ParallelControl control;
    auto slots = make_unique<ProgressSlot[]>(options.threads);
    const auto start = Clock::now();
    control.deadline = start + chrono::duration_cast<Clock::duration>(chrono::duration<double>(seconds));
    control.stop.store(cancel == Cancel::Preset);
    jthread stopper;
    if (cancel == Cancel::Async) stopper = jthread([&] {
        this_thread::sleep_for(chrono::milliseconds(5));
        control.stop.store(true,memory_order_release); // deliberately no queue notification
    });
    Run run;
    run.result = RunHeuristic(initial,limit,tools,options,collector,control,run.stats,slots.get(),options.threads);
    if (stopper.joinable()) stopper.join();
    run.seconds = Seconds(start);
    run.stopped = control.stop.load(); run.found = control.found.load(); run.timedOut = control.timedOut.load();
    for (size_t i = 0; i < options.threads; ++i) {
        const auto s = slots[i].Read();
        Require(!s.active && s.depth <= limit, "progress slot remains active/invalid after join");
    }
    Require(Fingerprint(initial) == before && initial.StateHash1() == h1 && initial.StateHash2() == h2,
            "RunHeuristic mutated its sealed initial graph");
    VerifyEntries(initial,limit,tools,collector);
    run.entries = collector.Entries();
    run.successful = collector.SuccessfulVisits(); run.duplicates = collector.DuplicateVisits();
    Require(collector.Count() <= quota && run.result.quotaReached == (collector.Count() == quota), "quota/count mismatch");
    Require(run.found == run.result.quotaReached && (!run.found || run.stopped), "global found is not quota semantics");
    Require(run.result.timedOut == (run.timedOut && !run.found), "timeout/quota precedence mismatch");
    Require(run.successful == collector.Count()+run.duplicates, "collector visits double-counted");
    Require(run.result.metrics.peakBeam <= options.beamWidth, "per-worker beam peak was summed");
    Require(run.stats.applied >= run.result.metrics.evaluated, "beam applies missing from merged stats");
    const uint64_t rootVisit = initial.GoalsMet() && cancel != Cancel::Preset ? 1 : 0;
    const uint64_t sources = SourceVisits(run.result.metrics)+rootVisit;
    // At quota, a racing beam/helper can observe a goal after collector admission
    // closed. Those verified visits may exceed admitted visits, not fall below.
    Require(sources >= run.successful, "lost successful source attribution");
    if (!run.found) Require(sources == run.successful, "partial-quota source visits were merged twice");
    return run;
}
string Summary(const Run& run) {
    const auto& m = run.result.metrics;
    ostringstream out;
    out << " [entries=" << run.entries.size() << " quota=" << run.result.quotaReached
        << " stop=" << run.stopped << " timeout=" << run.result.timedOut
        << " visits=" << run.successful << " beam=" << m.beamSolutions
        << " helper=" << m.helperSolutions << " coverage=" << m.coverageSolutions
        << " tasks=" << m.coverageTasks << " seconds=" << run.seconds << ']';
    return out.str();
}
void Quick(const Run& run, const string& why) {
    Require(run.seconds < .2, why + " waited for deadline/slow join" + Summary(run));
}
void Solved(const Run& run, size_t minimumPaid, const string& why) {
    Require(run.result.quotaReached && !run.result.timedOut && !run.entries.empty(), why + " did not reach quota");
    for (const auto& entry : run.entries)
        Require(entry.newElements.size() >= minimumPaid, why + " undercounted actual paid operations");
    Quick(run,why);
}

void SuccessMatrix() {
    Require(HeuristicOptions{}.coverageThreads == -1 && HeuristicOptions{}.restarts == 0,
            "default auto coverage/restarts changed");
    for (uint32_t threads : {4u,8u}) for (int coverage : {-1,0,2}) {
        auto o = Options(threads);
        if (coverage != -1) o.coverageThreads = coverage; // truly exercise the default
        const string label = "threads="+to_string(threads)+" coverage="+to_string(coverage);
        for (const Graph& g : {LineGoal(),PointGoal()}) {
            Require(!g.GoalsMet(), "success fixture is already satisfied");
            const auto run = Search(g,4,1,o);
            Solved(run,1,label);
            if (coverage == 0)
                Require(run.result.metrics.coverageTasks == 0 && run.result.metrics.coverageSolutions == 0,
                        "coverage=0 still used coverage lane");
        }
    }
}
void PartialQuotaAndDrain() {
    for (uint32_t threads : {4u,8u}) for (int coverage : {-1,2}) {
        auto o = Options(threads); o.coverageThreads = coverage;
        // Family merging can discard alternative orders, so an unlimited beam
        // may stop naturally or restart until the short budget. Neither outcome
        // proves exhaustion; both must preserve the one distinct partial result.
        const auto partial = Search(TriangleGoals(),4,1,o,2,.025);
        Require(partial.entries.size() == 1 && partial.entries[0].newElements.size() == 3 &&
                !partial.result.quotaReached && partial.stopped == partial.result.timedOut,
                "triangle partial collection lost its certificate or reported full quota" + Summary(partial));
        Require(partial.result.metrics.coverageTasks > 0, "partial quota never consumed a prefix task");
        Require(SourceVisits(partial.result.metrics) == partial.successful, "partial visits duplicated on metrics merge");
        Quick(partial,"partial-quota budget cancellation");
        // A fresh run/collector must not inherit tasks, quota flags or counters.
        const auto fresh = Search(TriangleGoals(),4,1,o);
        Solved(fresh,3,"fresh triangle quota");
    }
    auto o = Options(8);
    const auto onlyOne = Search(LineGoal(),4,1,o,2);
    Require(onlyOne.entries.size() == 1 && !onlyOne.stopped && !onlyOne.result.timedOut,
            "one legal line was mistaken for two distinct constructions");
    Quick(onlyOne,"single-pair partial quota");
}
void MultipleGoalsAndQuota() {
    for (int coverage : {-1,2}) {
        auto o = Options(8); o.coverageThreads = coverage;
        const auto mixed = Search(MixedGoals(),4,1,o);
        Solved(mixed,4,"eight-thread mixed line/point goals");
        // Multiple distinct certificates, not repeated visits from eight roots.
        Graph g = TriangleGoals(); g.goalElements.resize(1);
        const auto multi = Search(g,4,1,o,3);
        Solved(multi,1,"eight-thread quota=3");
        Require(multi.entries.size() == 3, "duplicate visits filled distinct quota");
    }
}
void DisabledCoverage() {
    for (uint32_t threads : {4u,8u}) for (int coverage : {-1,2}) {
        auto finite = Options(threads); finite.coverageThreads = coverage; finite.restarts = 1;
        const auto run = Search(TriangleGoals(),4,1,finite,2);
        Require(run.entries.size() == 1 && !run.result.timedOut &&
                run.result.metrics.coverageTasks == 0 && run.result.metrics.coverageSolutions == 0,
                "finite restarts=1 enabled coverage");
        Require(run.result.metrics.restarts <= threads, "restarts=1 was not per worker");
        Quick(run,"finite restarts");
        auto pure = Options(threads); pure.coverageThreads = coverage; pure.tailCandidates = 0;
        const auto beam = Search(LineGoal(),4,1,pure,2);
        const auto& m = beam.result.metrics;
        Require(beam.entries.size() == 1 && !beam.result.timedOut && m.coverageTasks == 0 &&
                m.coverageSolutions == 0 && m.tailCalls == 0 && m.helperSolutions == 0 &&
                m.rendezvousSolutions == 0 && m.chainSolutions == 0 && m.pointJoinSolutions == 0 &&
                m.beamSolutions == beam.successful && m.beamSolutions > 0,
                "tailCandidates=0 did not remain pure beam on the tiny fixture" + Summary(beam));
        Quick(beam,"tailCandidates=0");
    }
    auto off = Options(8); off.coverageThreads = 0; off.tailCandidates = 0;
    Solved(Search(PointGoal(),4,1,off),1,"explicit coverage=0 pure beam");
}
void Cancellation() {
    for (uint32_t threads : {4u,8u}) for (int coverage : {-1,2}) {
        auto o = Options(threads); o.coverageThreads = coverage;
        const Graph g = CancelGoal();
        const auto preset = Search(g,6,1,o,1,1,Cancel::Preset);
        Require(preset.stopped && !preset.result.timedOut && preset.entries.empty() &&
                preset.stats.nodes == 0 && preset.stats.rawCandidates == 0 &&
                preset.stats.applied == 0 && preset.result.metrics.coverageTasks == 0,
                "preset external stop did work or became timeout");
        Quick(preset,"preset stop");
        const auto expired = Search(g,6,1,o,1,-.001);
        Require(expired.stopped && expired.result.timedOut && expired.entries.empty() &&
                expired.stats.nodes == 0 && expired.stats.rawCandidates == 0 && expired.stats.applied == 0,
                "expired budget did search work or lost timeout");
        Quick(expired,"expired budget");
        const auto cancelled = Search(g,6,1,o,1,1,Cancel::Async);
        Require(cancelled.stopped && !cancelled.result.quotaReached && !cancelled.result.timedOut &&
                cancelled.entries.empty(), "caller stop became timeout/quota");
        Quick(cancelled,"asynchronous global.stop");
        const auto timed = Search(g,6,1,o,1,.008);
        Require(timed.result.timedOut && timed.stopped && !timed.result.quotaReached && timed.entries.empty(),
                "live deadline failed to stop coverage/beam workers");
        Quick(timed,"live deadline");
    }
}

void RejectOptions(const Graph& initial, int limit, const HeuristicOptions& options, bool preset = false) {
    SolutionCollector collector(1);
    ParallelControl control;
    control.deadline = Clock::now()+chrono::seconds(1); control.stop.store(preset);
    SearchStats stats;
    const string before = Fingerprint(initial);
    bool rejected = false;
    try { RunHeuristic(initial,limit,1,options,collector,control,stats); }
    catch (const invalid_argument& e) { rejected = *e.what() != '\0'; }
    Require(rejected, "expected explicit invalid_argument: threads="+to_string(options.threads)+
            " coverage="+to_string(options.coverageThreads)+" limit="+to_string(limit));
    Require(collector.Count() == 0 && stats.nodes == 0 && stats.rawCandidates == 0 && stats.applied == 0 &&
            !control.found.load() && control.stop.load() == preset,
            "invalid options did work/changed control before rejection");
    Require(Fingerprint(initial) == before, "rejected initial graph mutated");
}
void InvalidCoverage() {
    // Validate the REQUESTED count, not only the resolved eligible lane count.
    // These used to slip through on finite/disabled/E0/already-met/stopped paths.
    for (uint32_t threads : {4u,8u}) for (int value : {1,int(threads),int(threads)+1,-2}) {
        for (int mode = 0; mode < 10; ++mode) {
            auto o = Options(threads); o.coverageThreads = value;
            Graph g = LineGoal(); int limit = 4; bool preset = false;
            if (mode == 1) o.restarts = 1;
            if (mode == 2) o.tailCandidates = 0;
            if (mode == 3) o.tailSeconds = 0;
            if (mode == 4) o.adaptive = false;
            if (mode == 5) limit = 3;
            if (mode == 6) limit = 0;
            if (mode == 7) g.goalElements.clear();
            if (mode == 8) preset = true;
            if (mode == 9) { g.goalElements.clear(); limit = 0; }
            try { RejectOptions(g,limit,o,preset); }
            catch (const exception& e) { throw runtime_error("validation mode="+to_string(mode)+": "+e.what()); }
        }
    }
    for (int value : {numeric_limits<int>::min(),numeric_limits<int>::max()}) {
        auto o = Options(); o.coverageThreads = value; RejectOptions(LineGoal(),4,o);
    }
}
void SealedRootAndVerifier() {
    for (int defect = 0; defect < 3; ++defect) {
        Graph g = PointGoal();
        if (defect == 0) --g.initialElementCount;
        if (defect == 1) ++g.initialElementCount;
        if (defect == 2) g.pointBirth.pop_back();
        RejectOptions(g,4,Options());
    }
    const Graph initial = PointGoal();
    Graph solved = initial;
    Require(solved.Apply(Definition(initial.points[0],initial.points[1],Type::Line),1), "verifier setup");
    Replay(initial,solved,1,1);
    auto rejects = [&](const Graph& g, int limit = 4) {
        bool rejected = false;
        try { Replay(initial,g,limit,1); } catch (const runtime_error&) { rejected = true; }
        Require(rejected, "independent verifier accepted forged certificate");
    };
    rejects(solved,0);
    Graph forged = solved; Seal(forged); rejects(forged);
    forged = solved; forged.pointBirth.back() = 0; rejects(forged);
    forged = initial; forged.AddPoint({1,0},0); rejects(forged);
    forged = initial;
    Require(forged.Apply(Element::FromCoefficients(0,1,1,Type::Line),1), "forged line setup");
    forged.goalPoints = {{1,1}}; rejects(forged);
}

void QueueDrain() {
    ParallelControl control; control.deadline = Clock::now()+chrono::seconds(1);
    PrefixTaskQueue queue(control,2);
    const Element a = Definition({0,0},{2,0},Type::Line);
    const Element b = Definition({0,0},{0,2},Type::Line);
    vector<Element> source{a,b};
    Require(queue.Push(source,1), "queue refused first suffix");
    source[1] = a; // task owns its ordered suffix; no shared Graph/vector state
    Require(queue.Push(source,1), "queue refused second suffix");
    queue.Close();
    PrefixTask task;
    Require(queue.Pop(task) && task.prefix.size() == 1 && Exact(task.prefix[0],b), "first task/suffix was mutated");
    Require(queue.Pop(task) && task.prefix.size() == 1 && Exact(task.prefix[0],a), "FIFO second task lost");
    Require(!queue.Pop(task) && !queue.Push(source,0), "closed queue did not terminate");
    const auto stats = queue.GetStats();
    Require(stats.produced == 2 && stats.claimed == 2 && stats.peak == 2 && !control.stop.load(),
            "natural queue drain metrics/control mismatch");
}
void QueueAtomicStop() {
    for (bool full : {false,true}) for (bool quota : {false,true}) {
        ParallelControl control; control.deadline = Clock::now()+chrono::seconds(1);
        PrefixTaskQueue queue(control,1);
        const vector<Element> elements{Definition({0,0},{2,0},Type::Line)};
        if (full) Require(queue.Push(elements,0), "full queue setup");
        atomic<bool> entered{false}, returned{false};
        bool accepted = true;
        jthread waiter([&] {
            entered.store(true,memory_order_release);
            if (full) accepted = queue.Push(elements,0);
            else { PrefixTask task; accepted = queue.Pop(task); }
            returned.store(true,memory_order_release);
        });
        while (!entered.load(memory_order_acquire)) this_thread::yield();
        this_thread::sleep_for(chrono::milliseconds(5));
        const bool wasWaiting = !returned.load(memory_order_acquire);
        const auto cancelled = Clock::now();
        if (quota) control.found.store(true,memory_order_release);
        control.stop.store(true,memory_order_release);
        // No Close/notify from the test: exercise both blocking wait directions.
        waiter.join();
        Require(wasWaiting && !accepted, "queue wait unexpectedly accepted work");
        Require(Seconds(cancelled) < .2, "queue waited until deadline after atomic stop");
        Require(!control.timedOut.load() && control.found.load() == quota, "queue stop relabeled as timeout/quota");
        const auto stats = queue.GetStats();
        Require(stats.produced == (full ? 1u : 0u) && stats.claimed == 0, "cancelled queue admitted/claimed work");
    }
}
} // namespace

int main() {
    Watchdog watchdog;
    EPS = 1e-10;
    const pair<const char*,function<void()>> tests[] = {
        {"sealed initial graph / independent known-pair replay",SealedRootAndVerifier},
        {"invalid coverage counts, including ineligible/early returns",InvalidCoverage},
        {"4/8 threads default auto and explicit coverage=0/2",SuccessMatrix},
        {"partial quota / natural prefix drain / fresh collectors",PartialQuotaAndDrain},
        {"8-thread multiple goals and distinct quota=3",MultipleGoalsAndQuota},
        {"finite restarts and tailCandidates=0 disable coverage",DisabledCoverage},
        {"expired/live budget and external global.stop join",Cancellation},
        {"queue ordered suffix ownership / natural drain",QueueDrain},
        {"queue empty/full waits: atomic stop and quota wake",QueueAtomicStop}
    };
    const auto start = Clock::now();
    size_t passed = 0;
    for (const auto& [name,test] : tests) {
        const auto begin = Clock::now();
        try {
            test(); ++passed;
            cout << "PASS " << name << " (" << Seconds(begin) << "s)\n" << flush;
        } catch (const exception& e) {
            cerr << "FAIL " << name << ": " << e.what() << '\n' << flush;
        }
    }
    cout << "coverage tests: " << passed << '/' << size(tests) << " groups passed; wall="
         << Seconds(start) << "s\n" << flush;
    return passed == size(tests) ? 0 : 1;
}
