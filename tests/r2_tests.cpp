// Standalone C++20 test; use -pthread -fno-fast-math -ffp-contract=off.
// No application main, private-access shim, or expensive benchmark suite.
#include "../src/family_pool.hpp"
#include "../src/rendezvous.hpp"
#include "../src/heuristic.hpp"
#include <filesystem>

using namespace bs;
namespace {
using Clock = chrono::steady_clock;
namespace fs = std::filesystem;

void Require(bool ok, const string& why) {
    if (!ok) throw runtime_error(why);
}
bool Exact(double a, double b) {
    return bit_cast<uint64_t>(a) == bit_cast<uint64_t>(b);
}
bool Exact(const Element& a, const Element& b) {
    return a.type == b.type && a.bound == b.bound &&
           Exact(a.a,b.a) && Exact(a.b,b.b) && Exact(a.c,b.c);
}
bool SameOperations(const vector<Element>& a, const vector<Element>& b) {
    return a.size() == b.size() && equal(a.begin(),a.end(),b.begin(),
        [](const Element& x,const Element& y) { return Exact(x,y); });
}
struct EpsScope {
    double old = EPS;
    explicit EpsScope(double value) { EPS = value; }
    ~EpsScope() { EPS = old; }
};
void Seal(Graph& g) { g.initialElementCount = g.elements.size(); }

// Independent coefficient regeneration: do not use MakeCandidate/FromPoints,
// goal incidences, the solver's reverse lookup, or any returned point as input.
Element Definition(Point p, Point q, Type type) {
    Require(!SamePoint(p,q),"operation needs two distinct known points");
    Element e;
    e.type = type;
    if (type == Type::Circle) {
        e.a = p.x; e.b = p.y;
        e.c = (p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y);
    } else {
        Require(type == Type::Line,"new bounded elements are not legal tools");
        e.a = q.y-p.y; e.b = p.x-q.x; e.c = p.x*q.y-p.y*q.x;
        if (abs(e.b) >= EPS) { e.a /= e.b; e.c /= e.b; e.b = 1; }
        else if (abs(e.a) >= EPS) { e.c /= e.a; e.a = 1; e.b = 0; }
    }
    if (abs(e.a) < EPS) e.a = 0;
    if (abs(e.b) < EPS) e.b = 0;
    if (abs(e.c) < EPS) e.c = 0;
    return e;
}
bool KnownPair(const Graph& graph, const Element& operation) {
    for (size_t i = 0; i < graph.points.size(); ++i)
        for (size_t j = 0; j < graph.points.size(); ++j) {
            if (i == j || SamePoint(graph.points[i],graph.points[j])) continue;
            if (Exact(Definition(graph.points[i],graph.points[j],operation.type),operation))
                return true;
        }
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
Graph Replay(const Graph& initial, const Graph& result, int limit, int tools) {
    Require(result.initialElementCount == initial.elements.size(),"changed initial element count");
    Require(result.elements.size() >= initial.elements.size(),"lost initial elements");
    Require(result.elements.size()-initial.elements.size() <= static_cast<size_t>(limit),"returned E exceeds budget");
    for (size_t i = 0; i < initial.elements.size(); ++i)
        Require(Exact(initial.elements[i],result.elements[i]),"changed initial operation");
    Graph replay = initial;
    for (size_t i = initial.elements.size(); i < result.elements.size(); ++i) {
        const Element& e = result.elements[i];
        Require(e.type == Type::Line || e.type == Type::Circle,"unsupported tool");
        Require(e.bound == NO_BOUND,"fabricated operation bound");
        Require(tools != 0 || e.type == Type::Circle,"tools=0 returned a straight line");
        Require(tools != 1 || e.type == Type::Line,"tools=1 returned a circle");
        Require(KnownPair(replay,e),"operation is not raw-coefficient regeneration from a known pair at step " +
                to_string(i-initial.elements.size()+1));
        Require(replay.Apply(e,static_cast<uint16_t>(i-initial.elements.size()+1)),"duplicate paid operation");
    }
    Require(replay.GoalsMet(),"ordered replay does not meet ORIGINAL goals");
    Require(result.GoalsMet(),"collector contains non-solution");
    Require(Fingerprint(replay) == Fingerprint(result),"returned points/births/goals/domain differ from ordered replay");
    for (Point p : replay.points) {
        Require(isfinite(p.x) && isfinite(p.y),"nonfinite returned point");
        if (initial.gridMode)
            Require(p.x >= 0 && p.x <= initial.gridM && p.y >= 0 && p.y <= initial.gridN,
                    "point outside the CLOSED grid rectangle");
    }
    return replay;
}
void VerifyEntries(const Graph& initial, int limit, int tools, const SolutionCollector& collector) {
    for (const auto& entry : collector.Entries()) {
        Replay(initial,entry.graph,limit,tools);
        const vector<Element> suffix(entry.graph.elements.begin()+initial.elements.size(),entry.graph.elements.end());
        Require(SameOperations(suffix,entry.newElements),"collector suffix differs from graph");
        Require(entry.circles == static_cast<size_t>(count_if(suffix.begin(),suffix.end(),
            [](const Element& e) { return e.type == Type::Circle; })),"collector circle count");
    }
}
template<class F> void Rejects(F&& f, const string& why) {
    bool rejected = false;
    try { f(); } catch (const runtime_error&) { rejected = true; }
    Require(rejected,"verifier accepted negative control: " + why);
}

struct Input {
    Graph graph;
    int limit = 0, mode = 2;
    int Tools() const { return mode == 3 ? 1 : mode; }
};
Input Parse(istream& in) {
    Input input;
    in >> input.limit >> input.mode;
    Require(bool(in) && input.limit >= 0 && input.mode >= 0 && input.mode <= 3,"invalid fixture header");
    if (input.mode == 3) {
        input.graph.gridMode = true;
        in >> input.graph.gridM >> input.graph.gridN;
        Require(input.graph.gridM >= 0 && input.graph.gridN >= 0,"invalid grid size");
    }
    int counts[5]{};
    for (int& n : counts) { in >> n; Require(n >= 0,"negative fixture count"); }
    for (int i = 0; i < counts[0]; ++i) {
        Point p; in >> p.x >> p.y; input.graph.AddPoint(p,0);
    }
    if (input.mode == 3) input.graph.AddAutomaticGridLines();
    for (int kind = 1; kind < 5; ++kind) for (int i = 0; i < counts[kind]; ++i) {
        double a = 0, b = 0, c = 0, d = 0;
        if (kind == 2 || kind == 3) {
            in >> a >> b >> c >> d;
            input.graph.AddInitialBounded({a,b},{c,d},kind == 2 ? Type::Ray : Type::Segment);
        } else {
            in >> a >> b >> c;
            input.graph.AddInitial(Element::FromCoefficients(a,b,kind == 4 ? c*c : c,
                                                            kind == 4 ? Type::Circle : Type::Line));
        }
    }
    Seal(input.graph);
    int goals[3]{};
    for (int& n : goals) { in >> n; Require(n >= 0,"negative goal count"); }
    for (int kind = 0; kind < 2; ++kind) for (int i = 0; i < goals[kind]; ++i) {
        double a = 0, b = 0, c = 0; in >> a >> b >> c;
        input.graph.goalElements.push_back(Element::FromCoefficients(a,b,kind ? c*c : c,
                                                                     kind ? Type::Circle : Type::Line));
    }
    for (int i = 0; i < goals[2]; ++i) { Point p; in >> p.x >> p.y; input.graph.goalPoints.push_back(p); }
    Require(bool(in),"incomplete fixture");
    return input;
}
fs::path FixturePath(const char* executable) {
    const fs::path suffix = fs::path("benchmarks") / "cases" / "t2_givenO_OX_E4.in";
    for (fs::path base : {fs::path(__FILE__).parent_path().parent_path(),
                         fs::current_path(),fs::absolute(executable).parent_path()}) {
        for (int depth = 0; depth < 4 && !base.empty(); ++depth) {
            if (fs::is_regular_file(base/suffix)) return fs::absolute(base/suffix);
            base = base.parent_path();
        }
    }
    throw runtime_error("cannot locate T2 fixture; pass its path as the only argument");
}

using FamilyKey = vector<heuristic_detail::RawElementKey>;
struct PoolEntry { double score; uint64_t serial; vector<Element> prefix; };
FamilyKey Key(const vector<Element>& prefix) {
    Require(!prefix.empty(),"empty family fixture");
    return heuristic_detail::ConstructionFamily(vector<Element>(prefix.begin(),prefix.end()-1),prefix.back());
}
void Offer(heuristic_detail::FamilyPool<PoolEntry>& pool, double score, uint64_t serial,
           vector<Element> prefix) {
    auto key = Key(prefix);
    pool.Offer(PoolEntry{score,serial,std::move(prefix)},std::move(key));
}
Element PoolLine(double c) { return Element::FromCoefficients(0,1,c,Type::Line); }
void FamilyRepresentative() {
    // Actual counterexample: the SAME raw operation multiset in another order
    // gives different EPS representatives, point counts, and even GoalsMet.
    // Family merging is incomplete diversity selection, NOT state equivalence.
    EpsScope eps(ldexp(1.0,-30));
    Graph initial;
    for (Point p : {Point{1,2},Point{2,3},Point{3,7+.75*EPS},Point{4,9+.75*EPS}})
        initial.AddPoint(p,0);
    initial.AddInitial(Element::FromCoefficients(1,0,0,Type::Line));
    initial.goalPoints = {{0,1+1.25*EPS}};
    Seal(initial);
    const Element a = Definition(initial.points[0],initial.points[1],Type::Line);
    const Element b = Definition(initial.points[2],initial.points[3],Type::Line);
    Graph ab = initial, ba = initial;
    Require(ab.Apply(a,1) && ab.Apply(b,2) && ba.Apply(b,1) && ba.Apply(a,2),"family order setup");
    Require(ab.points.size() != ba.points.size() && !ab.GoalsMet() && ba.GoalsMet(),
            "family regression must demonstrate unequal ordered states");
    const vector<Element> first{a,b}, better{b,a};
    Require(Key(first) == Key(better),"operation family is not order independent");
    const auto family = Key(first);
    Require(is_sorted(family.begin(),family.end()),"family key is not sorted");
    Require(Key({a,a}).size() == 2,"family accidentally discarded multiplicity");
    uint64_t discarded = 0, merged = 0; bool limited = false;
    {
        heuristic_detail::FamilyPool<PoolEntry> pool(4,discarded,merged,limited);
        Offer(pool,1,1,first);
        Offer(pool,9,9,better);  // score wins over the earlier serial
        Offer(pool,3,0,first);  // a later lower score must not replace it
        auto selected = pool.Take();
        Require(selected.size() == 1 && selected[0].score == 9 && selected[0].serial == 9,
                "wrong family representative");
        Require(SameOperations(selected[0].prefix,better),"family sorted the retained execution order");
        Replay(initial,ba,2,1);
        Require(pool.Take().empty(),"Take returned entries twice");
        Require(discarded == 2 && merged == 2 && limited,"family merge must count incomplete discards");
    }
    Require(discarded == 2 && merged == 2,"Take plus destructor double counted family discards");
}
void FamilyReplacementAndCaps() {
    const Element a = PoolLine(1), b = PoolLine(2), c = PoolLine(3), d = PoolLine(4);
    uint64_t discarded = 0, merged = 0; bool limited = false;
    {
        heuristic_detail::FamilyPool<PoolEntry> pool(3,discarded,merged,limited);
        Offer(pool,10,9,{a}); Offer(pool,20,2,{b}); Offer(pool,30,3,{c});
        Offer(pool,40,4,{a});  // move the former worst to best in the ordered index
        Offer(pool,25,5,{d});  // must evict b, not the replaced a
        Offer(pool,35,6,{b});  // evicted key must be insertable again; evicts d
        Offer(pool,30,1,{c}); Offer(pool,30,99,{c});
        auto result = pool.Take();
        Require(result.size() == 3 && result[0].serial == 4 && result[1].serial == 6 && result[2].serial == 1,
                "replacement/eviction/tie-break ordering is stale");
        Require(discarded == 5 && merged == 3 && limited,"replacement discard accounting");
        Require(pool.Take().empty(),"second Take is not empty");
        Offer(pool,1,10,{a});
        Require(pool.Take().size() == 1 && discarded == 5 && merged == 3,"Take did not reset family index");
    }
    Require(discarded == 5,"drained pool destructor counted entries twice");
    discarded = merged = 0; limited = false;
    {
        heuristic_detail::FamilyPool<PoolEntry> pool(1,discarded,merged,limited);
        Offer(pool,7,9,{a}); Offer(pool,6,1,{b});
        Offer(pool,7,10,{a}); Offer(pool,7,4,{a});
        Offer(pool,7,3,{b}); Offer(pool,8,0,{a});
        auto result = pool.Take();
        Require(result.size() == 1 && result[0].serial == 0 && Exact(result[0].prefix[0],a),"cap=1 selection");
    }
    Require(discarded == 5 && merged == 2 && limited,"cap=1 accounting");
    discarded = merged = 0; limited = false;
    {
        heuristic_detail::FamilyPool<PoolEntry> pool(0,discarded,merged,limited);
        Offer(pool,1,0,{a}); Offer(pool,2,1,{a});
        Require(pool.Take().empty(),"cap=0 retained a family");
    }
    Require(discarded == 2 && merged == 0 && limited,"cap=0 accounting");
    discarded = merged = 0; limited = false;
    {
        heuristic_detail::FamilyPool<PoolEntry> abandoned(2,discarded,merged,limited);
        Offer(abandoned,1,0,{a}); Offer(abandoned,2,1,{b});
    }
    Require(discarded == 2 && merged == 0 && limited,"unconsumed entries were not counted on cancellation");
    discarded = merged = 0; limited = false;
    {
        heuristic_detail::FamilyPool<PoolEntry> pool(4,discarded,merged,limited);
        Element signedZero = a, nearElement = a, circle = a;
        signedZero.a = -0.0; nearElement.c += .5*EPS; circle.type = Type::Circle;
        Require(SameElement(a,signedZero) && SameElement(a,nearElement),"raw-key negative control setup");
        Offer(pool,1,0,{a}); Offer(pool,1,1,{signedZero});
        Offer(pool,1,2,{nearElement}); Offer(pool,1,3,{circle});
        auto result = pool.Take();
        Require(result.size() == 4,"family key conflated raw bits/type with EPS equality");
        for (size_t i = 0; i < result.size(); ++i) Require(result[i].serial == i,"equal-score serial order");
    }
    Require(discarded == 0 && merged == 0 && !limited,"raw-distinct families incorrectly counted as merged");
}

struct RendezvousRun {
    bool quota = false, stopped = false, found = false, timedOut = false;
    SearchStats stats;
    rendezvous_detail::Counts counts;
    vector<SolutionCollector::Entry> entries;
    double seconds = 0;
};
RendezvousRun Find(const Graph& initial, int limit, int tools, double seconds = 1.0,
                   bool presetStop = false) {
    const string before = Fingerprint(initial);
    SolutionCollector collector(1);
    ParallelControl control;
    const auto start = Clock::now();
    control.deadline = start + chrono::seconds(2);
    control.stop.store(presetStop);
    const auto deadline = start + chrono::duration_cast<Clock::duration>(chrono::duration<double>(seconds));
    RendezvousRun run;
    run.quota = rendezvous_detail::Find(initial,limit,tools,collector,control,run.stats,run.counts,deadline,nullptr);
    run.seconds = chrono::duration<double>(Clock::now()-start).count();
    run.stopped = control.stop.load(); run.found = control.found.load(); run.timedOut = control.timedOut.load();
    Require(Fingerprint(initial) == before,"Find mutated the initial graph");
    Require(run.quota == (collector.Count() == 1) && run.quota == run.found,"Find return is not quota semantics");
    VerifyEntries(initial,limit,tools,collector);
    run.entries = collector.Entries();
    return run;
}
void NoWork(const RendezvousRun& r) {
    Require(!r.quota && !r.found && r.entries.empty(),"ineligible/cancelled Find returned a solution");
    Require(r.stats.nodes == 0 && r.stats.applied == 0 && r.stats.rawCandidates == 0 &&
            r.stats.uniqueCandidates == 0 && r.counts.proposals == 0 && r.counts.replayed == 0 &&
            r.counts.successes == 0 && r.counts.reachablePoints == 0,"ineligible/cancelled Find did extra work");
}
HeuristicOptions Options(uint32_t threads = 1) {
    HeuristicOptions options;
    options.beamWidth = 4; options.branchLimit = 8; options.restarts = 1;
    options.threads = threads; options.tailSeconds = 0; options.tailCandidates = 0;
    return options; // Do not override adaptive: verify r2 is the default.
}
struct EngineRun {
    HeuristicResult result;
    SearchStats stats;
    vector<SolutionCollector::Entry> entries;
    bool stopped = false, found = false;
    double seconds = 0;
};
enum class Cancel { None, Preset, Asynchronous };
EngineRun Search(const Graph& initial, int limit, int tools, HeuristicOptions options,
                 double seconds = 1, size_t quota = 1, Cancel cancel = Cancel::None) {
    const string before = Fingerprint(initial);
    SolutionCollector collector(quota);
    ParallelControl control;
    auto slots = make_unique<ProgressSlot[]>(options.threads);
    control.stop.store(cancel == Cancel::Preset);
    const auto start = Clock::now();
    control.deadline = start + chrono::duration_cast<Clock::duration>(chrono::duration<double>(seconds));
    // jthread joins even if RunHeuristic or a validation check throws.
    jthread stopper;
    if (cancel == Cancel::Asynchronous) stopper = jthread([&] {
        this_thread::sleep_for(chrono::milliseconds(2));
        control.stop.store(true,memory_order_release);
    });
    EngineRun run;
    run.result = RunHeuristic(initial,limit,tools,options,collector,control,run.stats,slots.get(),options.threads);
    if (stopper.joinable()) stopper.join();
    run.seconds = chrono::duration<double>(Clock::now()-start).count();
    run.stopped = control.stop.load(); run.found = control.found.load();
    for (size_t i = 0; i < options.threads; ++i) {
        const auto snapshot = slots[i].Read();
        Require(!snapshot.active && snapshot.depth <= limit,"worker/progress slot not joined cleanly");
    }
    Require(Fingerprint(initial) == before,"RunHeuristic mutated initial graph");
    VerifyEntries(initial,limit,tools,collector);
    Require(collector.Count() <= quota && run.result.quotaReached == (collector.Count() == quota),"engine quota mismatch");
    Require(run.result.quotaReached == run.found,"control.found did not mean quota");
    Require(!(run.result.quotaReached && run.result.timedOut),"quota incorrectly reported as timeout");
    Require(run.result.metrics.peakBeam <= options.beamWidth,"per-worker beam storage cap exceeded");
    Require(run.result.metrics.tailCalls == 0,"disabled tail helpers still ran");
    Require(run.seconds < max(0.0,seconds)+1.0,"search exceeded cancellation/deadline grace");
    run.entries = collector.Entries();
    return run;
}
void RealT2(const Input& input) {
    Require(EPS == 1e-10 && input.mode == 3 && input.Tools() == 1 && input.limit == 4,"T2 test settings");
    Require(input.graph.gridM == 6 && input.graph.gridN == 6 && !input.graph.GoalsMet(),"wrong T2 fixture");
    const auto first = Find(input.graph,4,1);
    Require(first.quota && first.entries.size() == 1 && first.entries[0].newElements.size() == 4,
            "real T2 rendezvous did not return a legal 4E construction");
    Require(first.counts.successes == 1 && first.counts.proposals > 0 && first.counts.replayed > 0 &&
            first.counts.reachablePoints > 0 && !first.timedOut,"T2 rendezvous metrics");
    const auto repeat = Find(input.graph,4,1);
    Require(repeat.quota && Fingerprint(first.entries[0].graph) == Fingerprint(repeat.entries[0].graph) &&
            first.counts.proposals == repeat.counts.proposals && first.counts.replayed == repeat.counts.replayed &&
            first.stats.rawCandidates == repeat.stats.rawCandidates,"fresh collector changed seedless rendezvous");
    Require(HeuristicOptions{}.adaptive,"r2 adaptive mode is not enabled by default");
    for (uint64_t seed : {uint64_t{1},uint64_t{17},numeric_limits<uint64_t>::max()}) {
        auto options = Options(); options.seed = seed;
        const auto run = Search(input.graph,4,1,options,2);
        Require(run.result.quotaReached && run.entries.size() == 1 && run.entries[0].newElements.size() == 4,
                "adaptive T2 failed for seed " + to_string(seed));
        Require(Fingerprint(run.entries[0].graph) == Fingerprint(first.entries[0].graph),"rendezvous construction depended on beam seed");
        Require(run.result.metrics.rendezvousSolutions == 1 && run.result.metrics.rendezvousReplays > 0 &&
                run.result.metrics.expanded == 0,"T2 was not solved by the structural rendezvous lane");
    }
    cout << "  T2 direct wall=" << first.seconds << "s, proposals=" << first.counts.proposals
         << ", replays=" << first.counts.replayed << ", E=4; seeds=1,17,UINT64_MAX\n";
}
void RendezvousGuards(const Input& input) {
    NoWork(Find(input.graph,3,1));
    NoWork(Find(input.graph,4,0));
    const auto stopped = Find(input.graph,4,1,1,true);
    NoWork(stopped);
    Require(stopped.stopped && !stopped.timedOut,"preset stop was lost or relabeled timeout");
    const auto expired = Find(input.graph,4,1,-.001);
    NoWork(expired);
    Require(!expired.stopped && !expired.timedOut,"local helper deadline poisoned global cancellation");
    // A bounded E3 beam may stop without finding a construction. That is not
    // an unsolvability proof; every returned entry must still obey E<=3.
    const auto e3 = Search(input.graph,3,1,Options(),.1);
    Require(e3.result.metrics.rendezvousReplays == 0 && e3.result.metrics.rendezvousSolutions == 0,
            "E3 invoked the four-operation rendezvous");
    Graph line;
    line.AddPoint({0,0},0); line.AddPoint({2,0},0);
    line.goalElements = {Definition(line.points[0],line.points[1],Type::Line)};
    Seal(line);
    const auto compass = Search(line,2,0,Options());
    Require(compass.entries.empty() && !compass.result.quotaReached,"compass-only search fabricated a goal line");
}
Graph ClosedGrid() {
    istringstream text("2\n3\n2 2\n0 0 0 0 0\n0 0 1\n0.5 0.5\n");
    Input input = Parse(text);
    Require(input.Tools() == 1 && input.graph.points.size() == 9 && input.graph.elements.size() == 6,
            "mode3 parser did not add automatic grid geometry");
    return input.graph;
}
void ClosedReplaysAndQuota() {
    const Graph initial = ClosedGrid();
    const Element a = Definition({0,0},{1,1},Type::Line);
    const Element b = Definition({0,1},{1,0},Type::Line);
    const Element c = Definition({0,2},{2,0},Type::Line);
    Graph ab = initial, ba = initial;
    Require(ab.Apply(a,1) && ab.Apply(b,2) && ba.Apply(b,1) && ba.Apply(a,2),"closed prefix fixture");
    Replay(initial,ab,2,1); Replay(initial,ba,2,1);
    for (int repeat = 0; repeat < 2; ++repeat) {
        SolutionCollector collector(2); ParallelControl control;
        Require(!collector.Submit(ab,&control) && collector.Count() == 1 && !control.found.load(),"first hit prematurely filled quota=2");
        Require(!collector.Submit(ba,&control) && collector.Count() == 1 && collector.DuplicateVisits() == 1 &&
                !control.stop.load(),"reordered duplicate prematurely filled quota=2");
        Graph distinct = ab;
        Require(KnownPair(distinct,c) && distinct.Apply(c,3),"second distinct certificate fixture");
        Require(collector.Submit(distinct,&control) && collector.Count() == 2 && control.found.load() &&
                control.stop.load(),"second distinct construction did not fill quota");
        VerifyEntries(initial,3,1,collector);
    }
    // Same Solver, but each root/prefix task gets its own graph, collector and
    // cancellation state. A closed/replayed earlier run must not leak success.
    Solver solver(1,false,true,true,0,0,.1);
    for (const PrefixTask& task : {PrefixTask{},PrefixTask{{a}},PrefixTask{{b}}}) {
        for (size_t quota : {size_t{1},size_t{2}}) {
            Graph replay = initial; SolutionCollector collector(quota); ParallelControl control;
            control.deadline = Clock::now()+chrono::milliseconds(100);
            solver.SetSolutionCollector(&collector);
            SearchStats stats;
            const bool reached = solver.SearchPrefixTask(replay,2,task,stats,&control);
            Require(collector.Count() == 1 && reached == (quota == 1) && control.found.load() == reached &&
                    control.stop.load() == reached,"root/prefix replay leaked quota/success state");
            Require(!solver.TimedOut() && !control.timedOut.load(),"tiny root/prefix task timed out");
            VerifyEntries(initial,2,1,collector);
        }
    }
    solver.SetSolutionCollector(nullptr);
    auto options = Options(); options.beamWidth = 32; options.branchLimit = 64;
    const auto partial = Search(initial,2,1,options,1,2);
    Require(partial.entries.size() == 1 && !partial.result.quotaReached && !partial.stopped,
            "adaptive closed-grid beam reported quota after one unique construction");
    Graph met = initial; met.goalPoints = {{0,0}};
    for (size_t quota : {size_t{1},size_t{2}}) {
        const auto root = Search(met,0,1,Options(),1,quota);
        Require(root.entries.size() == 1 && root.entries[0].newElements.empty() &&
                root.result.quotaReached == (quota == 1),"already-satisfied root quota semantics");
    }
    Rejects([&] { Replay(initial,ab,1,1); },"E2 certificate under E1");
    Rejects([&] { Replay(initial,ab,2,0); },"line certificate under tools=0");
    Graph forged = ab; forged.pointBirth.back() = 0;
    Rejects([&] { Replay(initial,forged,2,1); },"forged point birth");
    forged = initial; forged.AddPoint({.5,.5},0);
    Rejects([&] { Replay(initial,forged,2,1); },"free goal point");
    forged = ab; forged.points.push_back({-.25*EPS,1}); forged.pointBirth.push_back(2);
    Rejects([&] { Replay(initial,forged,2,1); },"EPS-expanded exterior point");
    Graph nearGraph;
    nearGraph.AddPoint({0,1+.75*EPS},0); nearGraph.AddPoint({2,1+.75*EPS},0);
    const Element target = Element::FromCoefficients(0,1,1,Type::Line);
    Require(SameElement(Definition(nearGraph.points[0],nearGraph.points[1],Type::Line),target) && !KnownPair(nearGraph,target),
            "origin verifier accepted EPS-near target coefficients instead of actual operation");
}
void InitialGraphContract(const Input& input) {
    for (int defect = 0; defect < 3; ++defect) {
        Graph invalid = input.graph;
        if (defect == 0) --invalid.initialElementCount;
        if (defect == 1) ++invalid.initialElementCount;
        if (defect == 2) invalid.pointBirth.pop_back();
        const string before = Fingerprint(invalid);
        for (bool direct : {true,false}) {
            SolutionCollector collector(1); ParallelControl control;
            control.deadline = Clock::now()+chrono::seconds(1);
            SearchStats stats; rendezvous_detail::Counts counts;
            bool rejected = false;
            try {
                if (direct)
                    rendezvous_detail::Find(invalid,4,1,collector,control,stats,counts,control.deadline,nullptr);
                else
                    RunHeuristic(invalid,4,1,Options(),collector,control,stats);
            } catch (const invalid_argument&) { rejected = true; }
            Require(rejected,"unsealed root or inconsistent births did not throw invalid_argument");
            Require(collector.Count() == 0 && stats.nodes == 0 && stats.rawCandidates == 0 && stats.applied == 0 &&
                    counts.replayed == 0 && !control.stop.load() && !control.found.load(),
                    "invalid root performed work before rejection");
            Require(Fingerprint(invalid) == before,"rejected root was mutated");
        }
    }
}
void LegacyBeamAndCancellation(const Input& input) {
    Graph line;
    line.AddPoint({0,0},0); line.AddPoint({2,0},0);
    line.goalElements = {Definition(line.points[0],line.points[1],Type::Line)};
    Seal(line);
    auto legacy = Options(); legacy.adaptive = false;
    const auto solved = Search(line,1,1,legacy);
    Require(solved.result.quotaReached && solved.result.metrics.beamSolutions > 0 &&
            solved.result.metrics.rendezvousReplays == 0 && solved.result.metrics.familyMerged == 0,
            "adaptive=false did not use the r1 beam path");
    Graph hard;
    hard.gridMode = true; hard.gridM = hard.gridN = 6; hard.AddAutomaticGridLines();
    hard.goalPoints = {{100.123,100.456}}; // outside the exact closed domain
    Seal(hard);
    for (bool adaptive : {true,false}) {
        auto options = Options(3); options.adaptive = adaptive; options.restarts = 0;
        const auto preset = Search(input.graph,4,1,options,1,1,Cancel::Preset);
        Require(preset.entries.empty() && preset.stopped && !preset.result.timedOut &&
                preset.stats.rawCandidates == 0 && preset.stats.applied == 0,"preset three-worker stop did work");
        const auto expired = Search(input.graph,4,1,options,-.001);
        Require(expired.entries.empty() && expired.stopped && expired.result.timedOut &&
                expired.stats.rawCandidates == 0 && expired.stats.applied == 0,"expired three-worker start did work");
        const auto cancelled = Search(hard,8,1,options,1,1,Cancel::Asynchronous);
        Require(cancelled.stopped && !cancelled.result.quotaReached && !cancelled.result.timedOut &&
                cancelled.entries.empty(),"asynchronous three-worker stop misreported result");
        const auto timed = Search(hard,8,1,options,.003);
        Require(timed.result.timedOut && timed.stopped && !timed.result.quotaReached && timed.entries.empty(),
                "three-worker wall budget did not cancel cleanly");
        Require(timed.result.metrics.budgetLimited && cancelled.result.metrics.budgetLimited,
                "cancelled incomplete work was not marked budget-limited");
    }
}
} // namespace

int main(int argc, char** argv) {
    try {
        Require(argc <= 2,"usage: r2_tests [T2_INPUT]");
        EPS = 1e-10;
        const fs::path fixture = argc == 2 ? fs::absolute(argv[1]) : FixturePath(argv[0]);
        ifstream in(fixture);
        Require(bool(in),"cannot open fixture: " + fixture.string());
        const Input input = Parse(in);
        cout << "Fixture: " << fixture.string() << "; EPS=" << EPS << '\n';
        const pair<const char*,function<void()>> tests[] = {
            {"FamilyPool representative / non-equivalence",FamilyRepresentative},
            {"FamilyPool replacement / raw keys / caps / accounting",FamilyReplacementAndCaps},
            {"real T2 rendezvous / known-pair replay / seed independence",[&] { RealT2(input); }},
            {"rendezvous E/tool/deadline/stop guards",[&] { RendezvousGuards(input); }},
            {"closed root/prefix replays / independent quota=2",ClosedReplaysAndQuota},
            {"sealed initial graph / point-birth contract",[&] { InitialGraphContract(input); }},
            {"r1 beam opt-out / three-thread cancellation",[&] { LegacyBeamAndCancellation(input); }}
        };
        const auto start = Clock::now();
        size_t passed = 0;
        for (const auto& [name,test] : tests) {
            const auto begin = Clock::now();
            try {
                test(); ++passed;
                cout << "PASS " << name << " (" << chrono::duration<double>(Clock::now()-begin).count() << "s)\n";
            } catch (const exception& e) {
                cerr << "FAIL " << name << ": " << e.what() << '\n';
            }
        }
        cout << "r2 tests: " << passed << '/' << size(tests) << " groups passed; wall="
             << chrono::duration<double>(Clock::now()-start).count() << "s\n";
        return passed == size(tests) ? 0 : 1;
    } catch (const exception& e) {
        cerr << "FAIL r2 tests: " << e.what() << '\n';
        return 1;
    }
}
