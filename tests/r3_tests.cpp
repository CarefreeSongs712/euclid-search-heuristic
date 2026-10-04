// Standalone C++20 correctness tests, not a performance benchmark.
// Build with -O2 -pthread -fno-fast-math -ffp-contract=off; never include main.cpp.
#include "../src/chain_join.hpp"
#include "../src/foundation.hpp"
#include "../src/certificate.hpp"
#if __has_include("../src/point_join.hpp")
#include "../src/point_join.hpp"
#define BS_R3_HAS_POINT_JOIN 1
#else
#define BS_R3_HAS_POINT_JOIN 0
#endif
#include <filesystem>

using namespace bs;
namespace {
using Clock = chrono::steady_clock;
namespace fs = std::filesystem;

void Require(bool ok, const string& why) {
    if (!ok) throw runtime_error(why);
}
bool Exact(double a, double b) { return bit_cast<uint64_t>(a) == bit_cast<uint64_t>(b); }
bool Exact(const Element& a, const Element& b) {
    return a.type == b.type && a.bound == b.bound &&
           Exact(a.a,b.a) && Exact(a.b,b.b) && Exact(a.c,b.c);
}
bool SameOperations(const vector<Element>& a, const vector<Element>& b) {
    return a.size() == b.size() && equal(a.begin(),a.end(),b.begin(),
        [](const Element& x,const Element& y) { return Exact(x,y); });
}
void Seal(Graph& graph) { graph.initialElementCount = graph.elements.size(); }

// Independent raw-coefficient regeneration, without FromPoints/MakeCandidate,
// proposal-index points, goal coordinates, or certificate-export witnesses.
Element Definition(Point p, Point q, Type type) {
    Require(!SamePoint(p,q),"operation needs two distinct known points");
    Element e;
    e.type = type;
    if (type == Type::Circle) {
        e.a = p.x; e.b = p.y;
        e.c = (p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y);
    } else {
        Require(type == Type::Line,"new bounded elements are not ordinary tools");
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
            if (Exact(Definition(graph.points[i],graph.points[j],operation.type),operation)) return true;
        }
    return false;
}
Point StoredPoint(const Graph& graph, Point requested) {
    for (Point p : graph.points) if (SamePoint(p,requested)) return p;
    throw runtime_error("fixture defining point is not stored in the initial graph");
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
Graph Replay(const Graph& initial, const Graph& result, int limit, int tools = 2,
             size_t toolsFrom = 0, bool requireGoals = true) {
    Require(initial.initialElementCount == initial.elements.size(),"unsealed replay root");
    Require(result.initialElementCount == initial.initialElementCount,"prefix was resealed/free");
    Require(result.elements.size() >= initial.elements.size(),"lost initial elements");
    Require(limit >= 0 && result.elements.size()-initial.elements.size() <= static_cast<size_t>(limit),
            "returned E exceeds budget");
    for (size_t i = 0; i < initial.elements.size(); ++i)
        Require(Exact(initial.elements[i],result.elements[i]),"changed initial element");
    Graph replay = initial;
    for (size_t i = initial.elements.size(); i < result.elements.size(); ++i) {
        const Element& e = result.elements[i];
        Require(e.type == Type::Line || e.type == Type::Circle,"unsupported paid tool");
        Require(e.bound == NO_BOUND && isfinite(e.a) && isfinite(e.b) && isfinite(e.c),"invalid paid element");
        Require(e.type == Type::Circle ? e.c > 0 : (e.a != 0 || e.b != 0),"zero-radius/zero-line paid element");
        if (i >= toolsFrom) {
            Require(tools != 0 || e.type == Type::Circle,"circle-only suffix returned a line");
            Require(tools != 1 || e.type == Type::Line,"line-only suffix returned a circle");
        }
        Require(KnownPair(replay,e),"no bitwise known-pair witness at paid step " +
                to_string(i-initial.elements.size()+1));
        Require(replay.Apply(e,static_cast<uint16_t>(i-initial.elements.size()+1)),"duplicate paid element");
    }
    if (requireGoals) Require(replay.GoalsMet() && result.GoalsMet(),"ordered replay misses ORIGINAL goals");
    Require(Fingerprint(replay) == Fingerprint(result),"returned state differs from ordered Apply replay");
    for (size_t i = 0; i < replay.points.size(); ++i) {
        const Point p = replay.points[i];
        Require(isfinite(p.x) && isfinite(p.y),"nonfinite stored point");
        Require(replay.pointBirth[i] <= result.elements.size()-initial.elements.size(),"point birth exceeds paid E");
        if (initial.gridMode)
            Require(p.x >= 0 && p.x <= initial.gridM && p.y >= 0 && p.y <= initial.gridN,
                    "point outside CLOSED grid rectangle");
    }
    return replay;
}
void CertificateGate(const Graph& initial, const Graph& solved) {
    ostringstream output;
    WriteConstructionCertificate(output,initial,solved);
    const string text = output.str();
    const size_t cost = solved.elements.size()-initial.elements.size();
    size_t count = 0, pos = 0;
    while ((pos = text.find("\"type\":",pos)) != string::npos) { ++count; ++pos; }
    Require(count == cost && text.ends_with("\"E\":" + to_string(cost) + "}"),
            "certificate exporter lost a paid prefix operation");
}
void VerifyEntries(const Graph& initial, const Graph& parent, int remaining, int tools,
                   const SolutionCollector& collector) {
    const int total = static_cast<int>(parent.elements.size()-initial.elements.size()) + remaining;
    for (const auto& entry : collector.Entries()) {
        Require(entry.graph.elements.size() >= parent.elements.size(),"join lost paid prefix");
        for (size_t i = 0; i < parent.elements.size(); ++i)
            Require(Exact(parent.elements[i],entry.graph.elements[i]),"join rewrote paid prefix");
        Replay(initial,entry.graph,total,tools,parent.elements.size());
        const vector<Element> paid(entry.graph.elements.begin()+initial.elements.size(),entry.graph.elements.end());
        Require(SameOperations(paid,entry.newElements),"collector omitted/changed paid prefix");
        Require(entry.circles == static_cast<size_t>(count_if(paid.begin(),paid.end(),
            [](const Element& e) { return e.type == Type::Circle; })),"collector circle count");
        CertificateGate(initial,entry.graph);
    }
}
template<class F> void Rejects(F&& f, const string& why) {
    bool rejected = false;
    try { f(); } catch (const runtime_error&) { rejected = true; }
    Require(rejected,"negative control was accepted: " + why);
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
    for (int& n : goals) { in >> n; Require(n >= 0,"negative fixture goal count"); }
    for (int kind = 0; kind < 2; ++kind) for (int i = 0; i < goals[kind]; ++i) {
        double a = 0, b = 0, c = 0; in >> a >> b >> c;
        input.graph.goalElements.push_back(Element::FromCoefficients(a,b,kind ? c*c : c,
                                                                     kind ? Type::Circle : Type::Line));
    }
    for (int i = 0; i < goals[2]; ++i) { Point p; in >> p.x >> p.y; input.graph.goalPoints.push_back(p); }
    Require(bool(in),"incomplete fixture");
    return input;
}
fs::path FixturePath(const char* executable, const char* name) {
    const fs::path suffix = fs::path("benchmarks") / "cases" / name;
    for (fs::path base : {fs::path(__FILE__).parent_path().parent_path(),
                         fs::current_path(),fs::absolute(executable).parent_path()}) {
        for (int depth = 0; depth < 4 && !base.empty(); ++depth) {
            if (fs::is_regular_file(base/suffix)) return fs::absolute(base/suffix);
            base = base.parent_path();
        }
    }
    throw runtime_error("cannot locate fixture: " + suffix.string());
}
Input Load(const fs::path& path) {
    ifstream in(path);
    Require(bool(in),"cannot open fixture: " + path.string());
    cout << "Fixture: " << path.string() << '\n';
    return Parse(in);
}

struct ChainRun {
    bool quota = false, stopped = false, found = false, timedOut = false;
    SearchStats stats;
    chain_join_detail::Counts counts;
    vector<SolutionCollector::Entry> entries;
    double seconds = 0;
};
ChainRun Chain(const Graph& initial, const Graph& parent, int remaining, int tools,
               size_t quota = 1, double seconds = .6, bool presetStop = false) {
    const string before = Fingerprint(parent);
    const auto hash1 = parent.StateHash1(), hash2 = parent.StateHash2();
    SolutionCollector collector(quota); ParallelControl control;
    const auto start = Clock::now();
    control.deadline = start + chrono::seconds(2);
    control.stop.store(presetStop);
    const auto deadline = start + chrono::duration_cast<Clock::duration>(chrono::duration<double>(seconds));
    ChainRun run;
    run.quota = chain_join_detail::Find(parent,remaining,tools,collector,control,run.stats,run.counts,deadline);
    run.seconds = chrono::duration<double>(Clock::now()-start).count();
    run.stopped = control.stop.load(); run.found = control.found.load(); run.timedOut = control.timedOut.load();
    Require(Fingerprint(parent) == before && parent.StateHash1() == hash1 && parent.StateHash2() == hash2,
            "chain Find mutated parent");
    Require(collector.Count() <= quota && run.quota == (collector.Count() == quota) && run.quota == run.found,
            "chain return/found is not collector quota semantics");
    Require(run.stopped == (presetStop || run.quota) && !run.timedOut,"local chain deadline poisoned global control");
    Require(run.stats.maxElements <= parent.elements.size()+static_cast<size_t>(max(0,remaining)),
            "chain applied an over-budget tail even if it was not submitted");
    VerifyEntries(initial,parent,remaining,tools,collector);
    run.entries = collector.Entries();
    return run;
}
void NoWork(const ChainRun& run) {
    Require(!run.quota && run.entries.empty() && !run.found,"cancelled/ineligible chain returned a solution");
    Require(run.stats.nodes == 0 && run.stats.applied == 0 && run.stats.rawCandidates == 0 &&
            run.stats.uniqueCandidates == 0 && run.counts.generated == 0 && run.counts.proposals == 0 &&
            run.counts.replays == 0 && run.counts.successes == 0 && run.counts.reachablePoints == 0,
            "cancelled/ineligible chain performed work");
}
Graph T1Parent(const Input& input) {
    Require(EPS == 1e-10 && input.mode == 3 && input.limit == 5 && input.graph.gridM == 6 &&
            input.graph.gridN == 5 && !input.graph.GoalsMet(),"wrong real T1 fixture/settings");
    Graph parent = input.graph;
    parent.SetStateHashingEnabled(true);
    const Element prefix = Definition(StoredPoint(parent,{6,2}),StoredPoint(parent,{3,3}),Type::Line);
    Require(KnownPair(parent,prefix) && parent.Apply(prefix,1),"legal T1 first operation failed");
    Require(parent.initialElementCount == input.graph.initialElementCount &&
            parent.elements.size() == input.graph.elements.size()+1,"T1 first E was free/resealed");
    Replay(input.graph,parent,1,1,0,false);
    return parent;
}
void RealT1(const Input& input) {
    const Graph parent = T1Parent(input);
    const auto run = Chain(input.graph,parent,4,1);
    Require(run.quota && run.entries.size() == 1 && run.entries[0].newElements.size() == 5,
            "T1 chain did not return a fully paid 5E solution");
    Require(run.counts.generated > 0 && run.counts.proposals > 0 && run.counts.replays > 0 &&
            run.counts.successes > 0 && run.counts.reachablePoints > 0,"T1 chain work counters missing");
    Require(run.entries[0].graph.elements.back().type == Type::Line &&
            SameElement(run.entries[0].graph.elements.back(),input.graph.goalElements[0]),"T1 final paid goal line missing");
    cout << "  T1 correctness call=" << run.seconds << "s, proposals=" << run.counts.proposals
         << ", replays=" << run.counts.replays << ", total E=5\n";
    const Graph& solved = run.entries[0].graph;
    Rejects([&] { Replay(input.graph,solved,4); },"paid prefix forgotten under E4");
    Graph forged = solved; forged.initialElementCount = parent.elements.size();
    Rejects([&] { Replay(input.graph,forged,5); },"resealed prefix");
    forged = solved; forged.pointBirth.back() = 0;
    Rejects([&] { Replay(input.graph,forged,5); },"forged birth");
    forged = solved; forged.points.push_back({-.25*EPS,1}); forged.pointBirth.push_back(5);
    Rejects([&] { Replay(input.graph,forged,5); },"EPS-expanded exterior strip");
}
void ChainGuards(const Input& input) {
    const Graph parent = T1Parent(input);
    NoWork(Chain(input.graph,parent,4,0));
    NoWork(Chain(input.graph,parent,2,1));
    NoWork(Chain(input.graph,parent,4,1,1,.6,true));
    NoWork(Chain(input.graph,parent,4,1,1,-.001));
    const auto limited = Chain(input.graph,parent,3,1,1,.15);
    Require(limited.entries.empty() && limited.counts.replays > 0,
            "three-E tail fabricated a fourth paid goal line or did not exercise replay");
    const auto plural = Chain(input.graph,parent,4,1,2,.45);
    Require(!plural.entries.empty() && plural.counts.successes > 0,"quota=2 did not exercise successful chain submission");
    cout << "  chain quota=2 retained=" << plural.entries.size() << ", reached=" << plural.quota
         << "; remaining=3 bounded replays=" << limited.counts.replays << '\n';
}

Graph TwoPoints(Point a = {0,0}, Point b = {2,0}) {
    Graph graph;
    Require(graph.AddPoint(a,0) && graph.AddPoint(b,0),"two-point fixture collapsed");
    Seal(graph);
    return graph;
}
array<Element,3> ScaffoldSequence(const Graph& graph) {
    return {Definition(graph.points[0],graph.points[1],Type::Line),
            Definition(graph.points[0],graph.points[1],Type::Circle),
            Definition(graph.points[1],graph.points[0],Type::Circle)};
}
void FoundationAccounting() {
    // Every subset of already-existing operations: no missing operation is free,
    // and no existing operation consumes E a second time.
    for (unsigned mask = 0; mask < 8; ++mask) {
        Graph initial = TwoPoints();
        const auto sequence = ScaffoldSequence(initial);
        for (unsigned i = 0; i < sequence.size(); ++i)
            if (mask & (1u << i)) Require(initial.AddInitial(sequence[i]),"initial scaffold subset");
        Seal(initial);
        Graph graph = initial;
        SearchStats stats; vector<Element> paid;
        const int missing = 3-static_cast<int>(popcount(mask));
        Require(foundation_detail::ApplyPairScaffold(graph,0,1,missing,stats,paid),"affordable scaffold failed");
        vector<Element> expected;
        for (unsigned i = 0; i < sequence.size(); ++i)
            if (!(mask & (1u << i))) expected.push_back(sequence[i]);
        Require(SameOperations(paid,expected) && graph.elements.size() == initial.elements.size()+paid.size(),
                "scaffold order/cost differs from line(A,B), circle(A,B), circle(B,A)");
        Require(stats.applied == paid.size() && stats.nodes == paid.size(),"scaffold 1E work accounting");
        Replay(initial,graph,missing,2,0,false);
        const string before = Fingerprint(graph);
        vector<Element> repeated;
        Require(foundation_detail::ApplyPairScaffold(graph,0,1,0,stats,repeated) && repeated.empty() &&
                Fingerprint(graph) == before && stats.applied == paid.size(),"existing scaffold charged twice");
    }
    const Graph initial = TwoPoints();
    Graph parent = initial;
    Require(parent.Apply(ScaffoldSequence(initial)[0],1),"paid scaffold prefix");
    SearchStats stats; vector<Element> paid;
    Require(foundation_detail::ApplyPairScaffold(parent,0,1,2,stats,paid) && paid.size() == 2,
            "existing PAID line was charged twice");
    Require(parent.initialElementCount == initial.initialElementCount,"scaffold resealed prefix");
    Replay(initial,parent,3,2,0,false);
}
void FoundationBudgetRollback() {
    const Graph initial = TwoPoints();
    cout << "  scaffold failure is atomic (budget:paid)";
    for (int budget : {-1,0,1,2}) {
        Graph graph = initial; graph.SetStateHashingEnabled(true);
        const string before = Fingerprint(graph);
        const auto hash1 = graph.StateHash1(), hash2 = graph.StateHash2();
        const Mark mark = graph.GetMark();
        SearchStats stats; vector<Element> paid;
        Require(!foundation_detail::ApplyPairScaffold(graph,0,1,budget,stats,paid),"insufficient scaffold budget accepted");
        Require(paid.empty() && stats.nodes == 0 && stats.applied == 0 && stats.maxPoints == 0 &&
                stats.maxElements == 0 && Fingerprint(graph) == before &&
                graph.StateHash1() == hash1 && graph.StateHash2() == hash2,
                "insufficient scaffold budget must have zero partial mutation");
        // r3 now prechecks the entire schedule. False is atomic, although the
        // caller's normal rollback must remain safe and preserve its parent.
        Replay(initial,graph,max(0,budget),2,0,false);
        cout << ' ' << budget << ':' << paid.size();
        graph.Rollback(mark);
        Require(Fingerprint(graph) == before && graph.StateHash1() == hash1 && graph.StateHash2() == hash2,
                "caller rollback did not restore failed scaffold parent");
    }
    cout << '\n';
}
void FoundationInputGuards() {
    for (int defect = 0; defect < 5; ++defect) {
        Graph graph = TwoPoints();
        uint32_t a = 0, b = 1;
        if (defect == 0) b = static_cast<uint32_t>(graph.points.size());
        if (defect == 1) b = a;
        if (defect == 2) { graph.points.push_back({.5*EPS,0}); graph.pointBirth.push_back(0); b = 2; }
        if (defect == 3) ++graph.initialElementCount;
        if (defect == 4) graph.pointBirth.pop_back();
        const string before = Fingerprint(graph);
        SearchStats stats; vector<Element> paid;
        bool result = false;
        try { result = foundation_detail::ApplyPairScaffold(graph,a,b,3,stats,paid); }
        catch (const invalid_argument&) {}
        Require(!result && paid.empty() && stats.nodes == 0 && stats.applied == 0 && Fingerprint(graph) == before,
                "invalid scaffold input mutated/accepted, defect=" + to_string(defect));
    }
    Graph graph = TwoPoints();
    vector<Element> paid{ScaffoldSequence(graph)[0]};
    const string before = Fingerprint(graph);
    const vector<Element> originalPaid = paid;
    SearchStats stats;
    Require(!foundation_detail::ApplyPairScaffold(graph,0,1,3,stats,paid) &&
            SameOperations(paid,originalPaid) && Fingerprint(graph) == before && stats.nodes == 0 && stats.applied == 0,
            "scaffold must reject a nonempty paid output without changing it");
}
void FoundationNumericGuards() {
    for (const Point b : {Point{2*EPS,0},Point{1e200,0}}) {
        const Graph initial = TwoPoints({0,0},b);
        Graph graph = initial;
        SearchStats stats; vector<Element> paid;
        bool result = false;
        try { result = foundation_detail::ApplyPairScaffold(graph,0,1,3,stats,paid); }
        catch (const invalid_argument&) {}
        Require(!result,"zero/overflow scaffold was accepted as complete");
        Require(paid.empty() && stats.applied == 0 && stats.nodes == 0 && stats.maxPoints == 0 &&
                stats.maxElements == 0 && Fingerprint(graph) == Fingerprint(initial),
                "numeric scaffold rejection must have zero partial mutation");
        Replay(initial,graph,3,2,0,false); // In particular no zero-radius or inf element.
        for (const Element& e : paid)
            Require(isfinite(e.a) && isfinite(e.b) && isfinite(e.c) &&
                    (e.type != Type::Circle || e.c > 0),"invalid numeric operation escaped in paid");
    }
    for (const double invalid : {numeric_limits<double>::infinity(),numeric_limits<double>::quiet_NaN()}) {
        Graph graph = TwoPoints();
        graph.points[1].x = invalid;
        const string before = Fingerprint(graph);
        SearchStats stats; vector<Element> paid;
        Require(!foundation_detail::ApplyPairScaffold(graph,0,1,3,stats,paid) && paid.empty() &&
                stats.nodes == 0 && stats.applied == 0 && Fingerprint(graph) == before,
                "nonfinite scaffold point was accepted or mutated state");
    }
    // Deliberately synthetic counts isolate uint16_t boundary guards without
    // constructing 65535 actual search steps or pretending this is a certificate.
    Graph graph = TwoPoints();
    const Element line = ScaffoldSequence(graph)[0];
    graph.elements.assign(numeric_limits<uint16_t>::max(),line);
    const size_t before = graph.elements.size();
    SearchStats stats; vector<Element> paid;
    Require(!foundation_detail::ApplyPairScaffold(graph,0,1,3,stats,paid) && paid.empty() &&
            graph.elements.size() == before && stats.applied == 0 && stats.nodes == 0,
            "scaffold wrapped the birth depth past uint16_t");
}

#if BS_R3_HAS_POINT_JOIN
struct PointRun {
    bool quota = false, stopped = false, found = false, timedOut = false;
    SearchStats stats;
    point_join_detail::Counts counts;
    vector<SolutionCollector::Entry> entries;
    double seconds = 0;
};
PointRun PointFind(const Graph& initial, const Graph& parent, int remaining, int tools,
                   bool explicitDepth = true, size_t quota = 1, double seconds = .4, bool presetStop = false) {
    const string before = Fingerprint(parent);
    const auto hash1 = parent.StateHash1(), hash2 = parent.StateHash2();
    SolutionCollector collector(quota); ParallelControl control;
    const auto start = Clock::now();
    control.deadline = start + chrono::seconds(2);
    control.stop.store(presetStop);
    const auto deadline = start + chrono::duration_cast<Clock::duration>(chrono::duration<double>(seconds));
    PointRun run;
    const int depth = static_cast<int>(parent.elements.size()-parent.initialElementCount);
    if (explicitDepth)
        run.quota = point_join_detail::Find(parent,remaining,depth,tools,collector,control,run.stats,run.counts,deadline);
    else
        run.quota = point_join_detail::Find(parent,remaining,tools,collector,control,run.stats,run.counts,deadline);
    run.seconds = chrono::duration<double>(Clock::now()-start).count();
    run.stopped = control.stop.load(); run.found = control.found.load(); run.timedOut = control.timedOut.load();
    Require(Fingerprint(parent) == before && parent.StateHash1() == hash1 && parent.StateHash2() == hash2,
            "point Find mutated parent");
    Require(collector.Count() <= quota && run.quota == (collector.Count() == quota) && run.quota == run.found,
            "point return/found is not collector quota semantics");
    Require(run.stopped == (presetStop || run.quota) && !run.timedOut,"local point deadline poisoned global control");
    Require(run.stats.maxElements <= parent.elements.size()+static_cast<size_t>(max(0,remaining)),
            "point join applied an over-budget suffix even if it was not submitted");
    VerifyEntries(initial,parent,remaining,tools,collector);
    run.entries = collector.Entries();
    return run;
}
void NoWork(const PointRun& run) {
    Require(!run.quota && run.entries.empty() && !run.found,"cancelled/ineligible point join returned a solution");
    Require(run.stats.nodes == 0 && run.stats.applied == 0 && run.stats.rawCandidates == 0 &&
            run.stats.uniqueCandidates == 0 && run.counts.generated == 0 && run.counts.proposals == 0 &&
            run.counts.replays == 0 && run.counts.successes == 0 && run.counts.points == 0,
            "cancelled/ineligible point join performed work");
}
Graph T5Parent(const Input& input) {
    Require(EPS == 1e-10 && input.mode == 2 && input.limit == 6 && input.graph.points.size() == 2 &&
            input.graph.elements.size() == 2 && !input.graph.GoalsMet(),"wrong real T5 fixture/settings");
    Graph parent = input.graph; parent.SetStateHashingEnabled(true);
    SearchStats stats; vector<Element> paid;
    Require(foundation_detail::ApplyPairScaffold(parent,0,1,3,stats,paid) && paid.size() == 3,
            "T5 scaffold was not exactly three ordinary paid steps");
    Require(stats.nodes == 3 && stats.applied == 3 && parent.initialElementCount == input.graph.initialElementCount,
            "T5 scaffold accounting");
    Replay(input.graph,parent,3,2,0,false);
    return parent;
}
void RealT5(const Input& input) {
    const Graph parent = T5Parent(input);
    const auto run = PointFind(input.graph,parent,3,2);
    Require(run.quota && run.entries.size() == 1 && run.entries[0].newElements.size() == 6,
            "T5 scaffold+point join did not return a legal total 6E construction");
    Require(run.counts.generated > 0 && run.counts.proposals > 0 && run.counts.replays > 0 &&
            run.counts.successes > 0 && run.counts.points > parent.points.size(),"T5 point-join counters missing");
    const auto inferred = PointFind(input.graph,parent,3,2,false);
    Require(inferred.quota && Fingerprint(inferred.entries[0].graph) == Fingerprint(run.entries[0].graph),
            "explicit/inferred paid-depth overload mismatch");
    cout << "  T5 correctness call=" << run.seconds << "s, proposals=" << run.counts.proposals
         << ", replays=" << run.counts.replays << ", total E=6\n";
    Rejects([&] { Replay(input.graph,run.entries[0].graph,5); },"T5 scaffold charged as fewer than 3E");
}
void PointGuards(const Input& input) {
    const Graph parent = T5Parent(input);
    NoWork(PointFind(input.graph,parent,3,0));
    NoWork(PointFind(input.graph,parent,0,2));
    NoWork(PointFind(input.graph,parent,3,2,true,1,.4,true));
    NoWork(PointFind(input.graph,parent,3,2,false,1,-.001));
    const auto limited = PointFind(input.graph,parent,2,2);
    Require(limited.entries.empty() && limited.counts.generated > 0,
            "remaining=2 returned T5's three-operation suffix or did not exercise the budget gate");
}
Graph OneLineGoal() {
    Graph graph = TwoPoints({0,1},{2,1});
    graph.AddInitial(Element::FromCoefficients(1,0,1,Type::Line));
    graph.goalPoints = {{1,1}};
    Seal(graph);
    return graph;
}
void PointNearDirectionAndAllGoals() {
    const Graph initial = OneLineGoal();
    for (int budget : {1,2,3}) {
        const auto run = PointFind(initial,initial,budget,1);
        Require(run.quota && run.entries[0].newElements.size() == 1 && run.counts.replays > 0,
                "one-E point witness/final-line completion was overcharged");
    }
    Graph mismatch = initial;
    mismatch.goalPoints = {{1,1+100*EPS}};
    const auto declined = PointFind(mismatch,mismatch,1,1);
    Require(declined.counts.proposals > 0 && declined.counts.replays > 0 && declined.stats.applied > 0 &&
            declined.counts.successes == 0 && declined.entries.empty(),
            "near-direction proposal was not replayed then rejected by GoalsMet");
    // Both missing points and the element must be met; the first known point
    // deliberately occurs first so finding a satisfied target cannot end Find.
    Graph multiple = initial;
    multiple.AddInitial(Element::FromCoefficients(1,0,1.5,Type::Line));
    Seal(multiple);
    multiple.goalPoints = {{0,1},{1,1},{1.5,1}};
    multiple.goalElements = {Definition(multiple.points[0],multiple.points[1],Type::Line)};
    const auto complete = PointFind(multiple,multiple,1,1);
    Require(complete.quota && complete.entries[0].newElements.size() == 1,"all-goal point completion failed");
    for (bool wrongElement : {false,true}) {
        Graph incomplete = multiple;
        if (wrongElement) incomplete.goalElements.push_back(Element::FromCoefficients(0,1,2,Type::Line));
        else incomplete.goalPoints.back().y += 100*EPS;
        const auto run = PointFind(incomplete,incomplete,1,1);
        Require(run.counts.replays > 0 && run.stats.applied > 0 && run.counts.successes == 0 && run.entries.empty(),
                "point join submitted when only some goals were met");
    }
    // EPS-equivalent coefficients are not a raw known-pair certificate witness.
    Graph raw = TwoPoints({0,1+.75*EPS},{2,1+.75*EPS});
    const Element target = Element::FromCoefficients(0,1,1,Type::Line);
    Require(SameElement(Definition(raw.points[0],raw.points[1],Type::Line),target) && !KnownPair(raw,target),
            "bitwise verifier conflated EPS-near coordinates with an actual operation");
    Graph forged = raw; Require(forged.Apply(target,1),"raw witness negative control setup");
    Rejects([&] { Replay(raw,forged,1,1); },"target coefficients substituted for a raw known pair");
    Rejects([&] { CertificateGate(raw,forged); },"exporter accepted an unwitnessed target line");
}
void PointQuotaAndClosedGrid() {
    const Graph initial = OneLineGoal();
    for (int repeat = 0; repeat < 2; ++repeat) {
        const auto partial = PointFind(initial,initial,1,1,true,2);
        Require(partial.entries.size() == 1 && !partial.quota && !partial.found && !partial.stopped,
                "one point-join success incorrectly filled quota=2");
    }
    SolutionCollector collector(2); ParallelControl control;
    control.deadline = Clock::now()+chrono::seconds(1);
    SearchStats stats; point_join_detail::Counts counts;
    for (int repeat = 0; repeat < 2; ++repeat) {
        Require(!point_join_detail::Find(initial,1,0,1,collector,control,stats,counts,control.deadline) &&
                collector.Count() == 1 && !control.stop.load() && !control.found.load(),
                "duplicate replay falsely completed quota=2");
    }
    Require(collector.SuccessfulVisits() == 2 && collector.DuplicateVisits() == 1,"point duplicate quota accounting");
    VerifyEntries(initial,initial,1,1,collector);
    Graph two = initial;
    two.AddPoint({0,2},0); two.AddPoint({2,0},0);
    const auto plural = PointFind(two,two,1,1,true,2);
    Require(plural.quota && plural.entries.size() == 2,"two distinct known-pair lines did not fill quota=2");

    Graph grid;
    grid.gridMode = true; grid.gridM = grid.gridN = 2;
    grid.AddPoint({0,.5},0); grid.AddPoint({2,.5},0);
    grid.AddAutomaticGridLines();
    grid.AddInitial(Definition({0,0},{2,2},Type::Line));
    grid.goalPoints = {{.5,.5}};
    Seal(grid);
    const auto closed = PointFind(grid,grid,1,1);
    Require(closed.quota && closed.entries.size() == 1,"closed-grid point completion failed");
    Require(!grid.AddPoint({-.25*EPS,.75},0) && !grid.AddPoint({2+.25*EPS,.75},0),
            "closed grid admitted EPS-expanded exterior points");
    for (Point outside : {Point{-.25*EPS,.75},Point{2+.25*EPS,.75},
                          Point{.75,-.25*EPS},Point{.75,2+.25*EPS}}) {
        Graph forbidden = grid; forbidden.goalPoints = {outside};
        NoWork(PointFind(forbidden,forbidden,3,1));
        NoWork(Chain(forbidden,forbidden,4,1));
    }
}
void PointPrefixContract(const Input& input) {
    const Graph parent = T5Parent(input);
    for (int defect = 0; defect < 5; ++defect) {
        Graph invalid = parent;
        int depth = 3;
        if (defect == 0) invalid.initialElementCount = invalid.elements.size()+1;
        if (defect == 1) invalid.pointBirth.pop_back();
        if (defect == 2) depth = 2;
        if (defect == 3) invalid.pointBirth.back() = 4;
        if (defect == 4) { invalid.pointBirth[0] = 1; invalid.pointBirth[1] = 0; }
        const string before = Fingerprint(invalid);
        SolutionCollector collector(1); ParallelControl control;
        control.deadline = Clock::now()+chrono::seconds(1);
        SearchStats stats; point_join_detail::Counts counts;
        bool rejected = false;
        try { point_join_detail::Find(invalid,3,depth,2,collector,control,stats,counts,control.deadline); }
        catch (const invalid_argument&) { rejected = true; }
        Require(rejected && collector.Count() == 0 && !control.stop.load() && !control.found.load() &&
                stats.nodes == 0 && stats.applied == 0 && stats.rawCandidates == 0 && counts.generated == 0 &&
                counts.proposals == 0 && counts.replays == 0 && Fingerprint(invalid) == before,
                "point prefix/depth/birth contract accepted malformed state, defect=" + to_string(defect));
    }
}
void JoinCancellation() {
    Graph graph;
    graph.gridMode = true; graph.gridM = graph.gridN = 8; graph.AddAutomaticGridLines();
    graph.goalPoints = {{sqrt(2.0),sqrt(3.0)}};
    Seal(graph);
    const string before = Fingerprint(graph);
    for (bool chain : {true,false}) {
        SolutionCollector collector(1); ParallelControl control;
        control.deadline = Clock::now()+chrono::seconds(1);
        const auto deadline = Clock::now()+chrono::milliseconds(250);
        SearchStats stats; chain_join_detail::Counts chainCounts; point_join_detail::Counts pointCounts;
        jthread stopper([&] {
            this_thread::sleep_for(chrono::milliseconds(2));
            control.stop.store(true,memory_order_release);
        });
        const bool reached = chain ? chain_join_detail::Find(graph,4,1,collector,control,stats,chainCounts,deadline) :
            point_join_detail::Find(graph,3,0,2,collector,control,stats,pointCounts,deadline);
        stopper.join();
        Require(!reached && collector.Count() == 0 && control.stop.load() && !control.found.load() &&
                !control.timedOut.load() && Fingerprint(graph) == before,
                "asynchronous join stop fabricated success/timeout or mutated parent");
    }
}
#else
void RealT5(const Input&) { throw runtime_error("point_join.hpp is required for final r3 acceptance (no skip)"); }
void PointGuards(const Input&) { throw runtime_error("point_join.hpp is required for final r3 acceptance (no skip)"); }
void PointNearDirectionAndAllGoals() { throw runtime_error("point_join.hpp is required (no skip)"); }
void PointQuotaAndClosedGrid() { throw runtime_error("point_join.hpp is required (no skip)"); }
void PointPrefixContract(const Input&) { throw runtime_error("point_join.hpp is required (no skip)"); }
void JoinCancellation() { throw runtime_error("point_join.hpp is required (no skip)"); }
#endif

} // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 1 || argc == 3,"usage: r3_tests [T1_INPUT T5_INPUT]");
        EPS = 1e-10;
        const Input t1 = Load(argc == 3 ? fs::absolute(argv[1]) : FixturePath(argv[0],"t1_conditional_E5.in"));
        const Input t5 = Load(argc == 3 ? fs::absolute(argv[2]) : FixturePath(argv[0],"t5_aux_J_E6.in"));
        const pair<const char*,function<void()>> tests[] = {
            {"T1 chain / paid prefix / bitwise ordered replay / certificate",[&] { RealT1(t1); }},
            {"chain E/tool/stop/deadline/quota guards",[&] { ChainGuards(t1); }},
            {"foundation ordinary 1E / all existing subsets / paid prefix",FoundationAccounting},
            {"foundation insufficient E is atomic / caller rollback",FoundationBudgetRollback},
            {"foundation invalid/duplicate input guards",FoundationInputGuards},
            {"foundation zero/overflow/birth-depth guards",FoundationNumericGuards},
            {"T5 scaffold + point join / bitwise replay / certificate / overloads",[&] { RealT5(t5); }},
            {"point join E/tool/stop/deadline guards",[&] { PointGuards(t5); }},
            {"point join near-direction rejection / all goals / raw witness",PointNearDirectionAndAllGoals},
            {"point join partial/duplicate/full quota / closed grid",PointQuotaAndClosedGrid},
            {"point join paid prefix/depth/birth contract",[&] { PointPrefixContract(t5); }},
            {"chain and point join asynchronous cancellation",JoinCancellation}
        };
        const auto start = Clock::now();
        size_t passed = 0;
        for (const auto& [name,test] : tests) {
            const auto begin = Clock::now();
            try {
                test(); ++passed;
                cout << "PASS " << name << " (" << chrono::duration<double>(Clock::now()-begin).count() << "s)\n";
            } catch (const exception& e) { cerr << "FAIL " << name << ": " << e.what() << '\n'; }
        }
        cout << "r3 tests: " << passed << '/' << size(tests) << " groups passed; wall="
             << chrono::duration<double>(Clock::now()-start).count()
             << "s; short correctness tests only, not hard-case performance evidence\n";
        return passed == size(tests) ? 0 : 1;
    } catch (const exception& e) {
        cerr << "FAIL r3 tests: " << e.what() << '\n';
        return 1;
    }
}
