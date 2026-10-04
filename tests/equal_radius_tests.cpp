// Standalone, short correctness regression; no performance benchmark or T6.
#include "../src/equal_radius_probe.hpp"
#include "../src/solver.hpp"
#include "../src/certificate.hpp"
#include <filesystem>

using namespace bs;
namespace {
using Clock = chrono::steady_clock;
namespace fs = std::filesystem;

void Require(bool ok, const char* message) {
    if (!ok) throw runtime_error(message);
}
bool Exact(Point a, Point b) {
    return bit_cast<uint64_t>(a.x) == bit_cast<uint64_t>(b.x) &&
           bit_cast<uint64_t>(a.y) == bit_cast<uint64_t>(b.y);
}
string Fingerprint(const Graph& graph) {
    ostringstream out;
    out << graph.initialElementCount << ',' << graph.gridMode << ',' << graph.gridFast << ','
        << graph.gridLinesReady << ',' << graph.gridM << ',' << graph.gridN << ','
        << graph.StateHash1() << ',' << graph.StateHash2() << ';';
    auto point = [&](Point p) { out << bit_cast<uint64_t>(p.x) << ',' << bit_cast<uint64_t>(p.y) << ';'; };
    auto element = [&](Element e) {
        out << int(e.type) << ',' << e.bound << ',' << bit_cast<uint64_t>(e.a) << ','
            << bit_cast<uint64_t>(e.b) << ',' << bit_cast<uint64_t>(e.c) << ';';
    };
    for (Point p : graph.points) point(p);
    out << '/'; for (auto birth : graph.pointBirth) out << birth << ',';
    out << '/'; for (Element e : graph.elements) element(e);
    out << '/'; for (Bound b : graph.bounds) { point(b.p1); point(b.p2); }
    out << '/'; for (Point p : graph.goalPoints) point(p);
    out << '/'; for (Element e : graph.goalElements) element(e);
    return out.str();
}

struct Input { Graph graph; int limit = 0, tools = 2; };
Input Load(const fs::path& path) {
    ifstream in(path);
    Require(bool(in), "cannot open equal-radius compact fixture");
    Input input;
    in >> input.limit >> input.tools;
    Require(input.limit >= 0 && input.tools >= 0 && input.tools <= 2, "invalid fixture header");
    int counts[5]{};
    for (int& n : counts) { in >> n; Require(n >= 0, "invalid given count"); }
    for (int i = 0; i < counts[0]; ++i) { Point p; in >> p.x >> p.y; input.graph.AddPoint(p, 0); }
    for (int kind = 1; kind < 5; ++kind) for (int i = 0; i < counts[kind]; ++i) {
        double a = 0, b = 0, c = 0, d = 0;
        if (kind == 2 || kind == 3) {
            in >> a >> b >> c >> d;
            input.graph.AddInitialBounded({a, b}, {c, d}, kind == 2 ? Type::Ray : Type::Segment);
        } else {
            in >> a >> b >> c;
            input.graph.AddInitial(Element::FromCoefficients(a, b, kind == 4 ? c*c : c,
                                                            kind == 4 ? Type::Circle : Type::Line));
        }
    }
    input.graph.initialElementCount = input.graph.elements.size();
    int goals[3]{};
    for (int& n : goals) { in >> n; Require(n >= 0, "invalid goal count"); }
    for (int kind = 0; kind < 2; ++kind) for (int i = 0; i < goals[kind]; ++i) {
        double a = 0, b = 0, c = 0; in >> a >> b >> c;
        input.graph.goalElements.push_back(Element::FromCoefficients(a, b, kind ? c*c : c,
                                                                    kind ? Type::Circle : Type::Line));
    }
    for (int i = 0; i < goals[2]; ++i) { Point p; in >> p.x >> p.y; input.graph.goalPoints.push_back(p); }
    Require(bool(in), "incomplete fixture");
    string extra; Require(!(in >> extra), "fixture has trailing tokens");
    input.graph.SetStateHashingEnabled(true);
    return input;
}
fs::path Fixture(const char* executable) {
    const fs::path relative = "benchmarks/euclidea8/cases/eu15_4_three_equal.in";
    const fs::path starts[] = {fs::current_path(), fs::absolute(executable).parent_path(),
                              fs::absolute(fs::path(__FILE__)).parent_path()};
    for (fs::path root : starts) for (int i = 0; i < 6; ++i) {
        if (fs::exists(root / relative)) return root / relative;
        if (root == root.parent_path()) break;
        root = root.parent_path();
    }
    throw runtime_error("cannot locate fixture; pass its absolute path");
}

// Every paid element must have an exact, ordinary known-pair witness BEFORE
// Apply; neither goals nor any predicted point are added to this replay.
void VerifyPrefix(const Graph& original, const Graph& result, int budget) {
    Require(result.initialElementCount == original.initialElementCount, "prefix was resealed");
    Require(result.elements.size() >= original.elements.size() &&
            result.elements.size() - original.elements.size() <= static_cast<size_t>(budget), "E budget");
    for (size_t i = 0; i < original.elements.size(); ++i)
        Require(SameElementBits(original.elements[i], result.elements[i]), "changed given element");
    Graph replay = original;
    for (size_t k = original.elements.size(); k < result.elements.size(); ++k) {
        bool witness = false;
        for (uint32_t i = 0; i < replay.points.size() && !witness; ++i)
            for (uint32_t j = i + 1; j < replay.points.size() && !witness; ++j)
                for (uint8_t tool = 0; tool < 3; ++tool)
                    if (!SamePoint(replay.points[i], replay.points[j]) &&
                        SameElementBits(replay.MakeCandidate({i, j, tool}), result.elements[k])) {
                        witness = true; break;
                    }
        Require(witness, "operation lacked exact known-pair witness");
        Require(replay.Apply(result.elements[k], static_cast<uint16_t>(k-original.initialElementCount+1)),
                "repeated paid operation");
    }
    Require(result.pointBirth == replay.pointBirth, "changed or renumbered point births");
    Require(result.points.size() == replay.points.size(), "replay point count differs");
    for (size_t i = 0; i < result.points.size(); ++i)
        Require(Exact(result.points[i], replay.points[i]), "replay point bits differ");
}
struct Run {
    bool quota = false;
    Graph solved;
    equal_radius_detail::Counts counts;
};
Run Solve(const Graph& original, const Graph& parent, int remaining, bool acknowledgeQuota = true) {
    Run run;
    const string before = Fingerprint(parent);
    const int budget = static_cast<int>(parent.elements.size()-parent.initialElementCount) + remaining;
    ParallelControl global;
    global.deadline = Clock::now() + chrono::seconds(20);
    SearchStats stats;
    run.quota = equal_radius_detail::Probe(parent, remaining, 2, global, stats, run.counts,
        Clock::now() + chrono::milliseconds(750), [&](const Graph& state, int left) {
            VerifyPrefix(original, state, budget);
            Require(state.elements.size()-state.initialElementCount+left == static_cast<size_t>(budget),
                    "callback remaining is not additional E");
            Require(left == remaining-5 || state.GoalsMet(), "unexpected callback depth");
            Graph replay = original;
            PrefixTask task;
            task.prefix.assign(state.elements.begin()+original.elements.size(), state.elements.end());
            // Use a private deadline; do not reseal state or run DFS at depth 0
            // on it. The legacy solver reconstructs the whole paid prefix.
            Solver tail(2, false, true, true, 4096, 0, 0.05);
            tail.SetExternalStop(&global.stop);
            ParallelControl local;
            local.deadline = min(global.deadline, Clock::now()+chrono::milliseconds(50));
            SearchStats tailStats;
            const bool found = tail.SearchPrefixTask(replay, budget, task, tailStats, &local);
            if (!found || !replay.GoalsMet()) return false;
            VerifyPrefix(original, replay, budget);
            ostringstream certificate;
            WriteConstructionCertificate(certificate, original, replay);
            run.solved = std::move(replay);
            if (acknowledgeQuota) return true;
            global.stop.store(true); // A partial result must still return false.
            return false;
        });
    Require(Fingerprint(parent) == before, "probe modified its const parent");
    Require(!global.timedOut.load(), "local deadline poisoned global timeout");
    return run;
}
Graph Transform(const Graph& source, double u, double v, Point shift) {
    auto point = [&](Point p) { return Point{u*p.x-v*p.y+shift.x, v*p.x+u*p.y+shift.y}; };
    auto element = [&](Element e) {
        if (e.type == Type::Circle) {
            Point p = point({e.a,e.b}); e.a=p.x; e.b=p.y; e.c*=u*u+v*v; return e;
        }
        const double a=(e.a*u-e.b*v)/(u*u+v*v), b=(e.a*v+e.b*u)/(u*u+v*v);
        Element result = Element::FromCoefficients(a,b,e.c+a*shift.x+b*shift.y,Type::Line);
        result.type=e.type; result.bound=e.bound; return result;
    };
    Graph result;
    for (Point p : source.points) result.AddPoint(point(p),0);
    for (Bound b : source.bounds) result.bounds.push_back({point(b.p1),point(b.p2)});
    for (Element e : source.elements) result.AddInitial(element(e));
    for (Point p : source.goalPoints) result.goalPoints.push_back(point(p));
    for (Element e : source.goalElements) result.goalElements.push_back(element(e));
    result.initialElementCount=result.elements.size();
    result.SetStateHashingEnabled(true);
    return result;
}
void RealRoute(const Input& input) {
    Run run = Solve(input.graph, input.graph, input.limit);
    Require(run.quota && run.solved.GoalsMet(), "real route not solved");
    Require(run.solved.elements.size()-run.solved.initialElementCount == static_cast<size_t>(input.limit),
            "expected full paid fixture route");
    Require(run.counts.prefixes && run.counts.replays && run.counts.callbacks && run.counts.successes == 1,
            "missing probe evidence");
    Run partial = Solve(input.graph,input.graph,input.limit,false);
    Require(!partial.quota && partial.solved.GoalsMet() && partial.counts.successes == 0,
            "callback false was misreported as quota");
}
void GenericAndPaid(const Input& input) {
    for (const auto& v : vector<array<double,4>>{{1,0,3,-2},{0,1,0,0},{2,0,0,0}}) {
        Graph transformed=Transform(input.graph,v[0],v[1],{v[2],v[3]});
        Require(Solve(transformed,transformed,input.limit).quota, "similarity transform not solved");
    }
    Graph reordered=input.graph;
    swap(reordered.points.front(),reordered.points.back());
    reordered.SetStateHashingEnabled(true);
    Require(Solve(reordered,reordered,input.limit).quota, "root point reorder not solved");
    Graph parent=input.graph;
    Require(parent.Apply(parent.MakeCandidate({1,2,0}),1), "fixture paid parent already exists");
    parent.SetStateHashingEnabled(true);
    Require(Solve(input.graph,parent,input.limit).quota, "paid parent not solved");
    for (bool pointOnly : {false,true}) {
        Graph graph=input.graph;
        if (pointOnly) graph.goalElements.clear(); else graph.goalPoints.clear();
        Require(Solve(graph,graph,input.limit).quota, "goal-only variant not solved");
    }
}
void Guards(const Input& input) {
    for (int kind=0;kind<8;++kind) {
        Graph graph=input.graph;
        ParallelControl control;
        control.deadline=Clock::now()+chrono::seconds(2);
        auto deadline=control.deadline;
        int remaining=input.limit, tools=2;
        if (kind==0) { graph.goalPoints.clear(); graph.goalElements.clear(); }
        if (kind==1) deadline=Clock::now()-chrono::seconds(1);
        if (kind==2) control.deadline=Clock::now()-chrono::seconds(1);
        if (kind==3) control.stop.store(true);
        if (kind==4) tools=0;
        if (kind==5) tools=1;
        if (kind==6) remaining=5;
        if (kind==7) for (int i=0;i<6;++i) graph.AddPoint({double(30+i),17},0);
        const string before=Fingerprint(graph);
        bool called=false;
        SearchStats stats; equal_radius_detail::Counts counts;
        const bool quota=equal_radius_detail::Probe(graph,remaining,tools,control,stats,counts,deadline,
            [&](const Graph&,int) { called=true; return true; });
        Require(!quota && !called && stats.applied==0 && !control.timedOut.load(), "eligibility/cancel guard");
        Require(Fingerprint(graph)==before, "guard modified parent");
    }
    for (int kind=0;kind<3;++kind) {
        Graph graph=input.graph;
        if (kind==0) graph.pointBirth.pop_back();
        if (kind==1) graph.initialElementCount=graph.elements.size()+1;
        if (kind==2) graph.pointBirth[1]=1;
        ParallelControl control; SearchStats stats; equal_radius_detail::Counts counts;
        bool rejected=false;
        try {
            equal_radius_detail::Probe(graph,input.limit,2,control,stats,counts,Clock::now()+chrono::seconds(1),
                [](const Graph&,int) { return true; });
        } catch (const invalid_argument&) { rejected=true; }
        Require(rejected, "malformed parent accepted");
    }
    // Shorten the given carriers without creating free endpoint points. The
    // supporting infinite lines would permit the old route; the segments do not.
    Graph clipped=input.graph;
    Require(clipped.bounds.size()==2, "fixture bounded carriers changed");
    for (Bound& b : clipped.bounds) {
        b.p2.x=b.p1.x+(b.p2.x-b.p1.x)/10;
        b.p2.y=b.p1.y+(b.p2.y-b.p1.y)/10;
    }
    for (Element& e : clipped.elements) if (e.type==Type::Ray) e.type=Type::Segment;
    Run run=Solve(clipped,clipped,input.limit);
    Require(!run.quota, "segment ranges were treated as infinite lines");
}
} // namespace

int main(int argc,char** argv) {
    try {
        Require(argc==1 || argc==2, "usage: equal_radius_tests [compact-input]");
        EPS=1e-11;
        const Input input=Load(argc==2 ? fs::absolute(argv[1]) : Fixture(argv[0]));
        const auto start=Clock::now();
        RealRoute(input);
        cout << "PASS equal-radius paid route / exact known pairs / callback remaining / quota\n";
        GenericAndPaid(input);
        cout << "PASS transforms / root reorder / paid parent / point-only and element-only goals\n";
        Guards(input);
        cout << "PASS no-goals / deadlines / tools / budget / births / strict segments\n";
        cout << "equal-radius correctness wall=" << chrono::duration<double>(Clock::now()-start).count() << "s\n";
        return 0;
    } catch (const exception& e) {
        cerr << "FAIL equal-radius tests: " << e.what() << '\n';
        return 1;
    }
}
