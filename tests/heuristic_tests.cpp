// Standalone target: C++20, -pthread -fno-fast-math -ffp-contract=off.
// -DBS_HEURISTIC_EXPECTED_ONLY checks certificates before heuristic.hpp exists.
#include "../src/solver.hpp"
#ifndef BS_HEURISTIC_EXPECTED_ONLY
#include "../src/heuristic.hpp"
#endif
#include <filesystem>

using namespace bs;
namespace {
void Require(bool ok, const string& why) {
    if (!ok) throw runtime_error(why);
}
bool Near(double a, double b) { return isfinite(a) && isfinite(b) && abs(a-b) < EPS; }
bool PointEqual(Point a, Point b) { return Near(a.x,b.x) && Near(a.y,b.y); }
bool ElementEqual(const Element& a, const Element& b) {
    return a.type == b.type && Near(a.a,b.a) && Near(a.b,b.b) && Near(a.c,b.c);
}
bool ExactElement(const Element& a, const Element& b) {
    return bit_cast<uint64_t>(a.a)==bit_cast<uint64_t>(b.a) &&
           bit_cast<uint64_t>(a.b)==bit_cast<uint64_t>(b.b) &&
           bit_cast<uint64_t>(a.c)==bit_cast<uint64_t>(b.c) &&
           a.type==b.type && a.bound==b.bound;
}
// Deliberately independent of Graph::MakeCandidate, Element::FromPoints,
// reporting's origin finder, goal incidence, and the heuristic's reverse search.
Element Definition(Point p, Point q, Type type) {
    Require(!PointEqual(p,q), "construction needs two distinct known points");
    Element e;
    e.type = type;
    if (type == Type::Circle) {
        e.a=p.x; e.b=p.y;
        e.c=(p.x-q.x)*(p.x-q.x)+(p.y-q.y)*(p.y-q.y);
    } else {
        Require(type == Type::Line, "new rays/segments are not legal tools");
        e.a=q.y-p.y; e.b=p.x-q.x; e.c=p.x*q.y-p.y*q.x;
        if (abs(e.b) >= EPS) { e.a/=e.b; e.c/=e.b; e.b=1; }
        else if (abs(e.a) >= EPS) { e.c/=e.a; e.a=1; e.b=0; }
    }
    if (abs(e.a)<EPS) e.a=0;
    if (abs(e.b)<EPS) e.b=0;
    if (abs(e.c)<EPS) e.c=0;
    return e;
}
bool Origin(const Graph& g, const Element& e) {
    for (size_t i=0;i<g.points.size();++i) {
        for (size_t j=0;j<g.points.size();++j) {
            if (i==j || PointEqual(g.points[i],g.points[j])) continue;
            if (e.type==Type::Line && j<i) continue;
            // A target may be EPS-near a candidate, but the stored operation
            // must be the actual constructed coefficients, not the goal's.
            if (ExactElement(Definition(g.points[i],g.points[j],e.type),e)) return true;
        }
    }
    return false;
}
Graph Replay(const Graph& initial, const Graph& result, int limit, int tools) {
    Require(result.initialElementCount==initial.elements.size(), "changed initial element count");
    Require(result.elements.size()>=initial.elements.size(), "missing initial elements");
    const size_t steps=result.elements.size()-initial.elements.size();
    Require(steps<=static_cast<size_t>(limit), "returned E exceeds maximum steps");
    for (size_t i=0;i<initial.elements.size();++i)
        Require(ExactElement(initial.elements[i],result.elements[i]), "modified initial element");
    Graph replay=initial;
    // Never copy result.points, result.goalPoints, or target coordinates into replay.
    for (size_t i=initial.elements.size();i<result.elements.size();++i) {
        const Element& e=result.elements[i];
        Require(e.type==Type::Line || e.type==Type::Circle, "unsupported new element");
        Require(e.bound==NO_BOUND, "new element has a fabricated bound");
        Require(tools!=0 || e.type==Type::Circle, "straightedge used in compass-only mode");
        Require(tools!=1 || e.type==Type::Line, "compass used in straightedge-only mode");
        Require(Origin(replay,e), "element has no two-point construction origin at step "+to_string(i-initial.elements.size()+1));
        Require(replay.Apply(e,static_cast<uint16_t>(i-initial.elements.size()+1)), "duplicate paid element");
    }
    Require(replay.bounds.size()==result.bounds.size(), "invented/deleted bounded ranges");
    for (size_t i=0;i<replay.bounds.size();++i)
        Require(PointEqual(replay.bounds[i].p1,result.bounds[i].p1) &&
                PointEqual(replay.bounds[i].p2,result.bounds[i].p2), "modified bounded range");
    Require(replay.GoalsMet(), "replayed construction does not meet ORIGINAL goals");
    Require(result.GoalsMet(), "collector contains a non-goal graph");
    Require(replay.points.size()==result.points.size(), "invented/deleted final points");
    Require(result.pointBirth.size()==result.points.size(), "missing birth metadata");
    for (size_t i=0;i<replay.points.size();++i) {
        Require(PointEqual(replay.points[i],result.points[i]), "point is not born by ordered intersection replay");
        Require(replay.PointAllowed(result.points[i]), "point outside closed grid domain");
        Require(replay.pointBirth[i]==result.pointBirth[i], "point birth does not match construction order");
    }
    return replay;
}
[[maybe_unused]] void VerifyEntries(const Graph& initial, int limit, int tools, const SolutionCollector& solutions) {
    for (const auto& entry: solutions.Entries()) {
        Replay(initial,entry.graph,limit,tools);
        Require(entry.newElements.size()==entry.graph.elements.size()-initial.elements.size(), "collector suffix size");
        for (size_t i=0;i<entry.newElements.size();++i)
            Require(ExactElement(entry.newElements[i],entry.graph.elements[initial.elements.size()+i]), "collector suffix mismatch");
        Require(entry.circles==static_cast<size_t>(count_if(entry.newElements.begin(),entry.newElements.end(),
            [](const Element& e){return e.type==Type::Circle;})), "collector circle count");
    }
}
struct Input { Graph graph; int limit=0, mode=2; int Tools() const {return mode==3?1:mode;} };
Input Load(const string& path) {
    ifstream in(path);
    Require(bool(in), "cannot open fixture: "+path);
    Input f;
    in >> f.limit >> f.mode;
    if (f.mode==3) {f.graph.gridMode=true; in >> f.graph.gridM >> f.graph.gridN;}
    int n[5]{};
    for (int& count:n) in >> count;
    for (int i=0;i<n[0];++i) {Point p; in >> p.x >> p.y; f.graph.AddPoint(p,0);}
    if (f.mode==3) f.graph.AddAutomaticGridLines();
    for (int kind=1;kind<5;++kind) for (int i=0;i<n[kind];++i) {
        double a,b,c,d;
        if (kind==2 || kind==3) {
            in >> a >> b >> c >> d;
            f.graph.AddInitialBounded({a,b},{c,d},kind==2?Type::Ray:Type::Segment);
        } else {
            in >> a >> b >> c;
            f.graph.AddInitial(Element::FromCoefficients(a,b,kind==4?c*c:c,kind==4?Type::Circle:Type::Line));
        }
    }
    f.graph.initialElementCount=f.graph.elements.size();
    int goals[3]{};
    for (int& nGoal:goals) in >> nGoal;
    for (int kind=0;kind<2;++kind) for (int i=0;i<goals[kind];++i) {
        double a,b,c; in >> a >> b >> c;
        f.graph.goalElements.push_back(Element::FromCoefficients(a,b,kind?c*c:c,kind?Type::Circle:Type::Line));
    }
    for (int i=0;i<goals[2];++i) {Point p; in >> p.x >> p.y; f.graph.goalPoints.push_back(p);}
    Require(bool(in), "incomplete fixture: "+path);
    Require(f.limit>=0 && f.mode>=0 && f.mode<=3, "invalid fixture limit/mode");
    return f;
}
Graph Base() { Graph g; g.AddPoint({0,0},0); g.AddPoint({2,0},0); return g; }
void Seal(Graph& g) {g.initialElementCount=g.elements.size();}
void AddWitness(Graph& g, Point a, Point b, Type t, uint16_t depth) {
    // Coordinate labels in certificates must resolve to ALREADY known points.
    auto known=[&](Point p) {
        for (Point old:g.points) if (PointEqual(old,p)) return old;
        throw runtime_error("certificate uses a point before its birth");
    };
    const Point p=known(a), q=known(b);
    Require(g.Apply(Definition(p,q,t),depth), "certificate repeats an element");
}
void ExpectedTests() {
    Graph initial=Base(); initial.goalPoints={{1,0}}; Seal(initial);
    Graph solved=initial;
    AddWitness(solved,{0,0},{2,0},Type::Circle,1);
    AddWitness(solved,{2,0},{0,0},Type::Circle,2);
    AddWitness(solved,{1,sqrt(3.0)},{1,-sqrt(3.0)},Type::Line,3);
    AddWitness(solved,{0,0},{2,0},Type::Line,4);
    Replay(initial,solved,4,2);
    auto rejects=[&](const Graph& bad, int limit, int tools) {
        bool rejected=false;
        try {Replay(initial,bad,limit,tools);} catch (const runtime_error&) {rejected=true;}
        Require(rejected,"origin verifier accepted an invalid negative control");
    };
    rejects(solved,3,2); rejects(solved,4,0); rejects(solved,4,1);
    Graph freePoint=initial; freePoint.AddPoint({1,0},0); rejects(freePoint,4,2);
    Graph freeGoalLine=initial;
    freeGoalLine.Apply(Element::FromCoefficients(1,0,1,Type::Line),1);
    freeGoalLine.Apply(Element::FromCoefficients(0,1,0,Type::Line),2);
    rejects(freeGoalLine,4,2);
    Graph future=initial;
    future.Apply(Element::FromCoefficients(1,0,1,Type::Circle),1);
    rejects(future,4,2);
    Graph forgedBirth=solved;
    forgedBirth.pointBirth.back()=0;
    rejects(forgedBirth,4,2);
    Graph invented=solved;
    invented.AddPoint({17,19},4);
    rejects(invented,4,2);
    Graph closeGoal;
    closeGoal.AddPoint({0,1+.75*EPS},0); closeGoal.AddPoint({2,1+.75*EPS},0);
    const Element exactGoal=Element::FromCoefficients(0,1,1,Type::Line);
    Require(ElementEqual(Definition(closeGoal.points[0],closeGoal.points[1],Type::Line),exactGoal),
            "EPS-near negative control setup");
    Require(!Origin(closeGoal,exactGoal),"verifier accepted goal coefficients instead of actual constructed coefficients");
    Graph noFreeCenters;
    noFreeCenters.AddInitial(Element::FromCoefficients(7,9,4,Type::Circle));
    Require(!noFreeCenters.HasPoint({7,9}),"initial circle center became a free known point");
    noFreeCenters.AddInitialBounded({0,0},{1,0},Type::Ray);
    Require(!noFreeCenters.HasPoint({0,0}),"bounded-element endpoints became free points");
    cout << "PASS independent origin verifier, legal E4 certificate, negative controls\n";
}
void Certificate(const string& path) {
    Input f=Load(path); Graph solved=f.graph;
    string type; Point a,b; uint16_t step=0;
    while (cin >> type >> a.x >> a.y >> b.x >> b.y) {
        Require(type=="line" || type=="circle", "unknown certificate tool");
        AddWitness(solved,a,b,type=="line"?Type::Line:Type::Circle,++step);
    }
    Require(cin.eof(), "malformed certificate");
    Replay(f.graph,solved,f.limit,f.Tools());
    cout << "{\"certificate_valid\":true,\"returned_e\":" << step << "}\n";
}
#ifndef BS_HEURISTIC_EXPECTED_ONLY
struct Run {
    HeuristicResult result;
    SearchStats stats;
    vector<SolutionCollector::Entry> entries;
    double seconds=0;
};
HeuristicOptions Options(uint32_t threads=1) {
    HeuristicOptions o; o.threads=threads; o.restarts=1;
    o.beamWidth=64; o.branchLimit=96; o.seed=17;
    o.tailSeconds=0; o.tailCandidates=0;
    return o;
}
Run Search(Graph g, int limit, int tools, HeuristicOptions options,
           double seconds=10, size_t quota=1, bool slots=true) {
    Seal(g);
    const auto savedPoints=g.points;
    const auto savedElements=g.elements;
    SolutionCollector collector(quota); ParallelControl control;
    const auto start=chrono::steady_clock::now();
    control.deadline=start+chrono::duration_cast<chrono::steady_clock::duration>(chrono::duration<double>(seconds));
    auto progress=make_unique<ProgressSlot[]>(max<size_t>(options.threads,1));
    Run r;
    r.result=RunHeuristic(g,limit,tools,options,collector,control,r.stats,
                         slots?progress.get():nullptr,slots?options.threads:0);
    r.seconds=chrono::duration<double>(chrono::steady_clock::now()-start).count();
    if (slots) for (size_t i=0;i<options.threads;++i) {
        const auto snapshot=progress[i].Read();
        Require(!snapshot.active,"progress slot still active after workers joined");
        Require(snapshot.depth<=limit,"progress depth exceeds maximum E");
    }
    Require(g.points.size()==savedPoints.size() && g.elements.size()==savedElements.size(),"const initial graph mutated");
    for (size_t i=0;i<savedPoints.size();++i)
        Require(PointEqual(savedPoints[i],g.points[i]),"const initial point mutated");
    for (size_t i=0;i<savedElements.size();++i)
        Require(ExactElement(savedElements[i],g.elements[i]),"const initial element mutated");
    VerifyEntries(g,limit,tools,collector);
    Require(collector.Count()<=quota,"collector exceeds quota");
    Require(r.result.quotaReached==(collector.Count()>=quota),"quotaReached disagrees with collector");
    Require(!(r.result.quotaReached && r.result.timedOut),"quota marked timeout");
    Require(r.result.metrics.peakBeam<=options.beamWidth*max<uint32_t>(1,options.threads),"beam storage cap exceeded");
    if (seconds>0) Require(r.seconds<seconds+3,"deadline exceeded by more than test grace");
    r.entries=collector.Entries();
    return r;
}
void Solved(const Run& r, const string& name) {
    Require(r.result.quotaReached && !r.entries.empty(),name+": expected known solution not found");
}
string Fingerprint(const Run& r) {
    ostringstream s;
    s << r.result.quotaReached << ':' << r.result.timedOut;
    for (const auto& entry:r.entries) {
        s << '|';
        for (const Element& e:entry.graph.elements)
            s << int(e.type) << ',' << bit_cast<uint64_t>(e.a) << ',' << bit_cast<uint64_t>(e.b)
              << ',' << bit_cast<uint64_t>(e.c) << ';';
        for (size_t i=0;i<entry.graph.points.size();++i)
            s << bit_cast<uint64_t>(entry.graph.points[i].x) << ','
              << bit_cast<uint64_t>(entry.graph.points[i].y) << ',' << entry.graph.pointBirth[i] << ';';
    }
    const auto& m=r.result.metrics;
    s << '/' << m.restarts << ',' << m.layers << ',' << m.expanded << ',' << m.generated << ','
      << m.evaluated << ',' << m.beamDiscarded << ',' << m.candidateDiscarded << ',' << m.tailCalls << ','
      << m.peakBeam << ',' << m.budgetLimited << '/' << r.stats.nodes << ',' << r.stats.applied;
    return s.str();
}
void CancellationRegression() {
    // Every point is EPS-incident on y=1, but every pair constructs a line
    // with slope 2^-36 > EPS.  The 33,550,336-pair reverse scan must poll.
    EPS=1e-11;
    Graph input;
    input.points.reserve(8192); input.pointBirth.reserve(8192);
    for (int i=0;i<8192;++i) {
        input.points.push_back({ldexp(double(i),-16),1+ldexp(double(i),-52)});
        input.pointBirth.push_back(0); // bypass AddPoint's O(n^2) setup scans
    }
    input.goalElements={Element::FromCoefficients(0,1,1,Type::Line)};
    Require(input.PointOnElement(input.points.back(),input.goalElements[0]),"cancellation regression incidence");
    Require(!ElementEqual(Definition(input.points.front(),input.points.back(),Type::Line),input.goalElements[0]),
            "cancellation regression candidate unexpectedly matches");
    Graph timed=input;
    ParallelControl local;
    local.deadline=chrono::steady_clock::now()+chrono::milliseconds(1);
    Solver solver(1,false,true,true,0,0,.001);
    SearchStats stats;
    auto start=chrono::steady_clock::now();
    Require(!solver.SearchPrefixTask(timed,1,PrefixTask{},stats,&local),"pair scan fabricated solution");
    const double seconds=chrono::duration<double>(chrono::steady_clock::now()-start).count();
    Require(solver.TimedOut() && local.timedOut.load(),"long reverse-pair scan did not report timeout");
    Require(seconds<1,"reverse-pair scan did not respond to 1ms deadline");
    Graph cancelled=input;
    atomic<bool> external{true};
    ParallelControl control;
    control.deadline=chrono::steady_clock::now()+chrono::hours(1);
    Solver stopped(1,false,true,true,0,0,60);
    stopped.SetExternalStop(&external);
    SearchStats cancelledStats;
    start=chrono::steady_clock::now();
    Require(!stopped.SearchPrefixTask(cancelled,1,PrefixTask{},cancelledStats,&control),"externally cancelled scan ran");
    Require(!stopped.TimedOut() && !control.timedOut.load(),"external stop mislabeled timeout");
    Require(chrono::duration<double>(chrono::steady_clock::now()-start).count()<1,"external stop did not cancel promptly");
    cout << "PASS reverse-pair deadline/external-cancel regression (deadline wall=" << seconds << "s)\n";
}
void EngineTests() {
    size_t checks=0;
    CancellationRegression();
    for (uint32_t threads:{1u,4u}) {
        auto o=Options(threads);
        Graph line=Base(); line.goalElements={Element::FromCoefficients(0,1,0,Type::Line)};
        Solved(Search(line,1,1,o),"mode1"); ++checks;
        Graph circle=Base(); circle.goalElements={Element::FromCoefficients(0,0,4,Type::Circle)};
        Solved(Search(circle,1,0,o),"mode0"); ++checks;
        Require(Search(line,1,0,o).entries.empty(),"mode0 produced target line"); ++checks;
        Require(Search(circle,1,1,o).entries.empty(),"mode1 produced target circle"); ++checks;
        Graph midpoint=Base(); midpoint.goalPoints={{1,0}};
        Solved(Search(midpoint,4,2,o),"mode2 midpoint"); ++checks;
        Require(Search(midpoint,1,2,o).entries.empty(),"midpoint illegally solved in E1"); ++checks;
        Graph grid; grid.gridMode=true; grid.gridM=grid.gridN=1; grid.AddAutomaticGridLines();
        grid.goalPoints={{.5,.5}};
        Solved(Search(grid,2,1,o),"mode3 grid diagonals"); ++checks;
        Graph met=Base(); met.goalPoints={{0,0}};
        auto zero=Search(met,0,2,o); Solved(zero,"E0 already met");
        Require(zero.entries[0].newElements.empty(),"E0 used paid steps"); ++checks;
        auto early=Search(met,4,2,o); Solved(early,"already met nonzero limit");
        Require(early.entries[0].newElements.empty(),"already met goal should return zero steps"); ++checks;
        Require(Search(line,0,1,o).entries.empty(),"E0 created goal line"); ++checks;
        // A ray is not its infinite carrier; the carrier must be paid for.
        Graph ray=Base(); ray.AddInitialBounded({0,0},{2,0},Type::Ray);
        ray.goalElements=line.goalElements;
        auto paid=Search(ray,1,1,o); Solved(paid,"ray paid supporting line");
        Require(paid.entries[0].newElements.size()==1,"ray carrier was free"); ++checks;
        Graph guard;
        guard.AddPoint({2,1},0); guard.AddPoint({2,2},0);
        guard.AddInitialBounded({0,0},{1,0},Type::Ray);
        guard.AddInitial(Element::FromCoefficients(0,1,0,Type::Line));
        guard.goalPoints={{2,0}};
        vector<uint32_t> forced; guard.CollectForcedTailPointIndices(1,forced);
        Require(forced==vector<uint32_t>{0},"ray plus paid line lost forced goal");
        Solved(Search(guard,1,1,o),"ray coincident carrier forced point"); ++checks;
        // Timeout checks use an unsatisfied goal, never count timeout as failure proof.
        Graph hard;
        for (int i=0;i<18;++i) hard.AddPoint({double(i),double((i*i*7)%31)},0);
        hard.goalPoints={{1000.1234567,2000.7654321}};
        auto expired=Search(hard,8,2,o,-1);
        Require(expired.result.timedOut && expired.entries.empty(),"expired deadline not respected"); ++checks;
        o.restarts=0;
        auto timed=Search(hard,8,2,o,.003);
        Require(timed.result.timedOut && !timed.result.quotaReached,"wall budget not reported as timeout"); ++checks;
        auto partialTimeoutOptions=o;
        partialTimeoutOptions.beamWidth=4; partialTimeoutOptions.branchLimit=8;
        auto partialTimeout=Search(met,5,2,partialTimeoutOptions,.005,100000);
        Require(partialTimeout.result.timedOut && !partialTimeout.result.quotaReached &&
                !partialTimeout.entries.empty(),"timeout lost already collected legal solutions"); ++checks;
        o=Options(threads); o.beamWidth=1; o.branchLimit=1;
        auto capped=Search(hard,3,2,o);
        Require(!capped.result.quotaReached && !capped.result.timedOut,"finite capped search status");
        Require(capped.result.metrics.budgetLimited || capped.result.metrics.beamDiscarded ||
                capped.result.metrics.candidateDiscarded,"resource discard not reported"); ++checks;
        // A partial collection must remain non-quota and every entry must replay.
        auto partial=Search(line,1,1,Options(threads),10,3);
        Require(partial.entries.size()==1 && !partial.result.quotaReached,"partial quota semantics"); ++checks;
    }
    for (double eps:{1e-11,1e-13}) {
        EPS=eps;
        Graph nearGraph;
        nearGraph.AddPoint({0,1+.6*EPS},0); nearGraph.AddPoint({.1,1-.6*EPS},0);
        nearGraph.AddPoint({2,1},0); nearGraph.AddPoint({3,1},0);
        nearGraph.goalElements={Element::FromCoefficients(0,1,1,Type::Line)};
        Require(!ElementEqual(Definition(nearGraph.points[0],nearGraph.points[1],Type::Line),nearGraph.goalElements[0]),"first pair regression invalid");
        Solved(Search(nearGraph,1,1,Options()),"v9 first-two invalid pair"); ++checks;
        nearGraph.goalElements.clear();
        nearGraph.AddInitial(Element::FromCoefficients(1,0,4,Type::Line));
        nearGraph.AddInitial(Element::FromCoefficients(1,0,5,Type::Line));
        nearGraph.goalPoints={{4,1},{5,1}};
        Solved(Search(nearGraph,1,1,Options()),"reverse-line later pair"); ++checks;
        // EPS equality is non-transitive: one constructed line meets BOTH goals.
        Graph overlap;
        overlap.AddPoint({0,1+.75*EPS},0); overlap.AddPoint({2,1+.75*EPS},0);
        overlap.goalElements={Element::FromCoefficients(0,1,1,Type::Line),
                              Element::FromCoefficients(0,1,1+1.5*EPS,Type::Line)};
        Require(overlap.MissingDistinctGoalElementCount()==2,"overlap regression setup");
        Solved(Search(overlap,1,1,Options()),"nontransitive goal count lower bound"); ++checks;
        // Residual tolerance is not the coordinate-wise SamePoint tolerance.
        Graph residual;
        residual.AddPoint({0,0},0); residual.AddPoint({2,2},0);
        residual.AddInitial(Element::FromCoefficients(1,1,2+1.5*EPS,Type::Line));
        residual.goalPoints={{1,1}};
        Require(residual.ExistingSupportCount({1,1})==0,"residual regression setup");
        Solved(Search(residual,1,1,Options()),"residual versus coordinate tolerance"); ++checks;
    }
    EPS=1e-11;
    Graph deterministic=Base(); deterministic.goalPoints={{1,0}};
    auto o=Options(); o.restarts=2; o.beamWidth=16; o.branchLimit=24;
    const Run first=Search(deterministic,4,2,o,20,3,false);
    Require(!first.result.timedOut,"determinism comparison was time limited");
    for (int repeat=0;repeat<3;++repeat)
        Require(Fingerprint(first)==Fingerprint(Search(deterministic,4,2,o,20,3,false)),"single-thread same seed changed paths/counters");
    ++checks;
    o=Options(); o.tailSeconds=.05; o.tailCandidates=4;
    Graph tail=Base(); tail.goalPoints={{1,sqrt(3.0)}};
    Solved(Search(tail,2,2,o),"exact-tail integration"); ++checks;
    cout << "PASS heuristic engine checks=" << checks << " (all collector entries independently replayed)\n";
}
void CaseRun(int argc, char** argv) {
    Input f=Load(argv[2]); auto o=Options(); double seconds=10; size_t quota=1;
    for (int i=3;i<argc;++i) {
        string key=argv[i]; Require(i+1<argc,"missing case option value"); string v=argv[++i];
        if (key=="--threads") o.threads=static_cast<uint32_t>(stoul(v));
        else if (key=="--seed") o.seed=stoull(v);
        else if (key=="--restarts") o.restarts=stoull(v);
        else if (key=="--beam-width") o.beamWidth=stoull(v);
        else if (key=="--branch-limit") o.branchLimit=stoull(v);
        else if (key=="--tail-seconds") o.tailSeconds=stod(v);
        else if (key=="--tail-candidates") o.tailCandidates=stoull(v);
        else if (key=="--seconds") seconds=stod(v);
        else if (key=="--quota") quota=stoull(v);
        else if (key=="--eps") EPS=stod(v);
        else throw runtime_error("unknown case option: "+key);
    }
    auto r=Search(f.graph,f.limit,f.Tools(),o,seconds,quota);
    cout << setprecision(17) << "{\"valid\":true,\"quota_reached\":" << (r.result.quotaReached?"true":"false")
         << ",\"timed_out\":" << (r.result.timedOut?"true":"false") << ",\"seconds\":" << r.seconds
         << ",\"returned_e\":[";
    for (size_t i=0;i<r.entries.size();++i) {if(i) cout << ','; cout << r.entries[i].newElements.size();}
    const auto& m=r.result.metrics;
    cout << "],\"metrics\":{\"restarts\":" << m.restarts << ",\"layers\":" << m.layers
         << ",\"expanded\":" << m.expanded << ",\"generated\":" << m.generated
         << ",\"evaluated\":" << m.evaluated << ",\"beamDiscarded\":" << m.beamDiscarded
         << ",\"candidateDiscarded\":" << m.candidateDiscarded << ",\"tailCalls\":" << m.tailCalls
         << ",\"peakBeam\":" << m.peakBeam << ",\"budgetLimited\":" << m.budgetLimited << "}}\n";
}
#endif
} // namespace
int main(int argc, char** argv) {
    try {
        if (argc>=3 && string(argv[1])=="--certificate") {Certificate(argv[2]); return 0;}
#ifndef BS_HEURISTIC_EXPECTED_ONLY
        if (argc>=3 && string(argv[1])=="--case") {CaseRun(argc,argv); return 0;}
#endif
        Require(argc==1,"usage: heuristic_tests [--certificate INPUT | --case INPUT options]");
        ExpectedTests();
#ifndef BS_HEURISTIC_EXPECTED_ONLY
        EngineTests();
#else
        cout << "SKIP engine tests: BS_HEURISTIC_EXPECTED_ONLY build\n";
#endif
        return 0;
    } catch (const exception& e) {
        cerr << "FAIL heuristic tests: " << e.what() << '\n';
        return 1;
    }
}
