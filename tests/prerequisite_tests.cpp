// Standalone short correctness tests; never include another test's main.
// C++20, -O2 -pthread -fno-fast-math -ffp-contract=off (MSVC: /fp:strict).
#include "../src/diameter_probe.hpp"
#include "../src/mirror_probe.hpp"
#include "../src/homothety_probe.hpp"
#include "../src/tangent_probe.hpp"
#include "../src/goal_finish.hpp"
#include <filesystem>

#ifdef __FAST_MATH__
#error Prerequisite witness tests require strict floating-point arithmetic
#endif

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
void Seal(Graph& graph) {
    graph.initialElementCount = graph.elements.size();
    graph.SetStateHashingEnabled(true);
}
// Independent raw-coefficient regeneration: no MakeCandidate/FromPoints,
// goal coordinates, analytic point insertion, or exporter-selected witnesses.
Element Definition(Point p, Point q, Type type) {
    Require(!SamePoint(p,q),"ordinary operation needs distinct known points");
    Element e; e.type = type;
    if (type == Type::Circle) {
        e.a = p.x; e.b = p.y;
        e.c = (p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y);
    } else {
        Require(type == Type::Line,"paid bounded element is not an ordinary tool");
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
    for (Point p : graph.points) for (Point q : graph.points) {
        if (!isfinite(p.x) || !isfinite(p.y) || !isfinite(q.x) || !isfinite(q.y) || SamePoint(p,q)) continue;
        if (Exact(Definition(p,q,operation.type),operation)) return true;
    }
    return false;
}
Point Stored(const Graph& graph, Point requested) {
    for (Point p : graph.points) if (SamePoint(p,requested)) return p;
    throw runtime_error("fixture requested a point not actually stored");
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
Graph Replay(const Graph& root, const Graph& result, int totalBudget,
             int tools = 2, size_t suffixFrom = 0, bool requireGoals = false) {
    Require(root.initialElementCount == root.elements.size(),"unsealed replay root");
    Require(result.initialElementCount == root.initialElementCount,"paid prefix was resealed/free");
    Require(result.elements.size() >= root.elements.size() && totalBudget >= 0 &&
            result.elements.size()-root.elements.size() <= static_cast<size_t>(totalBudget),"returned E exceeds budget");
    for (size_t i = 0; i < root.elements.size(); ++i)
        Require(Exact(root.elements[i],result.elements[i]),"changed given element");
    Graph replay = root;
    for (size_t i = root.elements.size(); i < result.elements.size(); ++i) {
        const Element& e = result.elements[i];
        Require(e.type == Type::Line || e.type == Type::Circle,"unsupported paid tool");
        Require(e.bound == NO_BOUND && isfinite(e.a) && isfinite(e.b) && isfinite(e.c),"nonfinite/bounded paid element");
        Require(e.type == Type::Circle ? e.c > 0 : e.a != 0 || e.b != 0,"degenerate paid element");
        if (i >= suffixFrom) {
            Require(tools != 0 || e.type == Type::Circle,"circle-only callback contains a paid line");
            Require(tools != 1 || e.type == Type::Line,"line-only callback contains a paid circle");
        }
        Require(KnownPair(replay,e),"no RAW-BIT known-pair witness before paid step " + to_string(i-root.elements.size()+1));
        Require(replay.Apply(e,static_cast<uint16_t>(i-root.initialElementCount+1)),"duplicate paid operation");
    }
    Require(Fingerprint(replay) == Fingerprint(result),"state/point bits/birth/goals differ from ordered Apply replay");
    Require(replay.GoalsMet() == result.GoalsMet(),"callback GoalsMet disagrees with replay");
    if (requireGoals) Require(replay.GoalsMet(),"ordered replay misses ORIGINAL goals");
    for (size_t i = 0; i < replay.points.size(); ++i) {
        const Point p = replay.points[i];
        Require(isfinite(p.x) && isfinite(p.y),"nonfinite stored point");
        Require(replay.pointBirth[i] <= result.elements.size()-root.elements.size(),"point birth exceeds total paid E");
        Require(replay.PointAllowed(p),"point outside closed allowed domain");
    }
    return replay;
}
void Pay(Graph& graph, Point p, Point q, Type type) {
    const Element e = Definition(Stored(graph,p),Stored(graph,q),type);
    Require(KnownPair(graph,e) && graph.Apply(e,static_cast<uint16_t>(graph.elements.size()-graph.initialElementCount+1)),
            "fixture paid operation invalid/already present");
}

enum class Family { Diameter, Mirror, Homothety, Tangent };
struct Work { uint64_t applied = 0, callbacks = 0, successes = 0; };
template<class Complete>
bool Dispatch(Family family, const Graph& parent, int remaining, int tools,
              ParallelControl& control, SearchStats& stats, Work& work,
              Clock::time_point deadline, Complete&& complete, ProgressSlot* slot = nullptr) {
    auto finish = [&](bool value, const auto& counts) {
        work = {counts.applied,counts.callbacks,counts.successes}; return value;
    };
    if (family == Family::Diameter) {
        diameter_detail::Counts counts;
        const bool value = diameter_detail::Probe(parent,remaining,tools,control,stats,counts,deadline,complete,slot);
        return finish(value,counts);
    }
    if (family == Family::Mirror) {
        mirror_detail::Counts counts;
        const bool value = mirror_detail::Probe(parent,remaining,tools,control,stats,counts,deadline,complete,slot);
        return finish(value,counts);
    }
    if (family == Family::Tangent) {
        tangent_detail::Counts counts;
        const bool value = tangent_detail::Probe(parent,remaining,tools,control,stats,counts,deadline,complete,slot);
        return finish(value,counts);
    }
    homothety_detail::Counts counts;
    const bool value = homothety_detail::Probe(parent,remaining,tools,control,stats,counts,deadline,complete,slot);
    return finish(value,counts);
}
struct Options { bool cancelled = false, expired = false, globalExpired = false, stopAfterCallback = false; };
struct Run {
    bool accepted = false, stopped = false;
    size_t callbacks = 0;
    SearchStats stats;
    Work work;
    optional<Graph> prefix; // A callback-approved PREFIX, not a solution/collector entry.
};
Run Invoke(Family family, const Graph& root, const Graph& parent, int remaining, int tools,
           const function<bool(const Graph&,int)>& accept, Options options = {}) {
    const string before = Fingerprint(parent);
    const auto hash1 = parent.StateHash1(), hash2 = parent.StateHash2();
    const size_t depth = parent.elements.size()-root.elements.size();
    Run run; ParallelControl control; ProgressSlot slot;
    const auto start = Clock::now();
    control.deadline = start + (options.globalExpired ? -chrono::seconds(1) : chrono::seconds(1));
    control.stop.store(options.cancelled);
    const auto deadline = start + (options.expired ? -chrono::milliseconds(1) : chrono::milliseconds(100));
    slot.BeginTask(run.stats);
    run.accepted = Dispatch(family,parent,remaining,tools,control,run.stats,run.work,deadline,
        [&](const Graph& state, int left) {
            ++run.callbacks;
            Require(left >= 0 && left <= remaining && state.elements.size() >= parent.elements.size(),"invalid callback remaining");
            Require(state.elements.size()-parent.elements.size()+static_cast<size_t>(left) == static_cast<size_t>(remaining),
                    "callback remaining omits a paid step");
            for (size_t i = 0; i < parent.elements.size(); ++i)
                Require(Exact(parent.elements[i],state.elements[i]),"callback rewrote a paid parent operation");
            Replay(root,state,static_cast<int>(depth)+remaining,tools,parent.elements.size());
            if (options.stopAfterCallback) control.stop.store(true,memory_order_release);
            if (!accept(state,left)) return false;
            run.prefix = state;
            return true;
        },&slot);
    slot.EndTask(run.stats);
    run.stopped = control.stop.load();
    Require(Fingerprint(parent) == before && parent.StateHash1() == hash1 && parent.StateHash2() == hash2,"probe mutated parent/hash");
    Require(!control.found.load() && !control.timedOut.load(),"prefix callback was packaged as a global solution/timeout");
    Require(run.accepted == run.prefix.has_value() && run.work.successes == (run.accepted ? 1u : 0u),"callback return/acceptance mismatch");
    Require(run.work.callbacks == run.callbacks && run.work.applied == run.stats.applied && run.stats.nodes == run.stats.applied,
            "probe work/callback accounting");
    Require(run.stats.maxElements <= parent.elements.size()+static_cast<size_t>(max(0,remaining)),"probe applied over-budget work");
    Require(slot.Read().nodes == run.stats.nodes && !slot.Read().active,"progress final accounting");
    return run;
}

struct Input { Graph graph; int limit = 0, tools = 2; };
Input Load(const fs::path& path) {
    ifstream in(path); Require(bool(in),"cannot open fixture: " + path.string());
    Input input; in >> input.limit >> input.tools;
    Require(input.limit >= 0 && input.tools >= 0 && input.tools <= 2,"invalid fixture header");
    int counts[5]{};
    for (int& n : counts) { in >> n; Require(n >= 0,"negative fixture count"); }
    for (int i = 0; i < counts[0]; ++i) { Point p; in >> p.x >> p.y; input.graph.AddPoint(p,0); }
    for (int kind = 1; kind < 5; ++kind) for (int i = 0; i < counts[kind]; ++i) {
        double a = 0, b = 0, c = 0, d = 0;
        if (kind == 2 || kind == 3) {
            in >> a >> b >> c >> d;
            input.graph.AddInitialBounded({a,b},{c,d},kind == 2 ? Type::Ray : Type::Segment);
        } else {
            in >> a >> b >> c;
            input.graph.AddInitial(Element::FromCoefficients(a,b,kind == 4 ? c*c : c,kind == 4 ? Type::Circle : Type::Line));
        }
    }
    Seal(input.graph);
    int goals[3]{};
    for (int& n : goals) { in >> n; Require(n >= 0,"negative goal count"); }
    for (int kind = 0; kind < 2; ++kind) for (int i = 0; i < goals[kind]; ++i) {
        double a = 0, b = 0, c = 0; in >> a >> b >> c;
        input.graph.goalElements.push_back(Element::FromCoefficients(a,b,kind ? c*c : c,kind ? Type::Circle : Type::Line));
    }
    for (int i = 0; i < goals[2]; ++i) { Point p; in >> p.x >> p.y; input.graph.goalPoints.push_back(p); }
    Require(bool(in),"incomplete fixture");
    string extra; Require(!(in >> extra),"fixture has trailing tokens");
    return input;
}
fs::path Fixture(const char* executable, const char* name = "eu9_7_minimum_perimeter.in") {
    const fs::path suffix = fs::path("benchmarks/r32/cases")/name;
    for (fs::path base : {fs::absolute(fs::path(__FILE__)).parent_path(),fs::current_path(),fs::absolute(executable).parent_path()})
        for (int i = 0; i < 6 && !base.empty(); ++i) {
            if (fs::is_regular_file(base/suffix)) return base/suffix;
            if (base == base.parent_path()) break;
            base = base.parent_path();
        }
    throw runtime_error("cannot locate r32 fixture: " + suffix.string());
}

const auto Goals = [](const Graph& g,int) { return g.GoalsMet(); };
const auto Decline = [](const Graph&,int) { return false; };
void NoWork(const Run& run) {
    Require(!run.accepted && run.callbacks == 0 && run.stats.applied == 0 && run.stats.nodes == 0 &&
            run.stats.rawCandidates == 0 && run.work.callbacks == 0 && run.work.successes == 0,"ineligible/cancelled call performed work");
}
Graph DiameterFixture(bool clipped = false) {
    Graph graph;
    graph.AddPoint({0,0},0); graph.AddPoint({2,0},0);
    if (clipped) graph.AddInitialBounded({10,0},{11,0},Type::Segment);
    else graph.AddInitial(Definition(graph.points[0],graph.points[1],Type::Line));
    graph.goalElements = {Element::FromCoefficients(1,0,1,Type::Circle)};
    Seal(graph);
    return graph;
}
void DiameterRootAndPaid() {
    const Graph root = DiameterFixture();
    Require(!root.HasPoint({1,0}) && !root.GoalsMet(),"diameter midpoint must not be a free fixture point");
    const auto run = Invoke(Family::Diameter,root,root,4,2,Goals);
    Require(run.accepted && run.prefix->elements.size()-root.elements.size() == 4,"diameter root needs four ordinary paid steps");
    Replay(root,*run.prefix,4,2,root.elements.size(),true);
    Graph parent = root;
    Pay(parent,{0,0},{2,0},Type::Circle);
    const auto paid = Invoke(Family::Diameter,root,parent,3,2,Goals);
    Require(paid.accepted && paid.prefix->elements.size()-root.elements.size() == 4,"diameter existing paid circle must leave exactly 3E");
    Replay(root,*paid.prefix,4,2,parent.elements.size(),true);
    Graph given = parent;
    // Explicitly a DIFFERENT root fixture, not a reseal inside a probe.
    for (auto& birth : given.pointBirth) birth = 0;
    Seal(given);
    const auto existing = Invoke(Family::Diameter,given,given,3,2,Goals);
    Require(existing.accepted && existing.prefix->elements.size()-given.elements.size() == 3,"given reciprocal circle was charged again");
    for (int budget : {0,1,2,3})
        Require(!Invoke(Family::Diameter,root,root,budget,2,Goals).accepted,"diameter overran a sub-four-E budget");
}
void OrthicFourStepPrefix(const Input& input) {
    const Graph& root = input.graph;
    Require(root.points.size() == 3 && root.goalPoints.size() == 3 && root.goalElements.size() == 3 && !root.GoalsMet(),
            "real Orthic fixture shape changed");
    const Point b = root.points[1], c = root.points[2];
    const Point middle{(b.x+c.x)/2,(b.y+c.y)/2};
    Element circle;
    circle.type = Type::Circle; circle.a = middle.x; circle.b = middle.y;
    circle.c = (middle.x-b.x)*(middle.x-b.x)+(middle.y-b.y)*(middle.y-b.y);
    const auto hasTwoFeet = [&](const Graph& state,int) {
        return state.HasElement(circle) && state.HasPoint(root.goalPoints[1]) && state.HasPoint(root.goalPoints[2]);
    };
    const auto run = Invoke(Family::Diameter,root,root,4,2,hasTwoFeet);
    Require(run.accepted && run.prefix->elements.size()-root.elements.size() == 4,"real Orthic diameter(BC) did not generate both feet in four E");
    Require(!run.prefix->GoalsMet(),"four-E Orthic prefix must not be presented as the eight-E solution");
    Graph parent = root;
    Pay(parent,b,c,Type::Circle);
    const auto paid = Invoke(Family::Diameter,root,parent,3,2,hasTwoFeet);
    Require(paid.accepted && paid.prefix->elements.size()-root.elements.size() == 4 && !paid.prefix->GoalsMet(),
            "Orthic paid-circle continuation lost prefix cost/goals");
    const auto shortRun = Invoke(Family::Diameter,root,root,3,2,hasTwoFeet);
    Require(!shortRun.accepted,"Orthic diameter feet were manufactured without four real operations");
}
void OrthicFinishedEightE(const Input& input) {
    const Graph& root = input.graph;
    for (int variant = 0; variant < 3; ++variant) {
        const bool paidParent = variant != 0;
        const int totalBudget = variant == 2 ? 7 : input.limit;
        Graph parent = root;
        if (paidParent) Pay(parent,root.points[1],root.points[2],Type::Circle);
        const string before = Fingerprint(parent);
        const auto hash1 = parent.StateHash1(), hash2 = parent.StateHash2();
        const int remaining = totalBudget-static_cast<int>(parent.elements.size()-root.elements.size());
        ParallelControl control;
        const auto deadline = Clock::now()+chrono::milliseconds(200);
        control.deadline = deadline;
        SolutionCollector collector(1);
        SearchStats probeStats, finishStats;
        diameter_detail::Counts probeCounts;
        goal_finish_detail::Counts finishCounts;
        size_t calls = 0;
        const bool quota = diameter_detail::Probe(parent,remaining,2,control,probeStats,probeCounts,deadline,
            [&](const Graph& prefix,int left) {
                ++calls;
                Replay(root,prefix,totalBudget,2,parent.elements.size());
                Require(prefix.elements.size()-parent.elements.size()+static_cast<size_t>(left) == static_cast<size_t>(remaining),
                        "Orthic finish received incorrect remaining E");
                const string prefixBefore = Fingerprint(prefix);
                const auto ph1 = prefix.StateHash1(), ph2 = prefix.StateHash2();
                const bool found = goal_finish_detail::Find(prefix,left,2,collector,control,finishStats,finishCounts,deadline);
                Require(Fingerprint(prefix) == prefixBefore && prefix.StateHash1() == ph1 && prefix.StateHash2() == ph2,
                        "goal finish modified its paid parent");
                return found;
            });
        Require(Fingerprint(parent) == before && parent.StateHash1() == hash1 && parent.StateHash2() == hash2,
                "Orthic integration mutated caller's paid parent");
        Require(probeStats.maxElements <= root.elements.size()+static_cast<size_t>(totalBudget) &&
                finishStats.maxElements <= root.elements.size()+static_cast<size_t>(totalBudget),"Orthic finish exceeded total E while searching");
        if (totalBudget == 7) {
            Require(!quota && collector.Count() == 0 && !control.found.load() && !control.stop.load() && !control.timedOut.load() &&
                    calls > 0 && finishStats.applied > 0,"seven-E Orthic guard fabricated a solution or never exercised finish");
            continue; // Unknown at this budget, not a proof of mathematical minimality.
        }
        Require(quota && collector.Count() == 1 && control.found.load() && control.stop.load() && !control.timedOut.load(),
                "diameter + goal finish did not return verified complete Orthic 8E");
        Require(calls > 0 && probeCounts.successes == 1 && finishCounts.successes > 0,
                "full Orthic solve lacks real probe/finish submission evidence");
        const auto& entry = collector.Entries().front();
        Replay(root,entry.graph,totalBudget,2,parent.elements.size(),true);
        Require(entry.newElements.size() == 8 && entry.graph.elements.size()-root.elements.size() == 8,
                "collector forgot/discounted a paid prefix step");
        for (size_t i = 0; i < entry.newElements.size(); ++i)
            Require(Exact(entry.newElements[i],entry.graph.elements[root.elements.size()+i]),"collector changed a raw paid operation");
        for (size_t i = 0; i < parent.elements.size(); ++i)
            Require(Exact(parent.elements[i],entry.graph.elements[i]),"full finish replaced a paid parent operation");
    }
}
void DiameterFiniteCarrier() {
    const Graph finite = DiameterFixture(true);
    Require(finite.points.size() == 2 && !finite.HasPoint({10,0}) && !finite.HasPoint({1,0}),"bounded endpoints leaked into known points");
    const auto shortRun = Invoke(Family::Diameter,finite,finite,4,2,Goals);
    Require(!shortRun.accepted && shortRun.callbacks == 0,"remote finite segment was used as its infinite carrier for free");
    const auto paidLine = Invoke(Family::Diameter,finite,finite,5,2,Goals);
    Require(paidLine.accepted && paidLine.prefix->elements.size()-finite.elements.size() == 5,
            "finite carrier needs one additional paid endpoint line");
    Replay(finite,*paidLine.prefix,5,2,finite.elements.size(),true);
}

Point Similarity(Point p, double u, double v, Point offset) {
    return {u*p.x-v*p.y+offset.x,v*p.x+u*p.y+offset.y};
}
Graph MirrorFixture(double u = 1, double v = 0, Point offset = {}, bool clipped = false) {
    Graph graph;
    auto point = [&](Point p) { return Similarity(p,u,v,offset); };
    graph.AddPoint(point({0,0}),0); graph.AddPoint(point({4,0}),0); graph.AddPoint(point({1,2}),0);
    if (clipped) graph.AddInitialBounded(point({1.5,0}),point({2.5,0}),Type::Segment);
    else graph.AddInitial(Definition(graph.points[0],graph.points[1],Type::Line));
    graph.goalPoints = {point({1,-2})};
    Seal(graph);
    return graph;
}
void MirrorGenericAndPaid() {
    for (const auto& t : vector<array<double,4>>{{1,0,0,0},{0,1,3,-2},{3,4,-2,1}}) {
        const Graph root = MirrorFixture(t[0],t[1],{t[2],t[3]});
        Require(!root.GoalsMet(),"reflection was inserted for free");
        for (int tools : {0,2}) {
            const auto run = Invoke(Family::Mirror,root,root,2,tools,Goals);
            Require(run.accepted && run.prefix->elements.size()-root.elements.size() == 2,"generic reflection must be produced by two real circles");
            for (size_t i = root.elements.size(); i < run.prefix->elements.size(); ++i)
                Require(run.prefix->elements[i].type == Type::Circle,"two-circle reflection spent a non-circle step");
            Replay(root,*run.prefix,2,tools,root.elements.size(),true);
        }
    }
    const Graph root = MirrorFixture();
    Graph parent = root;
    Pay(parent,parent.points[0],parent.points[2],Type::Circle);
    Require(parent.points.size() > root.points.size() && parent.pointBirth.back() == 1,"paid mirror fixture needs real nonzero-birth points");
    for (int tools : {0,2}) {
        const auto paid = Invoke(Family::Mirror,root,parent,1,tools,Goals);
        Require(paid.accepted && paid.prefix->elements.size()-parent.elements.size() == 1,"existing paid radius circle must leave only one E");
        Replay(root,*paid.prefix,2,tools,parent.elements.size(),true);
    }
    Graph given = root;
    given.AddInitial(Definition(given.points[0],given.points[2],Type::Circle));
    Seal(given);
    const auto existing = Invoke(Family::Mirror,given,given,1,0,Goals);
    Require(existing.accepted && existing.prefix->elements.size()-given.elements.size() == 1,"initial radius circle was charged a second time");
    for (int budget : {0,1})
        Require(!Invoke(Family::Mirror,root,root,budget,0,Goals).accepted,"mirror overran missing-circle E budget");
}
void MirrorLineAndFiniteCarrier() {
    Graph root = MirrorFixture();
    root.goalElements = {Definition(root.points[2],root.goalPoints[0],Type::Line)};
    const auto joined = Invoke(Family::Mirror,root,root,3,2,Goals);
    Require(joined.accepted && joined.prefix->elements.size()-root.elements.size() == 3,"mirror optional join must be a third paid ordinary line");
    Replay(root,*joined.prefix,3,2,root.elements.size(),true);
    const auto circlesOnly = Invoke(Family::Mirror,root,root,3,0,Goals);
    Require(!circlesOnly.accepted && circlesOnly.callbacks > 0,"circle-only mirror must offer circles but never a disabled join line");
    const Graph finite = MirrorFixture(1,0,{},true);
    const auto clipped = Invoke(Family::Mirror,finite,finite,2,0,Goals);
    Require(!clipped.accepted && clipped.callbacks == 0 && clipped.stats.applied == 0,
            "mirror selected anchors outside a finite segment as though it were infinite");
}

Graph HomothetyFixture(Point goal = {-6,0}, bool unresolvedLine = false) {
    Graph graph;
    graph.AddPoint({0,0},0); graph.AddPoint({6,0},0);
    graph.AddInitial(Element::FromCoefficients(0,0,1,Type::Circle));
    graph.AddInitial(Element::FromCoefficients(6,0,4,Type::Circle));
    graph.goalPoints = {goal};
    if (unresolvedLine) graph.goalElements = {Element::FromCoefficients(0,1,123,Type::Line)};
    Seal(graph);
    return graph;
}
void HomothetyRootAndPaid() {
    // Unequal radii 1 and 2 at centers 0 and 6: external center -6,
    // internal center 2. No r31 near-tangent target line is used as exact data.
    for (Point target : {Point{-6,0},Point{2,0}}) {
        const Graph root = HomothetyFixture(target);
        Require(root.points.size() == 2 && !root.HasPoint(target),"similarity center must not be a given point");
        const auto run = Invoke(Family::Homothety,root,root,4,2,Goals);
        Require(run.accepted && run.prefix->elements.size()-root.elements.size() == 4,
                "homothety point goal needs four ordinary known-pair operations");
        Replay(root,*run.prefix,4,2,root.elements.size(),true);
        const Type expected[] = {Type::Line,Type::Circle,Type::Circle,Type::Line};
        for (size_t i = 0; i < size(expected); ++i)
            Require(run.prefix->elements[root.elements.size()+i].type == expected[i],"homothety L-C-C-L witness changed");
        Graph parent = root;
        Pay(parent,{0,0},{6,0},Type::Line);
        Require(parent.points.size() > root.points.size() && parent.pointBirth.back() == 1,"axis parent must contain real paid circle intersections");
        const auto paid = Invoke(Family::Homothety,root,parent,3,2,Goals);
        Require(paid.accepted && paid.prefix->elements.size()-root.elements.size() == 4,"paid homothety axis must leave three E, without resealing births");
        Replay(root,*paid.prefix,4,2,parent.elements.size(),true);
    }
    const Graph prefixRoot = HomothetyFixture({-6,0},true);
    const auto prefix = Invoke(Family::Homothety,prefixRoot,prefixRoot,4,2,
                              [](const Graph& g,int) { return g.HasPoint({-6,0}); });
    Require(prefix.accepted && !prefix.prefix->GoalsMet(),"homothety legitimate intermediate is not an eight-E tangent solution");
    const Graph pointRoot = HomothetyFixture();
    for (int budget : {0,1,2,3})
        Require(!Invoke(Family::Homothety,pointRoot,pointRoot,budget,2,Goals).accepted,"homothety generated a fake center within a sub-four-E budget");
    Graph missing = HomothetyFixture();
    missing.points.pop_back(); missing.pointBirth.pop_back(); missing.SetStateHashingEnabled(true);
    NoWork(Invoke(Family::Homothety,missing,missing,4,2,Goals));
}

Graph TangentFixture(double radius = 1, double distance = 3, double u = 1, double v = 0,
                     Point offset = {}, int side = 1, bool givenAxis = true) {
    Require(radius > 0 && distance > radius && (side == 1 || side == -1),"invalid generic tangent fixture");
    Graph graph;
    const Point o = Similarity({0,0},u,v,offset), h = Similarity({-distance,0},u,v,offset);
    graph.AddPoint(o,0); graph.AddPoint(h,0);
    graph.AddInitial(Element::FromCoefficients(o.x,o.y,radius*radius*(u*u+v*v),Type::Circle));
    // The near axial rim P is produced by the actual given line-circle
    // intersection, not by AddPoint of an analytically computed prerequisite.
    if (givenAxis) graph.AddInitial(Definition(o,h,Type::Line));
    // Analytical coefficients specify a goal ONLY; the witness must construct
    // a real known-pair line. Both orientations and non-axis-aligned cases run.
    const long double r = radius, d = distance, scale2 = static_cast<long double>(u)*u+static_cast<long double>(v)*v;
    const long double a = -r/d, b = side*sqrt(d*d-r*r)/d;
    const long double aa = (u*a-v*b)/scale2, bb = (v*a+u*b)/scale2;
    graph.goalElements = {Element::FromCoefficients(static_cast<double>(aa),static_cast<double>(bb),
        static_cast<double>(r+aa*offset.x+bb*offset.y),Type::Line)};
    Seal(graph);
    return graph;
}
void TangentResidual(const Element& line, const Element& circle) {
    Require(line.type == Type::Line && circle.type == Type::Circle,"tangency residual needs line and circle");
    const long double a = line.a, b = line.b, c = line.c;
    const long double residual = a*circle.a+b*circle.b-c;
    const long double distance2 = residual*residual/(a*a+b*b);
    // This is a binary64 roundoff sanity check, NOT high-precision or symbolic
    // proof. Independent decimal high-precision verification is a separate gate.
    const long double tolerance = 256*numeric_limits<double>::epsilon()*(1+abs(static_cast<long double>(circle.c)));
    Require(abs(distance2-circle.c) < tolerance,"constructed line only approximately resembles a tangent beyond roundoff");
}
void TangentFourPaidGeneric() {
    const array<double,6> cases[] = {
        {1,3,1,0,0,0}, {1.3,5.7,0,1,3,-2},
        {2.25,9.125,.6,.8,-1.25,2.5}, {.7,1.1,.8,-.6,2,-1}
    };
    for (const auto& t : cases) for (int side : {-1,1}) for (bool givenAxis : {true,false}) {
        const Graph root = TangentFixture(t[0],t[1],t[2],t[3],{t[4],t[5]},side,givenAxis);
        Graph parent = root;
        const Point o = parent.points[0], h = parent.points[1];
        if (!givenAxis) Pay(parent,o,h,Type::Line);
        const Point p = Similarity({-t[0],0},t[2],t[3],{t[4],t[5]});
        const Point middle{(o.x+h.x)/2,(o.y+h.y)/2};
        Require(parent.HasPoint(p) && !parent.HasPoint(middle) && !parent.GoalsMet(),
                "tangent fixture must contain real axial rim P but no free Thales midpoint");
        if (!givenAxis) Require(parent.pointBirth.back() == 1,"paid axis intersection lost its nonzero birth");
        const auto run = Invoke(Family::Tangent,root,parent,4,2,Goals);
        Require(run.accepted && run.prefix->elements.size()-parent.elements.size() == 4,
                "generic h>r tangent failed the four-paid-step doubling-chord route");
        Replay(root,*run.prefix,givenAxis ? 4 : 5,2,parent.elements.size(),true);
        const Graph& solved = *run.prefix;
        const size_t first = parent.elements.size();
        for (size_t i = 0; i < 3; ++i)
            Require(solved.elements[first+i].type == Type::Circle,"new four-E tangent route is C-C-C-L, not midpoint scaffolding");
        Require(solved.elements[first+3].type == Type::Line,"tangent final line must cost one ordinary E");
        Require(Exact(solved.elements[first],Definition(Stored(parent,o),Stored(parent,h),Type::Circle)),
                "first tangent circle must be C(O,H)");
        Require(Exact(solved.elements[first+1],Definition(Stored(parent,p),Stored(parent,h),Type::Circle)),
                "second tangent circle must use real near-axis rim P and H");
        Graph afterTwo = parent;
        for (size_t i = 0; i < 2; ++i)
            Require(afterTwo.Apply(solved.elements[first+i],static_cast<uint16_t>(afterTwo.elements.size()-afterTwo.initialElementCount+1)),
                    "tangent doubling-chord prefix replay failed");
        const Point w = Stored(afterTwo,{2*o.x-h.x,2*o.y-h.y});
        const Point actualP = Stored(parent,p);
        const Point vertex = Stored(afterTwo,{2*actualP.x-h.x,2*actualP.y-h.y});
        Require(Exact(solved.elements[first+2],Definition(w,vertex,Type::Circle)),
                "third tangent circle must be C(W,V), with both points actually generated");
        TangentResidual(solved.elements.back(),root.elements.front());
        const auto shortRun = Invoke(Family::Tangent,root,parent,3,2,Goals);
        Require(!shortRun.accepted && shortRun.stats.applied > 0,"remaining=3 must exercise and reject the incomplete paid tangent route");
    }
}
void ExternalHomothetyTangentEightE(const Input& input) {
    const Graph& root = input.graph;
    Require(input.limit == 8 && input.tools == 2 && root.points.size() == 2 && root.elements.size() == 2 &&
            root.goalElements.size() == 1 && root.goalPoints.empty() && !root.GoalsMet(),"wrong real external-tangent fixture");
    Require(root.elements[0].type == Type::Circle && root.elements[1].type == Type::Circle,"external fixture needs two circles");
    for (int variant = 0; variant < 3; ++variant) {
        const int totalBudget = variant == 2 ? 7 : 8;
        Graph parent = root;
        if (variant != 0) Pay(parent,root.points[0],root.points[1],Type::Line);
        const string before = Fingerprint(parent);
        const auto hash1 = parent.StateHash1(), hash2 = parent.StateHash2();
        const int leftAtParent = totalBudget-static_cast<int>(parent.elements.size()-root.elements.size());
        ParallelControl control; control.deadline = Clock::now()+chrono::milliseconds(200);
        SearchStats similarityStats, tangentStats;
        homothety_detail::Counts similarityCounts;
        uint64_t tangentApplied = 0, tangentSuccesses = 0;
        size_t similarityCallbacks = 0, tangentCallbacks = 0;
        SolutionCollector collector(1);
        const bool quota = homothety_detail::Probe(parent,leftAtParent,2,control,similarityStats,similarityCounts,control.deadline,
            [&](const Graph& similarity,int left) {
                ++similarityCallbacks;
                Replay(root,similarity,totalBudget,2,parent.elements.size());
                Require(similarity.elements.size()-root.elements.size() == 4 && left == totalBudget-4,
                        "homothety must pay exactly four E before tangent continuation");
                Require(!similarity.GoalsMet(),"homothety-only prefix was mistaken for the final common tangent");
                const string similarityBefore = Fingerprint(similarity);
                const auto sh1 = similarity.StateHash1(), sh2 = similarity.StateHash2();
                tangent_detail::Counts counts;
                const bool found = tangent_detail::Probe(similarity,left,2,control,tangentStats,counts,control.deadline,
                    [&](const Graph& result,int remaining) {
                        ++tangentCallbacks;
                        Replay(root,result,totalBudget,2,similarity.elements.size());
                        Require(result.elements.size()-similarity.elements.size()+static_cast<size_t>(remaining) == static_cast<size_t>(left),
                                "tangent callback remaining lost a paid step");
                        if (!result.GoalsMet()) return false; // A legal prefix alone is never submitted.
                        Replay(root,result,totalBudget,2,parent.elements.size(),true);
                        TangentResidual(result.elements.back(),root.elements[0]);
                        TangentResidual(result.elements.back(),root.elements[1]);
                        return collector.Submit(result,&control);
                    });
                tangentApplied += counts.applied; tangentSuccesses += counts.successes;
                Require(Fingerprint(similarity) == similarityBefore && similarity.StateHash1() == sh1 && similarity.StateHash2() == sh2,
                        "tangent mutated the paid homothety parent");
                return found;
            });
        Require(Fingerprint(parent) == before && parent.StateHash1() == hash1 && parent.StateHash2() == hash2,
                "external tangent integration mutated caller parent");
        Require(similarityStats.maxElements <= root.elements.size()+static_cast<size_t>(totalBudget) &&
                tangentStats.maxElements <= root.elements.size()+static_cast<size_t>(totalBudget) &&
                tangentApplied == tangentStats.applied,"external route search exceeded E or miscounted applied work");
        Require(similarityCallbacks > 0 && tangentApplied > 0,"external regression never exercised the chained construction");
        if (totalBudget == 7) {
            Require(!quota && collector.Count() == 0 && !control.stop.load() && !control.found.load() && !control.timedOut.load() &&
                    tangentSuccesses == 0,"incomplete seven-E route was submitted as an external tangent solution");
            continue; // This bounded heuristic result is Unknown, not an impossibility proof.
        }
        Require(quota && collector.Count() == 1 && control.stop.load() && control.found.load() && !control.timedOut.load() &&
                similarityCounts.successes == 1 && tangentSuccesses == 1 && tangentCallbacks > 0,
                "real external fixture homothety4+tangent4 did not submit a complete eight-E solution");
        const auto& entry = collector.Entries().front();
        Replay(root,entry.graph,8,2,parent.elements.size(),true);
        Require(entry.newElements.size() == 8 && entry.graph.elements.size()-root.elements.size() == 8,
                "external collector omitted paid homothety/tangent operations");
        const Type expected[] = {Type::Line,Type::Circle,Type::Circle,Type::Line,
                                 Type::Circle,Type::Circle,Type::Circle,Type::Line};
        for (size_t i = 0; i < size(expected); ++i)
            Require(entry.newElements[i].type == expected[i] && Exact(entry.newElements[i],entry.graph.elements[root.elements.size()+i]),
                    "external eight-E route is not raw L-C-C-L + C-C-C-L");
    }
}

array<pair<Family,Graph>,4> Fixtures() {
    return {{{Family::Diameter,DiameterFixture()},{Family::Mirror,MirrorFixture()},
             {Family::Homothety,HomothetyFixture()},{Family::Tangent,TangentFixture()}}};
}
int Budget(Family family) { return family == Family::Mirror ? 2 : 4; }
void GuardsAndCallbackContract() {
    for (const auto& [family,root] : Fixtures()) {
        const int budget = Budget(family);
        NoWork(Invoke(family,root,root,budget,2,Goals,{true,false,false,false}));
        NoWork(Invoke(family,root,root,budget,2,Goals,{false,true,false,false}));
        NoWork(Invoke(family,root,root,budget,2,Goals,{false,false,true,false}));
        NoWork(Invoke(family,root,root,0,2,Goals));
        NoWork(Invoke(family,root,root,-1,2,Goals));
        NoWork(Invoke(family,root,root,budget,1,Goals));
        if (family != Family::Mirror) NoWork(Invoke(family,root,root,budget,0,Goals));
        const auto declined = Invoke(family,root,root,budget,2,Decline,{false,false,false,true});
        Require(!declined.accepted && declined.callbacks == 1 && declined.stopped && declined.work.successes == 0,
                "callback false/atomic stop was misreported as accepted or ignored");
        const auto prefix = Invoke(family,root,root,budget,2,[](const Graph&,int) { return true; });
        Require(prefix.accepted && prefix.callbacks == 1 && !prefix.stopped,"plain true callback must mean prefix acceptance, not global quota");
    }
}

template<class F> void Rejects(F&& test, const string& why) {
    bool rejected = false;
    try { test(); } catch (const runtime_error&) { rejected = true; }
    Require(rejected,"witness negative control was accepted: " + why);
}
void WitnessNegativeControls() {
    const Graph root = MirrorFixture();
    const auto run = Invoke(Family::Mirror,root,root,2,2,Goals);
    Require(run.accepted,"negative controls need a legal mirror result");
    const Graph& valid = *run.prefix;
    Rejects([&] { Replay(root,valid,1); },"forgotten paid E");
    Graph forged = valid; forged.initialElementCount = forged.elements.size();
    Rejects([&] { Replay(root,forged,2); },"resealed paid prefix");
    forged = valid; forged.pointBirth.back() = 0;
    Rejects([&] { Replay(root,forged,2); },"reset paid birth to zero");
    forged = valid; forged.points.push_back({73,91}); forged.pointBirth.push_back(2);
    Rejects([&] { Replay(root,forged,2); },"free analytic point");
    forged = valid; forged.goalPoints.clear();
    Rejects([&] { Replay(root,forged,2); },"changed original goals");
    // EPS-equivalent target coefficients still are NOT a raw known-pair witness.
    Graph approximate;
    approximate.AddPoint({0,1+.75*EPS},0); approximate.AddPoint({2,1+.75*EPS},0); Seal(approximate);
    const Element target = Element::FromCoefficients(0,1,1,Type::Line);
    Require(SameElement(Definition(approximate.points[0],approximate.points[1],Type::Line),target) && !KnownPair(approximate,target),
            "witness checker confused EPS proximity with raw operation identity");
    forged = approximate; Require(forged.Apply(target,1),"near-target negative control setup");
    Rejects([&] { Replay(approximate,forged,1); },"substituted near target coefficients");
}
void InvalidParentsAndNonfinite() {
    for (const auto& [family,fixture] : Fixtures()) {
        for (int defect = 0; defect < 9; ++defect) {
            Graph parent = fixture;
            if (defect == 0) parent.initialElementCount = parent.elements.size()+1;
            if (defect == 1) parent.pointBirth.pop_back();
            if (defect == 2) parent.pointBirth.back() = 1;
            if (defect == 3) parent.points.back().x = numeric_limits<double>::infinity();
            if (defect == 4) {
                parent.points.push_back({numeric_limits<double>::quiet_NaN(),7}); parent.pointBirth.push_back(0);
            }
            if (defect == 5) parent.elements.front().c = numeric_limits<double>::infinity();
            if (defect == 6) parent.goalPoints.push_back({numeric_limits<double>::quiet_NaN(),0});
            if (defect == 7) parent.goalElements.push_back(Element::FromCoefficients(0,0,numeric_limits<double>::quiet_NaN(),Type::Circle));
            if (defect == 8) {
                Pay(parent,parent.points[0],parent.points[family == Family::Mirror ? 2 : 1],
                    family == Family::Homothety ? Type::Line : Type::Circle);
                parent.pointBirth[0] = 1; // Within paid depth, but followed by an older birth 0.
            }
            const string before = Fingerprint(parent);
            const auto h1 = parent.StateHash1(), h2 = parent.StateHash2();
            ParallelControl control; control.deadline = Clock::now()+chrono::seconds(1);
            SearchStats stats; Work work; bool callback = false, accepted = false;
            try {
                accepted = Dispatch(family,parent,Budget(family),2,control,stats,work,Clock::now()+chrono::milliseconds(30),
                                    [&](const Graph&,int) { callback = true; return true; });
            } catch (const invalid_argument&) {}
            Require(!accepted && !callback && stats.applied == 0,"invalid/nonfinite parent was used, family=" +
                    to_string(int(family)) + " defect=" + to_string(defect));
            Require(Fingerprint(parent) == before && parent.StateHash1() == h1 && parent.StateHash2() == h2 &&
                    !control.stop.load() && !control.found.load() && !control.timedOut.load(),"invalid parent rejection mutated state/control");
        }
    }
    // All inputs/element coefficients remain finite, but the almost-vertical
    // carrier's (a*a+b*b)*radiusSquared overflows during line-circle Apply.
    // A probe may decline this numerical case, never expose nonfinite points.
    const Graph extreme = MirrorFixture(1,1e80);
    const auto numeric = Invoke(Family::Mirror,extreme,extreme,2,0,Goals);
    Require(numeric.stats.rawCandidates > 0,"finite-overflow regression did not exercise an ordinary operation");
    for (const Point second : {Point{2*EPS,0},Point{1e200,0}}) {
        Graph root; root.AddPoint({0,0},0); root.AddPoint(second,0); Seal(root);
        const auto run = Invoke(Family::Diameter,root,root,5,2,[](const Graph&,int) { return true; });
        Require(!run.accepted && run.stats.applied == 0,"zero/overflow diameter radius must not create a paid element");
    }
}
} // namespace

int main(int argc, char** argv) {
    try {
        Require(argc >= 1 && argc <= 3,"usage: prerequisite_tests [ORTHIC_INPUT [EXTERNAL_TANGENT_INPUT]]");
        EPS = 1e-11;
        const auto start = Clock::now();
        const Input orthic = Load(argc >= 2 ? fs::absolute(argv[1]) : Fixture(argv[0]));
        const Input external = Load(argc == 3 ? fs::absolute(argv[2]) :
                                    (argc == 2 ? fs::absolute(argv[1]).parent_path()/"eu10_2_external_tangent.in" :
                                                 Fixture(argv[0],"eu10_2_external_tangent.in")));
        Require(orthic.limit == 8 && orthic.tools == 2 && orthic.graph.bounds.size() == 3,"wrong real Orthic fixture");
        const pair<const char*,function<void()>> tests[] = {
            {"diameter root / paid and given circles / exact GoalsMet",DiameterRootAndPaid},
            {"real Orthic diameter(BC) / two feet / four-E PREFIX ONLY",[&] { OrthicFourStepPrefix(orthic); }},
            {"real Orthic diameter + goal finish / all goals / paid eight-E collector",[&] { OrthicFinishedEightE(orthic); }},
            {"diameter finite segment != infinite carrier / fifth paid E",DiameterFiniteCarrier},
            {"mirror generic axes / two real circles / circle-only / remaining one E",MirrorGenericAndPaid},
            {"mirror optional paid join / disabled line / strict finite anchors",MirrorLineAndFiniteCarrier},
            {"homothety external and internal centers / four ordinary E / paid axis",HomothetyRootAndPaid},
            {"generic tangent h>r / C-C-C-L four E / given and paid axes / remaining three rejects",TangentFourPaidGeneric},
            {"real external tangent homothety4 + tangent4 / GoalsMet / all eight raw witnesses",[&] { ExternalHomothetyTangentEightE(external); }},
            {"tools / budget / local+global deadlines / atomic cancel / callback contract",GuardsAndCallbackContract},
            {"raw-bit witness negative controls / no free analytic points",WitnessNegativeControls},
            {"malformed parent / nonfinite / zero and overflowing radius",InvalidParentsAndNonfinite}
        };
        size_t passed = 0;
        for (const auto& [name,test] : tests) {
            try { test(); ++passed; cout << "PASS " << name << '\n'; }
            catch (const exception& e) { cerr << "FAIL " << name << ": " << e.what() << '\n'; }
        }
        const double elapsed = chrono::duration<double>(Clock::now()-start).count();
        cout << "prerequisite tests: " << passed << '/' << size(tests) << " groups; wall=" << elapsed
             << "s; prefix acceptance is NOT solution evidence; no long benchmark\n";
        Require(elapsed < 3.0,"short correctness suite exceeded three seconds");
        return passed == size(tests) ? 0 : 1;
    } catch (const exception& e) {
        cerr << "FAIL prerequisite tests: " << e.what() << '\n';
        return 1;
    }
}
