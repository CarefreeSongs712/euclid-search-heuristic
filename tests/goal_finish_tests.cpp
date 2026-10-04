#include "../src/goal_finish.hpp"
#include "../src/certificate.hpp"
#include <filesystem>

using namespace bs;
namespace {
using Clock = chrono::steady_clock;
namespace fs = std::filesystem;
namespace gf = bs::goal_finish_detail;

void Require(bool ok, const string& why) {
    if (!ok) throw runtime_error(why);
}
bool Exact(double a, double b) { return bit_cast<uint64_t>(a) == bit_cast<uint64_t>(b); }
void Seal(Graph& g) { g.initialElementCount = g.elements.size(); }

Graph Load(const fs::path& path) {
    ifstream in(path);
    Require(bool(in), "cannot open orthic fixture: " + path.string());
    Graph g;
    int limit = 0, mode = 0, counts[5]{}, goals[3]{};
    in >> limit >> mode;
    Require(limit == 8 && mode == 2, "wrong orthic fixture header");
    for (int& n : counts) { in >> n; Require(n >= 0, "negative input count"); }
    for (int i = 0; i < counts[0]; ++i) { Point p; in >> p.x >> p.y; g.AddPoint(p, 0); }
    for (int kind = 1; kind < 5; ++kind) for (int i = 0; i < counts[kind]; ++i) {
        double a = 0, b = 0, c = 0, d = 0;
        if (kind == 2 || kind == 3) {
            in >> a >> b >> c >> d;
            g.AddInitialBounded({a, b}, {c, d}, kind == 2 ? Type::Ray : Type::Segment);
        } else {
            in >> a >> b >> c;
            g.AddInitial(Element::FromCoefficients(a, b, kind == 4 ? c*c : c,
                                                  kind == 4 ? Type::Circle : Type::Line));
        }
    }
    Seal(g);
    for (int& n : goals) { in >> n; Require(n >= 0, "negative goal count"); }
    for (int kind = 0; kind < 2; ++kind) for (int i = 0; i < goals[kind]; ++i) {
        double a = 0, b = 0, c = 0;
        in >> a >> b >> c;
        g.goalElements.push_back(Element::FromCoefficients(a, b, kind ? c*c : c,
                                                          kind ? Type::Circle : Type::Line));
    }
    for (int i = 0; i < goals[2]; ++i) { Point p; in >> p.x >> p.y; g.goalPoints.push_back(p); }
    Require(bool(in), "incomplete fixture");
    Require(g.points.size() == 3 && g.elements.size() == 3 &&
            g.goalPoints.size() == 3 && g.goalElements.size() == 3, "wrong orthic fixture contents");
    return g;
}
fs::path FixturePath(const char* executable) {
    const fs::path suffix = fs::path("benchmarks") / "r32" / "cases" / "eu9_7_minimum_perimeter.in";
    for (fs::path base : {fs::path(__FILE__).parent_path().parent_path(),
                         fs::current_path(), fs::absolute(executable).parent_path()})
        for (int i = 0; i < 4 && !base.empty(); ++i, base = base.parent_path())
            if (fs::is_regular_file(base / suffix)) return fs::absolute(base / suffix);
    throw runtime_error("cannot locate orthic fixture; pass its path as the first argument");
}
uint32_t Known(const Graph& g, Point p) {
    for (uint32_t i = 0; i < g.points.size(); ++i)
        if (SamePoint(g.points[i], p)) return i;
    throw runtime_error("prefix witness is not actually known");
}
void Paid(Graph& g, Point p, Point q, bool circle) {
    const Element e = g.MakeCandidate(gf::KnownPair(Known(g, p), Known(g, q), circle));
    Require(gf::LegalFinite(e), "invalid prefix element");
    Require(g.Apply(e, static_cast<uint16_t>(g.elements.size() - g.initialElementCount + 1)),
            "repeated prefix element");
}
Graph OrthicPrefix(const Graph& root) {
    Graph g = root;
    // Old certificate endpoints are lookup keys only: never AddPoint them.
    Paid(g, {4.2, -0.2}, {-2.7, 0.1}, true);
    Paid(g, {-2.7, 0.1}, {4.2, -0.2}, true);
    Paid(g, {0.49019237886466849, -6.0255752861126277},
            {1.009807621135332, 5.9255752861126272}, false);
    Paid(g, {0.75000000000000022, -0.050000000000000003}, {-2.7, 0.1}, true);
    Require(g.elements.size() - g.initialElementCount == 4, "prefix must cost four E");
    return g;
}
string Fingerprint(const Graph& g) {
    ostringstream out;
    out << g.initialElementCount << ',' << g.gridMode << ',' << g.gridFast << ','
        << g.gridLinesReady << ',' << g.gridM << ',' << g.gridN << ';';
    auto point = [&](Point p) { out << bit_cast<uint64_t>(p.x) << ',' << bit_cast<uint64_t>(p.y) << ';'; };
    auto element = [&](Element e) {
        out << int(e.type) << ',' << e.bound << ',' << bit_cast<uint64_t>(e.a) << ','
            << bit_cast<uint64_t>(e.b) << ',' << bit_cast<uint64_t>(e.c) << ';';
    };
    for (Point p : g.points) point(p);
    out << '/';
    for (uint16_t b : g.pointBirth) out << b << ',';
    out << '/';
    for (Element e : g.elements) element(e);
    out << '/';
    for (Bound b : g.bounds) { point(b.p1); point(b.p2); }
    out << '/';
    for (Point p : g.goalPoints) point(p);
    out << '/';
    for (Element e : g.goalElements) element(e);
    return out.str();
}

// Independent endpoint arithmetic checks each paid witness before ordered replay.
Element Definition(Point p, Point q, Type type) {
    Element e;
    e.type = type;
    if (type == Type::Circle) {
        e.a = p.x; e.b = p.y;
        e.c = (p.x-q.x)*(p.x-q.x) + (p.y-q.y)*(p.y-q.y);
    } else {
        e.a = q.y-p.y; e.b = p.x-q.x; e.c = p.x*q.y-p.y*q.x;
        if (abs(e.b) >= EPS) { e.a /= e.b; e.c /= e.b; e.b = 1; }
        else if (abs(e.a) >= EPS) { e.c /= e.a; e.a = 1; e.b = 0; }
    }
    if (abs(e.a) < EPS) e.a = 0;
    if (abs(e.b) < EPS) e.b = 0;
    if (abs(e.c) < EPS) e.c = 0;
    return e;
}
bool HasWitness(const Graph& g, const Element& e) {
    for (uint32_t i = 0; i < g.points.size(); ++i)
        for (uint32_t j = i + 1; j < g.points.size(); ++j) {
            if (SamePoint(g.points[i], g.points[j])) continue;
            if (SameElementBits(Definition(g.points[i], g.points[j], e.type), e)) return true;
            if (e.type == Type::Circle &&
                SameElementBits(Definition(g.points[j], g.points[i], e.type), e)) return true;
        }
    return false;
}
void Verify(const Graph& root, const Graph& parent, const SolutionCollector& collector, int left, int tools) {
    for (const auto& entry : collector.Entries()) {
        const Graph& g = entry.graph;
        Require(g.GoalsMet(), "false positive goal");
        Require(g.initialElementCount == root.initialElementCount, "lost paid prefix");
        Require(g.elements.size() <= parent.elements.size() + size_t(left), "E overflow");
        Require(g.elements.size() >= parent.elements.size(), "lost parent elements");
        for (size_t i = 0; i < parent.elements.size(); ++i)
            Require(SameElementBits(g.elements[i], parent.elements[i]), "changed prefix");
        for (size_t i = 0; i < parent.points.size(); ++i)
            Require(Exact(g.points[i].x, parent.points[i].x) && Exact(g.points[i].y, parent.points[i].y) &&
                    g.pointBirth[i] == parent.pointBirth[i], "changed parent points/births");
        Graph replay = root;
        int extra = 0;
        for (size_t i = root.elements.size(); i < g.elements.size(); ++i) {
            const Element& e = g.elements[i];
            Require(gf::LegalFinite(e) && e.bound == NO_BOUND, "illegal paid element");
            Require(HasWitness(replay, e), "no bit-exact actual known-pair witness");
            if (i >= parent.elements.size()) {
                Require(tools != 0 || e.type == Type::Circle, "circle-only violation");
                Require(tools != 1 || e.type == Type::Line, "line-only violation");
                bool goal = false;
                for (const Element& target : replay.goalElements)
                    if (!replay.HasElement(target) && SameElement(e, target)) goal = true;
                if (!goal) ++extra;
            }
            Require(replay.Apply(e, static_cast<uint16_t>(i - root.initialElementCount + 1)), "repeat paid element");
        }
        Require(extra <= 1, "more than one auxiliary operation");
        Require(Fingerprint(replay) == Fingerprint(g), "result differs from ordered Apply replay");
        for (Point p : replay.points)
            Require(isfinite(p.x) && isfinite(p.y) && replay.PointAllowed(p), "invalid/outside known point");
        Require(entry.newElements.size() == g.elements.size() - root.initialElementCount,
                "collector omitted paid prefix");
        ostringstream cert;
        WriteConstructionCertificate(cert, root, g);
    }
}
struct Result {
    bool quota;
    size_t count;
    gf::Counts counts;
    double milliseconds;
};
Result Run(const Graph& root, const Graph& parent, int left, int tools,
           size_t quota = 1, double seconds = 2.0, ostream* certificate = nullptr) {
    const string before = Fingerprint(parent);
    const auto h1 = parent.StateHash1(), h2 = parent.StateHash2();
    SolutionCollector collector(quota);
    ParallelControl global;
    SearchStats stats;
    gf::Counts counts;
    ProgressSlot progress;
    progress.BeginTask(stats);
    const auto start = Clock::now();
    global.deadline = start + chrono::seconds(5);
    const bool full = gf::Find(parent, left, tools, collector, global, stats, counts,
        start + chrono::duration_cast<Clock::duration>(chrono::duration<double>(seconds)), &progress);
    const double ms = chrono::duration<double, milli>(Clock::now() - start).count();
    progress.EndTask(stats);
    Require(before == Fingerprint(parent) && h1 == parent.StateHash1() && h2 == parent.StateHash2(),
            "mutated const parent");
    Require(!global.timedOut, "local probe poisoned global timeout");
    Require(full == (collector.Count() >= quota), "incorrect quota status");
    Require(global.found == full && global.stop == full, "incorrect global flag status");
    Require(counts.successes == collector.SuccessfulVisits(), "incorrect successful visit statistics");
    Require(counts.applied == stats.applied && counts.candidates == stats.rawCandidates, "incorrect work statistics");
    Require(counts.goalCandidates <= gf::MaxGoalCandidates &&
            counts.auxiliaryCandidates <= gf::MaxAuxiliaryCandidates, "operation cap exceeded");
    Verify(root, parent, collector, left, tools);
    if (certificate && !collector.Entries().empty()) {
        *certificate << "{\"version\":\"v11\",\"eps\":" << setprecision(17) << EPS << ",\"solutions\":[";
        WriteConstructionCertificate(*certificate, root, collector.Entries().front().graph);
        *certificate << "]}\n";
    }
    return {full, collector.Count(), counts, ms};
}
void Report(const char* label, const Result& r) {
    cout << label << " quota=" << r.quota << " count=" << r.count << " ms=" << r.milliseconds
         << " candidates=" << r.counts.candidates << " goal=" << r.counts.goalCandidates
         << " aux=" << r.counts.auxiliaryCandidates << " previews=" << r.counts.previews
         << " replays=" << r.counts.replays << " applied=" << r.counts.applied
         << " visits=" << r.counts.successes << " caps=" << r.counts.capHits << '\n';
}
Graph LineRoot() {
    Graph g;
    g.AddPoint({0, 0}, 0); g.AddPoint({1, 0}, 0);
    g.goalElements.push_back(Element::FromCoefficients(0, 1, 0, Type::Line));
    return g;
}
void TestOrthic(const Graph& root, ostream* certificate) {
    Graph parent = OrthicPrefix(root);
    parent.SetStateHashingEnabled(true);
    const Result orthic = Run(root, parent, 4, 2, 1, 2.0, certificate);
    Report("orthic prefix4 +4 (E8)", orthic);
    Require(orthic.quota && orthic.counts.auxiliaryApplied != 0, "orthic E8 missing");
    const Result shortBudget = Run(root, parent, 3, 2);
    Report("orthic prefix4 +3 (Unknown)", shortBudget);
    Require(!shortBudget.quota && !shortBudget.counts.budgetLimited, "unexpected orthic E7 or incomplete short probe");
    const Result multi = Run(root, parent, 4, 2, 2);
    Report("orthic quota2", multi);
    Require(multi.quota && multi.count == 2 && multi.counts.successes > 2, "quota2/duplicate backtracking failed");
}
void TestTargetsAndTools() {
    Graph line = LineRoot();
    Require(Run(line, line, 1, 1).quota, "pure line failed");
    Require(!Run(line, line, 1, 0).quota, "compass produced line");
    line.goalPoints.push_back({.5, 0});
    Require(!Run(line, line, 1, 1).quota, "point on goal line added for free");
    line.goalPoints.clear();
    line.goalElements = {Element{.75*EPS, 1, 0, NO_BOUND, Type::Line},
                         Element{-.75*EPS, 1, 0, NO_BOUND, Type::Line}};
    Require(!SameElement(line.goalElements[0], line.goalElements[1]), "EPS fixture collapsed");
    Require(Run(line, line, 1, 1).quota, "one candidate must meet multiple EPS goals");

    Graph nearLine;
    nearLine.AddPoint({0, .75*EPS}, 0); nearLine.AddPoint({1, -.75*EPS}, 0);
    nearLine.goalElements = LineRoot().goalElements;
    Require(!Run(nearLine, nearLine, 1, 1).quota, "incidence pair accepted without verifying actual coefficients");
    nearLine.AddPoint({2, 0}, 0);
    const auto alternatives = Run(nearLine, nearLine, 1, 1);
    Require(alternatives.quota && alternatives.counts.goalCandidates >= 2, "stopped after first incidence pair");

    Graph circle;
    circle.AddPoint({0, 0}, 0); circle.AddPoint({1, 0}, 0);
    circle.goalElements.push_back(Element::FromCoefficients(0, 0, 1, Type::Circle));
    Require(Run(circle, circle, 1, 0).quota, "pure circle failed");
    Require(!Run(circle, circle, 1, 1).quota, "straightedge produced circle");
    circle.goalElements[0].c = 0;
    Require(!Run(circle, circle, 1, 2).quota, "zero circle accepted");
    circle.goalElements[0].c = numeric_limits<double>::infinity();
    Require(!Run(circle, circle, 1, 2).quota, "nonfinite circle accepted");
    Graph missingCenter;
    missingCenter.AddPoint({1, 0}, 0); missingCenter.AddPoint({-1, 0}, 0);
    missingCenter.goalElements.push_back(Element::FromCoefficients(0, 0, 1, Type::Circle));
    Require(!Run(missingCenter, missingCenter, 1, 0).quota, "circle center added for free");
    Graph nearCircle;
    nearCircle.AddPoint({.75*EPS, 0}, 0); nearCircle.AddPoint({1, 0}, 0);
    nearCircle.goalElements = missingCenter.goalElements;
    Require(!Run(nearCircle, nearCircle, 1, 0).quota, "circle radius not actually verified");

    Graph multi = LineRoot();
    multi.AddPoint({0, 1}, 0);
    const auto distinct = Run(multi, multi, 2, 1, 2);
    Report("goal-only hit then quota2", distinct);
    Require(distinct.quota && distinct.count == 2 && distinct.counts.successes > 2, "duplicate ended search early");
    Graph solved = LineRoot();
    solved.AddInitial(solved.goalElements.front()); Seal(solved);
    Require(Run(solved, solved, 0, 1).quota, "already solved root rejected at remaining=0");
}
void TestBoundedGeometry() {
    for (Type type : {Type::Ray, Type::Segment}) for (bool reaches : {false, true}) {
        Graph g = LineRoot();
        g.AddInitialBounded({.5, 1}, {.5, reaches ? -1.0 : 2.0}, type);
        Seal(g);
        g.goalPoints.push_back({.5, 0});
        Require(Run(g, g, 1, 1).quota == reaches, "bounded intersection range ignored");
    }
    for (bool fast : {false, true}) {
        Graph grid;
        grid.gridMode = true; grid.gridFast = fast; grid.gridM = 2; grid.gridN = 2;
        grid.AddAutomaticGridLines(); Seal(grid);
        grid.goalElements.push_back(Element::FromCoefficients(-.5, 1, 0, Type::Line));
        grid.goalPoints.push_back({1, .5});
        Require(Run(grid, grid, 1, 1).quota, "grid direct goal failed");
        grid.goalPoints.push_back({3, 3});
        Require(!Run(grid, grid, 1, 1).quota, "outside-grid point accepted");
    }
}
Graph DenseTargets() {
    Graph g;
    for (int y = 0; y < 8; ++y) {
        for (int x = 0; x < 4; ++x) g.AddPoint({double(x), double(y)}, 0);
        g.goalElements.push_back(Element::FromCoefficients(0, 1, y, Type::Line));
    }
    return g;
}
void TestCapsAndControls(const Graph& parent) {
    const Graph dense = DenseTargets();
    const Result capped = Run(dense, dense, 6, 1, 1, 5.0);
    Report("goal operation cap", capped);
    Require(!capped.quota && capped.counts.goalCandidates == gf::MaxGoalCandidates &&
            capped.counts.capHits && capped.counts.budgetLimited, "goal cap or no-missing-count-prune failed");
    Graph wide;
    for (int x = 0; x < 128; ++x) wide.AddPoint({double(x), 0}, 0);
    wide.goalElements.push_back(Element::FromCoefficients(0, 1, 1, Type::Line));
    const Result auxiliaryCap = Run(wide, wide, 1, 1);
    Require(!auxiliaryCap.quota && auxiliaryCap.counts.auxiliaryCandidates == gf::MaxAuxiliaryCandidates &&
            auxiliaryCap.counts.capHits && auxiliaryCap.counts.budgetLimited, "auxiliary cap failed");
    wide.AddPoint({128, 0}, 0);
    Require(Run(wide, wide, 1, 1).counts.gated == 1, "parent point gate failed");
    Graph tooMany = LineRoot();
    tooMany.goalElements.resize(9, tooMany.goalElements.front());
    Require(Run(tooMany, tooMany, 1, 1).counts.gated == 1, "goal count gate failed");
    tooMany = LineRoot();
    for (int x = 0; x < 97; ++x) tooMany.AddInitial(Element::FromCoefficients(1, 0, x, Type::Line));
    Seal(tooMany);
    Require(Run(tooMany, tooMany, 1, 1).counts.gated == 1, "parent element gate failed");

    ParallelControl control;
    SolutionCollector collector(1);
    SearchStats stats;
    gf::Counts counts;
    auto call = [&](const Graph& g, int left, int tools = 2) {
        return gf::Find(g, left, tools, collector, control, stats, counts, Clock::now() + chrono::seconds(2));
    };
    auto invalid = [&](const Graph& g, int left, int tools = 2) {
        bool threw = false;
        try { call(g, left, tools); } catch (const invalid_argument&) { threw = true; }
        Require(threw, "invalid paid-parent/budget/tool input accepted");
    };
    Graph bad = parent;
    bad.initialElementCount = bad.elements.size() + 1; invalid(bad, 4);
    bad = parent; bad.pointBirth.pop_back(); invalid(bad, 4);
    bad = parent; bad.pointBirth.back() = 5; invalid(bad, 4);
    bad = parent; bad.pointBirth.front() = 1; invalid(bad, 4);
    invalid(parent, 65532); invalid(parent, -1); invalid(parent, 4, 3);
    Require(!call(parent, 7) && counts.gated == 1, "remaining gate failed");
    control.stop = true;
    Require(!call(parent, 4) && counts.cancellations == 1, "ignored preset cancellation");
    control.stop = false;
    control.deadline = Clock::now() - chrono::seconds(1);
    Require(!call(parent, 4), "ignored global deadline");
    Require(!control.stop && !control.timedOut && !control.found, "global deadline changed shared flags");
    control.deadline = {};
    Require(!gf::Find(parent, 4, 2, collector, control, stats, counts, Clock::now() - chrono::seconds(1)),
            "ignored local deadline");
    Require(!control.stop && !control.timedOut && !control.found && counts.timeouts == 2,
            "local deadline changed shared flags");
    Require(Run(dense, dense, 6, 1, 1, .0001).counts.timeouts == 1, "in-flight local budget ignored");
    thread canceller([&] { this_thread::sleep_for(chrono::milliseconds(1)); control.stop.store(true, memory_order_release); });
    bool quota = false;
    try { quota = call(dense, 6, 1); } catch (...) { canceller.join(); throw; }
    canceller.join();
    Require(!quota && !control.found && !control.timedOut && collector.Count() == 0,
            "in-flight cancellation corrupted collector/control");
}
} // namespace

int main(int argc, char** argv) {
    try {
        const fs::path fixture = argc > 1 ? fs::path(argv[1]) : FixturePath(argv[0]);
        const Graph root = Load(fixture);
        ofstream certificate;
        if (argc > 2) { certificate.open(argv[2]); Require(bool(certificate), "cannot open certificate output"); }
        cout << "Fixture: " << fixture.string() << '\n';
        TestOrthic(root, argc > 2 ? &certificate : nullptr);
        TestTargetsAndTools();
        TestBoundedGeometry();
        TestCapsAndControls(OrthicPrefix(root));
        cout << "goal_finish_tests: all checks passed\n";
        return 0;
    } catch (const exception& e) {
        cerr << "goal_finish_tests: " << e.what() << '\n';
        return 1;
    }
}
