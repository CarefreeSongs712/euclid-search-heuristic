#include "../src/solver.hpp"
#include <cstring>

// Optional differential/microbenchmark build against an unmodified solver.hpp:
// -DBS_BASELINE_SOLVER_HEADER='"C:/.../solver_before.hpp"' -I../src
#ifdef BS_BASELINE_SOLVER_HEADER
namespace baseline {
using namespace ::bs;
#include BS_BASELINE_SOLVER_HEADER
}
#endif

namespace bs {
struct SolverTestAccess {
    static void Initialize(Solver& s, size_t depth = 4) {
        s.scratch_.resize(depth);
        s.deadline_ = chrono::steady_clock::now() + chrono::hours(1);
    }
    static optional<Candidate> ReverseLine(Solver& s, const Graph& g,
                                           const Element& line) {
        Initialize(s);
        SearchStats stats;
        return s.FindReverseLineRepresentation(g, line, 0, nullopt, stats);
    }
    static void CheckContextReuse() {
        Solver s(2, false, true, true, 4096, 0, 30);
        Initialize(s);
        Graph g;
        g.AddPoint({0, 0}, 0);
        g.goalElements.push_back(Element::FromCoefficients(0, 1, 0, Type::Line));
        const auto& first = s.BuildPreApplyContext(g, 2, 0);
        assert(first.previewGoal == 0 && first.knownLineHits == 1);
        g.goalElements.clear();
        g.goalPoints.push_back({3, 4});
        const auto& second = s.BuildPreApplyContext(g, 3, 0);
        assert(second.previewGoal == -1 && second.knownLineHits == 0);
        assert(second.missingGoals.empty() && second.missingGoalPoints.size() == 1);
        for (int k = 0; k != 8; ++k) {
            const Element e = Element::FromCoefficients(k, 1, k - 2, Type::Line);
            assert(s.GoalPriority(g, e, second) == g.GoalPriority(e));
        }
        // Do not collapse non-transitive approximate equality in the priority
        // cache: A~B and B~C do not imply A~C.
        g.goalElements = {Element::FromCoefficients(0, 1, 2, Type::Line),
                         Element::FromCoefficients(0, 1, 2 + .75 * EPS, Type::Line)};
        const Element candidate = Element::FromCoefficients(0, 1, 2 + 1.5 * EPS, Type::Line);
        const auto& third = s.BuildPreApplyContext(g, 3, 0);
        assert(third.missingGoals.size() == 2);
        assert(s.GoalPriority(g, candidate, third) == 2);
    }
};
}

using namespace bs;

static bool ExactElement(const Element& a, const Element& b) {
    return bit_cast<uint64_t>(a.a) == bit_cast<uint64_t>(b.a) &&
           bit_cast<uint64_t>(a.b) == bit_cast<uint64_t>(b.b) &&
           bit_cast<uint64_t>(a.c) == bit_cast<uint64_t>(b.c) &&
           a.type == b.type && a.bound == b.bound;
}

static Graph NearLineGraph() {
    Graph g;
    g.AddPoint({0, 1 + .6 * EPS}, 0);
    g.AddPoint({.1, 1 - .6 * EPS}, 0);
    g.AddPoint({2, 1}, 0);
    g.AddPoint({3, 1}, 0);
    return g;
}

static void Regressions() {
    SolverTestAccess::CheckContextReuse();
    const Element line = Element::FromCoefficients(0, 1, 1, Type::Line);
    {
        Graph g = NearLineGraph();
        assert(!SameElement(g.MakeCandidate({0, 1, 2}), line));
        Solver s(1, false, true, true, 4096, 0, 30);
        auto candidate = SolverTestAccess::ReverseLine(s, g, line);
        assert(candidate && SameElement(g.MakeCandidate(*candidate), line));
    }
    for (bool lowMemory : {false, true}) {
        // v9: an incident first pair may not construct the goal line.
        Graph g = NearLineGraph();
        g.goalElements.push_back(line);
        Solver s(1, false, true, lowMemory, 4096, 0, 30);
        SearchStats stats;
        assert(s.Search(g, 1, stats) && g.GoalsMet());
        // v9: ALL missing goal points are forced even when two supporting
        // initial elements happen to be parallel within intersection EPS.
        Graph points;
        points.AddPoint({0, 0}, 0);
        points.goalPoints = {{2, 3}, {4, 5}};
        points.AddInitial(Element::FromCoefficients(0, 1, 3, Type::Line));
        points.AddInitial(Element::FromCoefficients(.2 * EPS, 1, 3, Type::Line));
        vector<uint32_t> forced;
        points.CollectForcedTailPointIndices(1, forced);
        assert(forced == vector<uint32_t>({0, 1}));
    }
    {
        // End-to-end one-step reverse regression. The first two EPS-incident
        // points define the wrong line; a later verified pair must be tried.
        Graph g = NearLineGraph();
        g.AddInitial(Element::FromCoefficients(1, 0, 4, Type::Line));
        g.AddInitial(Element::FromCoefficients(1, 0, 5, Type::Line));
        g.goalPoints = {{4, 1}, {5, 1}};
#ifdef BS_BASELINE_SOLVER_HEADER
        Graph oldGraph = g;
        baseline::bs::Solver oldSolver(1, false, true, true, 4096, 0, 30);
        SearchStats oldStats;
        assert(!oldSolver.Search(oldGraph, 1, oldStats));
#endif
        Solver s(1, false, true, true, 4096, 0, 30);
        SearchStats stats;
        assert(s.Search(g, 1, stats) && g.GoalsMet());
    }
    {
        // A worker reuses the same Solver across independent prefix tasks.
        // Compare to a fresh solver after replacing goals and point counts.
        Solver reused(2, false, true, true, 64, 0, 30);
        for (int iteration = 0; iteration < 12; ++iteration) {
            Graph input;
            input.AddPoint({0, 0}, 0);
            input.AddPoint({1, 0}, 0);
            input.AddPoint({0, 1}, 0);
            if (iteration % 2) input.AddPoint({1, 1}, 0);
            input.goalPoints = {{.5, .5}};
            if (iteration % 3 == 0)
                input.goalElements = {Element::FromCoefficients(0, 1, 0, Type::Line)};
            PrefixTask task;
            task.prefix.push_back(input.MakeCandidate({0, 1, 2}));
            Graph a = input, b = input;
            Solver fresh(2, false, true, true, 64, 0, 30);
            SearchStats sa, sb;
            const bool ra = reused.SearchPrefixTask(a, 3, task, sa, nullptr);
            const bool rb = fresh.SearchPrefixTask(b, 3, task, sb, nullptr);
            assert(ra == rb && sa.nodes == sb.nodes && sa.applied == sb.applied);
            assert(a.elements.size() == b.elements.size());
            for (size_t i = 0; i < a.elements.size(); ++i)
                assert(ExactElement(a.elements[i], b.elements[i]));
        }
    }
    cout << "solver regressions passed\n";
}

#ifdef BS_BASELINE_SOLVER_HEADER
static Graph BenchmarkGraph(size_t n, bool goals = true) {
    Graph g;
    for (size_t i = 0; i < n; ++i) {
        const double x = static_cast<double>((i * 17) % 101) / 7;
        const double y = static_cast<double>((i * i * 13 + i * 7) % 103) / 11;
        g.AddPoint({x, y}, 0);
    }
    g.AddInitial(Element::FromCoefficients(0, 1, 2.12345, Type::Line));
    if (goals) g.goalPoints = {{1.123, 2.12345}, {2.7182818, 3.1415926}};
    else g.goalElements = {Element::FromCoefficients(1, 1, 1000, Type::Line)};
    g.initialElementCount = g.elements.size();
    return g;
}

struct Trace {
    vector<vector<Element>> prefixes;
    SearchStats stats;
    double ms = 0;
};
template<class SolverType>
static Trace RunTrace(const Graph& input, bool lowMemory, bool goalFirst,
                      bool symmetry, size_t dedup, int tools, uint16_t split = 1) {
    Graph g = input;
    SolverType solver(tools, symmetry, goalFirst, lowMemory, dedup, 0, 60);
    Trace trace;
    function<bool(const Graph&)> sink = [&](const Graph& node) {
        trace.prefixes.emplace_back(node.elements.begin() + input.elements.size(), node.elements.end());
        return true;
    };
    const auto begin = chrono::steady_clock::now();
    solver.ProduceFrontierTasks(g, 3, trace.stats, nullptr, sink, split);
    trace.ms = chrono::duration<double, milli>(chrono::steady_clock::now() - begin).count();
    assert(!solver.TimedOut());
    assert(g.points.size() == input.points.size() && g.elements.size() == input.elements.size());
    return trace;
}
static void Compare(const Trace& before, const Trace& after) {
    assert(before.prefixes.size() == after.prefixes.size());
    for (size_t i = 0; i < before.prefixes.size(); ++i) {
        assert(before.prefixes[i].size() == after.prefixes[i].size());
        for (size_t j = 0; j < before.prefixes[i].size(); ++j)
            assert(ExactElement(before.prefixes[i][j], after.prefixes[i][j]));
    }
    assert(before.stats.nodes == after.stats.nodes);
    assert(before.stats.applied == after.stats.applied);
    assert(before.stats.uniqueCandidates == after.stats.uniqueCandidates);
    assert(before.stats.duplicateCandidates == after.stats.duplicateCandidates);
}
static void Differential() {
    size_t checks = 0;
    for (size_t n : {size_t(6), size_t(20), size_t(60)}) {
        for (bool lowMemory : {false, true}) for (bool first : {false, true}) {
            for (int tools : {0, 1, 2}) {
                const Graph g = BenchmarkGraph(n);
                for (size_t dedup : {size_t(0), size_t(64)}) {
                    Compare(RunTrace<baseline::bs::Solver>(g, lowMemory, first, false, dedup, tools),
                            RunTrace<Solver>(g, lowMemory, first, false, dedup, tools));
                    ++checks;
                }
            }
        }
    }
    for (bool lowMemory : {false, true}) for (bool symmetry : {false, true}) {
        const Graph g = BenchmarkGraph(6);
        Compare(RunTrace<baseline::bs::Solver>(g, lowMemory, true, symmetry, 64, 2, 2),
                RunTrace<Solver>(g, lowMemory, true, symmetry, 64, 2, 2));
        ++checks;
    }
    // Exercise all three nonempty priority bands, rather than only the usual
    // mostly auxiliary random pairs, including repeated EPS-near target goals.
    for (bool lowMemory : {false, true}) for (size_t n : {size_t(8), size_t(60)}) {
        Graph g = BenchmarkGraph(n);
        const Point a = g.points[0], b = g.points[1];
        g.goalPoints = {{(a.x + b.x) / 2, (a.y + b.y) / 2}};
        g.goalElements = {g.MakeCandidate({2, 3, 2})};
        Element nearGoal = g.goalElements[0];
        nearGoal.c += .75 * EPS;
        g.goalElements.push_back(nearGoal);
        Compare(RunTrace<baseline::bs::Solver>(g, lowMemory, true, false, 64, 2),
                RunTrace<Solver>(g, lowMemory, true, false, 64, 2));
        ++checks;
    }
    // Initial rays/segments and grid fast-path must not change the accepted
    // prefix order.
    for (bool lowMemory : {false, true}) {
        Graph g;
        g.gridMode = true;
        g.gridM = 3;
        g.gridN = 3;
        g.AddAutomaticGridLines();
        g.AddInitialBounded({0, 0}, {2, 3}, Type::Segment);
        g.AddInitialBounded({3, 0}, {1, 3}, Type::Ray);
        g.goalPoints = {{1.234567, 1.234567}};
        g.initialElementCount = g.elements.size();
        Compare(RunTrace<baseline::bs::Solver>(g, lowMemory, true, false, 64, 2),
                RunTrace<Solver>(g, lowMemory, true, false, 64, 2));
        ++checks;
    }
    cout << "bitwise ordered-prefix differential checks=" << checks << " passed\n";
    for (size_t n : {size_t(20), size_t(60)}) for (bool lowMemory : {false, true}) {
        const Graph g = BenchmarkGraph(n);
        vector<double> beforeTimes, afterTimes;
        Trace before, after;
        for (int i = 0; i < 15; ++i) {
            before = RunTrace<baseline::bs::Solver>(g, lowMemory, true, false, 64, 2);
            after = RunTrace<Solver>(g, lowMemory, true, false, 64, 2);
            Compare(before, after);
            beforeTimes.push_back(before.ms);
            afterTimes.push_back(after.ms);
        }
        sort(beforeTimes.begin(), beforeTimes.end());
        sort(afterTimes.begin(), afterTimes.end());
        cout << "micro n=" << n << " stream=" << lowMemory
             << " baseline_ms=" << beforeTimes[7] << " optimized_ms=" << afterTimes[7]
             << " speedup=" << beforeTimes[7] / afterTimes[7]
             << " raw=" << before.stats.rawCandidates << "->" << after.stats.rawCandidates
             << " prefixes=" << after.prefixes.size() << '\n';
    }
}
static void FullBenchmark(const char* path) {
    ifstream in(path);
    int limit, mode;
    in >> limit >> mode;
    Graph input;
    if (mode == 3) {
        input.gridMode = true;
        in >> input.gridM >> input.gridN;
    }
    int counts[5];
    for (int& n : counts) in >> n;
    input.Reserve(static_cast<size_t>(counts[1] + counts[2] + counts[3] + counts[4] + limit));
    for (int i = 0; i < counts[0]; ++i) {
        Point p;
        in >> p.x >> p.y;
        input.AddPoint(p, 0);
    }
    if (mode == 3) input.AddAutomaticGridLines();
    for (int type = 1; type <= 4; ++type) for (int i = 0; i < counts[type]; ++i) {
        double a, b, c, d;
        if (type == 2 || type == 3) {
            in >> a >> b >> c >> d;
            input.AddInitialBounded({a, b}, {c, d}, type == 2 ? Type::Ray : Type::Segment);
        } else {
            in >> a >> b >> c;
            input.AddInitial(Element::FromCoefficients(a, b, type == 4 ? Sq(c) : c,
                                                      type == 4 ? Type::Circle : Type::Line));
        }
    }
    input.initialElementCount = input.elements.size();
    int goals[3];
    for (int& n : goals) in >> n;
    for (int type = 0; type < 2; ++type) for (int i = 0; i < goals[type]; ++i) {
        double a, b, c;
        in >> a >> b >> c;
        input.goalElements.push_back(Element::FromCoefficients(a, b, type == 1 ? Sq(c) : c,
                                                               type == 1 ? Type::Circle : Type::Line));
    }
    for (int i = 0; i < goals[2]; ++i) {
        Point p;
        in >> p.x >> p.y;
        input.goalPoints.push_back(p);
    }
    if (!in) throw runtime_error("Invalid benchmark input");
    auto run = [&]<class SolverType>(const char* name) {
        Graph g = input;
        SolverType solver(mode == 3 ? 2 : mode, true, true, true, 2048, 0, 120);
        SearchStats stats;
        const auto begin = chrono::steady_clock::now();
        const bool result = solver.Search(g, limit, stats);
        const double seconds = chrono::duration<double>(chrono::steady_clock::now() - begin).count();
        cout << name << " result=" << result << " timeout=" << solver.TimedOut()
             << " seconds=" << seconds << " nodes=" << stats.nodes
             << " applied=" << stats.applied << " raw=" << stats.rawCandidates << '\n';
        assert(!solver.TimedOut());
        return pair(result, stats.nodes);
    };
    const auto before = run.template operator()<baseline::bs::Solver>("baseline");
    const auto after = run.template operator()<Solver>("optimized");
    assert(before.first == after.first);
}
#endif

int main(int argc, char** argv) {
#ifdef BS_BASELINE_SOLVER_HEADER
    if (argc == 2) {
        FullBenchmark(argv[1]);
        return 0;
    }
#else
    (void)argc;
    (void)argv;
#endif
    Regressions();
#ifdef BS_BASELINE_SOLVER_HEADER
    Differential();
#endif
}
