#ifdef NDEBUG
#undef NDEBUG // These tests use assert even in a Release build.
#endif
#include "../src/geometry.hpp"
#include <random>

using namespace bs;

#if defined(__GNUC__) || defined(__clang__)
#define TEST_NOINLINE __attribute__((noinline))
#elif defined(_MSC_VER)
#define TEST_NOINLINE __declspec(noinline)
#else
#define TEST_NOINLINE
#endif

static bool ScalarHasPoint(const Graph& g, const Point& p) {
    if (g.gridMode && g.gridFast && g.gridLinesReady && EPS < 0.5 && g.PointAllowed(p) &&
        p.x == static_cast<int>(p.x) && p.y == static_cast<int>(p.y)) return true;
    for (const Point& old : g.points) if (SamePoint(old, p)) return true;
    return false;
}

static bool ScalarAddPoint(Graph& g, const Point& p, uint16_t birth) {
    if (!g.PointAllowed(p)) return false;
    if (g.gridMode && g.gridFast && g.gridLinesReady && EPS < 0.5 &&
        p.x == static_cast<int>(p.x) && p.y == static_cast<int>(p.y)) return false;
    for (const Point& old : g.points) if (SamePoint(old, p)) return false;
    g.points.push_back(p);
    g.pointBirth.push_back(birth);
    return true;
}

static void AssertSamePoints(const Graph& a, const Graph& b) {
    assert(a.points.size() == b.points.size());
    assert(a.pointBirth == b.pointBirth);
    for (size_t i = 0; i < a.points.size(); ++i) {
        assert(bit_cast<uint64_t>(a.points[i].x) == bit_cast<uint64_t>(b.points[i].x));
        assert(bit_cast<uint64_t>(a.points[i].y) == bit_cast<uint64_t>(b.points[i].y));
    }
}

static void TestPointSequences() {
    mt19937_64 rng(0x71ae3436ULL);
    uniform_real_distribution<double> unit(-10.0, 10.0);
    for (double eps : {1e-11, 1e-13, 0.125}) {
        EPS = eps;
        Graph fast, ref;
        vector<Point> input{
            {0.0, -0.0}, {-0.0, 0.0},
            {nextafter(eps, 0.0), 0.0}, {eps, 0.0},
            {nextafter(eps, numeric_limits<double>::infinity()), 0.0},
            {0.0, eps}, {0.0, -eps}, {eps, eps},
            {10.0, 10.0}, {10.0 + 0.75 * eps, 10.0},
            {10.0 + 1.5 * eps, 10.0},
            {numeric_limits<double>::quiet_NaN(), 0.0},
            {0.0, numeric_limits<double>::quiet_NaN()},
            {numeric_limits<double>::infinity(), 0.0},
            {-numeric_limits<double>::infinity(), 0.0},
            {0.0, numeric_limits<double>::infinity()},
            {numeric_limits<double>::max(), -numeric_limits<double>::max()},
            {numeric_limits<double>::denorm_min(), -numeric_limits<double>::denorm_min()}
        };
        for (size_t i = 0; i < 1200; ++i) {
            Point p{unit(rng), unit(rng)};
            if (i % 3 == 0 && !input.empty()) {
                p = input[static_cast<size_t>(rng() % input.size())];
                p.x += (static_cast<int>(rng() % 7) - 3) * 0.5 * EPS;
                p.y += (static_cast<int>(rng() % 7) - 3) * 0.5 * EPS;
            }
            input.push_back(p);
        }
        for (size_t i = 0; i < input.size(); ++i) {
            const Point p = input[i];
            assert(fast.HasPoint(p) == ScalarHasPoint(ref, p));
            assert(fast.AddPoint(p, static_cast<uint16_t>(i % 20)) ==
                   ScalarAddPoint(ref, p, static_cast<uint16_t>(i % 20)));
            AssertSamePoints(fast, ref);
        }
        // Every SIMD lane, partial final block, and early-hit position.
        for (size_t n = 0; n <= 67; ++n) {
            Graph g;
            for (size_t i = 0; i < n; ++i) g.AddPoint({double(i), double(i % 5)}, 0);
            for (size_t i = 0; i < n; ++i) {
                for (const Point p : {g.points[i], Point{double(i), -1.0},
                                     Point{double(i) + EPS, double(i % 5)}})
                    assert(g.HasPoint(p) == ScalarHasPoint(g, p));
            }
            assert(!g.HasPoint({-2.0, -3.0}));
        }
    }
    EPS = 1e-11;
}

static void TestGridAndRollback() {
    for (bool gridFast : {false, true}) {
        Graph fast;
        fast.gridMode = true;
        fast.gridFast = gridFast;
        fast.gridM = 4;
        fast.gridN = 3;
        fast.AddAutomaticGridLines();
        Graph ref = fast;
        for (const Point p : vector<Point>{{0, 0}, {4, 3}, {1.25, 2.75},
                {-EPS / 2, 1}, {4 + EPS / 2, 1}, {1, -EPS / 2},
                {1, 3 + EPS / 2}, {numeric_limits<double>::quiet_NaN(), 0},
                {numeric_limits<double>::infinity(), 0}}) {
            assert(fast.HasPoint(p) == ScalarHasPoint(ref, p));
            assert(fast.AddPoint(p, 1) == ScalarAddPoint(ref, p, 1));
        }
        AssertSamePoints(fast, ref);
    }
    Graph g;
    g.AddPoint({-1, 0}, 0); g.AddPoint({1, 0}, 0);
    g.Apply(Element::FromCoefficients(0, 1, 0, Type::Line), 0);
    g.SetStateHashingEnabled(true);
    const auto mark = g.GetMark();
    const uint64_t h1 = g.StateHash1(), h2 = g.StateHash2();
    const Graph original = g;
    g.Apply(Element::FromCoefficients(0, 0, 4, Type::Circle), 1);
    g.Apply(Element::FromCoefficients(1, 0, 0, Type::Line), 2);
    assert(g.HasPoint({0, 2}) && g.HasPoint({0, -2}));
    g.Rollback(mark);
    assert(g.StateHash1() == h1 && g.StateHash2() == h2);
    AssertSamePoints(g, original);
    assert(!g.HasPoint({0, 2}));
    Graph copied = g;
    assert(copied.AddPoint({7, 8}, 3));
    assert(!g.HasPoint({7, 8}));
    Graph moved = std::move(copied);
    assert(moved.HasPoint({7, 8}));
}

static void TestApplyOrder() {
    Graph fast, ref;
    fast.AddPoint({-2, 0}, 0); fast.AddPoint({2, 0}, 0);
    ref = fast;
    vector<Element> operations{
        Element::FromCoefficients(0, 0, 4, Type::Circle),
        Element::FromCoefficients(0, 1, 0, Type::Line),
        Element::FromCoefficients(1, 0, 0, Type::Line),
        Element::FromCoefficients(1, 0, 4, Type::Circle),
        Element::FromCoefficients(0, 1, 2, Type::Line),
        Element::FromCoefficients(0, 1, 2 + EPS / 16, Type::Line),
        Element::FromCoefficients(1, 1, 1, Type::Line)
    };
    for (size_t i = 0; i < operations.size(); ++i) {
        const auto& e = operations[i];
        for (const auto& old : ref.elements) ref.VisitIntersections(e, old, [&](const Point& p) {
            ScalarAddPoint(ref, p, static_cast<uint16_t>(i + 1)); return false;
        });
        ref.elements.push_back(e);
        fast.ApplyKnownNew(e, static_cast<uint16_t>(i + 1));
        AssertSamePoints(fast, ref);
    }
    // v9 guard: coincident ray + paid supporting line do not remove a missing
    // point from the one-operation forced-point set.
    Graph bounded;
    bounded.AddInitialBounded({0, 0}, {1, 0}, Type::Ray);
    bounded.AddInitial(Element::FromCoefficients(0, 1, 0, Type::Line));
    bounded.goalPoints.push_back({2, 0});
    vector<uint32_t> forced;
    bounded.CollectForcedTailPointIndices(1, forced);
    assert(forced == vector<uint32_t>{0});
}

TEST_NOINLINE static uint64_t ScanScalar(const Graph& g, const vector<Point>& probes,
                                        size_t repeats) {
    uint64_t hits = 0;
    for (size_t r = 0; r < repeats; ++r)
        for (const Point& p : probes) hits += ScalarHasPoint(g, p);
    return hits;
}
TEST_NOINLINE static uint64_t ScanOptimized(const Graph& g, const vector<Point>& probes,
                                           size_t repeats) {
    uint64_t hits = 0;
    for (size_t r = 0; r < repeats; ++r)
        for (const Point& p : probes) hits += g.HasPoint(p);
    return hits;
}

static void Benchmark() {
    uint64_t checksum = 0;
    for (size_t n : {8u, 16u, 32u, 128u, 512u, 2048u}) {
        for (const string mode : {"miss", "hit", "same-x-miss", "all-same-x", "first-hit"}) {
            Graph g;
            for (size_t i = 0; i < n; ++i)
                g.AddPoint({mode == "all-same-x" ? 0.0 : double(i), double(i)}, 0);
            vector<Point> probes;
            for (size_t q = 0; q < 256; ++q) {
                const size_t i = (q * 73) % n;
                probes.push_back(mode == "hit" ? g.points[i] :
                    mode == "first-hit" ? g.points[0] :
                    mode == "all-same-x" ? Point{0, -7} :
                    mode == "same-x-miss" ? Point{double(i), -7} : Point{double(i) + 0.125, -7});
            }
            const size_t repeats = max<size_t>(64, 500000 / n);
            auto measure = [&](auto scan) {
                const auto start = chrono::steady_clock::now();
                checksum += scan(g, probes, repeats);
                return chrono::duration<double, milli>(chrono::steady_clock::now() - start).count();
            };
            double scalar = 1e100, optimized = 1e100;
            for (int trial = 0; trial < 3; ++trial) {
                if (trial % 2 == 0) {
                    scalar = min(scalar, measure(ScanScalar));
                    optimized = min(optimized, measure(ScanOptimized));
                } else {
                    optimized = min(optimized, measure(ScanOptimized));
                    scalar = min(scalar, measure(ScanScalar));
                }
            }
            cout << "points=" << n << " mode=" << mode << " scalar_ms=" << scalar
                 << " optimized_ms=" << optimized << " ratio=" << scalar / optimized << '\n';
        }
    }
    cout << "benchmark_checksum=" << checksum << '\n';
}

int main(int argc, char** argv) {
    TestPointSequences();
    TestGridAndRollback();
    TestApplyOrder();
    cout << "geometry tests passed\n";
    if (argc > 1 && string(argv[1]) == "--benchmark") Benchmark();
}
