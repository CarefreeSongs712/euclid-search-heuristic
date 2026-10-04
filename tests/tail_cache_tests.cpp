#include "tail_cache.hpp"
#include <cassert>
#include <random>

using namespace bs;

int main() {
    std::mt19937_64 rng(20261004);
    for (double eps : {1e-11, 1e-13, 4e-5}) {
        EPS = eps;
        for (int trial = 0; trial < 100; ++trial) {
            Graph g;
            for (int i = 0; i < 32; ++i)
                g.AddPoint({static_cast<double>(static_cast<int>(rng() % 100) - 50) / 7,
                            static_cast<double>(static_cast<int>(rng() % 100) - 50) / 7}, 0);
            const size_t n = g.points.size();
            TailPrefixCache cache;
            cache.Reset(n);
            const Point target{0.12345, 0.6789};
            const auto& entries = cache.Get(g, target, 2);
            assert(entries.size() == n);
            vector<Element> tried;
            for (uint32_t q = 0; q < n; ++q) {
                const auto& e = entries[q];
                if (!SamePoint(target, g.points[q])) {
                    const auto line = Element::FromPoints(target, g.points[q], Type::Line);
                    assert(SameElement(e.line, line));
                    const bool dup = any_of(tried.begin(), tried.end(), [&](const Element& old) {
                        return SameElement(old, line);
                    });
                    assert(e.duplicateLine == dup);
                    if (!dup) {
                        tried.push_back(line);
                        vector<uint32_t> hits;
                        g.VisitPointIncidences(line, [&](uint32_t i) { hits.push_back(i); });
                        assert(e.first == (hits.empty() ? NO_BOUND : hits[0]));
                        assert(e.second == (hits.size() < 2 ? NO_BOUND : hits[1]));
                    }
                }
                if (e.circleValid) {
                    uint32_t first = NO_BOUND;
                    g.VisitPointIncidences(e.circle, [&](uint32_t i) {
                        if (i != q && first == NO_BOUND) first = i;
                    });
                    assert(e.circumference == first);
                }
            }
            const auto mark = g.GetMark();
            g.AddPoint({999.0, 17.0}, 1);
            assert(&cache.Get(g, target, 2) == &entries);
            g.Rollback(mark);
            cache.Reset(g.points.size());
            assert(cache.Get(g, {0.0, -0.0}, 1).size() == n);
        }
    }
    cout << "tail cache exact ordered-prefix tests passed\n";
}
