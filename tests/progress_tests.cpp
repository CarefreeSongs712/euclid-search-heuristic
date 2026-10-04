#include "progress.hpp"
#include <cassert>

using namespace bs;

int main() {
    ProgressSlot slot;
    SearchStats a;
    a.nodes = 100;
    slot.BeginTask(a);
    a.nodes += 1000;
    a.rawCandidates += 2000;
    a.uniqueCandidates += 500;
    slot.Publish(a, 7);
    slot.SetRootFraction(0.25);
    auto s = slot.Read();
    assert(s.nodes == 1000 && s.rawCandidates == 2000 && s.uniqueCandidates == 500);
    assert(s.depth == 7 && s.active && s.rootFraction == 0.25);
    slot.EndTask(a);
    slot.EndTask(a);
    assert(!slot.Read().active && slot.Read().nodes == 1000);
    SearchStats b;
    slot.BeginTask(b);
    b.nodes = 42;
    slot.EndTask(b);
    assert(slot.Read().nodes == 1042 && slot.Read().rawCandidates == 2000);
    slot.SetRootFraction(numeric_limits<double>::infinity());
    assert(slot.Read().rootFraction < 0);
    slot.SetRootFraction(2);
    assert(slot.Read().rootFraction == 1);

    ProgressState state(4, 1, false);
    const auto start = chrono::steady_clock::now();
    {
        ProgressReporter reporter(state, &slot, 1, start, 30, 100, true);
        reporter.Finish();
    }
    assert(chrono::duration<double>(chrono::steady_clock::now() - start).count() < 1);
    cout << "progress accumulation and immediate shutdown tests passed\n";
}
