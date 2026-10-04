#pragma once
#include "geometry.hpp"

namespace bs {

// Optional, score-only backward features. Nothing here is an operation source,
// a reachability test, or a pruning/success predicate. In particular, a virtual
// point/carrier is NEVER inserted into a Graph. Build once on the initial graph;
// afterwards this object is immutable and may be shared between workers.
class BackwardLandmarks {
public:
    static constexpr size_t MaxAnchors = 64, MaxGuides = 64;
    static constexpr size_t MaxProbes = 128, MaxPointScan = 512;

private:
    struct Anchor { Point point; double weight = 0.0; };
    struct Guide { Element curve; double weight = 0.0; };
    struct NoPoll { void operator()() const {} };
    array<Anchor, MaxAnchors> anchors_{};
    array<Guide, MaxGuides> guides_{};
    size_t anchorCount_ = 0, guideCount_ = 0, probes_ = 0;
    size_t initialElementCount_ = 0;

    // Fixed prefix plus evenly spaced samples, including the last item. Both
    // point and element scans are bounded, even without a cancellation callback.
    // Avoid k*n overflow on very large inputs. Sampling can miss useful geometry;
    // it only loses a bonus, never a real candidate.
    template<class Visit>
    static void Scan(size_t begin, size_t end, size_t cap, Visit&& visit) {
        begin = min(begin, end);
        const size_t n = end - begin, count = min(n, cap);
        if (!count) return;
        if (count == n) {
            for (size_t i = begin; i < end; ++i) if (visit(i)) return;
            return;
        }
        const size_t head = min<size_t>(32, count / 2);
        for (size_t k = 0; k < head; ++k) if (visit(begin + k)) return;
        const size_t rest = count - head, span = n - head - 1;
        for (size_t k = 0; k < rest; ++k) {
            const size_t offset = rest == 1 ? span :
                k * (span / (rest - 1)) + k * (span % (rest - 1)) / (rest - 1);
            if (visit(begin + head + offset)) return;
        }
    }
    static bool Finite(const Point& p) { return isfinite(p.x) && isfinite(p.y); }
    static bool Finite(const Element& e) {
        return isfinite(e.a) && isfinite(e.b) && isfinite(e.c) &&
            (e.type == Type::Circle ? e.c > 0.0 :
             e.type == Type::Line && (e.a != 0.0 || e.b != 0.0));
    }
    template<class Poll>
    static bool Known(const Graph& g, const Point& p, size_t begin, Poll& poll) {
        bool found = false;
        Scan(begin, g.points.size(), MaxPointScan, [&](size_t i) {
            poll();
            return found = SamePoint(p, g.points[i]);
        });
        return found;
    }
    template<class Poll>
    static bool Present(const Graph& g, const Element& e, size_t begin, Poll& poll) {
        bool found = false;
        Scan(begin, g.elements.size(), MaxPointScan, [&](size_t i) {
            poll();
            return found = g.SameStoredElement(e, g.elements[i]);
        });
        return found;
    }
    void AnchorAt(const Graph& g, const Point& p, double weight) {
        if (!Finite(p) || !g.PointAllowed(p)) return;
        for (size_t i = 0; i < anchorCount_; ++i) {
            if (SamePoint(p, anchors_[i].point)) {
                anchors_[i].weight = max(anchors_[i].weight, weight);
                return;
            }
        }
        size_t slot = anchorCount_;
        if (slot == MaxAnchors) {
            slot = 0;
            for (size_t i = 1; i < anchorCount_; ++i)
                if (anchors_[i].weight < anchors_[slot].weight) slot = i;
            if (weight <= anchors_[slot].weight) return;
        } else ++anchorCount_;
        anchors_[slot] = {p, weight};
    }
    void GuideOn(const Element& e, double weight) {
        if (!Finite(e)) return;
        for (size_t i = 0; i < guideCount_; ++i) {
            if (SameElement(e, guides_[i].curve)) {
                guides_[i].weight = max(guides_[i].weight, weight);
                return;
            }
        }
        size_t slot = guideCount_;
        if (slot == MaxGuides) {
            slot = 0;
            for (size_t i = 1; i < guideCount_; ++i)
                if (guides_[i].weight < guides_[slot].weight) slot = i;
            if (weight <= guides_[slot].weight) return;
        } else ++guideCount_;
        guides_[slot] = {e, weight};
    }
    static void Include(array<double, 3>& best, double value) {
        for (double& old : best) if (value > old) swap(value, old);
    }
    static double Total(const array<double, 3>& best) {
        return best[0] + 0.5 * best[1] + 0.25 * best[2];
    }
    static double Bounded(double score) {
        return isfinite(score) ? min(8.0, max(0.0, score)) : 0.0;
    }

public:
    size_t AnchorCount() const { return anchorCount_; }
    size_t GuideCount() const { return guideCount_; }
    size_t ProbeCount() const { return probes_; }

    void Build(const Graph& initial, int tools) { Build(initial, tools, NoPoll{}); }

    // tools: 0 compass, 1 straightedge, 2 both (3 also means grid straightedge).
    // poll() returns void and may throw to cancel. Scores have the same optional
    // final callback. Internal fixed-size feature scans do not call Graph's
    // unbounded HasPoint/HasElement/VisitPointIncidences routines.
    template<class Poll>
    void Build(const Graph& initial, int tools, Poll&& poll) {
        anchorCount_ = guideCount_ = probes_ = 0;
        initialElementCount_ = initial.elements.size();
        poll();
        if (tools < 0 || tools > 3) throw invalid_argument("landmark tool type must be 0..3");
        if (initial.goalPoints.empty() && initial.goalElements.empty()) return;
        const bool lines = tools != 0;
        vector<Point> goals;
        vector<Element> targets, carriers;
        goals.reserve(16); targets.reserve(16); carriers.reserve(64);
        Scan(0, initial.goalPoints.size(), 16, [&](size_t i) {
            poll();
            const Point p = initial.goalPoints[i];
            if (Finite(p) && initial.PointAllowed(p) && !Known(initial, p, 0, poll)) {
                goals.push_back(p); AnchorAt(initial, p, 1.0);
            }
            return false;
        });
        Scan(0, initial.goalElements.size(), 16, [&](size_t i) {
            poll();
            const Element e = initial.goalElements[i];
            if (!Finite(e) || Present(initial, e, 0, poll)) return false;
            targets.push_back(e);
            if ((e.type == Type::Line && lines) || (e.type == Type::Circle && tools != 1 && tools != 3))
                GuideOn(e, 1.0);
            if (e.type == Type::Circle) {
                const Point center{e.a, e.b};
                if (!Known(initial, center, 0, poll)) AnchorAt(initial, center, 1.0);
            }
            return false;
        });
        Scan(0, initial.elements.size(), 64, [&](size_t i) {
            poll(); carriers.push_back(initial.elements[i]); return false;
        });

        // Level one: useful points on a target carrier, not points granted by
        // that carrier. Prefer circles/non-grid geometry before abundant grid
        // intersections. VisitIntersections retains bounded-curve/grid clipping.
        for (int pass = 0; pass < 2; ++pass) for (const Element& old : carriers) {
            poll();
            if (initial.IsAutomaticGridLine(old) != (pass == 1)) continue;
            for (const Element& target : targets) {
                poll();
                initial.VisitIntersections(target, old, [&](const Point& p) {
                    poll();
                    if (Finite(p) && !Known(initial, p, 0, poll)) AnchorAt(initial, p, 0.45);
                    return false;
                });
            }
        }

        size_t circles = 0, joins = 0;
        for (const Element& circle : carriers) {
            poll();
            if (circle.type != Type::Circle || !Finite(circle)) continue;
            if (++circles > 8) break;
            const Point center{circle.a, circle.b};
            const bool centerKnown = Known(initial, center, 0, poll);
            array<Point, 16> rim{}, opposite{};
            array<bool, 16> missing{};
            size_t count = 0;
            Scan(0, initial.points.size(), MaxPointScan, [&](size_t i) {
                poll();
                const Point p = initial.points[i];
                if (Finite(p) && initial.PointOnElement(p, circle)) rim[count++] = p;
                return count == rim.size();
            });
            for (size_t i = 0; i < count; ++i) {
                poll();
                opposite[i] = {2.0 * center.x - rim[i].x, 2.0 * center.y - rim[i].y};
                const Point p = opposite[i];
                missing[i] = Finite(p) && initial.PointAllowed(p) && !Known(initial, p, 0, poll);
                if (!missing[i]) continue;
                AnchorAt(initial, p, 0.35);
                if (lines && centerKnown) GuideOn(Element::FromPoints(center, rim[i], Type::Line), 0.4);
            }
            for (const Point& p : goals) {
                poll();
                if (initial.PointOnElement(p, circle)) {
                    const Point oppositeGoal{2.0 * center.x - p.x, 2.0 * center.y - p.y};
                    if (Finite(oppositeGoal) && !Known(initial, oppositeGoal, 0, poll))
                        AnchorAt(initial, oppositeGoal, 0.65);
                }
            }
            if (!lines || !centerKnown) continue;

            // Level two: a chord H meets the target at Y. A carrier from an
            // existing rim point P through Y may need a missing antipode U.
            // The diameter through the known center can expose U. Reward H,
            // that diameter, P-U and U, even though H/diameter need not make
            // immediate target progress. No root operation closure is built.
            // MaxProbes counts chord probes; joins has a separate fixed budget.
            for (size_t i = 0; i < count && probes_ < MaxProbes; ++i) {
                for (size_t j = i + 1; j < count && probes_ < MaxProbes; ++j) {
                    poll(); ++probes_;
                    const Element chord = Element::FromPoints(rim[i], rim[j], Type::Line);
                    if (!Finite(chord)) continue;
                    auto meet = [&](const Point& y) {
                        poll();
                        if (!Finite(y) || !initial.PointAllowed(y) || Known(initial, y, 0, poll)) return false;
                        for (size_t u = 0; u < count; ++u) if (missing[u]) {
                            for (size_t p = 0; p < count; ++p) {
                                poll();
                                if (joins++ >= 16384) return true;
                                if (SamePoint(y, rim[p]) || SamePoint(y, opposite[u])) continue;
                                const Element next = Element::FromPoints(rim[p], opposite[u], Type::Line);
                                if (!Finite(next) || SameElement(chord, next) ||
                                    !initial.PointOnElement(y, next)) continue;
                                AnchorAt(initial, opposite[u], 1.8);
                                AnchorAt(initial, y, 1.2);
                                GuideOn(chord, 2.0);
                                GuideOn(Element::FromPoints(center, rim[u], Type::Line), 1.75);
                                GuideOn(next, 1.6);
                            }
                        }
                        return false;
                    };
                    for (const Element& target : targets) {
                        poll();
                        if (joins >= 16384) break;
                        initial.VisitIntersections(chord, target, meet);
                    }
                    for (const Point& goal : goals) {
                        poll();
                        if (joins >= 16384) break;
                        if (initial.PointOnElement(goal, chord)) meet(goal);
                    }
                    if (joins >= 16384) break;
                }
                if (joins >= 16384) break;
            }
        }
    }

    double ScoreCandidate(const Graph& g, const Element& e, size_t pointBegin) const {
        return ScoreCandidate(g, e, pointBegin, NoPoll{});
    }
    // Before Apply, pass g.points.size(): this only scores the curve. After a
    // real Apply, pass mark.pointCount: actual new landmark points also count.
    // Returns a dimensionless [0,8] bonus, e.g. multiply by prerequisite weight.
    template<class Poll>
    double ScoreCandidate(const Graph& g, const Element& e, size_t pointBegin, Poll&& poll) const {
        poll();
        if (!Finite(e)) return 0.0;
        double curve = 0.0, support = 0.0;
        array<double, 3> born{};
        for (size_t i = 0; i < guideCount_; ++i) {
            poll();
            if (SameElement(e, guides_[i].curve)) curve = max(curve, guides_[i].weight);
        }
        for (size_t i = 0; i < anchorCount_; ++i) {
            poll();
            const Anchor& a = anchors_[i];
            if (pointBegin < g.points.size() && Known(g, a.point, pointBegin, poll))
                Include(born, a.weight);
            else if (g.PointOnElement(a.point, e) && !Known(g, a.point, 0, poll))
                support = max(support, 0.35 * a.weight);
        }
        return Bounded(curve + support + Total(born));
    }

    double ScoreState(const Graph& g, size_t originalPointCount) const {
        return ScoreState(g, originalPointCount, NoPoll{});
    }
    // Cumulative evidence from real added points/elements, independent of the
    // most recent step. No nearest/farthest point assumption or hard cutoff on
    // the search itself; only a bounded sample is inspected for scoring.
    template<class Poll>
    double ScoreState(const Graph& g, size_t originalPointCount, Poll&& poll) const {
        poll();
        array<double, 3> points{}, curves{};
        for (size_t i = 0; i < anchorCount_; ++i) {
            poll();
            if (Known(g, anchors_[i].point, originalPointCount, poll)) Include(points, anchors_[i].weight);
        }
        for (size_t i = 0; i < guideCount_; ++i) {
            poll();
            if (Present(g, guides_[i].curve, initialElementCount_, poll)) Include(curves, guides_[i].weight);
        }
        return Bounded(Total(points) + Total(curves));
    }
};

} // namespace bs
