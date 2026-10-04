#pragma once
#include "core.hpp"

namespace bs {
class Graph {
public:
    // Small-search optimization: points/elements are deduplicated by direct
    // linear scans. For the intended shallow construction searches, these
    // sets stay small, avoiding unordered_map bucket hashing and rollback cost.
    vector<Point> points;
    vector<uint16_t> pointBirth;
    vector<Element> elements;
    vector<Bound> bounds;
    vector<Point> goalPoints;
    vector<Element> goalElements;
    size_t initialElementCount = 0;

    // Mode 3: the allowed point domain is a CLOSED rectangle. No EPS-expanded
    // exterior strip and no implicit clipping of an outside point onto an edge.
    // Curves keep their supporting equations; only in-domain intersections can
    // enter the known-point set or be used by any reverse/preview search.
    bool gridMode = false;
    bool gridFast = true;
    bool gridLinesReady = false;
    int gridM = 0, gridN = 0;

    bool PointAllowed(const Point& p) const {
        return !gridMode || (isfinite(p.x) && isfinite(p.y) &&
            p.x >= 0.0 && p.x <= gridM && p.y >= 0.0 && p.y <= gridN);
    }

    bool IsAutomaticGridLine(const Element& e) const {
        if (!gridMode || !gridLinesReady || EPS >= 0.5 || e.type != Type::Line) return false;
        if (e.a == 1.0 && e.b == 0.0)
            return e.c >= 0.0 && e.c <= gridM && static_cast<int>(e.c) == e.c;
        if (e.a == 0.0 && e.b == 1.0)
            return e.c >= 0.0 && e.c <= gridN && static_cast<int>(e.c) == e.c;
        return false;
    }

private:
    uint64_t stateHash1_ = 0x243f6a8885a308d3ULL;
    uint64_t stateHash2_ = 0x13198a2e03707344ULL;
    bool stateHashEnabled_ = false;

    // Preserve SamePoint's coordinate-wise strict EPS test and point order.
    // There is no quantization, representative replacement, or extra state to
    // rebuild after rollback/copy. Only x tests are batched; y retains the
    // scalar short circuit (including unordered/NaN comparisons).
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((always_inline))
#elif defined(_MSC_VER)
    __forceinline
#endif
    bool ContainsStoredPoint(const Point& p) const {
#if BS_HAS_SSE2 && !defined(BS_DISABLE_POINT_SCAN_SIMD)
        if (points.size() >= 16) {
            // Avoid SIMD setup for frequent hits on the first defining point.
            if (SamePoint(points.front(), p)) return true;
            size_t i = 1;
            const __m128d x = _mm_set1_pd(p.x);
            const __m128d eps = _mm_set1_pd(EPS);
            const __m128d sign = _mm_set1_pd(-0.0);
            for (; i + 3 < points.size(); i += 4) {
                const __m128d x01 = _mm_set_pd(points[i + 1].x, points[i].x);
                const __m128d x23 = _mm_set_pd(points[i + 3].x, points[i + 2].x);
                const __m128d hit01 = _mm_cmplt_pd(_mm_andnot_pd(sign, _mm_sub_pd(x01, x)), eps);
                const __m128d hit23 = _mm_cmplt_pd(_mm_andnot_pd(sign, _mm_sub_pd(x23, x)), eps);
                if (_mm_movemask_pd(_mm_or_pd(hit01, hit23)) == 0) continue;
                const int hits = _mm_movemask_pd(hit01) | (_mm_movemask_pd(hit23) << 2);
                if ((hits & 1) && IsZero(points[i].y - p.y)) return true;
                if ((hits & 2) && IsZero(points[i + 1].y - p.y)) return true;
                if ((hits & 4) && IsZero(points[i + 2].y - p.y)) return true;
                if ((hits & 8) && IsZero(points[i + 3].y - p.y)) return true;
            }
            for (; i < points.size(); ++i)
                if (SamePoint(points[i], p)) return true;
            return false;
        }
#endif
        for (const Point& old : points)
            if (SamePoint(old, p)) return true;
        return false;
    }

    void TogglePointHash(const Point& p) {
        if (!stateHashEnabled_) return;
        stateHash1_ ^= HashPointRaw(p, 0xa4093822299f31d0ULL);
        stateHash2_ ^= HashPointRaw(p, 0x082efa98ec4e6c89ULL);
    }

    void ToggleElementHash(const Element& e) {
        if (!stateHashEnabled_) return;
        stateHash1_ ^= HashElementRaw(e, 0x452821e638d01377ULL);
        stateHash2_ ^= HashElementRaw(e, 0xbe5466cf34e90c6cULL);
    }

    bool IsInRange(const Element& e, const Point& p) const {
        if (e.type == Type::Line) return true;
        if (e.type == Type::Ray || e.type == Type::Segment) {
            if (e.bound == NO_BOUND || e.bound >= bounds.size()) return false;
            const Bound& bd = bounds[e.bound];
            if (e.type == Type::Ray) {
                const double dotx = (bd.p1.x - p.x) * (bd.p1.x - bd.p2.x);
                const double doty = (bd.p1.y - p.y) * (bd.p1.y - bd.p2.y);
                return dotx + doty > -EPS;
            }
            const double dotx = (bd.p1.x - p.x) * (bd.p2.x - p.x);
            const double doty = (bd.p1.y - p.y) * (bd.p2.y - p.y);
            return dotx + doty < EPS;
        }
        return false;
    }


    template <class Visitor>
    bool VisitLineCircle(const Element& line, const Element& circle,
                         Visitor&& visitor) const {
        const double denom = Sq(line.a) + Sq(line.b);
        if (IsZero(denom)) return false;
        const double dist = line.a * circle.a + line.b * circle.b - line.c;
        const double delta = denom * circle.c - Sq(dist);
        if (IsZero(delta)) {
            Point p{circle.a - line.a * dist / denom,
                    circle.b - line.b * dist / denom};
            if (IsInRange(line, p) && visitor(p)) return true;
        } else if (delta > EPS) {
            const double root = sqrt(delta);
            Point p1{circle.a - (line.a * dist + line.b * root) / denom,
                     circle.b - (line.b * dist - line.a * root) / denom};
            Point p2{circle.a - (line.a * dist - line.b * root) / denom,
                     circle.b - (line.b * dist + line.a * root) / denom};
            if (IsInRange(line, p1) && visitor(p1)) return true;
            if (IsInRange(line, p2) && visitor(p2)) return true;
        }
        return false;
    }

public:
    void Reserve(size_t maxElements) {
        elements.reserve(maxElements);
        const size_t pointHint = 2 * maxElements * maxElements + 16;
        points.reserve(gridMode ? min<size_t>(pointHint, 1'000'000) : pointHint);
        pointBirth.reserve(points.capacity());
        bounds.reserve(maxElements);
    }

    bool HasPoint(const Point& p) const {
        if (gridMode && gridFast && gridLinesReady && EPS < 0.5 && PointAllowed(p) &&
            p.x == static_cast<int>(p.x) && p.y == static_cast<int>(p.y))
            return true; // every integer grid vertex is already initially known
        return ContainsStoredPoint(p);
    }

    bool SameStoredElement(const Element& e, const Element& f) const {
        if (!SameElement(e, f)) return false;
        if (e.type != Type::Ray && e.type != Type::Segment) return true;

        if (e.bound == NO_BOUND || f.bound == NO_BOUND ||
            e.bound >= bounds.size() || f.bound >= bounds.size()) {
            return false;
        }
        const Bound& be = bounds[e.bound];
        const Bound& bf = bounds[f.bound];

        if (e.type == Type::Segment) {
            // Segment orientation is irrelevant: [A,B] == [B,A].
            return (SamePoint(be.p1, bf.p1) && SamePoint(be.p2, bf.p2)) ||
                   (SamePoint(be.p1, bf.p2) && SamePoint(be.p2, bf.p1));
        }

        // A ray is determined by its start point and direction.  The second
        // defining point may be anywhere farther along the same ray, so do not
        // require it to be identical. SameElement above has already checked
        // that both rays lie on the same supporting line.
        if (!SamePoint(be.p1, bf.p1)) return false;
        const double ex = be.p2.x - be.p1.x;
        const double ey = be.p2.y - be.p1.y;
        const double fx = bf.p2.x - bf.p1.x;
        const double fy = bf.p2.y - bf.p1.y;
        return ex * fx + ey * fy > EPS;
    }

    bool HasElement(const Element& e) const {
        if (gridFast && IsAutomaticGridLine(e)) return true;
        for (const Element& old : elements) {
            if (SameStoredElement(old, e)) return true;
        }
        return false;
    }

    bool AddPoint(const Point& p, uint16_t birth) {
        if (!PointAllowed(p)) return false;
        if (gridMode && gridFast && gridLinesReady && EPS < 0.5 &&
            p.x == static_cast<int>(p.x) && p.y == static_cast<int>(p.y))
            return false; // exact existing grid vertex: no linear point scan
        if (ContainsStoredPoint(p)) return false;
        points.push_back(p);
        pointBirth.push_back(birth);
        TogglePointHash(p);
        return true;
    }

    bool PointOnElement(const Point& p, const Element& e) const {
        if (e.type == Type::Circle) {
            return IsZero(Sq(p.x - e.a) + Sq(p.y - e.b) - e.c);
        }
        if (!IsZero(e.a * p.x + e.b * p.y - e.c)) return false;
        return IsInRange(e, p);
    }

    // Same per-point arithmetic and comparison as PointOnElement; visit
    // matches in their original index order.
    template <class Visitor>
    void VisitPointIncidences(const Element& e, Visitor&& visit, uint32_t begin = 0) const {
        uint32_t i = begin;
#if BS_HAS_SSE2
        if (e.type == Type::Line || e.type == Type::Circle) {
            const __m128d aa = _mm_set1_pd(e.a), bb = _mm_set1_pd(e.b);
            const __m128d cc = _mm_set1_pd(e.c), eps = _mm_set1_pd(EPS);
            const __m128d sign = _mm_set1_pd(-0.0);
            for (; i + 1 < points.size(); i += 2) {
                const __m128d p0 = _mm_loadu_pd(&points[i].x);
                const __m128d p1 = _mm_loadu_pd(&points[i+1].x);
                const __m128d x = _mm_unpacklo_pd(p0, p1);
                const __m128d y = _mm_unpackhi_pd(p0, p1);
                __m128d residual;
                if (e.type == Type::Circle) {
                    const __m128d dx = _mm_sub_pd(x, aa);
                    const __m128d dy = _mm_sub_pd(y, bb);
                    residual = _mm_sub_pd(_mm_add_pd(_mm_mul_pd(dx, dx),
                                                    _mm_mul_pd(dy, dy)), cc);
                } else {
                    residual = _mm_sub_pd(_mm_add_pd(_mm_mul_pd(aa, x),
                                                    _mm_mul_pd(bb, y)), cc);
                }
                const int hits = _mm_movemask_pd(_mm_cmplt_pd(
                    _mm_andnot_pd(sign, residual), eps));
                if (hits & 1) visit(i);
                if (hits & 2) visit(i + 1);
            }
        }
#endif
        for (; i < points.size(); ++i)
            if (PointOnElement(points[i], e)) visit(i);
    }

    template <class Visitor>
    bool VisitUnclippedIntersections(const Element& e1, const Element& e2,
                            Visitor&& visitor) const {
        if (e1.type == Type::Circle) {
            if (e2.type == Type::Circle) {
                double a = 2.0 * (e1.a - e2.a);
                double b = 2.0 * (e1.b - e2.b);
                if (IsZero(a) && IsZero(b)) return false;
                double c = Sq(e1.a) - Sq(e2.a) + Sq(e1.b) - Sq(e2.b) - e1.c + e2.c;
                Element radical = Element::FromCoefficients(a, b, c, Type::Line);
                return VisitLineCircle(radical, e1, visitor);
            }
            return VisitLineCircle(e2, e1, visitor);
        }
        if (e2.type == Type::Circle)
            return VisitLineCircle(e1, e2, visitor);
        const double det = e1.a * e2.b - e1.b * e2.a;
        if (IsZero(det)) return false;
        Point p{(e1.c * e2.b - e1.b * e2.c) / det,
                (e1.a * e2.c - e1.c * e2.a) / det};
        return IsInRange(e1, p) && IsInRange(e2, p) && visitor(p);
    }

    template <class Visitor>
    bool VisitIntersections(const Element& e1, const Element& e2,
                            Visitor&& visitor) const {
        // Central filter also covers circle-circle radical-axis intersections,
        // prerequisite previews, and direct Graph users, not just AddPoint.
        if (!gridMode) return VisitUnclippedIntersections(e1, e2, visitor);
        auto inside = [&](const Point& p) {
            return PointAllowed(p) && visitor(p);
        };
        return VisitUnclippedIntersections(e1, e2, inside);
    }

    void Intersect(const Element& e1, const Element& e2, uint16_t birth) {
        VisitIntersections(e1, e2, [&](const Point& p) {
            AddPoint(p, birth);
            return false;
        });
    }

    Mark GetMark() const { return {points.size(), elements.size()}; }

    void ApplyKnownNew(const Element& e, uint16_t birth) {
        for (const Element& old : elements) Intersect(e, old, birth);
        elements.push_back(e);
        ToggleElementHash(e);
    }

    bool Apply(const Element& e, uint16_t birth) {
        if (HasElement(e)) return false;
        ApplyKnownNew(e, birth);
        return true;
    }

    void SetStateHashingEnabled(bool enabled) {
        stateHashEnabled_ = enabled;
        stateHash1_ = 0x243f6a8885a308d3ULL;
        stateHash2_ = 0x13198a2e03707344ULL;
        if (!enabled) return;
        for (const Point& p : points) TogglePointHash(p);
        for (const Element& e : elements) ToggleElementHash(e);
    }

    uint64_t StateHash1() const { return stateHash1_; }
    uint64_t StateHash2() const { return stateHash2_; }

    void Rollback(const Mark& mark) {
        while (elements.size() > mark.elementCount) {
            const uint32_t id = static_cast<uint32_t>(elements.size() - 1);
            ToggleElementHash(elements[id]);
            elements.pop_back();
        }
        while (points.size() > mark.pointCount) {
            const uint32_t id = static_cast<uint32_t>(points.size() - 1);
            TogglePointHash(points[id]);
            points.pop_back();
            pointBirth.pop_back();
        }
    }

    bool AddInitialBounded(const Point& p1, const Point& p2, Type t) {
        const uint32_t bi = static_cast<uint32_t>(bounds.size());
        bounds.push_back({p1, p2});
        Element e = Element::FromPoints(p1, p2, t, bi);
        if (!Apply(e, 0)) {
            bounds.pop_back();
            return false;
        }
        return true;
    }

    bool AddInitial(const Element& e) { return Apply(e, 0); }

    void AddAutomaticGridLines() {
        if (!gridMode || gridLinesReady) return;
        // Before user elements are added, all auto-grid intersections are known
        // exactly: one lattice point for each (integer x, integer y). Generate
        // them in the SAME row-major order as sequential horizontal/vertical
        // intersections, avoiding a quadratic scan of earlier lattice points.
        if (gridFast && EPS < 0.5 && elements.empty()) {
            for (int x = 0; x <= gridM; ++x) {
                const Element e = Element::FromCoefficients(1, 0, x, Type::Line);
                elements.push_back(e); ToggleElementHash(e);
            }
            for (int y = 0; y <= gridN; ++y) {
                const Element e = Element::FromCoefficients(0, 1, y, Type::Line);
                elements.push_back(e); ToggleElementHash(e);
            }
            const size_t explicitCount = points.size();
            for (int y = 0; y <= gridN; ++y) {
                for (int x = 0; x <= gridM; ++x) {
                    const Point p{static_cast<double>(x), static_cast<double>(y)};
                    bool duplicate = false;
                    for (size_t i = 0; i < explicitCount; ++i) {
                        if (SamePoint(points[i], p)) { duplicate = true; break; }
                    }
                    if (!duplicate) {
                        points.push_back(p); pointBirth.push_back(0); TogglePointHash(p);
                    }
                }
            }
        } else {
            for (int x = 0; x <= gridM; ++x)
                AddInitial(Element::FromCoefficients(1, 0, x, Type::Line));
            for (int y = 0; y <= gridN; ++y)
                AddInitial(Element::FromCoefficients(0, 1, y, Type::Line));
        }
        gridLinesReady = true;
    }


    bool GoalsMet() const {
        for (const Point& p : goalPoints) if (!HasPoint(p)) return false;
        for (const Element& e : goalElements) if (!HasElement(e)) return false;
        return true;
    }

    // Candidate priority used by goal-first search:
    //   2 = the candidate itself is a still-missing target line/circle;
    //   1 = it is not a target element, but it passes through a missing target point;
    //   0 = ordinary auxiliary construction.
    // This does not prune any candidate; it only changes search order.
    int GoalPriority(const Element& candidate) const {
        // Test the cheap geometric predicate first.  Almost every candidate is
        // unrelated to a particular goal, so avoid the linear HasElement /
        // HasPoint scan unless the candidate can actually satisfy that goal.
        for (const Element& e : goalElements) {
            if (SameElement(e, candidate) && !HasElement(e)) return 2;
        }
        for (const Point& p : goalPoints) {
            if (PointOnElement(p, candidate) && !HasPoint(p)) return 1;
        }
        return 0;
    }

    bool IsGoalDirected(const Element& candidate) const {
        return GoalPriority(candidate) > 0;
    }

    bool HasMissingGoalPoints() const {
        for (const Point& p : goalPoints) {
            if (!HasPoint(p)) return true;
        }
        return false;
    }

    int MissingPointLowerBound() const {
        int lower = 0;
        for (const Point& p : goalPoints) {
            if (HasPoint(p)) continue;
            int incidence = 0;
            for (const Element& e : elements) {
                if (PointOnElement(p, e)) {
                    ++incidence;
                    if (incidence >= 1) break;
                }
            }
            lower = max(lower, incidence ? 1 : 2);
        }
        return lower;
    }

    // Each construction adds at most one new element. Therefore the number
    // of distinct missing line/circle goals is a strict lower bound on the
    // number of remaining constructions.
    int MissingDistinctGoalElementCount() const {
        int missing = 0;
        for (size_t i = 0; i < goalElements.size(); ++i) {
            const Element& goal = goalElements[i];
            if (HasElement(goal)) continue;

            bool duplicate = false;
            for (size_t j = 0; j < i; ++j) {
                if (SameElement(goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) ++missing;
        }
        return missing;
    }

    // Count how many currently existing elements pass through a still-missing
    // target point.  In a consistent state a missing target point normally has
    // at most one such support, because the intersection of two existing
    // elements would already have been inserted as a known point. Still
    // count up to two to keep the test conservative under floating-point EPS.
    int ExistingSupportCount(const Point& p) const {
        int count = 0;
        for (const Element& e : elements) {
            if (PointOnElement(p, e) && ++count >= 2) break;
        }
        return count;
    }

    // Number of distinct still-missing target line/circle elements incident on
    // p.  These elements must be constructed anyway, so they can optimistically
    // supply incidence needed to create p without consuming an auxiliary E.
    int MissingDistinctGoalElementsThroughPoint(const Point& p) const {
        int count = 0;
        for (size_t i = 0; i < goalElements.size(); ++i) {
            const Element& goal = goalElements[i];
            if (HasElement(goal) || !PointOnElement(p, goal)) continue;

            bool duplicate = false;
            for (size_t j = 0; j < i; ++j) {
                if (SameElement(goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) ++count;
        }
        return count;
    }

    // Joint strict lower bound for target points after reserving one step for
    // every distinct missing target line/circle.  Let k be that number and
    // s=remaining-k the number of steps that could possibly be auxiliary.
    // A missing target point needs two incident elements in total. Existing
    // supports and still-missing target elements through the point can cover
    // part of that requirement; anything left must come from auxiliary steps.
    int RequiredAuxiliaryStepsForGoalPoints() const {
        int lower = 0;
        auto includeRequiredPoint = [&](const Point& p) {
            if (HasPoint(p)) return;
            const int existing = min(2, ExistingSupportCount(p));
            const int forcedTargets = MissingDistinctGoalElementsThroughPoint(p);
            const int need = max(0, 2 - existing - forcedTargets);
            lower = max(lower, need);
        };

        for (const Point& p : goalPoints) includeRequiredPoint(p);

        // A missing target circle has an implicit prerequisite: its center must
        // already be a known point before the circle can be constructed by this
        // program's center->circumference compass operation.  Treat that center
        // as an additional required point for the joint lower bound.  The goal
        // circle itself does not pass through its center (for nonzero radius),
        // so reserving the circle step cannot falsely help create the center.
        for (size_t i = 0; i < goalElements.size(); ++i) {
            const Element& goal = goalElements[i];
            if (goal.type != Type::Circle || HasElement(goal)) continue;
            bool duplicate = false;
            for (size_t j = 0; j < i; ++j) {
                if (SameElement(goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) includeRequiredPoint({goal.a, goal.b});
        }
        return lower;
    }

    // Collect missing target points for which every remaining construction is
    // forced to pass through the point.  Computing this once per DFS node avoids
    // rescanning all existing elements for every candidate in the last two layers.
    void CollectForcedTailPointIndices(int remaining, vector<uint32_t>& out) const {
        out.clear();
        if (remaining <= 0 || remaining > 2) return;
        for (uint32_t i = 0; i < goalPoints.size(); ++i) {
            const Point& p = goalPoints[i];
            if (HasPoint(p)) continue;
            const int existing = min(2, ExistingSupportCount(p));
            const int need = max(0, 2 - existing);
            // One step left: the sole new element must pass every still-missing
            // target point.  Duplicate coincident supports of the same carrier
            // must not drop the point from the forced set.
            if (remaining == 1 || need == remaining) out.push_back(i);
        }
    }

    bool CandidatePassesForcedTailPoints(const Element& candidate,
                                         const vector<uint32_t>& forced) const {
        for (uint32_t gi : forced) {
            if (!PointOnElement(goalPoints[gi], candidate)) return false;
        }
        return true;
    }

    // True if at least one distinct missing target line/circle can be drawn now
    // from the currently known points using the selected tool.  This is used
    // when remaining == number of missing target elements: the next step must
    // be one of those targets, so if none is directly drawable the node is dead.
    bool AnyMissingGoalElementDrawableNow(int toolType) const {
        for (size_t gi = 0; gi < goalElements.size(); ++gi) {
            const Element& goal = goalElements[gi];
            if (HasElement(goal)) continue;
            bool duplicate = false;
            for (size_t j = 0; j < gi; ++j) {
                if (SameElement(goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;

            if (goal.type == Type::Line) {
                if (toolType == 0) continue;
                int hits = 0;
                for (const Point& p : points) {
                    if (PointOnElement(p, goal) && ++hits >= 2) return true;
                }
            } else if (goal.type == Type::Circle) {
                if (toolType == 1 || IsZero(goal.c)) continue;
                int center = -1;
                for (size_t i = 0; i < points.size(); ++i) {
                    if (SamePoint(points[i], {goal.a, goal.b})) {
                        center = static_cast<int>(i);
                        break;
                    }
                }
                if (center < 0) continue;
                for (size_t i = 0; i < points.size(); ++i) {
                    if (static_cast<int>(i) != center && PointOnElement(points[i], goal))
                        return true;
                }
            }
        }
        return false;
    }

    // Strict necessary conditions for completing all still-missing element
    // goals in one construction.  If one distinct goal element is missing,
    // that element must already be directly constructible from known points:
    //   line   -> two known points on the line;
    //   circle -> known center plus a known point on the circumference.
    // More than one distinct missing element can never be completed by one E.
    bool MissingGoalElementsDrawableInOneStep(int toolType) const {
        const Element* missingGoal = nullptr;
        for (size_t i = 0; i < goalElements.size(); ++i) {
            const Element& goal = goalElements[i];
            if (HasElement(goal)) continue;

            bool duplicate = false;
            for (size_t j = 0; j < i; ++j) {
                if (SameElement(goalElements[j], goal)) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) continue;
            if (missingGoal != nullptr) return false;
            missingGoal = &goal;
        }

        if (missingGoal == nullptr) return true;
        const Element& goal = *missingGoal;

        if (goal.type == Type::Line) {
            if (toolType == 0) return false; // compass only
            int onLine = 0;
            for (const Point& p : points) {
                if (PointOnElement(p, goal) && ++onLine >= 2) return true;
            }
            return false;
        }

        if (goal.type == Type::Circle) {
            if (toolType == 1 || IsZero(goal.c)) return false; // straightedge only / zero-radius impossible
            const Point center{goal.a, goal.b};
            int centerId = -1;
            for (size_t i = 0; i < points.size(); ++i) {
                if (SamePoint(points[i], center)) {
                    centerId = static_cast<int>(i);
                    break;
                }
            }
            if (centerId < 0) return false;
            for (size_t i = 0; i < points.size(); ++i) {
                if (static_cast<int>(i) != centerId && PointOnElement(points[i], goal)) return true;
            }
            return false;
        }

        return false;
    }

    Element MakeCandidate(const Candidate& c) const {
        const Point& pi = points[c.i];
        const Point& pj = points[c.j];
        if (c.tool == 0) return Element::FromPoints(pi, pj, Type::Circle);
        if (c.tool == 1) return Element::FromPoints(pj, pi, Type::Circle);
        return Element::FromPoints(pi, pj, Type::Line);
    }

    void PrintSolution() const {
        cout << "Solution found!\n";
        cout << "Points (" << points.size() << "):\n";
        cout << setprecision(12);
        for (const Point& p : points) cout << '(' << CleanZero(p.x) << ',' << CleanZero(p.y) << ")\n";
        cout << "Elements (" << elements.size() << "):\n";
        for (const Element& e : elements) {
            if (e.type == Type::Circle) {
                cout << "(x-" << CleanZero(e.a) << ")^2+(y-" << CleanZero(e.b)
                     << ")^2=" << sqrt(max(0.0, e.c)) << "^2\n";
            } else {
                cout << CleanZero(e.a) << "x+" << CleanZero(e.b) << "y="
                     << CleanZero(e.c) << " [" << static_cast<int>(e.type) << "]\n";
            }
        }
    }
};

// Exact, bounded per-depth candidate deduplication. If the table reaches its
// load limit, new candidates are left untracked rather than discarded, so the

} // namespace bs
