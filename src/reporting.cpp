#include "reporting.hpp"

namespace bs {
// Reporting only: invoked after the search clock has stopped and workers joined.
// No fields, provenance strings, or callbacks are added to the live search state.
namespace readable_report {
#if defined(__GNUC__) || defined(__clang__)
#define READABLE_ONLY __attribute__((noinline, cold))
#elif defined(_MSC_VER)
#define READABLE_ONLY __declspec(noinline)
#else
#define READABLE_ONLY
#endif
constexpr size_t NONE = numeric_limits<size_t>::max();

struct Origin {
    size_t element = NONE;
    size_t other = NONE;
};
struct Definition {
    size_t first = NONE;
    size_t second = NONE;
};
struct Trace {
    vector<Origin> origins;
    vector<Definition> definitions;
    vector<size_t> pointGoals;
    vector<size_t> elementGoals;
    vector<size_t> pointNumbers;
    vector<string> elementNames;
};

READABLE_ONLY bool ExactNumber(double a, double b) {
    return bit_cast<uint64_t>(a) == bit_cast<uint64_t>(b);
}
READABLE_ONLY bool ExactPoint(const Point& a, const Point& b) {
    return ExactNumber(a.x, b.x) && ExactNumber(a.y, b.y);
}
READABLE_ONLY bool ExactElement(const Element& a, const Element& b) {
    return a.type == b.type && a.bound == b.bound &&
           ExactNumber(a.a, b.a) && ExactNumber(a.b, b.b) && ExactNumber(a.c, b.c);
}

// Report-private numerical helpers. They deliberately do not call Graph's
// mutating or candidate routines, so adding a reporter does not add callers
// that change their interprocedural optimization or return-value specialization.
struct RPoint { double x, y; };
struct RElement { double a, b, c; Type type; };
READABLE_ONLY bool RZero(double v) { return std::abs(v) < EPS; }
READABLE_ONLY double RClean(double v) { return RZero(v) ? 0.0 : v; }
READABLE_ONLY bool RSameElement(const RElement& a, const Element& b) {
    return a.type == b.type && RZero(a.a - b.a) &&
        RZero(a.b - b.b) && RZero(a.c - b.c);
}
READABLE_ONLY RElement RNormalize(RElement e) {
    if (!RZero(e.b)) {
        e.a /= e.b; e.c /= e.b; e.b = 1.0;
    } else if (!RZero(e.a)) {
        e.c /= e.a; e.a = 1.0; e.b = 0.0;
    }
    e.a = RClean(e.a); e.b = RClean(e.b); e.c = RClean(e.c);
    return e;
}
READABLE_ONLY RElement RMake(const Point& p, const Point& q, Type type) {
    RElement e{0, 0, 0, type};
    if (type == Type::Circle) {
        e.a = p.x; e.b = p.y;
        const double dx = p.x - q.x, dy = p.y - q.y;
        e.c = dx * dx + dy * dy;
    } else {
        e.a = q.y - p.y; e.b = p.x - q.x;
        e.c = p.x * q.y - p.y * q.x;
        e = RNormalize(e);
    }
    e.a = RClean(e.a); e.b = RClean(e.b); e.c = RClean(e.c);
    return e;
}
READABLE_ONLY bool RExactElement(const RElement& a, const Element& b) {
    return a.type == b.type && ExactNumber(a.a, b.a) &&
        ExactNumber(a.b, b.b) && ExactNumber(a.c, b.c);
}
READABLE_ONLY bool RSamePoint(const Point& p, const Point& q) {
    return RZero(p.x - q.x) && RZero(p.y - q.y);
}
READABLE_ONLY bool RInRange(const Graph& g, size_t ei, const RPoint& p) {
    if (!g.PointAllowed({p.x, p.y})) return false;
    const Element& e = g.elements[ei];
    if (e.type == Type::Line) return true;
    if (e.type != Type::Ray && e.type != Type::Segment) return false;
    if (e.bound == NO_BOUND || e.bound >= g.bounds.size()) return false;
    const Bound& b = g.bounds[e.bound];
    if (e.type == Type::Ray) {
        const double dx = (b.p1.x - p.x) * (b.p1.x - b.p2.x);
        const double dy = (b.p1.y - p.y) * (b.p1.y - b.p2.y);
        return dx + dy > -EPS;
    }
    const double dx = (b.p1.x - p.x) * (b.p2.x - p.x);
    const double dy = (b.p1.y - p.y) * (b.p2.y - p.y);
    return dx + dy < EPS;
}
READABLE_ONLY bool RRootMatches(const RPoint& p, const Point& goal) {
    return RZero(p.x - goal.x) && RZero(p.y - goal.y);
}
READABLE_ONLY bool RLineCircleContains(const Graph& g, size_t lineIndex,
        const RElement& l, const RElement& c, const Point& goal) {
    const double denom = l.a * l.a + l.b * l.b;
    if (RZero(denom)) return false;
    const double dist = l.a * c.a + l.b * c.b - l.c;
    const double delta = denom * c.c - dist * dist;
    auto match = [&](const RPoint& p) {
        // NONE means the infinite radical axis of two circles.
        return g.PointAllowed({p.x, p.y}) &&
            (lineIndex == NONE || RInRange(g, lineIndex, p)) && RRootMatches(p, goal);
    };
    if (RZero(delta)) {
        return match({c.a - l.a * dist / denom, c.b - l.b * dist / denom});
    } else if (delta > EPS) {
        const double root = sqrt(delta);
        return match({c.a - (l.a * dist + l.b * root) / denom,
                      c.b - (l.b * dist - l.a * root) / denom}) ||
               match({c.a - (l.a * dist - l.b * root) / denom,
                      c.b - (l.b * dist + l.a * root) / denom});
    }
    return false;
}
READABLE_ONLY bool RIntersectionContains(const Graph& g, size_t first,
                                        size_t second, const Point& goal) {
    const Element& x = g.elements[first];
    const Element& y = g.elements[second];
    const RElement a{x.a, x.b, x.c, x.type}, b{y.a, y.b, y.c, y.type};
    if (a.type == Type::Circle) {
        if (b.type == Type::Circle) {
            const double aa = 2.0 * (a.a - b.a), bb = 2.0 * (a.b - b.b);
            if (RZero(aa) && RZero(bb)) return false;
            const double cc = a.a * a.a - b.a * b.a + a.b * a.b - b.b * b.b - a.c + b.c;
            return RLineCircleContains(g, NONE, RNormalize({aa, bb, cc, Type::Line}), a, goal);
        }
        return RLineCircleContains(g, second, b, a, goal);
    }
    if (b.type == Type::Circle) return RLineCircleContains(g, first, a, b, goal);
    const double det = a.a * b.b - a.b * b.a;
    if (RZero(det)) return false;
    const RPoint p{(a.c * b.b - a.b * b.c) / det,
                   (a.a * b.c - a.c * b.a) / det};
    return RInRange(g, first, p) && RInRange(g, second, p) && RRootMatches(p, goal);
}

READABLE_ONLY Definition FindDefinition(const Graph& g, size_t known, const Element& e) {
    if (e.type == Type::Circle) {
        for (size_t i = 0; i < known; ++i) {
            if (!RSamePoint(g.points[i], {e.a, e.b})) continue;
            for (size_t j = 0; j < known; ++j) {
                if (i != j && RSameElement(RMake(g.points[i], g.points[j], Type::Circle), e))
                    return {i, j};
            }
        }
    } else if (e.type == Type::Line) {
        for (size_t i = 0; i < known; ++i)
            for (size_t j = i + 1; j < known; ++j)
                if (RSameElement(RMake(g.points[i], g.points[j], Type::Line), e)) return {i, j};
    }
    return {NONE, NONE};
}

READABLE_ONLY Trace Reconstruct(const Graph& solved, size_t givenPointCount) {
    if (givenPointCount > solved.points.size() ||
        solved.initialElementCount > solved.elements.size() ||
        solved.pointBirth.size() != solved.points.size())
        throw runtime_error("invalid initial counts");
    Trace t;
    t.origins.resize(solved.points.size());
    t.definitions.resize(solved.elements.size());
    t.elementNames.reserve(solved.elements.size());
    array<size_t, 4> counts{};
    const array<char, 4> prefixes{'C', 'L', 'R', 'S'};
    for (const Element& e : solved.elements) {
        const size_t k = static_cast<size_t>(e.type);
        t.elementNames.push_back(string(1, prefixes.at(k)) + to_string(++counts.at(k)));
    }
    // pointBirth is already stored by the solver. Recover each point's exact
    // parent intersection and each element's already-available defining pair.
    for (size_t pi = 0; pi < solved.points.size(); ++pi) {
        if (pi && solved.pointBirth[pi] < solved.pointBirth[pi - 1])
            throw runtime_error("point births are out of order");
        if (pi < givenPointCount) {
            if (solved.pointBirth[pi] != 0) throw runtime_error("invalid given point birth");
            continue;
        }
        const size_t birth = solved.pointBirth[pi];
        const size_t begin = birth ? solved.initialElementCount + birth - 1 : 0;
        const size_t end = birth ? begin + 1 : solved.initialElementCount;
        if (end > solved.elements.size()) throw runtime_error("invalid point birth");
        bool found = false;
        for (size_t ei = begin; ei < end && !found; ++ei)
            for (size_t old = 0; old < ei; ++old)
                if (RIntersectionContains(solved, ei, old, solved.points[pi])) {
                    t.origins[pi] = {ei, old}; found = true; break;
                }
        if (!found) {
            const size_t available = birth
                ? min(solved.elements.size(), solved.initialElementCount + birth)
                : solved.initialElementCount;
            for (size_t a = 0; a < available && !found; ++a)
                for (size_t b = 0; b < a; ++b)
                    if (RIntersectionContains(solved, a, b, solved.points[pi])) {
                        t.origins[pi] = {a, b}; found = true; break;
                    }
        }
    }
    size_t known = 0;
    for (size_t ei = solved.initialElementCount; ei < solved.elements.size(); ++ei) {
        const size_t step = ei - solved.initialElementCount + 1;
        while (known < solved.points.size() && solved.pointBirth[known] < step) ++known;
        t.definitions[ei] = FindDefinition(solved, known, solved.elements[ei]);
    }

    vector<bool> needed(solved.points.size(), false);
    for (size_t i = 0; i < givenPointCount; ++i) needed[i] = true;
    for (size_t ei = solved.initialElementCount; ei < t.definitions.size(); ++ei) {
        if (t.definitions[ei].first != NONE) needed.at(t.definitions[ei].first) = true;
        if (t.definitions[ei].second != NONE) needed.at(t.definitions[ei].second) = true;
    }
    for (const Point& goal : solved.goalPoints) {
        size_t pi = 0;
        while (pi < solved.points.size() && !RSamePoint(solved.points[pi], goal)) ++pi;
        if (pi == solved.points.size()) throw runtime_error("missing point goal");
        needed[pi] = true;
        t.pointGoals.push_back(pi);
    }
    for (const Element& goal : solved.goalElements) {
        size_t ei = 0;
        while (ei < solved.elements.size() && !(solved.elements[ei].type == goal.type &&
                RZero(solved.elements[ei].a - goal.a) && RZero(solved.elements[ei].b - goal.b) &&
                RZero(solved.elements[ei].c - goal.c))) ++ei;
        if (ei == solved.elements.size()) throw runtime_error("missing element goal");
        t.elementGoals.push_back(ei);
    }
    t.pointNumbers.resize(solved.points.size());
    size_t number = 0;
    for (size_t pi = 0; pi < needed.size(); ++pi)
        if (needed[pi]) t.pointNumbers[pi] = ++number;
    return t;
}

READABLE_ONLY void WriteNumber(ostream& out, double value) {
    char buffer[64];
    const auto result = to_chars(buffer, buffer + sizeof(buffer), CleanZero(value));
    if (result.ec == errc{}) out.write(buffer, result.ptr - buffer);
    else out << setprecision(17) << CleanZero(value);
}
READABLE_ONLY void WritePoint(ostream& out, const Point& p) {
    out << '('; WriteNumber(out, p.x); out << ','; WriteNumber(out, p.y); out << ')';
}
READABLE_ONLY void WriteNamedPoint(ostream& out, const string& name, const Point& p) {
    out << name << '='; WritePoint(out, p);
}
READABLE_ONLY void WriteLineEquation(ostream& out, const Element& e) {
    out << '('; WriteNumber(out, e.a); out << ")x+(";
    WriteNumber(out, e.b); out << ")y="; WriteNumber(out, e.c);
}
READABLE_ONLY void WriteCircleEquation(ostream& out, const Element& e) {
    out << "(x-("; WriteNumber(out, e.a); out << "))^2+(y-(";
    WriteNumber(out, e.b); out << "))^2="; WriteNumber(out, e.c);
}

READABLE_ONLY string Format(const Graph& solved, size_t givenPointCount, const Trace& t) {
    ostringstream out;
    out << defaultfloat << setprecision(17);
    auto pointName = [&](size_t pi) { return "P" + to_string(t.pointNumbers.at(pi)); };
    auto writePointRef = [&](size_t pi) {
        WriteNamedPoint(out, pointName(pi), solved.points.at(pi));
    };
    auto writeEquation = [&](size_t ei) {
        const Element& e = solved.elements.at(ei);
        if (e.type == Type::Circle) WriteCircleEquation(out, e);
        else WriteLineEquation(out, e);
    };

    out << "Solution found!\n";
    out << "作图步骤（" << solved.elements.size() - solved.initialElementCount << "E）：\n";
    out << "已知对象：\n";
    for (size_t pi = 0; pi < givenPointCount; ++pi) {
        out << "  "; writePointRef(pi); out << "。\n";
    }
    for (size_t ei = 0; ei < solved.initialElementCount; ++ei) {
        const Element& e = solved.elements[ei];
        out << "  已知";
        if (e.type == Type::Circle) {
            out << "圆" << t.elementNames[ei] << "；方程：";
            WriteCircleEquation(out, e);
        } else if (e.type == Type::Line) {
            out << "直线" << t.elementNames[ei] << "；方程：";
            WriteLineEquation(out, e);
        } else {
            const Bound& bd = solved.bounds.at(e.bound);
            out << (e.type == Type::Ray ? "射线" : "线段") << t.elementNames[ei] << "：";
            if (e.type == Type::Ray) out << "起点";
            else out << "端点";
            WritePoint(out, bd.p1);
            out << (e.type == Type::Ray ? "，经过" : "和");
            WritePoint(out, bd.p2);
            out << "；所在直线方程：";
            WriteLineEquation(out, e);
        }
        out << "。\n";
    }

    auto emitIntersection = [&](size_t pi) {
        const Origin& o = t.origins[pi];
        if (o.element == NONE || o.other == NONE) {
            out << "  得到"; writePointRef(pi);
            out << "";
            return;
        }
        const size_t a = min(o.element, o.other), b = max(o.element, o.other);
        out << "  取" << t.elementNames.at(a) << ',' << t.elementNames.at(b)
            << "交点";
        writePointRef(pi);
        out << "。\n";
    };

    bool initialHeading = false;
    for (size_t pi = givenPointCount; pi < solved.points.size(); ++pi) {
        if (!t.pointNumbers[pi]) continue;
        if (t.origins[pi].element == NONE || t.origins[pi].element >= solved.initialElementCount) continue;
        if (!initialHeading) { out << "已知元素的交点：\n"; initialHeading = true; }
        emitIntersection(pi);
    }
    out << '\n';

    for (size_t ei = solved.initialElementCount; ei < solved.elements.size(); ++ei) {
        const Definition& d = t.definitions[ei];
        const Element& e = solved.elements[ei];
        out << "第" << ei - solved.initialElementCount + 1 << "步：";
        if (d.first != NONE && d.second != NONE) {
            if (e.type == Type::Circle) {
                out << "以"; writePointRef(d.first); out << "为圆心，";
                writePointRef(d.second); out << "为圆周点作圆" << t.elementNames[ei];
            } else {
                out << "过"; writePointRef(d.first); out << "，";
                writePointRef(d.second); out << "作直线" << t.elementNames[ei];
            }
        } else {
            out << (e.type == Type::Circle ? "作圆" : "作直线") << t.elementNames[ei]
                << "";
        }
        out << "；方程：";
        if (e.type == Type::Circle) WriteCircleEquation(out, e);
        else WriteLineEquation(out, e);
        out << "。\n";

        const size_t currentStep = ei - solved.initialElementCount + 1;
        for (size_t pi = givenPointCount; pi < solved.points.size(); ++pi) {
            if (!t.pointNumbers[pi]) continue;
            if (solved.pointBirth[pi] == currentStep) emitIntersection(pi);
        }
    }

    if (solved.gridMode) {
        out << "\n网格校验：全部 " << solved.points.size()
            << " 个已知点均限制在 [0," << solved.gridM << "]×[0," << solved.gridN
            << "] 内（含边界）\n";
    }
    out << "\n目标对应：\n";
    size_t lineGoal = 0, circleGoal = 0;
    for (size_t gi = 0; gi < solved.goalElements.size(); ++gi) {
        const bool circle = solved.goalElements[gi].type == Type::Circle;
        const size_t ei = t.elementGoals[gi];
        out << "  " << (circle ? "目标圆GC" : "目标直线GL")
            << (circle ? ++circleGoal : ++lineGoal) << '='
            << t.elementNames[ei] << "；方程：";
        if (circle) WriteCircleEquation(out, solved.elements[ei]);
        else WriteLineEquation(out, solved.elements[ei]);
        out << "。\n";
    }
    for (size_t gi = 0; gi < t.pointGoals.size(); ++gi) {
        const size_t pi = t.pointGoals[gi];
        out << "  目标点GP" << gi + 1 << '=';
        writePointRef(pi);
        out << "。\n";
    }
    if (solved.elements.size() == solved.initialElementCount)
        out << "目标已由给定对象满足，无须新增作图元素。\n";
    return out.str();
}

#if defined(__GNUC__) || defined(__clang__)
__attribute__((noinline, cold))
#endif
void Print(const Graph& solved, size_t givenPointCount) {
    // Re-evaluate numerical definitions/parent intersections only after success.
    // Build and validate the entire report before writing any steps. Failure is
    // a reporting failure only.
    try {
        const Trace trace = Reconstruct(solved, givenPointCount);
        cout << Format(solved, givenPointCount, trace);
    } catch (const exception& ex) {
        for (size_t pi = 0; pi < solved.points.size(); ++pi) {
            cout << "  P" << pi + 1 << '='; WritePoint(cout, solved.points[pi]); cout << "。\n";
        }
        for (size_t ei = 0; ei < solved.elements.size(); ++ei) {
            const Element& e = solved.elements[ei];
            cout << "  " << (e.type == Type::Circle ? "圆C" : e.type == Type::Line ? "直线L" : e.type == Type::Ray ? "射线R" : "线段S")
                 << ei + 1 << "；" << (e.type == Type::Circle ? "方程：" : "所在直线方程：");
            if (e.type == Type::Circle) WriteCircleEquation(cout, e);
            else WriteLineEquation(cout, e);
            cout << "。\n";
        }
    }
}
#undef READABLE_ONLY
} // namespace readable_report

} // namespace bs
