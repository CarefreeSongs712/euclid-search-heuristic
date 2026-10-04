#include "solver.hpp"
#include <fstream>

using namespace bs;

int main(int argc, char** argv) {
    if (argc != 3) return 1;
    ifstream input(argv[1]), certificate(argv[2]);
    int budget, mode, np, nl, nr, ns, nc;
    input >> budget >> mode >> np >> nl >> nr >> ns >> nc;
    if (!input || mode != 2 || nl || nr || nc) return 1;
    Graph graph;
    for (int i = 0; i < np; ++i) {
        Point p;
        input >> p.x >> p.y;
        graph.AddPoint(p, 0);
    }
    for (int i = 0; i < ns; ++i) {
        Point a, b;
        input >> a.x >> a.y >> b.x >> b.y;
        graph.AddInitialBounded(a, b, Type::Segment);
    }
    graph.initialElementCount = graph.elements.size();
    int gl, gc, gp;
    input >> gl >> gc >> gp;
    if (gc) return 1;
    for (int i = 0; i < gl; ++i) {
        double a, b, c;
        input >> a >> b >> c;
        graph.goalElements.push_back(Element::FromCoefficients(a, b, c, Type::Line));
    }
    for (int i = 0; i < gp; ++i) {
        Point p;
        input >> p.x >> p.y;
        graph.goalPoints.push_back(p);
    }
    if (!input) return 1;
    int steps;
    certificate >> steps;
    for (int step = 1; step <= steps; ++step) {
        int type;
        Point a, b;
        certificate >> type >> a.x >> a.y >> b.x >> b.y;
        if (!certificate) return 1;
        auto known = [&](Point target) -> const Point* {
            for (const auto& point : graph.points) if (SamePoint(target, point)) return &point;
            return nullptr;
        };
        const Point *pa = known(a), *pb = known(b);
        if (!pa || !pb) {
            cerr << "Unknown defining point at step " << step << '\n';
            return 2;
        }
        const Element element = Element::FromPoints(*pa, *pb, type == 0 ? Type::Circle : Type::Line);
        if (!graph.Apply(element, static_cast<uint16_t>(step))) {
            cerr << "Repeated element at step " << step << '\n';
            return 2;
        }
    }
    if (!graph.GoalsMet()) {
        cerr << "Certificate failed to realize every point and QR/ST line\n";
        return 2;
    }
    cout << "PASS: " << steps << " paid operations, only ABC initially known, "
         << "bounded AB/AC/BC, both QR/ST and all four target points verified\n";
    return 0;
}
