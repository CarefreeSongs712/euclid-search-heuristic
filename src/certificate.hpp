#pragma once
#include "geometry.hpp"

namespace bs {

inline bool SameElementBits(const Element& a, const Element& b) {
    return a.type == b.type && a.bound == b.bound &&
        bit_cast<uint64_t>(a.a) == bit_cast<uint64_t>(b.a) &&
        bit_cast<uint64_t>(a.b) == bit_cast<uint64_t>(b.b) &&
        bit_cast<uint64_t>(a.c) == bit_cast<uint64_t>(b.c);
}

// Cold-path export only: every paid element needs a witness using points born
// before that step. A goal coordinate alone is never an acceptable witness.
inline void WriteConstructionCertificate(ostream& out, const Graph& initial,
                                         const Graph& solved) {
    Graph replay = initial;
    replay.SetStateHashingEnabled(false);
    out << setprecision(17) << "{\"steps\":[";
    for (size_t index = initial.elements.size(); index < solved.elements.size(); ++index) {
        const Element& expected = solved.elements[index];
        optional<Candidate> witness;
        for (uint32_t i = 0; i < replay.points.size() && !witness; ++i) {
            for (uint32_t j = i + 1; j < replay.points.size() && !witness; ++j) {
                const uint8_t begin = expected.type == Type::Line ? 2 : 0;
                const uint8_t end = expected.type == Type::Line ? 3 : 2;
                for (uint8_t tool = begin; tool < end; ++tool) {
                    Candidate c{i,j,tool};
                    if (SameElementBits(replay.MakeCandidate(c),expected)) {
                        witness=c;
                        break;
                    }
                }
            }
        }
        if (!witness) throw runtime_error("Cannot certify paid element from known points");
        const Point first = replay.points[witness->tool == 1 ? witness->j : witness->i];
        const Point second = replay.points[witness->tool == 1 ? witness->i : witness->j];
        if (index != initial.elements.size()) out << ',';
        out << "{\"type\":\"" << (expected.type==Type::Line ? "line" : "circle")
            << "\",\"first\":[" << first.x << ',' << first.y
            << "],\"second\":[" << second.x << ',' << second.y
            << "],\"element\":[" << expected.a << ',' << expected.b << ',' << expected.c << "]}";
        if (!replay.Apply(expected,static_cast<uint16_t>(index-initial.elements.size()+1)))
            throw runtime_error("Repeated paid element in certificate");
    }
    if (!replay.GoalsMet()) throw runtime_error("Certificate replay did not satisfy goals");
    out << "],\"E\":" << solved.elements.size()-initial.elements.size() << '}';
}

} // namespace bs
