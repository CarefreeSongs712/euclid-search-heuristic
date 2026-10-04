#include "../src/novelty_pool.hpp"
#include <random>

using namespace bs;
using namespace bs::heuristic_detail;

namespace {

struct Value {
    double score;
    uint64_t serial, diversity, id;
};
using Key = vector<RawElementKey>;

Key Family(uint64_t id) { return {{id, 0, 0, 0}, {0, id * 7, 0, 0}}; }

uint64_t Priority(const Key& key, uint64_t salt) {
    uint64_t rank = SplitMix64(salt ^ SplitMix64(static_cast<uint64_t>(key.size())));
    for (const auto& element : key)
        for (uint64_t word : element) rank = SplitMix64(rank ^ word);
    return rank;
}

bool Better(const Value& a, const Value& b) {
    return a.score > b.score || (a.score == b.score && a.serial < b.serial);
}
bool ScoreOrder(const Value& a, const Value& b) {
    return Better(a, b) || (!Better(b, a) && Family(a.id) < Family(b.id));
}
void Require(bool condition, const char* message) {
    if (!condition) throw runtime_error(message);
}
vector<uint64_t> Ids(const vector<Value>& values) {
    vector<uint64_t> result;
    for (const auto& value : values) result.push_back(value.id);
    return result;
}

// Deliberately unbounded OFFLINE oracle: retain every family's best offer, then
// select score elites and random-ranked non-elites. Production cannot keep this
// history. Testing promotions/reentries against it catches lost-rank contenders.
vector<Value> Oracle(const vector<Value>& input, size_t capacity) {
    map<Key, Value> best;
    for (const auto& value : input) {
        auto it = best.find(Family(value.id));
        if (it == best.end() || Better(value, it->second))
            best.insert_or_assign(Family(value.id), value);
    }
    vector<Value> ranked;
    for (const auto& [key, value] : best) {
        (void)key;
        ranked.push_back(value);
    }
    sort(ranked.begin(), ranked.end(), ScoreOrder);
    const size_t eliteLimit = capacity - (capacity > 1 ? max<size_t>(1, capacity / 4) : 0);
    const size_t count = min(eliteLimit, ranked.size());
    vector<Value> output(ranked.begin(), ranked.begin() + count);
    vector<Value> rest(ranked.begin() + count, ranked.end());
    const uint64_t salt = input.empty() ? 0 : input.front().diversity;
    sort(rest.begin(), rest.end(), [&](const Value& a, const Value& b) {
        const auto x = Priority(Family(a.id), salt), y = Priority(Family(b.id), salt);
        return x > y || (x == y && Family(a.id) < Family(b.id));
    });
    for (const auto& value : rest) {
        if (output.size() == capacity) break;
        output.push_back(value);
    }
    sort(output.begin(), output.end(), ScoreOrder);
    return output;
}

void RandomizedOracle() {
    mt19937_64 random(379); // Fixed stream; no geometry/search/time-limit dependency.
    for (size_t capacity = 0; capacity <= 65; ++capacity) {
        for (int trial = 0; trial < 12; ++trial) {
            vector<Value> stream;
            uint64_t discarded = 0, merged = 0;
            bool limited = false;
            size_t returned = 0;
            {
                NoveltyPool<Value> pool(capacity, discarded, merged, limited);
                for (uint64_t i = 0; i < 500; ++i) {
                    Value value{double(random() % 19), i, random(), random() % 150};
                    stream.push_back(value);
                    pool.Offer(value, Family(value.id));
                }
                auto actual = pool.Take(), expected = Oracle(stream, capacity);
                Require(Ids(actual) == Ids(expected), "online selection differs from offline family oracle");
                for (size_t i = 0; i < actual.size(); ++i) {
                    Require(actual[i].serial == expected[i].serial, "wrong best representative");
                    Require(actual[i].diversity == expected[i].diversity, "representative diversity was overwritten");
                }
                returned = actual.size();
                Require(discarded + returned == stream.size(), "Take discard accounting");
                Require(merged <= discarded && limited, "merge/limited accounting");
                Require(pool.Take().empty(), "second Take not empty");
            }
            Require(discarded + returned == stream.size(), "destructor double-counted after Take");
        }
    }
}

void Cancellation() {
    for (size_t capacity : {size_t(0), size_t(1), size_t(4), size_t(64)}) {
        uint64_t discarded = 0, merged = 0;
        bool limited = false;
        {
            NoveltyPool<Value> pool(capacity, discarded, merged, limited);
            for (uint64_t i = 0; i < 300; ++i)
                pool.Offer({double(i % 9), i, i * 11, i % 80}, Family(i % 80));
        }
        Require(discarded == 300 && limited, "cancelled ownership accounting");
    }
}

void Reuse() {
    uint64_t discarded = 0, merged = 0;
    bool limited = false;
    NoveltyPool<Value> pool(4, discarded, merged, limited);
    vector<Value> first, second;
    for (uint64_t i = 0; i < 20; ++i) {
        Value value{double(i % 3), i, i + 4, i};
        first.push_back(value);
        pool.Offer(value, Family(i));
    }
    Require(Ids(pool.Take()) == Ids(Oracle(first, 4)), "reuse first batch");
    for (uint64_t i = 0; i < 20; ++i) {
        Value value{double(i % 5), i, i + 777, i};
        second.push_back(value);
        pool.Offer(value, Family(i));
    }
    Require(Ids(pool.Take()) == Ids(Oracle(second, 4)), "reuse random salt");
    Require(discarded == 32, "reuse discard count");
}

void RepeatedFamily() {
    uint64_t discarded = 0, merged = 0;
    bool limited = false;
    NoveltyPool<Value> pool(8, discarded, merged, limited);
    vector<Value> stream;
    for (uint64_t i = 0; i < 200; ++i) {
        Value value{double(i % 8), i, i + 19, i};
        stream.push_back(value);
        pool.Offer(value, Family(i));
    }
    for (uint64_t i = 0; i < 800; ++i) {
        Value value{0, 1000 + i, numeric_limits<uint64_t>::max() - i, 80};
        stream.push_back(value);
        pool.Offer(value, Family(80));
    }
    Require(Ids(pool.Take()) == Ids(Oracle(stream, 8)), "duplicate permutations increased family lottery");
    Require(discarded == 992, "repeated family count");
}

void RankCollision() {
    // SplitMix64 is bijective. Invert xor-shifts and multiplications to create
    // DISTINCT full keys with exactly equal random rank; neither may be merged.
    auto unxor = [](uint64_t x, int shift) {
        uint64_t y = x;
        for (int s = shift; s < 64; s += shift) y ^= x >> s;
        return y;
    };
    auto inverse = [&](uint64_t x) {
        x = unxor(x, 31); x *= 0x319642b2d24d8ec3ULL;
        x = unxor(x, 27); x *= 0x96de1b173f119089ULL;
        x = unxor(x, 30);
        return x - 0x9e3779b97f4a7c15ULL;
    };
    const uint64_t salt = 5;
    Key a{{1, 2, 3, 4}}, b{{9, 2, 3, 0}};
    uint64_t before = SplitMix64(salt ^ SplitMix64(uint64_t(1)));
    for (int i = 0; i < 3; ++i) before = SplitMix64(before ^ b[0][i]);
    b[0][3] = before ^ inverse(Priority(a, salt));
    Require(a != b && Priority(a, salt) == Priority(b, salt), "test failed to create rank collision");
    uint64_t discarded = 0, merged = 0;
    bool limited = false;
    NoveltyPool<Value> pool(4, discarded, merged, limited);
    pool.Offer({1, 1, salt, 1}, a);
    pool.Offer({1, 2, 99, 2}, b);
    const auto output = pool.Take();
    Require(output.size() == 2 && discarded == 0 && merged == 0 && !limited,
            "hash collision merged full keys");
}

void BestRepresentativeAndQuota() {
    uint64_t discarded = 0, merged = 0;
    bool limited = false;
    NoveltyPool<Value> pool(8, discarded, merged, limited);
    for (uint64_t i = 0; i < 6; ++i)
        pool.Offer({100.0 - double(i), i, 17 + i, i}, Family(i));
    for (uint64_t i = 6; i < 100; ++i)
        pool.Offer({0, i, 17 + i, i}, Family(i));
    // Replacing an elite by a better score and then a lower serial must preserve
    // that offered representative verbatim, not whichever has higher diversity.
    pool.Offer({110, 500, 0, 0}, Family(0));
    pool.Offer({110, 499, 1, 0}, Family(0));
    pool.Offer({110, 501, numeric_limits<uint64_t>::max(), 0}, Family(0));
    const auto output = pool.Take();
    Require(output.size() == 8 && output.front().serial == 499 && output.front().diversity == 1,
            "best-score/serial representative changed");
    size_t lowScore = 0;
    for (const auto& value : output) if (value.score == 0) ++lowScore;
    Require(lowScore == 2 && merged == 3 && discarded == 95, "3/4 elite + 1/4 novelty quota/count");
}

} // namespace

int main() {
    try {
        RandomizedOracle();
        Cancellation();
        Reuse();
        RepeatedFamily();
        RankCollision();
        BestRepresentativeAndQuota();
        cout << "PASS novelty pool: 792 fixed-seed oracle streams, cancellation, reuse, "
                "duplicate fairness, rank collisions, representatives and quotas\n";
        return 0;
    } catch (const exception& error) {
        cerr << "FAIL novelty pool: " << error.what() << '\n';
        return 1;
    }
}
