#pragma once
#include "family_pool.hpp"

namespace bs::heuristic_detail {

// A bounded, INCOMPLETE family selector, not an equivalence/deduplication proof.
// Keep the best 3/4 by score and a stable pseudorandom 1/4 of the other families.
// A representative keeps its original ordered prefix; a full raw-vector key is
// only a diversity family. EPS-near keys and hash collisions remain distinct.
//
// The first offered diversity value salts this pool's family ranks. Later offers
// of the same key keep the same rank, even after eviction: operation permutations
// cannot buy extra lottery tickets. Duplicate representatives compete by score
// and serial only, never by their per-offer diversity value.
//
// Two indexes retain top-elite scores and top-capacity family ranks. The latter
// needs capacity (not merely the random quota): up to eliteLimit ranks may be
// occupied by elites, including families promoted by a later better offer. Their
// union stores at most eliteLimit+capacity records, O(capacity*prefix_depth), and
// Take returns at most capacity distinct families. Spare ranked records count as
// discards at Take/cancellation. No history-sized seen-key table is required.
template<class T>
class NoveltyPool {
    using Key = vector<RawElementKey>;
    struct Record {
        T value;
        uint64_t priority;
        const Key* key = nullptr; // Immutable map key; stable until this record is erased.
        bool elite = false, random = false;
    };
    struct ScoreOrder {
        bool operator()(const Record* a, const Record* b) const {
            if (a->value.score != b->value.score) return a->value.score > b->value.score;
            if (a->value.serial != b->value.serial) return a->value.serial < b->value.serial;
            return *a->key < *b->key;
        }
    };
    struct RandomOrder {
        bool operator()(const Record* a, const Record* b) const {
            if (a->priority != b->priority) return a->priority > b->priority;
            return *a->key < *b->key; // Hash equality is never family equality.
        }
    };
    size_t capacity_, eliteLimit_, randomLimit_;
    map<Key, Record> records_;
    std::set<Record*, ScoreOrder> elite_;
    std::set<Record*, RandomOrder> random_;
    uint64_t& discarded_;
    uint64_t& merged_;
    bool& limited_;
    uint64_t salt_ = 0;
    bool seeded_ = false;

    static bool Better(const T& a, const T& b) {
        return a.score > b.score || (a.score == b.score && a.serial < b.serial);
    }
    uint64_t Priority(const Key& key) const {
        uint64_t rank = SplitMix64(salt_ ^ SplitMix64(static_cast<uint64_t>(key.size())));
        for (const RawElementKey& element : key)
            for (uint64_t word : element) rank = SplitMix64(rank ^ word);
        return rank;
    }
    void Drop(bool merged = false) {
        ++discarded_;
        if (merged) ++merged_;
        limited_ = true;
    }
    void EraseUnused(Record* record) {
        if (record->elite || record->random) return;
        const auto it = records_.find(*record->key);
        records_.erase(it);
        Drop();
    }
    void OfferElite(Record* record) {
        if (!eliteLimit_ || (elite_.size() >= eliteLimit_ &&
            !ScoreOrder{}(record, *elite_.rbegin()))) return;
        elite_.insert(record);
        record->elite = true;
        if (elite_.size() > eliteLimit_) {
            auto worst = prev(elite_.end());
            Record* victim = *worst;
            elite_.erase(worst);
            victim->elite = false;
            EraseUnused(victim);
        }
    }
    void OfferRandom(Record* record) {
        if (!randomLimit_ || (random_.size() >= randomLimit_ &&
            !RandomOrder{}(record, *random_.rbegin()))) return;
        random_.insert(record);
        record->random = true;
        if (random_.size() > randomLimit_) {
            auto worst = prev(random_.end());
            Record* victim = *worst;
            random_.erase(worst);
            victim->random = false;
            EraseUnused(victim);
        }
    }

public:
    NoveltyPool(size_t capacity, uint64_t& discarded, uint64_t& merged, bool& limited)
        : capacity_(capacity),
          eliteLimit_(capacity - (capacity > 1 ? max<size_t>(1, capacity / 4) : 0)),
          randomLimit_(capacity > 1 ? capacity : 0),
          discarded_(discarded), merged_(merged), limited_(limited) {}
    NoveltyPool(const NoveltyPool&) = delete;
    NoveltyPool& operator=(const NoveltyPool&) = delete;
    // Every representative still owned during cancellation is unfinished work.
    ~NoveltyPool() {
        if (!records_.empty()) {
            discarded_ += static_cast<uint64_t>(records_.size());
            limited_ = true;
        }
    }
    void Offer(T value, Key key) {
        if (!seeded_) { salt_ = value.diversity; seeded_ = true; }
        auto existing = records_.find(key);
        if (existing != records_.end()) {
            Record* record = &existing->second;
            if (Better(value, record->value)) {
                // Erase before mutating the score used by the elite comparator.
                const bool wasElite = record->elite;
                if (wasElite) elite_.erase(record);
                record->value = std::move(value);
                if (wasElite) elite_.insert(record); // Improvement cannot demote an elite.
                else OfferElite(record);
            }
            Drop(true);
            return;
        }
        if (!capacity_) { Drop(); return; }
        const uint64_t priority = Priority(key);
        auto [it, inserted] = records_.emplace(std::move(key), Record{std::move(value), priority});
        (void)inserted;
        Record* record = &it->second;
        record->key = &it->first;
        OfferElite(record);
        OfferRandom(record);
        EraseUnused(record);
    }
    vector<T> Take() {
        vector<Record*> selected;
        selected.reserve(min(capacity_, records_.size()));
        for (Record* record : elite_) selected.push_back(record);
        for (Record* record : random_) {
            if (selected.size() >= capacity_) break;
            if (!record->elite) selected.push_back(record);
        }
        sort(selected.begin(), selected.end(), ScoreOrder{});
        vector<T> result;
        result.reserve(selected.size());
        const size_t extras = records_.size() - selected.size();
        if (extras) {
            discarded_ += static_cast<uint64_t>(extras);
            limited_ = true;
        }
        // No ordered index may observe a moved-from representative.
        elite_.clear();
        random_.clear();
        for (Record* record : selected) result.push_back(std::move(record->value));
        records_.clear();
        seeded_ = false;
        return result;
    }
};

} // namespace bs::heuristic_detail
