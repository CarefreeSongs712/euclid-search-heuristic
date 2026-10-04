#pragma once
#include "geometry.hpp"
#include <map>
#include <set>

namespace bs::heuristic_detail {

using RawElementKey = array<uint64_t, 4>;
inline RawElementKey RawKey(const Element& e) {
    return {static_cast<uint64_t>(e.type), bit_cast<uint64_t>(e.a),
            bit_cast<uint64_t>(e.b), bit_cast<uint64_t>(e.c)};
}
inline vector<RawElementKey> ConstructionFamily(const vector<Element>& prefix,
                                               const Element& last) {
    vector<RawElementKey> key;
    key.reserve(prefix.size()+1);
    for (const Element& e : prefix) key.push_back(RawKey(e));
    key.push_back(RawKey(last));
    sort(key.begin(),key.end());
    return key;
}

// Operation sets define heuristic diversity families, NOT interchangeable
// ordered floating-point states. Keep the selected prefix's original order.
template<class T>
class FamilyPool {
    using Key=vector<RawElementKey>;
    struct Record {T value; Key key;};
    struct Order {
        const vector<Record>* records;
        bool operator()(size_t a,size_t b)const {
            const T& x=(*records)[a].value;
            const T& y=(*records)[b].value;
            if(x.score!=y.score)return x.score<y.score;
            if(x.serial!=y.serial)return x.serial>y.serial;
            return a<b;
        }
    };
    size_t capacity_;
    vector<Record> slots_;
    map<Key,size_t> families_;
    std::set<size_t,Order> order_;
    uint64_t& discarded_;
    uint64_t& merged_;
    bool& limited_;
    static bool Better(const T& a,const T& b) {
        return a.score>b.score || (a.score==b.score && a.serial<b.serial);
    }
    void Drop(bool merged=false) {
        ++discarded_;if(merged)++merged_;limited_=true;
    }
public:
    FamilyPool(size_t capacity,uint64_t& discarded,uint64_t& merged,bool& limited)
        :capacity_(capacity),order_(Order{&slots_}),discarded_(discarded),merged_(merged),limited_(limited) {
        slots_.reserve(capacity_);
    }
    FamilyPool(const FamilyPool&)=delete;
    FamilyPool& operator=(const FamilyPool&)=delete;
    ~FamilyPool() {
        if(!slots_.empty()){discarded_+=slots_.size();limited_=true;}
    }
    void Offer(T value,Key key) {
        auto existing=families_.find(key);
        if(existing!=families_.end()) {
            const size_t id=existing->second;
            if(Better(value,slots_[id].value)) {
                order_.erase(id);
                slots_[id].value=std::move(value);
                order_.insert(id);
            }
            Drop(true);return;
        }
        if(!capacity_){Drop();return;}
        if(slots_.size()<capacity_) {
            const size_t id=slots_.size();
            slots_.push_back({std::move(value),std::move(key)});
            families_.emplace(slots_[id].key,id);order_.insert(id);
            return;
        }
        const size_t worst=*order_.begin();
        if(Better(value,slots_[worst].value)) {
            order_.erase(order_.begin());families_.erase(slots_[worst].key);
            slots_[worst]={std::move(value),std::move(key)};
            families_.emplace(slots_[worst].key,worst);order_.insert(worst);
        }
        Drop();
    }
    vector<T> Take() {
        vector<T> result;result.reserve(slots_.size());
        order_.clear();families_.clear();
        for(auto& slot:slots_)result.push_back(std::move(slot.value));
        slots_.clear();
        sort(result.begin(),result.end(),Better);
        return result;
    }
};

} // namespace bs::heuristic_detail
