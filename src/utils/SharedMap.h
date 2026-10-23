#pragma once

#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

// Copies share read-only storage; every explicit write first detaches its owner.
template<typename Map>
class SharedMap
{
    struct Storage
    {
        Map values;
        std::optional<size_t> hash;
    };

public:
    using key_type = typename Map::key_type;
    using mapped_type = typename Map::mapped_type;

    // A preparation-scoped index owns no maps and verifies equality after hashing.
    class Pool
    {
    public:
        template<typename Hash>
        bool share(SharedMap& map, Hash hash)
        {
            if (map.empty()) {
                map.clear();
                return false;
            }
            if (!map.storage_->hash) {
                map.storage_->hash = hash(map.read());
            }
            auto& candidates = buckets_[*map.storage_->hash];
            for (auto it = candidates.begin(); it != candidates.end();) {
                auto candidate = it->lock();
                if (!candidate) {
                    it = candidates.erase(it);
                    continue;
                }
                if (candidate == map.storage_) {
                    return false;
                }
                if (candidate->values == map.read()) {
                    map.storage_ = std::move(candidate);
                    return true;
                }
                ++it;
            }
            candidates.emplace_back(map.storage_);
            return false;
        }

    private:
        std::unordered_map<size_t, std::vector<std::weak_ptr<Storage>>> buckets_;
    };

    const Map& read() const
    {
        if (storage_) return storage_->values;
        static const Map empty;
        return empty;
    }

    Map& write()
    {
        if (!storage_) {
            storage_ = std::make_shared<Storage>();
        }
        else if (!storage_.unique()) {
            storage_ = std::make_shared<Storage>(Storage{storage_->values, {}});
        }
        storage_->hash.reset();
        return storage_->values;
    }

    bool empty() const { return read().empty(); }
    size_t size() const { return read().size(); }
    auto begin() const { return read().begin(); }
    auto end() const { return read().end(); }
    auto find(const key_type& key) const { return read().find(key); }
    bool contains(const key_type& key) const { return read().contains(key); }
    const mapped_type& at(const key_type& key) const { return read().at(key); }
    mapped_type& operator[](const key_type& key) { return write()[key]; }
    void clear() { storage_.reset(); }
    size_t erase(const key_type& key) { return contains(key) ? write().erase(key) : 0; }

    template<typename... Args>
    auto emplace(Args&&... args) { return write().emplace(std::forward<Args>(args)...); }

    template<typename... Args>
    auto try_emplace(Args&&... args) { return write().try_emplace(std::forward<Args>(args)...); }

    template<typename... Args>
    auto insert_or_assign(Args&&... args) { return write().insert_or_assign(std::forward<Args>(args)...); }

    bool sharesStorageWith(const SharedMap& other) const { return storage_ == other.storage_; }

private:
    std::shared_ptr<Storage> storage_;
};
