#pragma once

#include "Docking.h"

#include <unordered_set>

namespace pnr {

// Necessary input proofs for one physical driver in one source-move view.
// Private output/earlier-input reservations must never enter these proofs.
class MovingInputCutCache {
public:
    struct Key {
        fpga::Tile* target = nullptr;
        NodeMask pins{};
        fpga::Coord source_hint{};
        int source_pos = -1;
        int radius = 0, depth = 0;
        // A null anchor denotes the unchanged driver's complete anchor set
        // and source takeoff probe; otherwise only this landing is a goal.
        fpga::Tile* anchor = nullptr;
        int dst = -1;
        bool operator==(const Key&) const = default;
    };
    struct Result { bool possible; bool reused; };

    explicit MovingInputCutCache(size_t max_entries=128,size_t max_nodes=16384)
        : max_entries(max_entries),max_nodes(max_nodes) {}

    // Compare complete numeric release sets, not branch labels or hashes
    // alone. Duplicate/order variants of the same cut share the same proof.
    template<class Search,class Cancel>
    Result prove(const Key& key,const std::vector<fpga::RouteCutNode>& releases,
                 const Search& search,const Cancel& cancelled) {
        if(revision!=fpga::routeMutationRevision())clear();
        if(cancelled())return {false,false};
        if(releases.size()>max_nodes)return {search(),false};
        Footprint footprint;
        for(const auto& cut:releases)
            footprint.insert({cut.tile.x,cut.tile.y,cut.type,cut.node});
        size_t hash=0;
        for(const auto& node:footprint)hash+=BackwardLocalKeyHash{}(node);
        for(const auto& entry:entries)
            if(entry.hash==hash && entry.key==key && entry.releases==footprint)
                return {entry.possible,true};
        const bool possible=search();
        // Cancellation or a live edit is not an exhausted immutable proof.
        if(cancelled())return {possible,false};
        if(revision!=fpga::routeMutationRevision()) {clear();return {possible,false};}
        if(max_entries && footprint.size()<=max_nodes) {
            if(entries.size()>=max_entries || nodes+footprint.size()>max_nodes)clear();
            nodes+=footprint.size();
            entries.push_back({key,std::move(footprint),hash,possible});
        }
        return {possible,false};
    }

private:
    using Footprint=std::unordered_set<BackwardLocalKey,BackwardLocalKeyHash>;
    struct Entry {Key key;Footprint releases;size_t hash;bool possible;};
    std::vector<Entry> entries;
    size_t nodes=0,max_entries,max_nodes;
    uint64_t revision=fpga::routeMutationRevision();
    void clear() {entries.clear();nodes=0;revision=fpga::routeMutationRevision();}
};

} // namespace pnr
