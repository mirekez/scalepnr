#pragma once

#include "Docking.h"

#include <unordered_set>
#include <type_traits>
#include <memory>

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

    explicit MovingInputCutCache(size_t max_entries=128,size_t max_nodes=16384,
                                 size_t max_boundary_nodes=262144)
        : max_entries(max_entries),max_nodes(max_nodes),max_boundary_nodes(max_boundary_nodes) {}

    // Count unique retained boundary nodes, not references from anchor proofs.
    size_t boundaryNodeCount() const { return boundary_nodes; }

    // Compare complete numeric release sets, not branch labels or hashes
    // alone. An observer-capable fixed-anchor search also records its exhausted
    // region's blocked boundary. Every occupied rejection must reach the
    // observer; another cut cannot help unless it opens that boundary.
    template<class Search,class Cancel>
    Result prove(const Key& key,const std::vector<fpga::RouteCutNode>& releases,
                 const Search& search,const Cancel& cancelled) {
        if(revision!=fpga::routeMutationRevision())clear();
        if(cancelled())return {false,false};
        Footprint boundary;
        // A fixed landing has no hidden takeoff/packing predicate. A driver
        // probe may reject a free SRC for reasons absent from this boundary;
        // keep those proofs scoped to the exact release set.
        bool boundary_complete=key.anchor && std::is_invocable_v<Search,const InputRouteBlocked&>;
        InputRouteBlocked blocked=[&](fpga::Tile& tile,fpga::CBNodeNameType kind,int node) {
            if(!boundary_complete)return;
            boundary.insert({tile.coord.x,tile.coord.y,kind,node});
            if(boundary.size()>max_boundary_nodes) {
                boundary.clear();boundary_complete=false;
            }
        };
        auto run=[&] {
            if constexpr(std::is_invocable_v<Search,const InputRouteBlocked&>) return search(blocked);
            else return search();
        };
        if(releases.size()>max_nodes)return {run(),false};
        Footprint footprint;
        for(const auto& cut:releases)
            footprint.insert({cut.tile.x,cut.tile.y,cut.type,cut.node});
        size_t hash=0;
        for(const auto& node:footprint)hash+=BackwardLocalKeyHash{}(node);
        for(const auto& entry:entries) {
            if(entry.key!=key)continue;
            if(entry.hash==hash && entry.releases==footprint)
                return {entry.possible,true};
            if(!entry.possible && entry.boundary) {
                bool opens=false;
                for(const auto& cut:footprint)if(entry.boundary->nodes.contains(cut)){opens=true;break;}
                // Removing a previous release only removes free resources;
                // new releases outside this boundary cannot expose a route.
                if(!opens)return {false,true};
            }
        }
        const bool possible=run();
        // Cancellation or a live edit is not an exhausted immutable proof.
        if(cancelled())return {possible,false};
        if(revision!=fpga::routeMutationRevision()) {clear();return {possible,false};}
        if(max_entries && footprint.size()<=max_nodes) {
            if(entries.size()>=max_entries || nodes+footprint.size()>max_nodes)clear();
            nodes+=footprint.size();
            std::shared_ptr<const Boundary> saved;
            if(!possible && boundary_complete) {
                // Different anchors can exhaust the same region. Share only
                // the immutable witness; proof keys and results stay separate.
                size_t boundary_hash=0;
                for(const auto& node:boundary)boundary_hash+=BackwardLocalKeyHash{}(node);
                for(const auto& entry:entries)
                    if(entry.boundary && entry.boundary->hash==boundary_hash &&
                       entry.boundary->nodes==boundary) {
                        saved=entry.boundary;break;
                    }
                // Bound aggregate boundary storage independently of cut-set
                // entries. Dropping a witness only causes recomputation.
                if(!saved) {
                    if(boundary_nodes+boundary.size()>max_boundary_nodes) {
                        for(auto& entry:entries)entry.boundary.reset();
                        boundary_nodes=0;
                    }
                    boundary_nodes+=boundary.size();
                    saved=std::make_shared<Boundary>(std::move(boundary),boundary_hash);
                }
            }
            entries.push_back({key,std::move(footprint),hash,possible,std::move(saved)});
        }
        return {possible,false};
    }

private:
    using Footprint=std::unordered_set<BackwardLocalKey,BackwardLocalKeyHash>;
    struct Boundary {Footprint nodes;size_t hash;};
    struct Entry {Key key;Footprint releases;size_t hash;bool possible;std::shared_ptr<const Boundary> boundary;};
    std::vector<Entry> entries;
    size_t nodes=0,max_entries,max_nodes;
    size_t boundary_nodes=0,max_boundary_nodes;
    uint64_t revision=fpga::routeMutationRevision();
    void clear() {entries.clear();nodes=boundary_nodes=0;revision=fpga::routeMutationRevision();}
};

} // namespace pnr
