#pragma once

#include "Element.h"
#include "Inst.h"
#include "Tile.h"
#include "RouteSource.h"
#include "MovingInputReachability.h"

#include <algorithm>
#include <functional>
#include <memory>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pnr {

// One moving cluster and placement snapshot. Packing survives routing-only
// changes; physical input proofs belong to one unchanged routing view.
// Rejection skips placement probing, never traversal through the Tile.
class MovingPlacementBlacklist {
public:
    explicit MovingPlacementBlacklist(std::vector<std::vector<rtl::Inst*>> orders)
        : orders_(std::move(orders)) {}

    // Exhaust packing positions without live route checks or first-fit assumptions.
    // Cancellation and routing failures are never evidence for blacklisting a tile.
    bool rejects(fpga::Tile& tile, const std::function<bool()>& cancelled = {}) {
        if (auto found = checked_.find(&tile); found != checked_.end()) {
            if (found->second) ++hits;
            return found->second;
        }
        if (cancelled && cancelled()) return false;
        ++checks;
        if (!tile.tile_type || tile.tile_type->elements.empty()) {
            checked_.emplace(&tile, true);
            ++blocked_count_;
            return true;
        }
        if (orders_.empty()) return false;
        // Source moving probes while the original cluster is still placed.
        // Detach references, not slots or wires, and restore them after preview.
        struct Saved {
            rtl::Inst* inst;
            Referable<fpga::Tile>* tile;
            fpga::Coord coord;
            int pos;
        };
        struct Restore {
            std::vector<Saved> saved;
            ~Restore() {
                for (auto& old : saved) {
                    old.inst->tile.clear();
                    if (old.tile) old.inst->tile.set(old.tile);
                    old.inst->coord = old.coord;
                    old.inst->pos = old.pos;
                }
            }
        } restore;
        // Original locations need their existing placement checked, not a free slot.
        for (auto* member : orders_.front())
            if (!member || !member->cell_ref.peer || member->tile.peer == &tile) return false;
        for (auto* member : orders_.front()) {
            restore.saved.push_back({member, member->tile.peer, member->coord, member->pos});
            member->tile.clear();
            member->pos = -1;
        }
        fpga::ElementPackingPreview preview(tile);
        bool interrupted = false;
        for (const auto& order : orders_) {
            auto pack = [&](auto&& self, size_t index) -> bool {
                if (cancelled && cancelled()) {
                    interrupted = true;
                    return false;
                }
                if (index == order.size()) return true;
                auto* member = order[index];
                for (int pos : tile.candidatePositions(member)) {
                    const size_t checkpoint = preview.checkpoint();
                    if (preview.reservePlacementAt(member, pos) >= 0) {
                        const bool fits = self(self, index + 1);
                        preview.rollback(checkpoint);
                        if (fits) return true;
                    }
                    if (interrupted) return false;
                }
                return false;
            };
            if (pack(pack, 0)) {
                // Remember the pure proof, not a route-dependent placement approval.
                // The caller must still validate every arrival and actual placement.
                checked_.emplace(&tile, false);
                return false;
            }
            if (interrupted) return false;
        }
        checked_.emplace(&tile, true);
        ++blocked_count_;
        return true;
    }

    size_t size() const { return blocked_count_; }
    size_t checks = 0;
    size_t hits = 0;
    size_t input_branch_previews = 0;
    size_t input_branch_expanded = 0;
    size_t input_branch_checks = 0;
    size_t input_branch_reuses = 0;
    size_t input_cut_skips = 0;
    size_t input_cut_proofs = 0, input_cut_reuses = 0;
    using InputBoundary = std::unordered_map<fpga::Tile*,std::array<NodeMask,5>>;
    // Bound retained typed masks independently of design size. Eviction only
    // loses a cut-filtering optimization, never a routing opportunity.
    static constexpr size_t max_input_boundary_tiles = 16384;

    // Independent input proofs describe physical pins, not register slots or
    // output exits. A failed multi-pin proof rejects all subsets; a successful
    // proof is only a necessary condition and never replaces actual routing.
    std::optional<bool> inputResult(const rtl::Inst *driver, const std::string &port,
                                   fpga::Tile &tile, const NodeMask &pins) {
        const auto source=rtl::routeSourceEndpoint(driver,port);
        auto found=inputs_.find({source.first,source.second,driver->tile.peer,driver->pos,&tile});
        if (found==inputs_.end() || pins==NodeMask{}) return {};
        if ((pins & found->second.failed)==pins) { ++input_hits; return false; }
        for (const auto &possible:found->second.possible)
            if ((possible & pins)==possible) { ++input_hits; return true; }
        return {};
    }
    void recordInput(const rtl::Inst *driver, const std::string &port,
                     fpga::Tile &tile, const NodeMask &pins, bool possible,
                     std::optional<InputBoundary> boundary = {}) {
        if (pins==NodeMask{}) return;
        const auto source=rtl::routeSourceEndpoint(driver,port);
        auto &entry=inputs_[{source.first,source.second,driver->tile.peer,driver->pos,&tile}];
        if (possible) entry.possible.push_back(pins);
        else {
            entry.failed |= pins;
            if(boundary && boundary->size()<=max_input_boundary_tiles) {
                if(input_boundary_tiles_+boundary->size()>max_input_boundary_tiles) {
                    for(auto& [key,value]:inputs_)value.boundaries.clear();
                    input_boundary_tiles_=0;
                }
                input_boundary_tiles_+=boundary->size();
                entry.boundaries.push_back({pins,std::move(*boundary)});
            }
        }
        ++input_proofs;
    }
    // An exhausted exact input proof remains negative after added private
    // reservations unless a cut opens its occupied boundary. Scope by pins
    // and physical driver; an empty recorded boundary is a valid proof too.
    bool inputCutMayHelp(const rtl::Inst* driver,const std::string& port,
                        fpga::Tile& tile,const NodeMask& pins,
                        const std::vector<fpga::RouteCutNode>& releases) {
        const auto source=rtl::routeSourceEndpoint(driver,port);
        auto found=inputs_.find({source.first,source.second,driver->tile.peer,driver->pos,&tile});
        if(found==inputs_.end() || pins==NodeMask{})return true;
        for(const auto& proof:found->second.boundaries) {
            if((pins & proof.pins)!=pins)continue;
            bool opens=false;
            for(const auto& cut:releases) {
                auto at=proof.nodes.find(fpga::Device::current().getTile(cut.tile.x,cut.tile.y));
                if(at!=proof.nodes.end() && cut.type>=0 && cut.type<5 &&
                   cut.node>=0 && cut.node<CB_MAX_NODES && at->second[cut.type].testBit(cut.node)) {
                    opens=true;break;
                }
            }
            if(!opens) {++input_cut_skips;return false;}
        }
        return true;
    }
    // A live cut invalidates routing proofs, but cannot change pure packing.
    void clearRoutingFailures() { inputs_.clear(); input_boundary_tiles_=0; }
    MovingInputReachability& inputRegion(const rtl::Inst* driver,
            const std::string& port, fpga::Tile& tile) {
        const auto source=rtl::routeSourceEndpoint(driver,port);
        auto& entry=inputs_[{source.first,source.second,driver->tile.peer,driver->pos,&tile}];
        if (!entry.region) entry.region=std::make_unique<MovingInputReachability>(tile);
        return *entry.region;
    }
    // Keep negative regions across cuts, but reconsider newly freed boundary
    // nodes. Exact failures in a region which can reach a source/prefix CB
    // remain routing-view dependent and must be checked again after any cut.
    void clearRoutingFailures(const std::vector<fpga::Tile*>& changed,
                              const BackwardTakeoffStateView* view=nullptr) {
        input_boundary_tiles_=0;
        for (auto it=inputs_.begin();it!=inputs_.end();) {
            auto& proof=it->second;
            // A live release may expose a new exact boundary even when the
            // coarse region remains negative. Never reuse a stale footprint.
            proof.boundaries.clear();
            if (!proof.region) {it=inputs_.erase(it);continue;}
            const bool extended=proof.region->released(changed,view);
            if (extended || proof.region->uncertain()) {
                proof.failed={};
                proof.possible.clear();
            }
            ++it;
        }
    }
    size_t input_hits = 0, input_proofs = 0, input_expanded = 0;

private:
    std::vector<std::vector<rtl::Inst*>> orders_;
    std::unordered_map<fpga::Tile*, bool> checked_;
    size_t blocked_count_ = 0;
    size_t input_boundary_tiles_ = 0;
    struct InputKey {
        const rtl::Inst *driver;
        std::string port;
        const fpga::Tile *source_tile;
        int source_pos;
        fpga::Tile *tile;
        bool operator==(const InputKey &) const = default;
    };
    struct InputHash {
        size_t operator()(const InputKey &key) const {
            return std::hash<const void*>{}(key.driver) ^
                (std::hash<const void*>{}(key.tile)<<1) ^ std::hash<std::string>{}(key.port) ^
                (std::hash<const void*>{}(key.source_tile)<<2) ^ std::hash<int>{}(key.source_pos);
        }
    };
    struct InputResult {
        struct Boundary {NodeMask pins;InputBoundary nodes;};
        NodeMask failed;
        std::vector<NodeMask> possible;
        std::unique_ptr<MovingInputReachability> region;
        std::vector<Boundary> boundaries;
    };
    std::unordered_map<InputKey,InputResult,InputHash> inputs_;
};

} // namespace pnr
