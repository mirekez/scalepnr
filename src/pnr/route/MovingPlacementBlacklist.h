#pragma once

#include "Element.h"
#include "Inst.h"
#include "Tile.h"

#include <functional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pnr {

// One moving cluster, one unchanged placement snapshot. A rejection skips only
// placement probing: the routing engine must still traverse this tile normally.
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

private:
    std::vector<std::vector<rtl::Inst*>> orders_;
    std::unordered_map<fpga::Tile*, bool> checked_;
    size_t blocked_count_ = 0;
};

} // namespace pnr
