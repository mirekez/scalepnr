#pragma once

#include "Module.h"

#include <array>
#include <limits>
#include <unordered_map>

namespace rtl {

// Routing may append passthrough nets. Keep offsets, not pointers, so appends
// do not rescan the module and vector relocation cannot stale lookup results.
class NetDesignatorIndex {
public:
    size_t update(const Module& module)
    {
        if (indexed_nets > module.nets.size()) {
            clear();
        }
        const size_t previous_count = indexed_nets;
        for (; indexed_nets < module.nets.size(); ++indexed_nets) {
            const Net& net = module.nets[indexed_nets];
            if (net.distributed_source) {
                const size_t value = net.distributed_one ? 1 : 0;
                if (distributed_constant_nets[value] == invalid_offset) {
                    distributed_constant_nets[value] = indexed_nets;
                }
            }
            for (int designator : net.designators) {
                net_by_designator.try_emplace(designator, indexed_nets);
            }
        }
        return indexed_nets - previous_count;
    }

    Net* find(Module& module, int designator)
    {
        update(module);
        auto found = net_by_designator.find(designator);
        return found == net_by_designator.end() ? nullptr
                                               : &module.nets[found->second];
    }

    // Constant route nets are indexed by offset alongside numeric aliases.
    Net* findDistributedConstant(Module& module, bool one)
    {
        update(module);
        const size_t offset = distributed_constant_nets[one ? 1 : 0];
        if (offset == invalid_offset) return nullptr;
        Net& net = module.nets[offset];
        return net.distributed_source && net.distributed_one == one ? &net : nullptr;
    }

    // In-place edits/reordering require an explicit reset; appends do not.
    void clear()
    {
        indexed_nets = 0;
        net_by_designator.clear();
        distributed_constant_nets = {invalid_offset, invalid_offset};
    }

private:
    static constexpr size_t invalid_offset = std::numeric_limits<size_t>::max();
    size_t indexed_nets = 0;
    std::unordered_map<int, size_t> net_by_designator;
    std::array<size_t, 2> distributed_constant_nets{invalid_offset, invalid_offset};
};

}
