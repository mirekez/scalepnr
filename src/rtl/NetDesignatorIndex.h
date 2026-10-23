#pragma once

#include "Module.h"

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
            for (int designator : module.nets[indexed_nets].designators) {
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

    // In-place edits/reordering require an explicit reset; appends do not.
    void clear()
    {
        indexed_nets = 0;
        net_by_designator.clear();
    }

private:
    size_t indexed_nets = 0;
    std::unordered_map<int, size_t> net_by_designator;
};

}
