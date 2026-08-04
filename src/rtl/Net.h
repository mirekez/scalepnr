#pragma once

#include "referable.h"
#include "Port.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace fpga {
struct Wire;
}

namespace rtl
{

struct Inst;

struct NetRouteBinding
{
    Inst* owner = nullptr;
    size_t route_index = std::numeric_limits<size_t>::max();
    Inst* from = nullptr;
    Inst* to = nullptr;
    std::string from_port;
    std::string to_port;
    std::string route_name;
};

struct Net
{
    // optional
    std::string name;
    std::vector<int> designators;

    Ref<fpga::Wire> wire;
    Ref<Port> src_port;
    Ref<Port> dst_port;
    bool void_net = false;
    // Infrastructure nets may reserve routing resources before ordinary routing.
    bool route_protected = false;
    // Distributed sources start independently from database-declared local
    // nodes instead of one placed resource endpoint.
    bool distributed_source = false;
    // Select which of the two database-declared distributed capabilities owns
    // this net; this is source identity and survives scheduler reconstruction.
    bool distributed_one = true;
    std::vector<int> void_designators;
    std::vector<NetRouteBinding> routes;

    bool routeCanBePreempted() const
    {
        return !route_protected;
    }

    bool designatorIsVoid(int designator) const
    {
        return void_net
            || std::find(void_designators.begin(), void_designators.end(), designator)
                != void_designators.end();
    }

    bool markDesignatorVoid(int designator)
    {
        if (designatorIsVoid(designator)) {
            return false;
        }
        void_designators.push_back(designator);
        void_net = !designators.empty()
            && std::all_of(designators.begin(), designators.end(),
                [&](int candidate) {
                    return std::find(void_designators.begin(), void_designators.end(), candidate)
                        != void_designators.end();
                });
        return true;
    }

    void clearVoidDesignators()
    {
        void_net = false;
        void_designators.clear();
    }

    std::string makeName(size_t limit = 200)
    {
        return name;
    }
};


}
