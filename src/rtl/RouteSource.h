#pragma once

#include "Inst.h"

#include <utility>

namespace rtl {

// Generated source helpers alias the upstream driver; resolve the same identity
// for scheduling, shared leases and repair without inspecting routing names.
inline std::pair<const Inst*, std::string> routeSourceEndpoint(
    const Inst* inst, std::string port)
{
    auto* current = const_cast<Inst*>(inst);
    for (size_t depth = 0; current && current->cell_ref.peer && depth < 64; ++depth) {
        auto kind = current->cell_ref->attributes.find("scalepnr_passthrough");
        if (kind == current->cell_ref->attributes.end() || kind->second != "source") break;
        Conn* input = nullptr;
        for (auto& conn : current->conns) {
            if (conn.port_ref.peer && conn.port_ref->type == Port::PORT_IN) {
                input = &conn;
                break;
            }
        }
        Conn* driver = input ? input->follow() : nullptr;
        if (!driver || !driver->inst_ref.peer || !driver->port_ref.peer ||
            driver->inst_ref.peer == current) break;
        current = driver->inst_ref.peer;
        port = driver->port_ref->makeName();
    }
    return {current, std::move(port)};
}

// Equal names or designators are insufficient: require the same resolved pin.
inline bool sameRouteSource(const Inst* left, const std::string& left_port,
                            const Inst* right, const std::string& right_port)
{
    if (!left || !right || left_port.empty() || right_port.empty()) return false;
    if (left == right && left_port == right_port) return true;
    return routeSourceEndpoint(left, left_port) == routeSourceEndpoint(right, right_port);
}

} // namespace rtl
