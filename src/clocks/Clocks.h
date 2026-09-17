#pragma once

#include "Clock.h"
#include "Design.h"
#include "getConns.h"
#include "referable.h"
#include "debug.h"

#include <deque>
#include <set>

namespace technology
{
    struct Tech;
}

namespace clk
{

struct Clocks
{
    // Timing forests and register bunches retain addresses of these objects.
    std::deque<Referable<rtl::Clock>> clocks_list;
    std::set<std::pair<std::string, std::string>> asynchronous_pairs;

    technology::Tech* tech = nullptr;

    bool addClocks(rtl::Design& design, const std::string& clk_name, const std::string& port_name, double period_ns, int duty);
    void getClocks(std::vector<rtl::Clock*>* clocks, const std::string& name, bool partial_name = true);

    bool asynchronous(const rtl::Clock& a, const rtl::Clock& b) const
    {
        return asynchronous_pairs.contains(std::minmax(a.name, b.name));
    }

    // Follow only explicitly described clock buffers, never combinational data.
    Referable<rtl::Clock>* findClock(rtl::Conn* connection,
        const std::multimap<std::string, std::string>& buffers)
    {
        std::set<rtl::Conn*> visited;
        while (connection && visited.insert(connection).second) {
            for (auto& clock : clocks_list)
                if (clock.conn_ptr == connection) return &clock;
            if (auto* driver = connection->follow()) {
                connection = driver;
                continue;
            }
            auto* inst = connection->inst_ref.peer;
            if (!inst || !inst->cell_ref.peer) return nullptr;
            auto [first, last] = buffers.equal_range(inst->cell_ref->type);
            bool output = false;
            for (; first != last; ++first)
                output |= first->second == connection->port_ref->name;
            if (!output) return nullptr;
            rtl::Conn* input = nullptr;
            for (auto& conn : inst->conns) {
                if (conn.port_ref->type != rtl::Port::PORT_IN) continue;
                if (input) return nullptr; // a mux needs an explicit clock model
                input = &conn;
            }
            connection = input;
        }
        return nullptr;
    }
};


}
