#include "Clocks.h"

using namespace clk;

bool Clocks::addClocks(rtl::Design& design, const std::string& clk_name, const std::string& port_name, double period_ns, int duty)
{
    PNR_LOG2("CLKS", "addClocks, name: {}, port: {}, period: {}, duty: {}", clk_name, port_name, period_ns, duty);
    clocks_list.reserve(MAX_CLOCKS);

    for (auto& clock : clocks_list) {
        if (clock.name == clk_name || port_name == clock.conn_name) {
            PNR_WARNING("clock '{}' already exists\n", clk_name);
            return false;
        }
    }

    std::vector<Referable<rtl::Conn>*> conns;
    rtl::connFilter filter;
    filter.partial = false;
    filter.port_name = port_name;
    rtl::getConns(&conns, std::move(filter), &design.top);

    bool found = false;
    for (auto* conn : conns) {
        auto name = conn->makeName();
        if (name == port_name) {
            clocks_list.emplace_back( rtl::Clock{.name = clk_name, .conn_ptr = conn, .conn_name = port_name, .period_ns = period_ns, .duty = duty} );
            std::print("\ncreated clock '{}' for port '{}'", clk_name, name);
            findBufs(clocks_list.back().conn_ptr, clocks_list.back());  // find and mark BUFGs
            return true;
        }
    }
    if (!found) {
        PNR_WARNING("cant find port '{}' for clock '{}'", port_name, clk_name);
    }
    return false;
}

void Clocks::getClocks(std::vector<rtl::Clock*>* clocks, const std::string& name, bool partial_name)
{
    PNR_LOG1("CLKS", "getClocks, name: '{}', partial_name: '{}'", name, partial_name);
    for (auto& clock : clocks_list) {
        PNR_LOG2("CLKS", "clock_name: '{}' (port '{}')", clock.name, clock.conn_name);
        if (name == clock.name || (partial_name && (name.length() == 0 || clock.name.find(name) != std::string::npos))) {
            PNR_LOG1("CLKS", "found_clock: '{}'", clock.name);
            clocks->push_back(&clock);
        }
    }
}

void Clocks::findBufs(Referable<rtl::Conn>* clk_conn, rtl::Clock& clk)
{
    PNR_LOG2("CLKS", "findBufs, clk_conn: '{}'", clk_conn->makeName());
    if (clk_conn->peers.size() == 0) {  // it's CLK input (should be BUFG or error)
        auto it = tech->buffers_ports.find(clk_conn->inst_ref->cell_ref->type);
        if (it == tech->buffers_ports.end()) {
            PNR_LOG2("CLKS", "skipping '{}'", clk_conn->makeName());
            if (!clk_conn->inst_ref->cell_ref->module_ref->is_blackbox) {
                PNR_WARNING("floating clock net: got terminal conn '{}' ('{}'), but it's not a is_blackbox ('{}')",
                    clk_conn->makeName(), clk_conn->inst_ref->cell_ref->type, clk_conn->inst_ref->cell_ref->module_ref->name);
            }
        }
        while (it != tech->buffers_ports.end() && it->first == clk_conn->inst_ref->cell_ref->type) {
            PNR_LOG2("CLKS", "found an iobuf: '{}' by conn '{}'", clk_conn->inst_ref->cell_ref->type, clk_conn->makeName());
            clk.bufg_ptr = clk_conn->inst_ref.peer;
            for (auto& next_conn : clk_conn->inst_ref->conns) {
                if (next_conn.port_ref->name == it->second) {  // it's output of BUFG
                    PNR_LOG2("CLKS", "trying next '{}' ('{}')", next_conn.inst_ref->makeName(), next_conn.inst_ref->cell_ref->type);
                    findBufs(&next_conn, clk);  // trying next
                    break;
                }
            }
            ++it;
        }
    }
    else
    if (clk_conn->peers.size() == 1)  // we want to skip all bufs till bufg
    for (auto* peer_ptr : clk_conn->peers) {  // it's CLK output, directly or from BUFG
        Referable<rtl::Conn>& peer = rtl::Conn::fromRef(*static_cast<Ref<rtl::Conn>*>(peer_ptr));
        PNR_LOG2("CLKT", "recursing '{}'", peer.makeName());
        findBufs(&peer, clk);
    }
}
