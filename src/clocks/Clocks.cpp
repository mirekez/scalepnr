#include "Clocks.h"
#include "Tech.h"
#include <cmath>
#include <stdexcept>

using namespace clk;

void Clocks::setMaxDelay(const std::string& from, const std::string& to, double ns)
{
    std::vector<rtl::Clock*> sources, sinks;
    getClocks(&sources, from, false);
    getClocks(&sinks, to, false);
    if (sources.size()!=1 || sinks.size()!=1)
        throw std::invalid_argument("set_max_delay requires known, explicit clock names");
    if (!std::isfinite(ns) || ns < 0.000001 || ns > 1e9)
        throw std::invalid_argument("maximum delay must be finite and between 0.000001 and 1e9 ns");
    if (asynchronous(*sources.front(), *sinks.front()))
        throw std::invalid_argument("maximum delay conflicts with an asynchronous clock-group exclusion");
    datapath_max_delays[{from,to}] = ns;
}

bool Clocks::addClocks(rtl::Design& design, const std::string& clk_name, const std::string& port_name, double period_ns, int duty)
{
    PNR_LOG1("CLKS", "addClocks, name: {}, port: {}, period: {}, duty: {}", clk_name, port_name, period_ns, duty);
    if (clk_name.empty() || port_name.empty() || !std::isfinite(period_ns)
        || period_ns < 0.000001 || period_ns > 1e9 || duty <= 0 || duty >= 100)
        return false;
    if (!design.top.cell_ref.peer || !design.top.cell_ref->module_ref.peer)
        return false;

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
