#pragma once

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>

namespace pnr {

// Opt-in timing of one task and its nested proofs, never per-node output.
// Exclusive time excludes nested scopes; disabled scopes do not read the clock.
class RouteTaskTiming {
public:
    inline static thread_local RouteTaskTiming* current = nullptr;
    RouteTaskTiming(std::string_view phase, std::string_view route = {})
        : parent(current), phase(phase), route(route.empty() && parent ? parent->route : route) {
        static const std::string selected = [] {
            const char* value = std::getenv("SCALEPNR_PROFILE_TASK_NET");
            return value ? std::string(value) : std::string{};
        }();
        active = parent || (!selected.empty() && this->route.find(selected) != std::string_view::npos);
        if (!active) return;
        id = ++sequence;
        started = Clock::now();
        current = this;
        std::cout << "TASK_TIME_BEGIN id=" << id << " parent=" << (parent ? parent->id : 0)
                  << " phase=" << std::quoted(phase) << " route=" << std::quoted(this->route) << '\n';
    }
    ~RouteTaskTiming() {
        if (!active) return;
        const double seconds = std::chrono::duration<double>(Clock::now() - started).count();
        std::cout << "TASK_TIME_END id=" << id << " parent=" << (parent ? parent->id : 0)
                  << " phase=" << std::quoted(phase) << " route=" << std::quoted(route)
                  << " seconds=" << seconds << " exclusive_seconds=" << seconds - child_seconds
                  << " target=(" << x << ',' << y << ") pos=" << pos
                  << " anchor=(" << anchor_x << ',' << anchor_y << ") dst=" << dst
                  << " result=" << result << " cached=" << cached
                  << " expanded=" << expanded << " edges=" << edges << " busy=" << busy
                  << " probes=" << probes << " pending=" << pending
                  << " region_hits=" << region_hits << '\n';
        if (parent) parent->child_seconds += seconds;
        current = parent;
    }
    RouteTaskTiming(const RouteTaskTiming&) = delete;
    RouteTaskTiming& operator=(const RouteTaskTiming&) = delete;
    bool enabled() const { return active; }
    int x = -1, y = -1, pos = -1, anchor_x = -1, anchor_y = -1, dst = -1;
    bool cached = false;
    std::string_view result = "returned";
    size_t expanded = 0, edges = 0, busy = 0, probes = 0, pending = 0, region_hits = 0;

private:
    using Clock = std::chrono::steady_clock;
    inline static thread_local size_t sequence = 0;
    RouteTaskTiming* parent;
    std::string_view phase, route;
    bool active = false;
    size_t id = 0;
    Clock::time_point started{};
    double child_seconds = 0;
};

} // namespace pnr
