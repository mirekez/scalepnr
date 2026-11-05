#include "RouteTaskTiming.h"

#include <sstream>
#include <stdexcept>

int main() {
    setenv("SCALEPNR_PROFILE_TASK_NET", "signal_71", 1);
    std::ostringstream output;
    struct Capture {
        std::streambuf* old = nullptr;
        ~Capture() { std::cout.rdbuf(old); }
    } capture{std::cout.rdbuf(output.rdbuf())};
    auto require = [](bool ok, const char* why) { if (!ok) throw std::runtime_error(why); };
    {
        // Unrelated tasks cannot enable timing or produce records.
        pnr::RouteTaskTiming ignored("move", "signal_72");
        require(!ignored.enabled() && !pnr::RouteTaskTiming::current, "unselected scope activated");
    }
    require(output.str().empty(), "disabled scope printed output");
    {
        pnr::RouteTaskTiming task("move", "prefix.signal_71");
        {
            // Nested input aliases inherit selection without enabling node traces.
            pnr::RouteTaskTiming input("input", "input_13");
            require(input.enabled() && pnr::RouteTaskTiming::current == &input, "nested input lost selection");
            input.x=4; input.y=7; input.expanded=709467; input.cached=true; input.result="failed";
        }
        // Ending a nested scope restores its live parent.
        require(pnr::RouteTaskTiming::current == &task, "parent timing scope not restored");
    }
    require(!pnr::RouteTaskTiming::current, "timing leaked to another task");
    const auto text=output.str();
    require(text.find("parent=1 phase=\"input\"") != std::string::npos &&
            text.find("target=(4,7)") != std::string::npos &&
            text.find("cached=1 expanded=709467") != std::string::npos &&
            text.find("exclusive_seconds=") != std::string::npos, "missing attribution or counters");
    size_t records=0;
    for(char c:text) records+=c=='\n';
    // Even a large search emits only one begin/end pair, never per-node output.
    require(records==4, "timing output scaled with search size");
}
