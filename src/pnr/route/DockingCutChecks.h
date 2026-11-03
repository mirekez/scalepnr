#pragma once

#include "Wire.h"
#include <functional>
#include <map>
#include <span>
#include <tuple>
#include <utility>

namespace pnr {

struct RouteDesign;

// Preflight physical owners without releasing any route. The query index lives
// for the docking attempt, while eligibility is evaluated for each cut.
bool provisionalDockingCutsEligible(RouteDesign *router,
    std::span<const fpga::RouteCutNode> cuts, rtl::Net *current_net,
    rtl::Inst *current_source, const std::string &current_source_port,
    fpga::RouteOwnerQueries &owners);

// One immutable docking attempt: eligibility is a property of a physical cut,
// not of the many candidate edges that reuse it. Discard before any mutation.
class DockingCutChecks {
public:
  using Check = std::function<bool(const fpga::RouteCutNode &)>;
  explicit DockingCutChecks(Check check) : check_(std::move(check)) {}
  bool eligible(std::span<const fpga::RouteCutNode> cuts) {
    for (const auto &cut : cuts) {
      const auto key = std::make_tuple(cut.tile.x, cut.tile.y, int(cut.type), cut.node);
      auto [entry, inserted] = results_.try_emplace(key, false);
      if (inserted) entry->second = check_(cut);
      if (!entry->second) return false;
    }
    return true;
  }
private:
  Check check_;
  std::map<std::tuple<int, int, int, int>, bool> results_;
};

} // namespace pnr
