#pragma once

#include "Wire.h"
#include <functional>
#include <map>
#include <span>
#include <tuple>
#include <utility>

namespace pnr {

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
