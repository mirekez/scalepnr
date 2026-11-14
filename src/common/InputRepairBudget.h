#pragma once

#include <cstddef>

namespace pnr {

// Share this expansion limit across all input-repair searches during one
// move. On exhaustion, stop repairing inputs, unroute their obsolete suffixes,
// and enqueue them for subsequent moving instead of rejecting the move.
inline constexpr size_t MAX_INPUT_REPAIRING_EXPANDED_STATES_PER_ONE_MOVING = 100000;
// All preliminary in-place searches for one destination share this allowance.
// Individual search quotas may be smaller; they must not reset this total.
inline constexpr size_t MAX_PRE_RELOCATION_REPAIR_EXPANDED_STATES = 32768;

struct InputRepairBudget {
    size_t limit = MAX_INPUT_REPAIRING_EXPANDED_STATES_PER_ONE_MOVING;
    size_t expanded = 0;
    bool defer_inputs = false;

    // Only bounded subsearches activate this context. Moving input searches use
    // defer_inputs for deferral; destination pre-relocation probes reuse the
    // counter with a smaller limit and hand exhaustion directly to relocation.
    // Leaving the scope restores the outer search without cancelling its stage.
    inline static thread_local InputRepairBudget* active = nullptr;
    struct Scope {
        InputRepairBudget* previous;
        explicit Scope(InputRepairBudget& budget) : previous(active) { active = &budget; }
        ~Scope() { active = previous; }
        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;
    };

    static bool stopped() { return active && active->defer_inputs; }
    static bool expand() {
        if (!active) return true;
        if (active->expanded >= active->limit) {
            active->defer_inputs = true;
            return false;
        }
        ++active->expanded;
        return true;
    }
};

} // namespace pnr
