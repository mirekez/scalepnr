#pragma once

#include <cstddef>

namespace pnr {

// Share this expansion limit across all input-repair searches during one source
// move. On exhaustion, stop repairing inputs, unroute their obsolete suffixes,
// and enqueue them for subsequent source moving instead of rejecting the move.
inline constexpr size_t MAX_INPUT_REPAIRING_EXPANDED_STATES_PER_ONE_MOVING = 1000000;

struct InputRepairBudget {
    size_t limit = MAX_INPUT_REPAIRING_EXPANDED_STATES_PER_ONE_MOVING;
    size_t expanded = 0;
    bool defer_inputs = false;

    // Only input searches activate this context; the outer trunk search is free
    // to continue after the input budget expires, without cancelling its stage.
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
