#pragma once

#include <algorithm>
#include <cmath>

namespace pnr {

// Convert a continuous outline coordinate to its containing mesh cell while
// keeping numerical spill at the grid edges inside the allocated array.
inline int outlineMeshIndex(float coordinate, int extent)
{
    if (extent <= 1) {
        return 0;
    }
    return std::clamp(static_cast<int>(std::floor(coordinate)), 0, extent - 1);
}

// The instance optimizer starts grid spreading after iteration 50 and moves
// at most one physical-grid step per pass. One full-device traversal is enough.
inline int outlineInstanceIterationLimit(int requested, int grid_width, int grid_height)
{
    int full_traversal = 51 + std::max(grid_width, grid_height);
    return std::min(requested, full_traversal);
}

// Bunch relaxation changes phase at iterations 50, 100, and 150. Beyond one
// hundred final-phase passes, very large designs repeat an already stable
// relaxation while cost grows as cells squared through the requested budget.
inline int outlineBunchIterationLimit(int cells)
{
    return std::min(std::max(1, cells/10), 251);
}

}
