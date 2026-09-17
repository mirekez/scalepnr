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

// Scale the requested budget with design size, capped at 100 passes.
// The instance optimizer inherits this budget, subject to its grid-size cap.
inline int outlineBunchIterationLimit(int cells)
{
    return std::min(std::max(1, cells/10), 100);
}

}
