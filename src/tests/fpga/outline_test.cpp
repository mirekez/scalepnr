#include "OutlineGrid.h"

#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message)
{
    if (!condition) {
        std::cerr << "outline_test: " << message << '\n';
        std::exit(1);
    }
}

}

int main()
{
    for (int index = 0; index < 10; ++index) {
        require(pnr::outlineMeshIndex(static_cast<float>(index) + 0.5F, 10)
                    == index,
                "a radial half-cell coordinate escaped its containing box");
    }
    require(pnr::outlineMeshIndex(-0.5F, 10) == 0,
            "a coordinate below the mesh was not clamped");
    require(pnr::outlineMeshIndex(10.25F, 10) == 9,
            "a coordinate above the mesh was not clamped");
    require(pnr::outlineMeshIndex(4.75F, 1) == 0,
            "a single-cell mesh did not select its only cell");

    // Small designs retain their requested optimization budget.
    require(pnr::outlineInstanceIterationLimit(40, 296, 418) == 40,
            "instance optimizer shortened a budget below one device traversal");
    // Large designs stop after warmup plus one complete physical-grid traversal.
    require(pnr::outlineInstanceIterationLimit(1164, 296, 418) == 469,
            "instance optimizer did not cap redundant full-device traversals");
    require(pnr::outlineBunchIterationLimit(40) == 4,
            "small bunch optimization budget was changed");
    require(pnr::outlineBunchIterationLimit(100000) == 251,
            "large bunch optimization budget was not capped");
    std::cout << "outline_test passed\n";
    return 0;
}
