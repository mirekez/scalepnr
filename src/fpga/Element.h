#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace fpga {

/*
Element-based tile packing model:
1. TileType::elements describes all element positions available in one tile type.
   Each Element has a primitive category and a bitmap_pos; all set bitmap_pos bits
   of the same ElementType are legal positions for that primitive category.
2. Element connectivity is stored in the TileType elements themselves. For each
   occupied bitmap_pos, left_blockers/right_blockers point to positions of other
   element types that are physically connected to this element from the left or
   to the right in the tile-local resource chain. The runtime elements_pos and
   elements_free arrays hold one uint16_t mask per ElementType, and
   elements_left/elements_right hold 16 uint16_t masks per ElementType. Each bit
   addresses one possible position, so one tile can model up to 16 positions of
   one element type.
3. When a concrete Tile is initialized, TileType::elements are copied into compact
   per-tile arrays for each ElementType. These arrays are indexed by element type
   in tile-column order from left to right: elements_pos has all existing positions
   of that type, elements_free tracks currently unused positions of that type, and
   elements_left/elements_right mirror neighbor connectivity masks for that type.
4. Tile::tryAdd maps the candidate cell to an ElementType, uses elements_free to
   search free positions of that type, checks that the chosen bit exists in
   elements_pos, uses elements_left/elements_right to find connected neighbor
   positions by indexing the same arrays with the neighbor type, and allows packing
   beside an occupied neighbor only when the real netlist cells are connected in
   the required direction. Otherwise the position is blocked for this cell.
*/

enum ElementType : uint16_t
{
    ELEMENT_LUT5 = 0,
    ELEMENT_LUT1,
    ELEMENT_MUXF7,
    ELEMENT_MUXF8,
    ELEMENT_CARRY,
    ELEMENT_FD,
    ELEMENT_TYPE_COUNT,
};

constexpr int ELEMENT_BITMAP_BITS = 16;

struct Element
{
    std::string name;
    ElementType type = ELEMENT_LUT5;
    uint16_t bitmap_pos = 0;
    std::array<uint16_t, ELEMENT_BITMAP_BITS> left_blockers{};
    std::array<uint16_t, ELEMENT_BITMAP_BITS> right_blockers{};
    int elements_to_left = 0;
};

const char* elementTypeName(ElementType type);

}
