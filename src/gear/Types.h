#pragma once

#include <vector>
#include <string>

namespace gear {

    struct Coord
    {
        int x = -1, y = -1;

        Coord operator+(const Coord& other) const {
            return Coord{x + other.x, y + other.y};
        }

        Coord operator-(const Coord& other) const {
            return Coord{x - other.x, y - other.y};
        }
    };

    struct Range
    {
        int a = -1, b = -1;
    };

    struct Rect
    {
        Coord a, b;

        int width()
        {
            return b.x - a.x;
        }

        int height()
        {
            return b.y - a.y;
        }
    };

    struct RectEx: public Rect
    {
        Coord name;  // name coords
        std::vector<Range> more_x;
    };
}

