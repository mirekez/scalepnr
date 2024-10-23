#pragma once

#include <vector>
#include <string>

namespace gear {

struct Coord
{
    int x = -1, y = -1;

    Coord operator+(const Coord& other) const
    {
        return Coord{x + other.x, y + other.y};
    }

    Coord operator-(const Coord& other) const
    {
        return Coord{x - other.x, y - other.y};
    }

    operator bool()
    {
        return x != -1;
    }
};

struct Range
{
    int a = -1, b = -1;
    operator bool()
    {
        return a != -1;
    }
    int len()
    {
        return b - a;
    }
};

struct Rect
{
    Range x, y;

    int width()
    {
        return x.len();
    }

    int height()
    {
        return y.len();
    }
};

struct RectEx: public Rect
{
    Coord name;  // name coords
    std::vector<Range> more_x;
};


}
