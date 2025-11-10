#pragma once

#include <vector>
#include <string>

namespace fpga {

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

    Coord operator+=(const Coord& other)
    {
        return (*this = *this + other);
    }

    Coord operator-=(const Coord& other)
    {
        return (*this = *this + other);
    }

    operator bool()
    {
        return x != -1;
    }

    bool operator<(const Coord& other)
    {
        if (x < other.x) return true;
        if (x > other.x) return false;
        return y < other.y;
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


}
