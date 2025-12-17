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
        return *this = *this + other;
    }

    Coord operator-=(const Coord& other)
    {
        return *this = *this - other;
    }

    bool operator==(Coord& other)
    {
        return x == other.x && y == other.y;
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

inline void radialSearch(Coord& coord, int& dir, int& steps, int& pos)
{
    switch (dir) {
        case 0: coord.x += 0; coord.y += 1; break;
        case 1: coord.x += 1; coord.y += 0; break;
        case 2: coord.x -= 0; coord.y -= 1; break;
        case 3: coord.x -= 1; coord.y -= 0; break;
    }
    ++pos;
    if (pos == steps) {
        dir = (dir + 1)%4;
    }
    if (pos == steps*2) {
        dir = (dir + 1)%4;
        ++steps;
        pos = 0;
    }
}


}
