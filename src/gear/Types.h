#pragma once

#include <vector>
#include <string>
#include <format>
#include <sstream>

#include "sscan.h"
#include "debug.h"

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

template<>
struct std::formatter<gear::Rect, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const gear::Rect& rect, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "");
        if (rect.a.x == rect.b.x && rect.a.y == rect.b.y) {
            std::format_to(ctx.out(), "{}:{}", rect.a.x, rect.a.y);
        }
        else
        if (rect.a.x == rect.b.x) {
            std::format_to(ctx.out(), "{}:({}-{})", rect.a.x, rect.a.y, rect.b.y);
        }
        else
        if (rect.a.y == rect.b.y) {
            std::format_to(ctx.out(), "({}-{}):{}", rect.a.x, rect.b.x, rect.a.y);
        }
        else {
            std::format_to(ctx.out(), "({}:{})-({}:{})", rect.a.x, rect.a.y, rect.b.x, rect.b.y);
        }
        return ret;
    }
};


template<>
struct std::formatter<gear::RectEx, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const gear::RectEx& rect, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "");
        std::formatter<gear::Rect> f;
        f.format(rect, ctx);
        if (rect.more_x.size()) {
            for (auto range : rect.more_x) {
                if (range.b == -1) {
                    std::format_to(ctx.out(), "|{}", range.a);
                }
                else {
                    std::format_to(ctx.out(), "|{}-{}", range.a, range.b);
                }
            }
        }
        return ret;
    }
};

template<>
struct std::formatter<gear::Coord, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(gear::Coord coord, FmtContext& ctx) const
    {
        return std::format_to(ctx.out(), "{}:{}", coord.x, coord.y);
    }
};

////// scanners

bool scanRect(std::string_view line, gear::Rect& rect)
{
    if (sscan(line, "{}:{}", rect.a.x, rect.a.y) == 2) {
        rect.b.x = rect.a.x;
        rect.b.y = rect.a.y;
//        std::print("{}.{}, ", rect.a.x, rect.a.y);
    }
    else
    if (sscan(line, "{}:({}-{})", rect.a.x, rect.a.y, rect.b.y) == 3) {
        rect.b.x = rect.a.x;
//        std::print("{}.({}-{}), ", rect.a.x, rect.a.y, rect.b.y);
    }
    else
    if (sscan(line, "({}-{}):{}", rect.a.x, rect.b.x, rect.a.y) == 3) {
        rect.b.y = rect.a.y;
//        std::print("({}-{}).{}, ", rect.a.x, rect.b.x, rect.a.y);
    }
    else
    if (sscan(line, "({}:{})-({}:{})", rect.a.x, rect.a.y, rect.b.x, rect.b.y) == 4) {
//        std::print("({}.{})-({}.{}), ", rect.a.x, rect.a.y, rect.b.x, rect.b.y);
    }
    else {
        GEAR_WARNING("cant read rect: '{}'\n", line);
        return false;
    }
}

bool scanRect(std::stringstream ss, gear::RectEx& rect)
{
    std::string line;
    bool first = true;
    while (std::getline(ss, line, '|')) {
        if (first) {
            if (!scanRect(line, rect)) {
                return false;
            }
        }
        else {
            gear::Range range;
            if (sscan(line, "{}-{}", range.a, range.b) == 2) {
//                std::print("+{}-{}", range.a, range.b);
            }
            else
            if (sscan(line, "{}", range.a) == 1) {
//                std::print("+{}", range.a);
            }
            else {
                GEAR_WARNING("cant read more_x: '{}'\n", line);
                return false;
            }
            rect.more_x.push_back(range);
        }
        first = false;
    }
    return true;
}
