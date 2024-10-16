#pragma once

#include <format>
#include <sstream>

#include "sscan.h"
#include "Types.h"

#define JSON_OBJECTS_IDENT 4  // the ident of which objects begin in JSON (to parse it on-fly)

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
                if (range.b == range.a) {
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
        PNR_WARNING("cant read rect: '{}'\n", line);
        return false;
    }
    return true;
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
                range.b = range.a;
//                std::print("+{}", range.a);
            }
            else {
                PNR_WARNING("cant read more_x: '{}'\n", line);
                return false;
            }
            rect.more_x.push_back(range);
        }
        first = false;
    }
    return true;
}
