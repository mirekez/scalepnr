#pragma once

#include <format>
#include <sstream>

#include "sscan.h"
#include "Types.h"

#define JSON_OBJECTS_IDENT 4  // the ident of which objects begin in JSON (to parse it on-fly)

template<>
struct std::formatter<std::ispanstream, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(std::ispanstream& ss, FmtContext& ctx) const
    {
        return std::format_to(ctx.out(), "{}", std::string(ss.span().data(), ss.span().size()));
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

template<>
struct std::formatter<gear::Range, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const gear::Range& range, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "");
        if (range.b == range.a) {
            std::format_to(ctx.out(), "{}", range.a);
        }
        else {
            std::format_to(ctx.out(), "{}-{}", range.a, range.b);
        }
        return ret;
    }
};

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
        return std::format_to(ctx.out(), "{}:{}", rect.x, rect.y);
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
                std::format_to(ctx.out(), ";{}", range);
            }
        }
        return ret;
    }
};


////// scanners (can not to reset position on errors)

inline bool scan(std::ispanstream& ss, gear::Coord& coord)
{
    if (sscan(ss, "{}:{}", coord.x, coord.y) != 2) {
        PNR_WARNING("cant read coord: '{}'\n", ss);
        return false;
    }
    return true;
}

inline bool scan(std::ispanstream& ss, gear::Range& range)
{
    if (sscan(ss, "{}-{}", range.a, range.b) == 2) {
        return true;
    }
    if (sscan(ss, "{}", range.a) == 1) {
        range.b = range.a;
        return true;
    }
    PNR_WARNING("cant read range: '{}'\n", ss);
    return false;
}

inline bool scan(std::ispanstream& ss, gear::Rect& rect)
{
    if (!scan(ss, rect.x) || ss.peek() != (int)':' || ss.get() != (int)':' || !scan(ss, rect.y)) {
        PNR_WARNING("cant read rect: '{}'\n", ss);
        return false;
    }
    return true;
}

inline bool scan(std::ispanstream& ss, gear::RectEx& rect)
{
    if (!scan(ss, (gear::Rect&)rect)) {
        return false;
    }
    while (ss.get() == (int)';') {
        gear::Range range;
        if (!scan(ss, range)) {
            PNR_WARNING("cant read more_x: '{}'\n", ss);
            return false;
        }
        rect.more_x.push_back(range);
    }
    return true;
}
