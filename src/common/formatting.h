#pragma once

#include <format>
#include <sstream>

#include <string_view>
#include <spanstream>
#include "Types.h"

#define JSON_OBJECTS_IDENT 4  // the ident of which objects begin in JSON (to parse it on-fly)

struct RangeEx: public fpga::Range
{
    int name_x;  // name's coord
};

struct RectEx: public fpga::Rect
{
    fpga::Coord name;  // name's coord
    std::vector<RangeEx> more_x;
};

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
        return std::format_to(ctx.out(), "{}/{}", std::string(ss.span().data(), ss.span().size()), (int)ss.tellg());
    }
};

template<>
struct std::formatter<fpga::Coord, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(fpga::Coord coord, FmtContext& ctx) const
    {
        return std::format_to(ctx.out(), "{}:{}", coord.x, coord.y);
    }
};

template<>
struct std::formatter<fpga::Range, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const fpga::Range& range, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "");
        if (range.b == range.a) {
            ret = std::format_to(ctx.out(), "{}", range.a);
        }
        else {
            ret = std::format_to(ctx.out(), "{}-{}", range.a, range.b);
        }
        return ret;
    }
};

template<>
struct std::formatter<RangeEx, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const RangeEx& range, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "");
        if (range.b == range.a) {
            ret = std::format_to(ctx.out(), "{}", range.a);
        }
        else {
            ret = std::format_to(ctx.out(), "{}-{}", range.a, range.b);
        }
        if (range.name_x != -1) {
            ret = std::format_to(ctx.out(), "/{}", range.name_x);
        }
        return ret;
    }
};

template<>
struct std::formatter<fpga::Rect, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const fpga::Rect& rect, FmtContext& ctx) const
    {
        return std::format_to(ctx.out(), "{}:{}", rect.x, rect.y);
    }
};


template<>
struct std::formatter<RectEx, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const RectEx& rect, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "");
        std::formatter<fpga::Rect> f;
        ret = f.format(rect, ctx);

        if (rect.name.y != -1) {
            ret = std::format_to(ctx.out(), "/{}", rect.name);
        }

        if (rect.more_x.size()) {
            for (auto range : rect.more_x) {
                ret = std::format_to(ctx.out(), ";{}", range);
            }
        }
        return ret;
    }
};


////// scanners (can not to reset position on errors)
namespace fpga {

inline std::ispanstream& operator>>(std::ispanstream& ss, fpga::Coord& coord)
{
    if (!(ss >> coord.x)) {
        ss.setstate(std::ios_base::failbit);
        PNR_WARNING("cant read coord 1: '{}'\n", ss);
        return ss;
    }
    if (ss.peek() != (int)':') {
        ss.setstate(std::ios_base::failbit);
        PNR_WARNING("cant read coord 2: '{}'\n", ss);
        return ss;
    }
    ss.ignore(1);
    if (!(ss >> coord.y)) {
        ss.setstate(std::ios_base::failbit);
        PNR_WARNING("cant read coord 3: '{}'\n", ss);
        return ss;
    }

    ss.clear();
    return ss;
}

inline std::ispanstream& operator>>(std::ispanstream& ss, Range& range)
{
    if (!(ss >> range.a)) {
        ss.setstate(std::ios_base::failbit);
        PNR_WARNING("cant read range1: '{}'\n", ss);
        return ss;
    }
    if (ss.peek() == (int)'-') {
        ss.ignore(1);
        if (!(ss >> range.b)) {
            ss.setstate(std::ios_base::failbit);
            PNR_WARNING("cant read range2: '{}'\n", ss);
            return ss;
        }
    }
    else {
        range.b = range.a;
        ss.clear();
    }
    return ss;
}

inline std::ispanstream& operator>>(std::ispanstream& ss, RangeEx& range)
{
    if (!(ss >> (Range&)range)) {
        ss.setstate(std::ios_base::failbit);
        return ss;
    }
    if (ss.peek() == (int)'/') {
        ss.ignore(1);
        if (!(ss >> range.name_x)) {
            ss.setstate(std::ios_base::failbit);
            PNR_WARNING("cant read name_x: '{}'\n", ss);
            return ss;
        }
    }
    return ss;
}

inline std::ispanstream& operator>>(std::ispanstream& ss, fpga::Rect& rect)
{
    if (!(ss >> rect.x)) {
        ss.setstate(std::ios_base::failbit);
        PNR_WARNING("cant read rect 1: '{}'\n", ss);
        return ss;
    }
    if (ss.peek() != (int)':') {
        ss.setstate(std::ios_base::failbit);
        PNR_WARNING("cant read rect 2: '{}'\n", ss);
        return ss;
    }
    ss.ignore(1);
    if (!(ss >> rect.y)) {
        ss.setstate(std::ios_base::failbit);
        PNR_WARNING("cant read rect 3: '{}'\n", ss);
        return ss;
    }

    ss.clear();
    return ss;
}

//inline bool scan(std::ispanstream& ss, fpga::Rect& rect)
//{
//    if (!scan(ss, rect.x) || ss.peek() != (int)':' || ss.get() != (int)':' || !scan(ss, rect.y)) {
//}

inline std::ispanstream& operator>>(std::ispanstream& ss, RectEx& rect)
{
    if (!(ss >> (fpga::Rect&)rect)) {
        ss.setstate(std::ios_base::failbit);
        return ss;
    }
    if (ss.peek() == (int)'/') {
        ss.ignore(1);
        if (!(ss >> rect.name)) {
            ss.setstate(std::ios_base::failbit);
            PNR_WARNING("cant read name: '{}'\n", ss);
            return ss;
        }
    }
    while (ss.peek() == (int)';' && ss.get()) {
        RangeEx range;
        if (!(ss >>range)) {
            ss.setstate(std::ios_base::failbit);
            PNR_WARNING("cant read more_x: '{}'\n", ss);
            return ss;
        }
        rect.more_x.push_back(range);
    }
    ss.clear();
    return ss;
}

}

////// other formatters

#include <vector>
#include <map>

template<>
struct std::formatter<std::vector<std::string>, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const std::vector<std::string>& vect, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "<");
        std::string delim = "";
        for (const auto& el : vect) {
            ret = std::format_to(ctx.out(), "{}{}", delim, el);
            delim = ", ";
        }
        ret = std::format_to(ctx.out(), ">");
        return ret;
    }
};

template<>
struct std::formatter<std::vector<bool>, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const std::vector<bool>& vect, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "<");
        std::string delim = "";
        for (const auto& el : vect) {
            ret = std::format_to(ctx.out(), "{}{}", delim, el);
            delim = ", ";
        }
        ret = std::format_to(ctx.out(), ">");
        return ret;
    }
};

template<>
struct std::formatter<std::vector<int>, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const std::vector<int>& vect, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "<");
        std::string delim = "";
        for (const auto& el : vect) {
            ret = std::format_to(ctx.out(), "{}{}", delim, el);
            delim = ", ";
        }
        ret = std::format_to(ctx.out(), ">");
        return ret;
    }
};

template<>
struct std::formatter<std::map<std::string,std::string>, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const std::map<std::string,std::string>& map, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "<");
        std::string delim = "";
        for (const auto& pair : map) {
            ret = std::format_to(ctx.out(), "{}{}:{}", delim, pair.first, pair.second);
            delim = ", ";
        }
        ret = std::format_to(ctx.out(), ">");
        return ret;
    }
};

template<>
struct std::formatter<std::multimap<std::string,std::string>, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const std::multimap<std::string,std::string>& multimap, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "<");
        std::string delim = "";
        for (const auto& pair : multimap) {
            ret = std::format_to(ctx.out(), "{}{}:{}", delim, pair.first, pair.second);
            delim = ", ";
        }
        ret = std::format_to(ctx.out(), ">");
        return ret;
    }
};

template<>
struct std::formatter<std::map<std::string,std::pair<std::string,std::string>>, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const std::map<std::string,std::pair<std::string,std::string>>& map, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "<");
        std::string delim = "";
        for (const auto& pair : map) {
            ret = std::format_to(ctx.out(), "{}{}:{};{}", delim, pair.first, pair.second.first, pair.second.second);
            delim = ", ";
        }
        ret = std::format_to(ctx.out(), ">");
        return ret;
    }
};

template<>
struct std::formatter<std::multimap<std::string,std::pair<std::string,std::string>>, char>
{
    template<class ParseContext>
    constexpr ParseContext::iterator parse(ParseContext& ctx)
    {
        return ctx.begin();
    }

    template<class FmtContext>
    FmtContext::iterator format(const std::multimap<std::string,std::pair<std::string,std::string>>& multimap, FmtContext& ctx) const
    {
        auto ret = std::format_to(ctx.out(), "<");
        std::string delim = "";
        for (const auto& pair : multimap) {
            ret = std::format_to(ctx.out(), "{}{}:{};{}", delim, pair.first, pair.second.first, pair.second.second);
            delim = ", ";
        }
        ret = std::format_to(ctx.out(), ">");
        return ret;
    }
};
