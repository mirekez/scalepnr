#pragma once
#include <array>
#include <cstdint>
#include <format>
#include <string>

struct u1024
{
    static constexpr int words_count = 8;
    std::array<__uint128_t, words_count> words{};

    u1024(__uint128_t hi = 0, __uint128_t lo = 0)
    {
        words[0] = lo;
        words[1] = hi;
    }

    u1024 operator&(const u1024& v) const
    {
        u1024 out;
        for (int i = 0; i < words_count; ++i) {
            out.words[i] = words[i] & v.words[i];
        }
        return out;
    }

    u1024 operator|(const u1024& v) const
    {
        u1024 out;
        for (int i = 0; i < words_count; ++i) {
            out.words[i] = words[i] | v.words[i];
        }
        return out;
    }

    u1024 operator~() const
    {
        u1024 out;
        for (int i = 0; i < words_count; ++i) {
            out.words[i] = ~words[i];
        }
        return out;
    }

    u1024& operator&=(const u1024& v)
    {
        for (int i = 0; i < words_count; ++i) {
            words[i] &= v.words[i];
        }
        return *this;
    }

    u1024& operator|=(const u1024& v)
    {
        for (int i = 0; i < words_count; ++i) {
            words[i] |= v.words[i];
        }
        return *this;
    }

    u1024 operator<<(int s) const
    {
        if (s <= 0) {
            return *this;
        }
        if (s >= 1024) {
            return {};
        }
        u1024 out;
        int word_shift = s / 128;
        int bit_shift = s % 128;
        for (int i = words_count - 1; i >= word_shift; --i) {
            out.words[i] |= words[i - word_shift] << bit_shift;
            if (bit_shift != 0 && i - word_shift - 1 >= 0) {
                out.words[i] |= words[i - word_shift - 1] >> (128 - bit_shift);
            }
        }
        return out;
    }

    u1024 operator>>(int s) const
    {
        if (s <= 0) {
            return *this;
        }
        if (s >= 1024) {
            return {};
        }
        u1024 out;
        int word_shift = s / 128;
        int bit_shift = s % 128;
        for (int i = 0; i + word_shift < words_count; ++i) {
            out.words[i] |= words[i + word_shift] >> bit_shift;
            if (bit_shift != 0 && i + word_shift + 1 < words_count) {
                out.words[i] |= words[i + word_shift + 1] << (128 - bit_shift);
            }
        }
        return out;
    }

    bool operator==(const u1024& v) const
    {
        return words == v.words;
    }

    bool operator!=(const u1024& v) const
    {
        return !(*this == v);
    }

    static int ctz128(__uint128_t x)
    {
        if (x == 0) {
            return 128;
        }
        uint64_t lo = static_cast<uint64_t>(x);
        if (lo != 0) {
            return __builtin_ctzll(lo);
        }
        uint64_t hi = static_cast<uint64_t>(x >> 64);
        return 64 + __builtin_ctzll(hi);
    }

    int ffs1024() const
    {
        for (int i = 0; i < words_count; ++i) {
            if (words[i] != 0) {
                return i * 128 + ctz128(words[i]);
            }
        }
        return -1;
    }

    int ffs256() const
    {
        return ffs1024();
    }

    template <typename F>
    bool for_each_set_bit128(__uint128_t part, int offset, F&& f) const
    {
        while (part) {
            __uint128_t lsb = part & -part;
            int index = 0;
            uint64_t low = static_cast<uint64_t>(lsb);
            if (low != 0) {
                index = __builtin_ctzll(low);
            } else {
                index = 64 + __builtin_ctzll(static_cast<uint64_t>(lsb >> 64));
            }
            if (f(offset + index)) {
                return true;
            }
            part &= part - 1;
        }
        return false;
    }

    template <typename F>
    bool for_each_set_bit(F&& f) const
    {
        for (int i = 0; i < words_count; ++i) {
            if (for_each_set_bit128(words[i], i * 128, f)) {
                return true;
            }
        }
        return false;
    }

    std::string str() const
    {
        std::string out;
        out.reserve(256);
        for (int i = words_count - 1; i >= 0; --i) {
            out += std::format("{:016x}{:016x}",
                static_cast<uint64_t>(words[i] >> 64),
                static_cast<uint64_t>(words[i]));
        }
        return out;
    }
};

using u256 = u1024;
