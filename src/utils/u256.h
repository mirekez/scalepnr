#pragma once
#include <cstdint>

struct u256
{
    __uint128_t hi, lo;

    u256() = default;
    u256(__uint128_t h, __uint128_t l) : hi(h), lo(l) {}
    u256(size_t i) { hi = 0; lo = i; }

    u256 operator&(const u256 &v) const { return {hi & v.hi, lo & v.lo}; }
    u256 operator|(const u256 &v) const { return {hi | v.hi, lo | v.lo}; }
    u256 operator~() const { return {~hi, ~lo}; }

    u256& operator&=(const u256 &v) { hi &= v.hi; lo &= v.lo; return *this; }
    u256& operator|=(const u256 &v) { hi |= v.hi; lo |= v.lo; return *this; }

    u256 operator<<(int s)
    {
        if (s == 0) {
            return *this;
        }
        if (s >= 256) {
            return u256(0);
        }
        if (s >= 128) {
            return u256( (__uint128_t)(lo << (s - 128)), 0 );
        } else {
            u256 tmp;
            tmp.hi = (hi << s) | (lo >> (128 - s));
            tmp.lo = (lo << s);
            return tmp;
        }
    }

    u256 operator>>(int s)
    {
        if (s == 0) {
            return *this;
        }
        if (s >= 256) {
            return u256(0, 0);
        }
        if (s >= 128) {
            return u256( 0, (__uint128_t)(hi >> (s - 128)) );
        } else {
            u256 tmp;
            tmp.lo = (lo >> s) | (hi << (128 - s));
            tmp.hi = (hi >> s);
            return tmp;
        }
    }

    bool operator==(const u256& v)
    {
        return hi == v.hi && lo == v.lo;
    }

    bool operator!=(const u256& v)
    {
        return !(*this == v);
    }

    int ctz128(__uint128_t x)
    {
        if (x == 0) {
            return 128;
        }

        uint64_t lo = (uint64_t)x;
        if (lo != 0) {
            return __builtin_ctzll(lo);
        }

        uint64_t hi = (uint64_t)(x >> 64);
        return 64 + __builtin_ctzll(hi);
    }

    int ffs256()
    {
        if (lo != 0) {
            return ctz128(lo);
        }
        if (hi != 0) {
            return 128 + ctz128(hi);
        }
        return -1;
    }

    std::string str()
    {
        std::string s;
        return std::to_string((uint64_t)(hi>>64)) + std::to_string((uint64_t)hi)
             + std::to_string((uint64_t)(lo>>64)) + std::to_string((uint64_t)lo);
    }
};
