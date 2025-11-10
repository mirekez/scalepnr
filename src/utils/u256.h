#pragma once
#include <cstdint>

struct u256
{
    __uint128_t hi, lo;

    u256() = default;
    u256(__uint128_t h, __uint128_t l) : hi(h), lo(l) {}

    u256 operator&(const u256 &o) const { return {hi & o.hi, lo & o.lo}; }
    u256 operator|(const u256 &o) const { return {hi | o.hi, lo | o.lo}; }

    u256& operator&=(const u256 &o){ hi &= o.hi; lo &= o.lo; return *this; }
    u256& operator|=(const u256 &o){ hi |= o.hi; lo |= o.lo; return *this; }
};
