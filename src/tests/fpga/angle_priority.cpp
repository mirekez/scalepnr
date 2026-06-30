#include "Crossbar.h"

#include <array>
#include <cstdio>
#include <format>
#include <random>
#include <string>
#include <vector>

namespace {

struct TestFailure
{
    std::string message;
};

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw TestFailure{message};
    }
}

NodeMask bit(int index)
{
    return NodeMask{0, 1} << index;
}

int encodeJump(int dx, int dy, int lane)
{
    auto encode = [](int value) {
        return value & 0xf;
    };
    return (encode(dx) << 7) | (encode(dy) << 3) | (lane & 0x7);
}

int decodeSigned4(int value)
{
    value &= 0xf;
    return (value & 0x8) ? value - 16 : value;
}

fpga::Coord encodedJumpDelta(int src)
{
    return {decodeSigned4((src >> 7) & 0xf), decodeSigned4((src >> 3) & 0xf)};
}

int scaledAxis(int value, int max_abs)
{
    if (value == 0 || max_abs == 0) {
        return 0;
    }
    int scaled = (std::abs(value) * 7 + max_abs / 2) / max_abs;
    return value < 0 ? -std::max(1, scaled) : std::max(1, scaled);
}

fpga::Coord targetBucket(fpga::Coord diff)
{
    int max_abs = std::max(std::abs(diff.x), std::abs(diff.y));
    return {scaledAxis(diff.x, max_abs), scaledAxis(diff.y, max_abs)};
}

int directionIndex(fpga::Coord delta)
{
    int sx = (delta.x > 0) - (delta.x < 0);
    int sy = (delta.y > 0) - (delta.y < 0);
    if (sx > 0 && sy == 0) {
        return 0;
    }
    if (sx > 0 && sy < 0) {
        return 1;
    }
    if (sx == 0 && sy < 0) {
        return 2;
    }
    if (sx < 0 && sy < 0) {
        return 3;
    }
    if (sx < 0 && sy == 0) {
        return 4;
    }
    if (sx < 0 && sy > 0) {
        return 5;
    }
    if (sx == 0 && sy > 0) {
        return 6;
    }
    if (sx > 0 && sy > 0) {
        return 7;
    }
    return -1;
}

int expectedFirst(const std::vector<int>& sources, fpga::Coord from, fpga::Coord to)
{
    fpga::Coord target = targetBucket(to - from);
    constexpr int max_cross = 98;
    constexpr int direction_order_offsets[8] = {0, -1, 1, -2, 2, -3, 3, 4};
    int base_direction = directionIndex(target);
    for (int direction_offset : direction_order_offsets) {
        int direction = base_direction >= 0 ? (base_direction + direction_offset + 8) & 7 : -1;
        for (int cross_abs = 0; cross_abs <= max_cross; ++cross_abs) {
            for (int length = 1; length <= 14; ++length) {
                for (int lane = 0; lane < 8; ++lane) {
                    int selected = -1;
                    for (int src : sources) {
                        if ((src & 0x7) != lane) {
                            continue;
                        }
                        fpga::Coord bucket = encodedJumpDelta(src);
                        if (base_direction >= 0 && directionIndex(bucket) != direction) {
                            continue;
                        }
                        int cross = bucket.x * target.y - bucket.y * target.x;
                        if (std::abs(cross) != cross_abs) {
                            continue;
                        }
                        if (std::abs(bucket.x) + std::abs(bucket.y) != length) {
                            continue;
                        }
                        if (selected < 0 || src < selected) {
                            selected = src;
                        }
                    }
                    if (selected >= 0) {
                        return selected;
                    }
                }
            }
        }
    }
    return -1;
}

fpga::CBType makeCrossbar(const std::vector<int>& sources, int local)
{
    fpga::CBType cb{};
    cb.name = "ANGLE_PRIORITY_CB";
    cb.type_id = 0;
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, local, "LOCAL_OUT");
    for (int src : sources) {
        cb.rememberNodeName(fpga::CB_NODE_SRC, src, std::format("SRC_{}", src));
        cb.rememberNodeName(fpga::CB_NODE_DST, src, std::format("DST_{}", src));
        cb.local_src[local].jump |= bit(src);
        fpga::CBJumpState dsts{};
        dsts.jump = bit(src);
        cb.dst_by_src[src].push_back(fpga::CBType::ResolvedJump{encodedJumpDelta(src), cb.type_id, dsts, false});
    }
    cb.rebuildOutgoingSrcs();
    cb.ensureDerivedMasks();
    return cb;
}

void setResolvedDelta(fpga::CBType& cb, int src, fpga::Coord delta)
{
    auto& entries = cb.dst_by_src[src];
    require(!entries.empty(), "test source has no resolved jump entry");
    for (auto& entry : entries) {
        entry.delta = delta;
    }
    cb.rebuildOutgoingSrcs();
}

void testLoadedDeltaOverridesEncodedShape()
{
    constexpr int local = 4;
    int encoded_diagonal_loaded_north = encodeJump(2, -2, 0);
    int encoded_east_loaded_east = encodeJump(1, 0, 0);
    std::vector<int> sources{encoded_diagonal_loaded_north, encoded_east_loaded_east};
    fpga::CBType cb = makeCrossbar(sources, local);
    setResolvedDelta(cb, encoded_diagonal_loaded_north, {0, -1});
    setResolvedDelta(cb, encoded_east_loaded_east, {1, 0});

    fpga::CBState state{};
    state.type = &cb;
    int first = state.iterate(false, local, {0, 20}, {30, 10}, -1);
    require(first == encoded_east_loaded_east,
        std::format("loaded-delta priority did not override encoded source shape: actual={}, expected={}",
            first, encoded_east_loaded_east));
}

void testAngleBeforeWrongDirection()
{
    constexpr int local = 5;
    std::vector<int> sources{
        encodeJump(-1, -1, 0),
        encodeJump(1, 1, 0),
        encodeJump(-1, 0, 0),
        encodeJump(0, -1, 0),
    };
    fpga::CBType cb = makeCrossbar(sources, local);
    fpga::CBState state{};
    state.type = &cb;

    int first = state.iterate(false, local, {10, 10}, {0, 0}, -1);
    require(first == encodeJump(-1, -1, 0),
        std::format("exact target angle was not first: actual={}, expected={}",
            first, encodeJump(-1, -1, 0)));
}

void testShortBeforeLongForSameAngle()
{
    constexpr int local = 7;
    std::vector<int> sources{
        encodeJump(-7, -7, 0),
        encodeJump(-1, -1, 0),
        encodeJump(-3, -3, 0),
    };
    fpga::CBType cb = makeCrossbar(sources, local);
    fpga::CBState state{};
    state.type = &cb;

    int first = state.iterate(false, local, {20, 20}, {0, 0}, -1);
    int second = state.iterate(false, local, {20, 20}, {0, 0}, first);
    int third = state.iterate(false, local, {20, 20}, {0, 0}, second);
    require(first == encodeJump(-1, -1, 0), "short exact-angle line was not first");
    require(second == encodeJump(-3, -3, 0), "medium exact-angle line was not second");
    require(third == encodeJump(-7, -7, 0), "long exact-angle line was not third");
}

void testLongCorrectAngleBeforeShortWrongAngle()
{
    constexpr int local = 8;
    int short_south = encodeJump(0, 1, 0);
    int long_west = encodeJump(-5, 0, 0);
    std::vector<int> sources{short_south, long_west};
    fpga::CBType cb = makeCrossbar(sources, local);
    setResolvedDelta(cb, short_south, {0, 1});
    setResolvedDelta(cb, long_west, {-5, 0});

    fpga::CBState state{};
    state.type = &cb;
    int first = state.iterate(false, local, {81, 85}, {0, 109}, -1);
    require(first == long_west,
        std::format("long line with better angle lost to short wrong-angle line: actual={}, expected={}",
            first, long_west));
}

void testForwardDirectionBeforeOppositeAngle()
{
    constexpr int local = 12;
    int east = encodeJump(1, 0, 0);
    int north = encodeJump(0, -1, 0);
    std::vector<int> sources{east, north};
    fpga::CBType cb = makeCrossbar(sources, local);

    fpga::CBState state{};
    state.type = &cb;
    int first = state.iterate(false, local, {49, 64}, {4, 23}, -1);
    require(first == north,
        std::format("forward north source lost to opposite east source: actual={}, expected={}",
            first, north));
}

void testBusyAndDeadendAreSkipped()
{
    constexpr int local = 9;
    int best = encodeJump(1, 0, 0);
    int next = encodeJump(2, 0, 0);
    int dead = encodeJump(3, 0, 0);
    std::vector<int> sources{dead, best, next};
    fpga::CBType cb = makeCrossbar(sources, local);
    fpga::CBState state{};
    state.type = &cb;
    state.src.jump |= bit(best);
    state.src_deadend.jump |= bit(dead);

    int first = state.iterate(false, local, {0, 0}, {10, 0}, -1);
    require(first == next,
        std::format("busy/deadend source was not skipped: actual={}, expected={}", first, next));
}

void testRandomMasks()
{
    constexpr int local = 11;
    std::mt19937 rng(0x51a1e5u);
    std::uniform_int_distribution<int> coord_dist(-40, 40);
    std::uniform_int_distribution<int> lane_dist(0, 7);
    std::uniform_int_distribution<int> count_dist(8, 48);

    for (int case_index = 0; case_index < 100; ++case_index) {
        fpga::Coord from{coord_dist(rng), coord_dist(rng)};
        fpga::Coord to{coord_dist(rng), coord_dist(rng)};
        if (from.x == to.x && from.y == to.y) {
            to.x += 13;
        }
        fpga::Coord target = targetBucket(to - from);
        std::vector<int> sources;
        sources.push_back(encodeJump(target.x == 0 ? 0 : (target.x < 0 ? -1 : 1),
                                     target.y == 0 ? 0 : (target.y < 0 ? -1 : 1),
                                     lane_dist(rng)));
        int count = count_dist(rng);
        for (int i = 0; i < count; ++i) {
            int dx = (coord_dist(rng) % 15);
            int dy = (coord_dist(rng) % 15);
            if (dx < -7) {
                dx += 15;
            }
            if (dx > 7) {
                dx -= 15;
            }
            if (dy < -7) {
                dy += 15;
            }
            if (dy > 7) {
                dy -= 15;
            }
            if (dx == 0 && dy == 0) {
                dx = target.x == 0 ? 1 : (target.x < 0 ? -1 : 1);
            }
            sources.push_back(encodeJump(dx, dy, lane_dist(rng)));
        }

        fpga::CBType cb = makeCrossbar(sources, local);
        fpga::CBState state{};
        state.type = &cb;
        int expected = expectedFirst(sources, from, to);
        int actual = state.iterate(false, local, from, to, -1);
        require(actual == expected,
            std::format("random priority mismatch case={}, actual={}, expected={}",
                case_index, actual, expected));
    }
}

} // namespace

int main()
{
    try {
        testAngleBeforeWrongDirection();
        testLoadedDeltaOverridesEncodedShape();
        testShortBeforeLongForSameAngle();
        testLongCorrectAngleBeforeShortWrongAngle();
        testForwardDirectionBeforeOppositeAngle();
        testBusyAndDeadendAreSkipped();
        testRandomMasks();
    }
    catch (const TestFailure& failure) {
        std::fprintf(stderr, "angle_priority failed: %s\n", failure.message.c_str());
        return 1;
    }
    catch (const std::exception& error) {
        std::fprintf(stderr, "angle_priority exception: %s\n", error.what());
        return 1;
    }
    return 0;
}
