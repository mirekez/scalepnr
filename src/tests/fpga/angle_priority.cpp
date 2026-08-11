#include "Crossbar.h"

#include <array>
#include <cmath>
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
    return (encode(dx) << 8) | (encode(dy) << 4) | (lane & 0xf);
}

int decodeSigned4(int value)
{
    value &= 0xf;
    return (value & 0x8) ? value - 16 : value;
}

fpga::Coord encodedJumpDelta(int src)
{
    return {decodeSigned4((src >> 8) & 0xf), decodeSigned4((src >> 4) & 0xf)};
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

int expectedFirst(const std::vector<int>& sources, fpga::Coord from, fpga::Coord to)
{
    fpga::Coord target = targetBucket(to - from);
    int selected = -1;
    long double selected_angle = 0;
    int selected_length = 0;
    for (int src : sources) {
        fpga::Coord delta = encodedJumpDelta(src);
        long double cross = std::abs(static_cast<long double>(delta.x * target.y - delta.y * target.x));
        long double dot = static_cast<long double>(delta.x * target.x + delta.y * target.y);
        long double angle = std::atan2(cross, dot);
        int length = std::abs(delta.x) + std::abs(delta.y);
        if (selected < 0 || angle < selected_angle
            || (angle == selected_angle && length < selected_length)
            || (angle == selected_angle && length == selected_length && (src & 0xf) < (selected & 0xf))
            || (angle == selected_angle && length == selected_length
                && (src & 0xf) == (selected & 0xf) && src < selected)) {
            selected = src;
            selected_angle = angle;
            selected_length = length;
        }
    }
    return selected;
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
        cb.dst_by_src[src].push_back(fpga::CBType::ResolvedJump{encodedJumpDelta(src), cb.type_id, dsts, {}, false});
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

void testMostlyWestTargetPrefersWestBeforeNorth()
{
    constexpr int local = 13;
    int west = encodeJump(-1, 0, 2);
    int north_short = encodeJump(0, -1, 0);
    int north_long = encodeJump(0, -6, 1);
    int east = encodeJump(1, 0, 0);
    int south = encodeJump(0, 6, 0);
    std::vector<int> sources{north_short, north_long, west, east, south};
    fpga::CBType cb = makeCrossbar(sources, local);
    fpga::CBState state{};
    state.type = &cb;

    // Regression for the real failed fanout geometry: (-10,-2) is almost west,
    // so an octant-based implementation must not exhaust north before west.
    int first = state.iterate(false, local, {132, 117}, {122, 115}, -1);
    int second = state.iterate(false, local, {132, 117}, {122, 115}, first);
    int third = state.iterate(false, local, {132, 117}, {122, 115}, second);
    int fourth = state.iterate(false, local, {132, 117}, {122, 115}, third);
    int fifth = state.iterate(false, local, {132, 117}, {122, 115}, fourth);
    require(first == west,
        std::format("mostly-west target selected wrong first jump: actual={}, expected={}", first, west));
    require(second == north_short, "short north line was not second after the west line");
    require(third == north_long, "long north line did not follow its shorter equal-angle line");
    require(fourth == south, "south line was not ordered before the opposite east line");
    require(fifth == east, "opposite east line was not last");
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

void testNodeSpecificPriorityExcludesUnreachableSources()
{
    constexpr int selected_local = 14;
    constexpr int unrelated_local = 15;
    int west_short = encodeJump(-1, 0, 0);
    int west_long = encodeJump(-4, 0, 0);
    int exact_north_but_unreachable = encodeJump(0, -1, 0);
    fpga::CBType cb = makeCrossbar(
        {west_long, exact_north_but_unreachable, west_short}, selected_local);
    cb.local_src[selected_local].jump = bit(west_short) | bit(west_long);
    cb.local_src[unrelated_local].jump = bit(exact_north_but_unreachable);
    cb.rebuildOutgoingSrcs();

    const std::vector<uint16_t>& ordered = cb.orderedSrcNodes(
        fpga::CB_NODE_LOCAL, selected_local, fpga::Coord{-20, 0});
    // Check: the hot-path cache preserves angle/length ordering while omitting
    // a globally valid source that cannot be reached from this local node.
    require(ordered.size() == 2 && ordered[0] == west_short
            && ordered[1] == west_long,
        "node-specific priority included an unreachable source or lost short-line order");
    require(std::find(ordered.begin(), ordered.end(),
                      exact_north_but_unreachable) == ordered.end(),
        "node-specific priority retained another local node's source");
}

void testRandomMasks()
{
    constexpr int local = 11;
    std::mt19937 rng(0x51a1e5u);
    std::uniform_int_distribution<int> coord_dist(-40, 40);
    std::uniform_int_distribution<int> lane_dist(0, 15);
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
        testMostlyWestTargetPrefersWestBeforeNorth();
        testBusyAndDeadendAreSkipped();
        testNodeSpecificPriorityExcludesUnreachableSources();
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
