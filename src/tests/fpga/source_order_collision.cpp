#include "Crossbar.h"

#include <array>
#include <cstdio>
#include <format>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace {

constexpr int endpoint = 161;
constexpr fpga::Coord source{54, 68};
constexpr fpga::Coord sink{54, 59};
// Numeric LOCAL exits and source occupancy from the failed Basic attempt.
constexpr std::array local_exits{
    18, 22, 66, 98, 162, 194, 243, 248, 259, 260, 546, 738,
    818, 978, 1028, 1029, 3077, 3078, 3378, 3538, 3618, 3810, 3843, 3845};
constexpr std::array occupied_sources{
    160, 161, 192, 240, 241, 242, 248, 249,
    256, 257, 258, 262, 547, 3808, 3809, 3846};
// Controlled prior query: distinct DST exits deliberately disjoint from LOCAL.
// The real trace did not capture the earlier cache writer or its exit list.
constexpr std::array arrival_exits{16, 64};

void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

template <size_t N>
NodeMask mask(const std::array<int, N>& nodes)
{
    NodeMask result{};
    for (int node : nodes) result.setBit(node);
    return result;
}

NodeMask mask(const std::vector<uint16_t>& nodes)
{
    NodeMask result{};
    for (int node : nodes) result.setBit(node);
    return result;
}

struct Fixture {
    fpga::CBType cb;
    fpga::CBState state{};

    Fixture()
    {
        cb.name = "abstract_matrix";
        cb.type_id = 0;
        cb.rememberNodeName(fpga::CB_NODE_LOCAL, endpoint, "drive_161");
        cb.rememberNodeName(fpga::CB_NODE_DST, endpoint, "arrival_161");
        cb.local_src[endpoint].jump = mask(local_exits);
        cb.dst_src[endpoint].jump = mask(arrival_exits);
        for (int src : local_exits) addExit(src);
        for (int src : arrival_exits) addExit(src);
        cb.rebuildOutgoingSrcs();
        state.type = &cb;
        state.src.jump = mask(occupied_sources);
    }

    void addExit(int src)
    {
        cb.rememberNodeName(fpga::CB_NODE_SRC, src, std::format("exit_{}", src));
        cb.rememberNodeName(fpga::CB_NODE_DST, src, std::format("landing_{}", src));
        fpga::CBJumpState landing{};
        landing.jump.setBit(src);
        // Abstract resolved jumps suffice: this test isolates source ordering,
        // not device loading, path completion or owner registration. Vary the
        // deltas so different target directions genuinely change the order.
        auto signedAxis = [](int value) { return value < 8 ? value : value - 16; };
        fpga::Coord delta{signedAxis((src >> 8) & 15), signedAxis((src >> 4) & 15)};
        if (src == arrival_exits[0]) delta = {1, 0};
        if (src == arrival_exits[1]) delta = {-1, 0};
        cb.dst_by_src[src].push_back({delta, cb.type_id, landing, {}, false});
    }

    std::vector<uint16_t> ordered(fpga::CBNodeNameType role)
    {
        return cb.orderedSrcNodes(role, endpoint, sink - source);
    }

    size_t active(const std::vector<uint16_t>& order, bool inspect_transit) const
    {
        NodeMask available = cb.local_src[endpoint].jump & ~state.src_deadend.jump;
        if (!inspect_transit) available &= ~state.src.jump;
        size_t count = 0;
        for (int src : order) {
            if (available.testBit(src) && !cb.dst_by_src[src].empty()) ++count;
        }
        return count;
    }
};

void snapshot()
{
    Fixture local;
    const auto order = local.ordered(fpga::CB_NODE_LOCAL);
    require(order.size() == local_exits.size() && mask(order) == mask(local_exits),
            "cold LOCAL order lost or duplicated reachable exits");
    require(local.active(order, true) == 24 && local.active(order, false) == 23,
            "snapshot must have 24 eligible exits, 23 free and one occupied");
    require((mask(local_exits) & local.state.src.jump) == mask(std::array{248}),
            "only exit_248 may be occupied among the source alternatives");
    require(local.state.src_deadend.jump == NodeMask{}, "snapshot has a deadend");
    const auto* reachable = local.cb.srcNodes(fpga::CB_NODE_LOCAL, endpoint);
    require(reachable && reachable->size() == 24, "source topology is incomplete");

    Fixture arrival;
    const auto arrival_order = arrival.ordered(fpga::CB_NODE_DST);
    require(arrival_order.size() == arrival_exits.size()
                && mask(arrival_order) == mask(arrival_exits),
            "cold DST query does not have the intended separate topology");
    require((mask(local_exits) & mask(arrival_exits)) == NodeMask{},
            "LOCAL and DST exits must be disjoint");
    std::puts("snapshot: 24 reachable LOCAL exits; 23 free; 1 occupied; no deadends; cold queries pass");
}

void collision(bool dst_first)
{
    // Compare with independent cold queries; never calculate an expected cache
    // key here, so the test specifies behavior rather than the implementation.
    Fixture cold_local, cold_dst, warmed;
    const auto expected_local = cold_local.ordered(fpga::CB_NODE_LOCAL);
    const auto expected_dst = cold_dst.ordered(fpga::CB_NODE_DST);
    const auto first_role = dst_first ? fpga::CB_NODE_DST : fpga::CB_NODE_LOCAL;
    const auto second_role = dst_first ? fpga::CB_NODE_LOCAL : fpga::CB_NODE_DST;
    const auto first = warmed.ordered(first_role);
    const auto second = warmed.ordered(second_role);
    const auto& expected_first = dst_first ? expected_dst : expected_local;
    const auto& expected_second = dst_first ? expected_local : expected_dst;
    std::printf("%s: first=%zu second=%zu expected_second=%zu\n",
                dst_first ? "DST then LOCAL" : "LOCAL then DST",
                first.size(), second.size(), expected_second.size());
    if (dst_first) {
        std::printf("LOCAL after DST: active=%zu/24; free_active=%zu/23\n",
                    warmed.active(second, true), warmed.active(second, false));
    }
    require(first == expected_first, "first query differs from its cold control");
    require(second == expected_second,
            "source ordering depends on an earlier query for a different node role");
    require(warmed.ordered(first_role) == expected_first,
            "second query corrupted the first role's cached order");
}

void directions()
{
    Fixture fixture;
    Fixture east, west;
    require(east.cb.orderedSrcNodes(fpga::CB_NODE_DST, endpoint, {7, 0})
                != west.cb.orderedSrcNodes(fpga::CB_NODE_DST, endpoint, {-7, 0}),
            "direction sweep needs a topology with direction-dependent ordering");
    // Exercise both shared-role and adjacent-direction collisions. The oracle
    // filters the global order directly, bypassing the per-node cache.
    for (int sweep = 0; sweep < 2; ++sweep) {
        for (int x = -7; x <= 7; ++x) for (int y = -7; y <= 7; ++y) {
            const fpga::Coord delta{sweep ? -x : x, sweep ? -y : y};
            for (auto role : {fpga::CB_NODE_DST, fpga::CB_NODE_LOCAL}) {
                const NodeMask reachable = role == fpga::CB_NODE_LOCAL
                    ? mask(local_exits) : mask(arrival_exits);
                std::vector<uint16_t> expected;
                for (auto src : fixture.cb.orderedSrcNodes(delta)) {
                    if (reachable.testBit(src)) expected.push_back(src);
                }
                require(fixture.cb.orderedSrcNodes(role, endpoint, delta) == expected,
                        "per-node order aliases another role or direction");
            }
        }
    }
    std::puts("directions: 900 queries match the independent global-order oracle");
}

} // namespace

int main(int argc, char** argv)
{
    try {
        require(argc == 2, "expected snapshot, dst_first, local_first or directions");
        const std::string_view mode = argv[1];
        if (mode == "snapshot") snapshot();
        else if (mode == "dst_first") collision(true);
        else if (mode == "local_first") collision(false);
        else if (mode == "directions") directions();
        else throw std::runtime_error("unknown test case");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "source order regression: %s\n", error.what());
        return 1;
    }
}
