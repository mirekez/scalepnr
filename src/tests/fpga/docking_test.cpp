#include "Device.h"
#include "Docking.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <set>
#include <stdexcept>
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

int encodedJump(int dx, int dy, int num = 0)
{
    auto encode = [](int value) {
        return value & 0xf;
    };
    return (encode(dx) << 8) | (encode(dy) << 4) | (num & 0xf);
}

void rememberJumpTarget(fpga::CBType& cb, int src, int dst, fpga::Coord delta)
{
    fpga::CBJumpState dsts{};
    dsts.jump = bit(dst);
    fpga::CBType::ResolvedJump entry{};
    entry.delta = delta;
    entry.target_cb_type_id = cb.type_id;
    entry.dsts = dsts;
    cb.dst_by_src[src].push_back(entry);
}

void rememberJumpTarget(fpga::CBType& cb, int src, int dst, fpga::Coord delta, int target_cb_type_id)
{
    fpga::CBJumpState dsts{};
    dsts.jump = bit(dst);
    fpga::CBType::ResolvedJump entry{};
    entry.delta = delta;
    entry.target_cb_type_id = target_cb_type_id;
    entry.dsts = dsts;
    entry.target_tile_coord = true;
    cb.dst_by_src[src].push_back(entry);
}

void rememberConn(fpga::CBType& cb, fpga::CBNodeNameType from_type, int from,
                  fpga::CBNodeNameType to_type, int to)
{
    const std::string* from_name = cb.nodeName(from_type, from);
    const std::string* to_name = cb.nodeName(to_type, to);
    cb.rememberConnName(from_type, from, to_type, to,
        from_name ? *from_name : std::to_string(from),
        to_name ? *to_name : std::to_string(to));
}

fpga::CBType makeDockingCrossbar()
{
    fpga::CBType cb{};
    cb.name = "DOCK";
    cb.type_id = 0;
    constexpr int dst = 0;
    constexpr int pin = 20;
    std::vector<std::pair<int, fpga::Coord>> srcs{
        {encodedJump(0, -1), fpga::Coord{0, -1}},
        {encodedJump(1, 0), fpga::Coord{1, 0}},
        {encodedJump(0, 1), fpga::Coord{0, 1}},
        {encodedJump(-1, 0), fpga::Coord{-1, 0}},
    };

    cb.rememberNodeName(fpga::CB_NODE_DST, dst, "D0");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN0");
    for (const auto& [src, delta] : srcs) {
        cb.rememberNodeName(fpga::CB_NODE_SRC, src, "S" + std::to_string(src));
        cb.dst_src[dst].jump |= bit(src);        rememberJumpTarget(cb, src, dst, delta);
        rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
    }
    cb.dst_local[dst].local |= bit(pin);
    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_LOCAL, pin);
    cb.rebuildOutgoingSrcs();
    return cb;
}

fpga::CBType makeTargetStepOutCrossbar()
{
    fpga::CBType cb{};
    cb.name = "DOCK_STEP_OUT";
    cb.type_id = 0;
    constexpr int enter_dst = 0;
    constexpr int blocked_dst = 1;
    constexpr int pin = 20;
    int east = encodedJump(1, 0);
    int west = encodedJump(-1, 0);

    cb.rememberNodeName(fpga::CB_NODE_DST, enter_dst, "ENTER_DST");
    cb.rememberNodeName(fpga::CB_NODE_DST, blocked_dst, "BLOCKED_DST");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN0");
    cb.rememberNodeName(fpga::CB_NODE_SRC, east, "EAST");
    cb.rememberNodeName(fpga::CB_NODE_SRC, west, "WEST");

    cb.dst_src[blocked_dst].jump |= bit(east);    rememberJumpTarget(cb, east, blocked_dst, fpga::Coord{1, 0});
    rememberConn(cb, fpga::CB_NODE_DST, blocked_dst, fpga::CB_NODE_SRC, east);

    cb.dst_src[blocked_dst].jump |= bit(west);    rememberJumpTarget(cb, west, enter_dst, fpga::Coord{-1, 0});
    rememberConn(cb, fpga::CB_NODE_DST, blocked_dst, fpga::CB_NODE_SRC, west);

    cb.dst_local[enter_dst].local |= bit(pin);
    rememberConn(cb, fpga::CB_NODE_DST, enter_dst, fpga::CB_NODE_LOCAL, pin);
    cb.rebuildOutgoingSrcs();
    return cb;
}

fpga::CBType makeForwardNamespaceCrossbar()
{
    fpga::CBType cb{};
    cb.name = "FORWARD_NS";
    cb.type_id = 0;
    constexpr int anchor_dst = 0;
    constexpr int target_dst = 7;
    int east = encodedJump(1, 0);

    cb.rememberNodeName(fpga::CB_NODE_DST, anchor_dst, "ANCHOR_DST");
    cb.rememberNodeName(fpga::CB_NODE_SRC, east, "EAST_SRC");
    cb.dst_src[anchor_dst].jump |= bit(east);
    fpga::CBJumpState target_dsts{};
    target_dsts.jump = bit(target_dst);
    cb.dst_by_src[east].push_back(fpga::CBType::ResolvedJump{
        fpga::Coord{1, 0},
        1,
        target_dsts,
        {},
        false
    });
    rememberConn(cb, fpga::CB_NODE_DST, anchor_dst, fpga::CB_NODE_SRC, east);
    cb.rebuildOutgoingSrcs();
    return cb;
}

fpga::CBType makeTargetNamespaceCrossbar()
{
    fpga::CBType cb{};
    cb.name = "TARGET_NS";
    cb.type_id = 1;
    constexpr int target_dst = 7;
    constexpr int pin = 20;

    cb.rememberNodeName(fpga::CB_NODE_DST, target_dst, "TARGET_DST");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "TARGET_PIN");
    cb.dst_local[target_dst].local |= bit(pin);
    rememberConn(cb, fpga::CB_NODE_DST, target_dst, fpga::CB_NODE_LOCAL, pin);
    cb.rebuildOutgoingSrcs();
    return cb;
}

fpga::CBType makeRandomBackwardCrossbar(int type_id, int dst,
                                        int first_target_dst, int second_target_dst)
{
    fpga::CBType cb{};
    cb.name = "RANDOM_BACKWARD_" + std::to_string(type_id);
    cb.type_id = type_id;
    constexpr int pin = 20;
    std::vector<std::pair<int, fpga::Coord>> srcs{
        {encodedJump(0, -1), fpga::Coord{0, -1}},
        {encodedJump(1, 0), fpga::Coord{1, 0}},
        {encodedJump(0, 1), fpga::Coord{0, 1}},
        {encodedJump(-1, 0), fpga::Coord{-1, 0}},
    };

    cb.rememberNodeName(fpga::CB_NODE_DST, dst, "D" + std::to_string(type_id));
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "P" + std::to_string(type_id));
    for (const auto& [src, delta] : srcs) {
        cb.rememberNodeName(fpga::CB_NODE_SRC, src,
            "S" + std::to_string(type_id) + "_" + std::to_string(src));
        cb.dst_src[dst].jump |= bit(src);
        rememberJumpTarget(cb, src, first_target_dst, delta, 0);
        rememberJumpTarget(cb, src, second_target_dst, delta, 1);
        rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
    }
    cb.dst_local[dst].local |= bit(pin);
    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_LOCAL, pin);
    cb.rebuildOutgoingSrcs();
    return cb;
}

std::vector<fpga::Tile*> resetTwoTypeGrid(int width, int height,
                                          fpga::CBType& forward_cb,
                                          fpga::CBType& target_cb,
                                          fpga::Coord target_coord)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    forward_cb.type_id = 0;
    target_cb.type_id = 1;
    device.cb_types.push_back(forward_cb);
    device.cb_types.push_back(target_cb);
    device.grid_spec.size = {width, height};
    device.size_width = width;
    device.size_height = height;
    device.tile_grid.resize(static_cast<size_t>(width * height));

    std::vector<fpga::Tile*> result;
    result.reserve(device.tile_grid.size());
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            fpga::Tile& tile = device.tile_grid[static_cast<size_t>(y * width + x)];
            tile.coord = {x, y};
            tile.cb_coord = tile.coord;
            tile.name = {x, y};
            tile.cb = {};
            tile.pin_state = {};
            tile.cb_type = (x == target_coord.x && y == target_coord.y)
                ? &device.cb_types[1]
                : &device.cb_types[0];
            tile.routedNets.clear();
            result.push_back(&tile);
        }
    }
    return result;
}

std::vector<fpga::Tile*> resetGrid(int width, int height, fpga::CBType& cb)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    cb.type_id = 0;
    device.cb_types.push_back(cb);
    device.grid_spec.size = {width, height};
    device.size_width = width;
    device.size_height = height;
    device.tile_grid.resize(static_cast<size_t>(width * height));

    std::vector<fpga::Tile*> result;
    result.reserve(device.tile_grid.size());
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            fpga::Tile& tile = device.tile_grid[static_cast<size_t>(y * width + x)];
            tile.coord = {x, y};
            tile.cb_coord = tile.coord;
            tile.name = {x, y};
            tile.cb = {};
            tile.pin_state = {};
            tile.cb_type = &cb;
            tile.routedNets.clear();
            result.push_back(&tile);
        }
    }
    return result;
}

std::vector<fpga::Tile*> resetRandomTwoTypeGrid(int width, int height,
                                                fpga::CBType& first_cb,
                                                fpga::CBType& second_cb,
                                                std::mt19937& rng)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    first_cb.type_id = 0;
    second_cb.type_id = 1;
    device.cb_types.push_back(first_cb);
    device.cb_types.push_back(second_cb);
    device.grid_spec.size = {width, height};
    device.size_width = width;
    device.size_height = height;
    device.tile_grid.resize(static_cast<size_t>(width * height));

    std::vector<fpga::Tile*> result;
    result.reserve(device.tile_grid.size());
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            fpga::Tile& tile = device.tile_grid[static_cast<size_t>(y * width + x)];
            tile.coord = {x, y};
            tile.cb_coord = tile.coord;
            tile.name = {x, y};
            tile.cb = {};
            tile.pin_state = {};
            tile.cb_type = &device.cb_types[rng() & 1u];
            tile.routedNets.clear();
            result.push_back(&tile);
        }
    }
    return result;
}

std::vector<fpga::Coord> makeCorridor(fpga::Coord source, fpga::Coord target, bool horizontal_first)
{
    std::vector<fpga::Coord> path;
    fpga::Coord curr = source;
    path.push_back(curr);
    auto step_x = [&]() {
        while (curr.x != target.x) {
            curr.x += curr.x < target.x ? 1 : -1;
            path.push_back(curr);
        }
    };
    auto step_y = [&]() {
        while (curr.y != target.y) {
            curr.y += curr.y < target.y ? 1 : -1;
            path.push_back(curr);
        }
    };
    if (horizontal_first) {
        step_x();
        step_y();
    }
    else {
        step_y();
        step_x();
    }
    return path;
}

void occupyBlockedTile(fpga::Tile& tile)
{
    tile.cb.dst.jump |= bit(0);
    tile.cb.src.jump |= bit(encodedJump(0, -1)) | bit(encodedJump(1, 0))
        | bit(encodedJump(0, 1)) | bit(encodedJump(-1, 0));
    tile.cb.local.local |= bit(20);
}

void docking_finds_one_random_free_path(unsigned seed)
{
    std::mt19937 rng(seed);
    fpga::CBType cb = makeDockingCrossbar();
    std::vector<fpga::Tile*> tiles = resetGrid(13, 13, cb);

    std::uniform_int_distribution<int> coord_dist(3, 9);
    fpga::Coord source{coord_dist(rng), coord_dist(rng)};
    fpga::Coord target;
    do {
        target = {coord_dist(rng), coord_dist(rng)};
    } while ((std::abs(target.x - source.x) + std::abs(target.y - source.y)) == 0
        || (std::abs(target.x - source.x) + std::abs(target.y - source.y)) > 5);

    std::vector<fpga::Coord> corridor = makeCorridor(source, target, (seed & 1u) != 0);
    std::set<std::pair<int, int>> free_tiles;
    for (fpga::Coord coord : corridor) {
        free_tiles.insert({coord.x, coord.y});
    }

    for (fpga::Tile* tile : tiles) {
        if (std::abs(tile->coord.x - target.x) <= 5 && std::abs(tile->coord.y - target.y) <= 5
            && !free_tiles.contains({tile->coord.x, tile->coord.y})) {
            occupyBlockedTile(*tile);
        }
    }

    fpga::Tile* source_tile = fpga::Device::current().getTile(source.x, source.y);
    fpga::Tile* target_tile = fpga::Device::current().getTile(target.x, target.y);
    require(source_tile && target_tile, "source or target tile missing");
    pnr::DockingResult result = pnr::dockGrounding(*source_tile, 0, "D0", *target_tile, bit(20), 5, 5);
    require(result.success,
        "dockGrounding did not find the deliberately freed corridor for seed " + std::to_string(seed)
        + " source=(" + std::to_string(source.x) + "," + std::to_string(source.y) + ")"
        + " target=(" + std::to_string(target.x) + "," + std::to_string(target.y) + ")"
        + " target_seeds=" + std::to_string(result.target_seed_count)
        + " fpop=" + std::to_string(result.forward_pop_count)
        + " fpush=" + std::to_string(result.forward_push_count)
        + " bpop=" + std::to_string(result.backward_pop_count)
        + " bpush=" + std::to_string(result.backward_push_count));
    require(result.fragments.size() >= 2, "dockGrounding returned an incomplete route suffix");
    require(result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN, "dockGrounding did not finish at a tile pin");
    require(result.fragments.back().local == 20, "dockGrounding finished at the wrong local input");

    for (const fpga::Wire& fragment : result.fragments) {
        if (fragment.type != fpga::Wire::WIRE_CROSSBAR || fragment.jump < 0) {
            continue;
        }
        require(free_tiles.contains({fragment.from.x, fragment.from.y}),
            "dockGrounding used a blocked source-side tile");
        require(free_tiles.contains({fragment.to.x, fragment.to.y}),
            "dockGrounding used a blocked destination-side tile");
    }
}

void docking_extends_from_existing_anchor_dst()
{
    fpga::CBType cb = makeDockingCrossbar();
    std::vector<fpga::Tile*> tiles = resetGrid(13, 13, cb);
    fpga::Coord source{4, 4};
    fpga::Coord target{8, 4};
    std::vector<fpga::Coord> corridor = makeCorridor(source, target, true);
    std::set<std::pair<int, int>> free_tiles;
    for (fpga::Coord coord : corridor) {
        free_tiles.insert({coord.x, coord.y});
    }

    for (fpga::Tile* tile : tiles) {
        if (std::abs(tile->coord.x - target.x) <= 5 && std::abs(tile->coord.y - target.y) <= 5
            && !free_tiles.contains({tile->coord.x, tile->coord.y})) {
            occupyBlockedTile(*tile);
        }
    }

    fpga::Tile* source_tile = fpga::Device::current().getTile(source.x, source.y);
    fpga::Tile* target_tile = fpga::Device::current().getTile(target.x, target.y);
    require(source_tile && target_tile, "source or target tile missing");

    // A continued partial route already owns its anchor dst; docking must only
    // allocate the new outgoing src from that anchor instead of rejecting it.
    source_tile->cb.dst.jump |= bit(0);
    pnr::DockingResult result = pnr::dockGrounding(*source_tile, 0, "D0", *target_tile, bit(20), 5, 5);
    require(result.success, "dockGrounding rejected an already-leased anchor dst");
    require(!result.fragments.empty(), "dockGrounding returned no continuation fragments");
    require(result.fragments.front().from.x == source.x && result.fragments.front().from.y == source.y,
        "dockGrounding did not start from the leased anchor tile");
}

void docking_backward_meets_an_existing_anchor_dst()
{
    fpga::CBType cb{};
    cb.name = "LEASED_ANCHOR";
    cb.type_id = 0;
    constexpr int dst = 0;
    constexpr int pin = 20;
    int east = encodedJump(1, 0);
    cb.rememberNodeName(fpga::CB_NODE_SRC, east, "EAST_SRC");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "TARGET_PIN");
    cb.dst_src[dst].jump |= bit(east);
    rememberJumpTarget(cb, east, dst, fpga::Coord{1, 0});
    for (int entry = 0; entry < 8; ++entry) {
        cb.rememberNodeName(fpga::CB_NODE_DST, entry, "ENTRY_" + std::to_string(entry));
        cb.dst_local[entry].local |= bit(pin);
        rememberConn(cb, fpga::CB_NODE_DST, entry, fpga::CB_NODE_LOCAL, pin);
    }
    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, east);
    cb.rebuildOutgoingSrcs();
    fpga::Coord source{5, 6};
    fpga::Coord target{6, 6};
    resetGrid(13, 13, cb);

    fpga::Tile* source_tile = fpga::Device::current().getTile(source.x, source.y);
    fpga::Tile* target_tile = fpga::Device::current().getTile(target.x, target.y);
    require(source_tile && target_tile, "leased-anchor docking source or target tile missing");

    // Reproduce a continued route: its final destination node is already
    // leased by the same route before backward grounding starts.
    source_tile->cb.dst.jump |= bit(dst);
    pnr::DockingResult result = pnr::dockGrounding(
        *source_tile, dst, "ANCHOR_DST", *target_tile, bit(pin), 5, 5, true);

    require(result.success,
        "backward docking could not reach the already-leased route anchor"
        " seeds=" + std::to_string(result.target_seed_count)
        + " bpop=" + std::to_string(result.backward_pop_count)
        + " bpush=" + std::to_string(result.backward_push_count)
        + " busy=" + std::to_string(result.backward_busy_reject_count)
        + " missing=" + std::to_string(result.backward_missing_prev_dst_count)
        + " reaches=" + std::to_string(result.backward_mapping_reaches_count));
    require(result.target_seed_count == 8,
        "leased-anchor regression did not create all eight free destination entries");
    require(result.backward_push_count == 1,
        "backward docking did not extend the free destination entry to the route anchor");
    require(result.forward_push_count == 0,
        "docking bypassed the leased-anchor regression through forward expansion");
    require(result.fragments.size() == 3,
        "leased-anchor backward docking returned an unexpected route length");

    // Focused tracing must retain every free target seed and the concrete
    // backward path that met the existing forward anchor.
    std::array<bool, 8> traced_seeds{};
    bool traced_meeting_path = false;
    for (const pnr::DockingBackwardAttempt& attempt : result.backward_attempts) {
        if (attempt.result == "seed" && attempt.target_dst >= 0
            && attempt.target_dst < static_cast<int>(traced_seeds.size())) {
            traced_seeds[attempt.target_dst] = true;
        }
        if (attempt.result == "meet" && attempt.target_dst == dst
            && attempt.fragments.size() == 3) {
            traced_meeting_path = true;
        }
    }
    require(std::all_of(traced_seeds.begin(), traced_seeds.end(), [](bool traced) {
        return traced;
    }), "backward docking trace omitted a free target destination seed");
    require(traced_meeting_path,
        "backward docking trace omitted the path meeting the existing anchor");
}

void docking_ignores_src_deadends()
{
    fpga::CBType cb = makeDockingCrossbar();
    resetGrid(13, 13, cb);
    fpga::Coord source{4, 4};
    fpga::Coord target{8, 4};

    for (fpga::Tile& tile : fpga::Device::current().tile_grid) {
        if (std::abs(tile.coord.x - target.x) <= 5 && std::abs(tile.coord.y - target.y) <= 5) {
            tile.cb.src_deadend.jump = tile.cb_type->dst_src[0].jump;
        }
    }

    fpga::Tile* source_tile = fpga::Device::current().getTile(source.x, source.y);
    fpga::Tile* target_tile = fpga::Device::current().getTile(target.x, target.y);
    require(source_tile && target_tile, "deadend docking source or target tile missing");

    // Docking is a bounded final-entry search; sticky deadends from earlier
    // forward attempts must not block an otherwise free local docking path.
    pnr::DockingResult result = pnr::dockGrounding(*source_tile, 0, "D0", *target_tile, bit(20), 5, 5);
    require(result.success, "dockGrounding incorrectly treated src_deadend as real occupancy");
    require(result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
        "deadend-ignoring docking did not finish at a tile pin");
}

void docking_steps_out_from_non_enterable_target_dst()
{
    fpga::CBType cb = makeTargetStepOutCrossbar();
    resetGrid(13, 13, cb);
    fpga::Coord target{6, 6};
    fpga::Tile* target_tile = fpga::Device::current().getTile(target.x, target.y);
    require(target_tile, "target tile missing");

    // The forward route has already reached the destination tile, but on a dst
    // rail that cannot reach the required local pin. Docking must step out.
    target_tile->cb.dst.jump |= bit(1);
    pnr::DockingResult result = pnr::dockGrounding(*target_tile, 1, "BLOCKED_DST", *target_tile, bit(20), 5, 5);
    require(result.success, "dockGrounding could not step out from a non-enterable target dst");
    require(result.fragments.size() >= 4, "dockGrounding returned too few fragments for step-out docking");
    require(result.fragments.front().type == fpga::Wire::WIRE_CROSSBAR,
        "step-out docking did not start with a crossbar step");
    require(result.fragments.front().from.x == target.x && result.fragments.front().from.y == target.y,
        "step-out docking did not start on the target tile");
    require(result.fragments.front().to.x != target.x || result.fragments.front().to.y != target.y,
        "step-out docking did not leave the target tile before entering");
    require(result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
        "step-out docking did not finish at a tile pin");
    require(result.fragments.back().local == 20, "step-out docking finished at the wrong local input");
}

void docking_iob_uses_wider_endpoint_window()
{
    fpga::CBType cb = makeDockingCrossbar();
    std::vector<fpga::Tile*> tiles = resetGrid(25, 25, cb);
    fpga::Coord source{6, 12};
    fpga::Coord target{18, 12};
    std::vector<fpga::Coord> corridor = makeCorridor(source, target, true);
    std::set<std::pair<int, int>> free_tiles;
    for (fpga::Coord coord : corridor) {
        free_tiles.insert({coord.x, coord.y});
    }

    for (fpga::Tile* tile : tiles) {
        if (std::abs(tile->coord.x - target.x) <= 12 && std::abs(tile->coord.y - target.y) <= 12
            && !free_tiles.contains({tile->coord.x, tile->coord.y})) {
            occupyBlockedTile(*tile);
        }
    }

    fpga::Tile* source_tile = fpga::Device::current().getTile(source.x, source.y);
    fpga::Tile* target_tile = fpga::Device::current().getTile(target.x, target.y);
    require(source_tile && target_tile, "source or target tile missing for IOB docking");

    // Generic CLB grounding is intentionally radius 5 and must not cover this
    // edge-style endpoint distance.
    pnr::DockingResult clb_result = pnr::dockGrounding(*source_tile, 0, "D0", *target_tile, bit(20), 5, 5);
    require(!clb_result.success, "generic docking unexpectedly crossed the IOB-sized gap");

    // IOB docking uses the same bitmask transitions, but with the wider edge
    // endpoint window needed for I/O route tiles.
    pnr::DockingResult iob_result = pnr::dockIOB(*source_tile, 0, "D0", *target_tile, bit(20));
    require(iob_result.success, "dockIOB did not find the deliberately freed I/O corridor");
    require(iob_result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN,
        "dockIOB did not finish at a tile pin");
    require(iob_result.fragments.back().local == 20, "dockIOB finished at the wrong local input");
    require(iob_result.forward_push_count < 160 && iob_result.backward_push_count < 160,
        "dockIOB searched too many side branches for a one-tile-wide directed corridor");
}

void docking_backward_uses_resolved_target_dst_namespace()
{
    fpga::CBType forward_cb = makeForwardNamespaceCrossbar();
    fpga::CBType target_cb = makeTargetNamespaceCrossbar();
    fpga::Coord source{5, 6};
    fpga::Coord target{6, 6};
    resetTwoTypeGrid(13, 13, forward_cb, target_cb, target);

    fpga::Tile* source_tile = fpga::Device::current().getTile(source.x, source.y);
    fpga::Tile* target_tile = fpga::Device::current().getTile(target.x, target.y);
    require(source_tile && target_tile, "namespace docking source or target tile missing");

    // Check: the previous tile has only local dst 0 for the outgoing source,
    // while the resolved destination tile uses dst 7 for the target pin.
    require((source_tile->cb_type->dsts_reaching_src[encodedJump(1, 0)].jump & bit(0)) != NodeMask{},
        "namespace docking setup lost previous-tile dst 0");
    require((source_tile->cb_type->dsts_reaching_src[encodedJump(1, 0)].jump & bit(7)) == NodeMask{},
        "namespace docking setup accidentally made previous-tile dst 7 valid");

    pnr::DockingResult result = pnr::dockGrounding(*source_tile, 0, "ANCHOR_DST", *target_tile, bit(20), 5, 5);
    require(result.success, "dockGrounding failed when previous dst and target dst used different numeric namespaces");
    require(result.backward_push_count == 1,
        "namespace docking should need exactly one backward push from target dst 7 to previous dst 0");
    require(result.fragments.size() == 3, "namespace docking returned an unexpected route length");
    require(result.fragments.front().from.x == source.x && result.fragments.front().to.x == target.x,
        "namespace docking did not use the direct resolved jump");
    require(result.fragments.front().local == 0 && result.fragments.front().jump == encodedJump(1, 0),
        "namespace docking used the wrong previous-tile dst/src");
    require(result.fragments[1].local == 7 && result.fragments.back().local == 20,
        "namespace docking did not enter the resolved target dst/local");
}

void docking_reports_only_the_reachable_blocked_terminal()
{
    fpga::CBType forward_cb = makeForwardNamespaceCrossbar();
    fpga::CBType target_cb = makeTargetNamespaceCrossbar();
    constexpr int reachable_dst = 7;
    constexpr int unreachable_dst = 9;
    constexpr int pin = 20;
    target_cb.rememberNodeName(fpga::CB_NODE_DST, unreachable_dst, "UNREACHABLE_DST");
    target_cb.dst_local[unreachable_dst].local |= bit(pin);
    rememberConn(target_cb, fpga::CB_NODE_DST, unreachable_dst, fpga::CB_NODE_LOCAL, pin);
    target_cb.rebuildOutgoingSrcs();

    fpga::Coord source{5, 6};
    fpga::Coord target{6, 6};
    resetTwoTypeGrid(13, 13, forward_cb, target_cb, target);
    fpga::Tile* source_tile = fpga::Device::current().getTile(source.x, source.y);
    fpga::Tile* target_tile = fpga::Device::current().getTile(target.x, target.y);
    require(source_tile && target_tile, "blocked-terminal test tiles missing");

    // Both terminal rails are occupied, but dst_by_src physically reaches only
    // dst 7. Docking must report dst 7 rather than an arbitrary busy entry.
    target_tile->cb.dst.jump |= bit(reachable_dst) | bit(unreachable_dst);
    pnr::DockingResult result = pnr::dockGrounding(
        *source_tile, 0, "ANCHOR_DST", *target_tile, bit(pin), 5, 5);
    require(!result.success, "blocked-terminal docking unexpectedly leased an occupied entry");
    require(result.blocked_terminal_reachable,
        "docking did not report the occupied entry reached by its forward search");
    require(result.blocked_dst == reachable_dst,
        "docking reported an occupied entry that the forward route cannot reach");
    require(result.blocked_pin == pin, "docking reported the wrong terminal pin");
}

void docking_builds_random_multi_hop_backward_routes(unsigned seed)
{
    std::mt19937 rng(seed);
    constexpr int first_dst = 3;
    constexpr int second_dst = 41;
    fpga::CBType first_cb = makeRandomBackwardCrossbar(0, first_dst, first_dst, second_dst);
    fpga::CBType second_cb = makeRandomBackwardCrossbar(1, second_dst, first_dst, second_dst);
    std::vector<fpga::Tile*> tiles = resetRandomTwoTypeGrid(13, 13, first_cb, second_cb, rng);

    std::uniform_int_distribution<int> coord_dist(3, 9);
    fpga::Coord source{coord_dist(rng), coord_dist(rng)};
    fpga::Coord target;
    do {
        target = {coord_dist(rng), coord_dist(rng)};
    } while (std::abs(target.x - source.x) + std::abs(target.y - source.y) < 4
        || std::abs(target.x - source.x) + std::abs(target.y - source.y) > 5);

    std::vector<fpga::Coord> corridor = makeCorridor(source, target, (seed & 1u) != 0);
    std::set<std::pair<int, int>> free_tiles;
    for (size_t i = 0; i < corridor.size(); ++i) {
        fpga::Coord coord = corridor[i];
        free_tiles.insert({coord.x, coord.y});
        fpga::Tile* tile = fpga::Device::current().getTile(coord.x, coord.y);
        require(tile, "random backward corridor tile missing");
        tile->cb_type = &fpga::Device::current().cb_types[(i + seed) & 1u];
    }
    for (fpga::Tile* tile : tiles) {
        if (!free_tiles.contains({tile->coord.x, tile->coord.y})) {
            occupyBlockedTile(*tile);
        }
    }

    fpga::Tile* source_tile = fpga::Device::current().getTile(source.x, source.y);
    fpga::Tile* target_tile = fpga::Device::current().getTile(target.x, target.y);
    require(source_tile && target_tile, "random backward source or target tile missing");
    int source_dst = source_tile->cb_type->type_id == 0 ? first_dst : second_dst;

    pnr::DockingResult result = pnr::dockGrounding(
        *source_tile, source_dst, "", *target_tile, bit(20), 5, 5);
    require(result.success,
        "random multi-hop backward docking failed for seed " + std::to_string(seed)
        + " bpop=" + std::to_string(result.backward_pop_count)
        + " bpush=" + std::to_string(result.backward_push_count));
    require(result.backward_push_count >= 2,
        "random docking did not exercise multi-hop backward expansion for seed " + std::to_string(seed));
    require(result.fragments.back().type == fpga::Wire::WIRE_TILE_PIN
            && result.fragments.back().local == 20,
        "random backward docking did not finish at the requested pin");

    int cross_type_edges = 0;
    for (const fpga::Wire& fragment : result.fragments) {
        if (fragment.type != fpga::Wire::WIRE_CROSSBAR || fragment.jump < 0) {
            continue;
        }
        fpga::Tile* from_tile = fpga::Device::current().getTile(fragment.from.x, fragment.from.y);
        fpga::Tile* to_tile = fpga::Device::current().getTile(fragment.to.x, fragment.to.y);
        require(from_tile && to_tile && from_tile->cb_type && to_tile->cb_type,
            "random backward route references a missing tile");
        require((from_tile->cb_type->dsts_reaching_src[fragment.jump].jump & bit(fragment.local)) != NodeMask{},
            "random backward route used a source unreachable from its previous dst");
        std::vector<fpga::TileJumpTarget> targets =
            fpga::Device::current().resolveJumpTargets(*from_tile, fragment.jump);
        bool resolved = std::any_of(targets.begin(), targets.end(), [&](const fpga::TileJumpTarget& candidate) {
            return candidate.tile == to_tile && candidate.dst_node == fragment.dst;
        });
        require(resolved, "random backward route edge is absent from dst_by_src");
        if (from_tile->cb_type != to_tile->cb_type) {
            ++cross_type_edges;
        }
    }
    require(cross_type_edges >= 2,
        "random backward route did not cross enough distinct destination namespaces");
}

void docking_memoizes_failed_backward_positions()
{
    fpga::CBType cb = makeDockingCrossbar();
    resetGrid(9, 9, cb);
    constexpr int radius = 3;
    constexpr int max_depth = 3;
    fpga::Coord source{1, 1};
    fpga::Coord target{4, 4};
    fpga::Tile* source_tile = fpga::Device::current().getTile(source.x, source.y);
    fpga::Tile* target_tile = fpga::Device::current().getTile(target.x, target.y);
    require(source_tile && target_tile, "memoized docking source or target tile missing");

    // Block every forward exit at the anchor so the backward search exhausts
    // a cyclic grid and must encounter already-expanded (coord,dst) positions.
    source_tile->cb.src.jump = source_tile->cb_type->dst_src[0].jump;
    pnr::DockingResult result = pnr::dockGrounding(
        *source_tile, 0, "D0", *target_tile, bit(20), max_depth, radius);
    require(!result.success, "memoized docking unexpectedly escaped the blocked anchor");
    require(result.backward_pop_count > 4,
        "memoized docking test did not expand enough backward positions");
    require(result.backward_deadend_count == result.backward_pop_count,
        "docking did not memoize every exhausted backward position");
    require(result.backward_deadend_reject_count > 0,
        "docking did not prune a branch returning to a memoized failed position");

    int one_window_scan_limit = 0;
    for (int y = target.y - radius; y <= target.y + radius; ++y) {
        for (int x = target.x - radius; x <= target.x + radius; ++x) {
            fpga::Tile* tile = fpga::Device::current().getTile(x, y);
            if (tile && tile->cb_type) {
                one_window_scan_limit +=
                    static_cast<int>(tile->cb_type->dst_by_src.values.size());
            }
        }
    }
    require(result.backward_mapping_scan_count <= one_window_scan_limit,
        "docking rescanned the reverse jump database after its temporary index was built");
}

} // namespace

int main()
{
    try {
        for (unsigned seed = 1; seed <= 20; ++seed) {
        docking_finds_one_random_free_path(seed);
        }
        docking_extends_from_existing_anchor_dst();
        docking_backward_meets_an_existing_anchor_dst();
        docking_ignores_src_deadends();
        docking_steps_out_from_non_enterable_target_dst();
        docking_iob_uses_wider_endpoint_window();
        docking_backward_uses_resolved_target_dst_namespace();
        docking_reports_only_the_reachable_blocked_terminal();
        for (unsigned seed = 1; seed <= 20; ++seed) {
            docking_builds_random_multi_hop_backward_routes(seed);
        }
        docking_memoizes_failed_backward_positions();
    }
    catch (const TestFailure& failure) {
        std::fprintf(stderr, "docking_test failed: %s\n", failure.message.c_str());
        return EXIT_FAILURE;
    }
    catch (const std::exception& ex) {
        std::fprintf(stderr, "docking_test exception: %s\n", ex.what());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
