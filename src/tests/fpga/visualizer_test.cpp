#include "TimingPath.h"
#include "RegBunch.h"
#include "Device.h"
#include "Visualizer.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>

namespace {

struct Failure : std::runtime_error
{
    using std::runtime_error::runtime_error;
};

void require(bool condition, const std::string& message)
{
    if (!condition) {
        throw Failure(message);
    }
}

void addBit(NodeMask& mask, int bit)
{
    mask.setBit(bit);
}

void prepareDevice(fpga::Device& device, int size = 5)
{
    device.tile_grid.clear();
    device.cb_types.clear();
    device.tile_types.clear();
    device.size_width = size;
    device.size_height = size;
    device.grid_spec.size = {size, size};
    device.cb_types.reserve(2);
    device.cb_types.emplace_back();
    fpga::CBType& cb = device.cb_types.back();
    cb.name = "GENERIC_VISUALIZER_CB";
    cb.type_id = 0;
    cb.base_type_id = 0;

    constexpr int source_local = 1;
    constexpr int source_joint = 2;
    constexpr int source_node = 0x100;
    constexpr int target_dst = 3;
    constexpr int target_local = 4;
    constexpr int free_local = 5;
    constexpr int constant_zero_local = 6;
    constexpr int constant_one_local = 7;
    addBit(cb.local_joint[source_local].joint, source_joint);
    addBit(cb.joint_src[source_joint].jump, source_node);
    addBit(cb.local_src[source_local].jump, source_node);
    addBit(cb.dst_local[target_dst].local, target_local);
    addBit(cb.local_output_nodes, source_local);
    addBit(cb.local_input_nodes, target_local);
    addBit(cb.local_input_nodes, free_local);
    addBit(cb.constant_zero_nodes, constant_zero_local);
    addBit(cb.constant_one_nodes, constant_one_local);
    addBit(cb.valid_dst_nodes, target_dst);
    fpga::CBJumpState destinations;
    addBit(destinations.jump, target_dst);
    cb.dst_by_src[source_node].push_back(
        {{1, 0}, cb.type_id, destinations, {}, false});

    cb.rememberNodeName(fpga::CB_NODE_LOCAL, source_local, "LOCAL_SOURCE");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, target_local, "LOCAL_TARGET");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, free_local, "LOCAL_FREE");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, constant_zero_local, "CONST_ZERO");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, constant_one_local, "CONST_ONE");
    cb.rememberNodeName(fpga::CB_NODE_JOINT, source_joint, "JOINT_SOURCE");
    cb.rememberNodeName(fpga::CB_NODE_SRC, source_node, "SRC_EAST");
    cb.rememberNodeName(fpga::CB_NODE_DST, target_dst, "DST_FROM_WEST");

    device.tile_grid.resize(size * size);
    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            fpga::Tile& tile = device.tile_grid[y * size + x];
            tile.coord = {x, y};
            tile.name = {x, y};
            tile.cb_type = &cb;
            tile.cb.type = &cb;
            tile.cb_coord = tile.coord;
            addBit(tile.incoming_dst_nodes, target_dst);
        }
    }
}

void runVisualizerRegression()
{
    fpga::Device& device = fpga::Device::current();
    prepareDevice(device);
    fpga::Tile* source = device.getTile(2, 2);
    fpga::Tile* target = device.getTile(3, 2);
    require(source && target, "visualizer fixture has no route tiles");

    constexpr int source_local = 1;
    constexpr int source_joint = 2;
    constexpr int source_node = 0x100;
    constexpr int target_dst = 3;
    constexpr int target_local = 4;
    constexpr int free_local = 5;
    constexpr int constant_zero_local = 6;
    constexpr int constant_one_local = 7;
    addBit(source->cb.local.local, source_local);
    addBit(source->cb.joint.jump, source_joint);
    addBit(source->cb.src.jump, source_node);
    addBit(target->cb.dst.jump, target_dst);
    addBit(target->cb.local.local, target_local);
    addBit(target->pin_state.leased_nodes, target_local);

    rtl::Inst owner;
    std::vector<fpga::Wire> route;
    fpga::Wire source_pin;
    source_pin.type = fpga::Wire::WIRE_TILE_PIN;
    source_pin.from = source->coord;
    source_pin.to = source->coord;
    source_pin.local = source_local;
    route.push_back(source_pin);
    fpga::Wire jump;
    jump.type = fpga::Wire::WIRE_CROSSBAR;
    jump.from = source->coord;
    jump.to = target->coord;
    jump.local = source_local;
    jump.pos = 0;
    jump.joint = source_joint;
    jump.jump = source_node;
    jump.route_jump = source_node;
    jump.dst = target_dst;
    route.push_back(jump);
    fpga::Wire enter;
    enter.type = fpga::Wire::WIRE_CROSSBAR;
    enter.from = target->coord;
    enter.to = target->coord;
    enter.local = target_dst;
    enter.pos = 1;
    route.push_back(enter);
    fpga::Wire target_pin;
    target_pin.type = fpga::Wire::WIRE_TILE_PIN;
    target_pin.from = target->coord;
    target_pin.to = target->coord;
    target_pin.local = target_local;
    route.push_back(target_pin);
    owner.wires.push_back(std::move(route));

    rtl::Net net;
    net.name = "GENERIC_VISUALIZER_LOGICAL_NET";
    const std::string highlighted_route = "GENERIC_VISUALIZER_ROUTE";
    net.appendRouteBinding({&owner, 0, nullptr, nullptr, "OUT", "IN",
                            highlighted_route});
    uint64_t route_id = net.routeId(0);
    source->routed_bindings.push_back({&net, route_id});
    target->routed_bindings.push_back({&net, route_id});

    // Draw an ordinary binding over the exact same physical path. The named
    // route must still be painted last and remain visibly red.
    rtl::Net overlapping_net;
    overlapping_net.name = "GENERIC_VISUALIZER_OVERLAP";
    overlapping_net.appendRouteBinding(
        {&owner, 0, nullptr, nullptr, "OUT", "IN", "OVERLAPPING_ROUTE"});
    uint64_t overlapping_id = overlapping_net.routeId(0);
    source->routed_bindings.push_back({&overlapping_net, overlapping_id});
    target->routed_bindings.push_back({&overlapping_net, overlapping_id});

    fpga::Visualizer visualizer(device);
    visualizer.drawCB(2, 2);
    auto src_point = visualizer.nodePosition(
        source->coord, fpga::CB_NODE_SRC, source_node);
    auto joint_point = visualizer.nodePosition(
        source->coord, fpga::CB_NODE_JOINT, source_joint);
    auto dst_point = visualizer.nodePosition(
        target->coord, fpga::CB_NODE_DST, target_dst);
    auto occupied_point = visualizer.nodePosition(
        target->coord, fpga::CB_NODE_LOCAL, target_local);
    auto free_point = visualizer.nodePosition(
        target->coord, fpga::CB_NODE_LOCAL, free_local);
    auto constant_zero_point = visualizer.nodePosition(
        target->coord, fpga::CB_NODE_LOCAL, constant_zero_local);
    auto constant_one_point = visualizer.nodePosition(
        target->coord, fpga::CB_NODE_LOCAL, constant_one_local);
    require(src_point && joint_point && dst_point && occupied_point && free_point
                && constant_zero_point && constant_one_point,
            "visualizer omitted a numeric crossbar node");
    require(free_point->x == occupied_point->x + 90
                && constant_zero_point->x == free_point->x
                && constant_one_point->x == free_point->x + 110,
            "local guides do not use the exceptional 90/110 pixel spacing");
    require(occupied_point->x
                == 3 * fpga::Visualizer::tile_pixels + 130,
            "local guide group was not shifted 10 pixels right");
    require(joint_point->x
                == 2 * fpga::Visualizer::tile_pixels + 350,
            "joint guide does not remain separate from local guides");
    require(constant_zero_point->y - free_point->y == 8,
            "middle local guide does not use the reduced node pitch");
    int target_origin_y = 2 * fpga::Visualizer::tile_pixels;
    require(free_point->y >= target_origin_y + 10 + 60 + 8
                && joint_point->y >= target_origin_y + 10 + 60 + 8,
            "local or joint nodes started before the lowered guide origin");
    require(src_point->x > 2 * fpga::Visualizer::tile_pixels
                               + fpga::Visualizer::tile_pixels - 32,
            "eastbound SRC was not placed on the source crossbar right edge");
    require(src_point->y == 3 * fpga::Visualizer::tile_pixels - 11 - 50 - 8,
            "right SRC lost its corner caption padding");
    require(dst_point->x < 3 * fpga::Visualizer::tile_pixels + 32,
            "west-arriving DST was not placed on the target crossbar left edge");
    require(dst_point->y == 3 * fpga::Visualizer::tile_pixels - 11 - 50 - 8,
            "left DST lost its corner caption padding");

    visualizer.drawRoutes(highlighted_route);
    require(visualizer.pixel(occupied_point->x, occupied_point->y)
                == fpga::Visualizer::Color{25, 185, 80, 255},
            "occupied local node is not green");
    require(visualizer.pixel(free_point->x, free_point->y)
                == fpga::Visualizer::Color{25, 105, 235, 255},
            "free local node is not blue");
    int jump_mid_x = (src_point->x + dst_point->x) / 2;
    int jump_mid_y = (src_point->y + dst_point->y) / 2;
    fpga::Visualizer::Color jump_pixel = visualizer.pixel(jump_mid_x, jump_mid_y);
    require(jump_pixel == fpga::Visualizer::Color{255, 0, 0, 255},
            "selected physical route was not highlighted in red");
    require(visualizer.pixel(jump_mid_x, jump_mid_y + 1)
                == fpga::Visualizer::Color{255, 0, 0, 255},
            "selected physical route is missing its lower highlight stripe");
    require(visualizer.pixel(jump_mid_x, jump_mid_y - 1)
                == fpga::Visualizer::Color{255, 0, 0, 255},
            "selected physical route was not drawn three pixels wide");
    auto has_black_pixel = [&](int x0, int y0, int x1, int y1) {
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                if (visualizer.pixel(x, y)
                    == fpga::Visualizer::Color{0, 0, 0, 255}) {
                    return true;
                }
            }
        }
        return false;
    };
    int target_origin_x = 3 * fpga::Visualizer::tile_pixels;
    bool has_coordinate_title = false;
    for (int y = target_origin_y + fpga::Visualizer::tile_pixels / 2 - 12;
         y <= target_origin_y + fpga::Visualizer::tile_pixels / 2 + 12; ++y) {
        for (int x = target_origin_x + fpga::Visualizer::tile_pixels / 2 - 60;
             x <= target_origin_x + fpga::Visualizer::tile_pixels / 2 + 60;
             ++x) {
            if (visualizer.pixel(x, y)
                == fpga::Visualizer::Color{105, 105, 105, 255}) {
                has_coordinate_title = true;
            }
        }
    }
    require(has_coordinate_title,
            "tile coordinate background title was not drawn");
    require(has_black_pixel(target_origin_x + 12, free_point->y - 3,
                            free_point->x - 4, free_point->y + 3),
            "local title was not drawn in black left of the local guide");
    bool has_highlight_name = false;
    for (int y = std::max(0, src_point->y - 16); y < src_point->y - 6; ++y) {
        for (int x = src_point->x + 7;
             x < std::min(fpga::Visualizer::image_pixels, src_point->x + 180);
             ++x) {
            if (visualizer.pixel(x, y)
                == fpga::Visualizer::Color{255, 0, 0, 255}) {
                has_highlight_name = true;
            }
        }
    }
    require(has_highlight_name,
            "highlighted route name was not drawn near its starting point");
    require(visualizer.stats().highlighted_labels == 1,
            "visualizer did not emit exactly one highlighted route title");
    require(visualizer.stats().node_titles > 0,
            "visualizer did not report rendered node titles");
    require(visualizer.stats().title_anchor_collisions == 0,
            "visualizer assigned different node titles to one text anchor");
    require(visualizer.stats().occupied_nodes >= 5,
            "visualizer did not count live leased nodes");
    require(visualizer.stats().routes == 2
                && visualizer.stats().route_segments > 0,
            "visualizer did not count indexed route bindings and segments");

    // An unfinished committed prefix may not be registered on its tiles yet;
    // direct rendering must still expose that physical route in diagnostics.
    size_t indexed_routes = visualizer.stats().routes;
    visualizer.drawRoute(owner.wires.front(), true);
    require(visualizer.stats().routes == indexed_routes + 1,
            "visualizer omitted an explicit unindexed partial route");

    // A failed task without any route still exposes its exact endpoint.
    require(visualizer.highlightNode(target->coord, fpga::CB_NODE_LOCAL,
                                     free_local, "unrouted target"),
            "visualizer could not mark an unresolved endpoint");
    require(visualizer.stats().highlighted_nodes == 1,
            "visualizer did not count the unresolved endpoint marker");

    std::string filename = "/tmp/scalepnr_visualizer_test.png";
    visualizer.writePNG(filename);
    std::ifstream png(filename, std::ios::binary);
    std::array<unsigned char, 24> header{};
    png.read(reinterpret_cast<char*>(header.data()), header.size());
    require(png.gcount() == static_cast<std::streamsize>(header.size()),
            "visualizer PNG is missing or truncated");
    const std::array<unsigned char, 8> signature{137, 80, 78, 71, 13, 10, 26, 10};
    require(std::equal(signature.begin(), signature.end(), header.begin()),
            "visualizer output has an invalid PNG signature");
    auto read_be32 = [&](size_t offset) {
        return (static_cast<uint32_t>(header[offset]) << 24)
            | (static_cast<uint32_t>(header[offset + 1]) << 16)
            | (static_cast<uint32_t>(header[offset + 2]) << 8)
            | static_cast<uint32_t>(header[offset + 3]);
    };
    require(read_be32(16) == fpga::Visualizer::image_pixels
                && read_be32(20) == fpga::Visualizer::image_pixels,
            "visualizer PNG is not 2560x2560");
    png.close();
    std::remove(filename.c_str());

    // Failure capture must retain one visible title even after an unrouted
    // net has no fragments in this window; an existing route title stays unique.
    visualizer.drawFailureLabel(highlighted_route);
    require(visualizer.stats().highlighted_labels == 1,
            "failure capture duplicated an existing highlighted route title");
    visualizer.drawCB(2, 2);
    visualizer.drawRoutes("UNROUTED_WITHOUT_FRAGMENTS");
    visualizer.drawFailureLabel("UNROUTED_WITHOUT_FRAGMENTS");
    require(visualizer.stats().highlighted_labels == 1 &&
                visualizer.stats().occupied_nodes > 0 &&
                visualizer.stats().route_segments > 0,
            "failure capture lost its title or live congestion routes");

    // A resource view shares its owner's crossbar, not a second free copy.
    fpga::Tile* resource = device.getTile(1, 2);
    fpga::Tile* no_crossbar = device.getTile(0, 2);
    resource->cb_coord = source->coord;
    no_crossbar->cb_type = nullptr;
    visualizer.drawCB(2, 2);
    visualizer.drawRoutes(highlighted_route);
    auto alias_point = visualizer.nodePosition(resource->coord, fpga::CB_NODE_SRC, source_node);
    auto owner_point = visualizer.nodePosition(source->coord, fpga::CB_NODE_SRC, source_node);
    // Owner and resource lookups identify the same leased green dot.
    require(alias_point && owner_point && alias_point->x == owner_point->x
                && alias_point->y == owner_point->y
                && visualizer.pixel(alias_point->x, alias_point->y)
                    == fpga::Visualizer::Color{25, 185, 80, 255},
            "resource view duplicated or lost its owner's lease");
    // Every in-grid slot stays visible, including the tile without a crossbar.
    require(visualizer.stats().tiles == 25 && visualizer.stats().crossbars == 23
                && visualizer.stats().attached_tiles == 1
                && visualizer.stats().tiles_without_crossbar == 1,
            "visualizer omitted a tile or counted a duplicate crossbar");
    require(visualizer.pixel(10, 2 * 512 + 10)
                == fpga::Visualizer::Color{70, 76, 86, 255}
                && !visualizer.nodePosition(no_crossbar->coord, fpga::CB_NODE_SRC, source_node),
            "tile without a crossbar lost its frame or gained fake nodes");
    // Resource tiles show the ownership label but no duplicate guides.
    require(has_black_pixel(512 + 22, 2 * 512 + 26, 2 * 512 - 20, 2 * 512 + 34)
                && visualizer.pixel(512 + 130, 2 * 512 + 100)
                    == fpga::Visualizer::Color{247, 248, 250, 255},
            "resource tile did not replace duplicate guides with its ownership label");
    // Indexed routes still render at the owner after the duplicate view is removed.
    require(visualizer.stats().routes == 2 && visualizer.stats().route_segments > 0
                && visualizer.pixel(jump_mid_x, jump_mid_y)
                    == fpga::Visualizer::Color{255, 0, 0, 255},
            "ownership-aware drawing lost the highlighted route");
    // A route pin recorded on the resource tile joins the following owner-side fragment.
    auto alias_route = owner.wires.front();
    alias_route.front().from = resource->coord;
    alias_route.front().to = resource->coord;
    size_t before_segments = visualizer.stats().route_segments;
    visualizer.drawRoute(owner.wires.front(), true, highlighted_route);
    size_t owner_segments = visualizer.stats().route_segments - before_segments;
    before_segments = visualizer.stats().route_segments;
    visualizer.drawRoute(alias_route, true, highlighted_route);
    require(visualizer.stats().route_segments - before_segments == owner_segments,
            "resource-side endpoint broke the owner's route segment sequence");
    visualizer.writePNG("/tmp/scalepnr_visualizer_ownership.png");
    // A resource whose owner is outside the window must not invent a local crossbar.
    visualizer.drawCB(-1, 2);
    require(!visualizer.nodePosition(resource->coord, fpga::CB_NODE_SRC, source_node),
            "out-of-window crossbar was duplicated in its resource tile");
    resource->cb_coord = resource->coord;
    no_crossbar->cb_type = source->cb_type;

    // A device-edge view keeps fixed slots and silently omits invalid tiles.
    visualizer.drawCB(0, 0);
    require(visualizer.nodePosition({0, 0}, fpga::CB_NODE_LOCAL, source_local)
                .has_value(),
            "device-edge visualization omitted the valid corner tile");
    require(!visualizer.nodePosition({-1, 0}, fpga::CB_NODE_LOCAL, source_local)
                 .has_value(),
            "device-edge visualization created an out-of-grid crossbar");

    // Even very dense edge classes stay in one row, reducing their pitch.
    fpga::CBType& dense_cb = device.cb_types.front();
    dense_cb.rememberNodeName(fpga::CB_NODE_SRC, 499,
                              "ANNOTATION_WITHOUT_PHYSICAL_SOURCE");
    (void)dense_cb.dst_by_src[499];
    for (int node = 500; node < 760; ++node) {
        dense_cb.rememberNodeName(fpga::CB_NODE_SRC, node,
                                  "DENSE_SOURCE_" + std::to_string(node));
        dense_cb.src_priority_deltas[node].push_back({1, 0});
        fpga::CBType::ResolvedJump jump_target;
        jump_target.delta = {1, 0};
        jump_target.target_cb_type_id = dense_cb.type_id;
        jump_target.dsts.jump.setBit(target_dst);
        dense_cb.dst_by_src[node].push_back(std::move(jump_target));
    }
    visualizer.drawCB(2, 2);
    require(!visualizer.nodePosition({2, 2}, fpga::CB_NODE_SRC, 499),
            "annotation-only source was drawn as a physical node");
    auto first_dense = visualizer.nodePosition({2, 2}, fpga::CB_NODE_SRC, 500);
    auto last_dense = visualizer.nodePosition({2, 2}, fpga::CB_NODE_SRC, 759);
    require(first_dense && last_dense && last_dense->x == first_dense->x
                && last_dense->y < first_dense->y
                && last_dense->y >= 2 * fpga::Visualizer::tile_pixels + 10,
            "dense edge nodes wrapped or ran outside their tile");
}

void runOffscreenIncomingDirectionRegression()
{
    fpga::Device& device = fpga::Device::current();
    prepareDevice(device, 17);
    fpga::CBType& cb = device.cb_types.front();
    const std::array<fpga::Coord, 4> directions{{{0, -6}, {6, 0},
                                               {0, 6}, {-6, 0}}};
    auto node_id = [](fpga::Coord direction, int lane) {
        return ((direction.x & 15) << 8) | ((direction.y & 15) << 4) | lane;
    };
    for (auto direction : directions) {
        for (int lane = 0; lane < 4; ++lane) {
            int node = node_id(direction, lane);
            fpga::CBJumpState destinations;
            addBit(destinations.jump, node);
            cb.dst_by_src[node].push_back(
                {direction, cb.type_id, destinations, {}, false});
            addBit(cb.dst_local[node].local, 4);
            addBit(cb.valid_dst_nodes, node);
            cb.rememberNodeName(fpga::CB_NODE_SRC, node,
                               "LONG_SOURCE_" + std::to_string(node));
            cb.rememberNodeName(fpga::CB_NODE_DST, node,
                               "LONG_TARGET_" + std::to_string(node));
        }
    }

    // Every incoming source is six tiles away, outside this 5x5 viewport.
    // DSTs must use the arrival side, opposite to the encoded travel direction.
    fpga::Visualizer visualizer(device);
    visualizer.drawCB(8, 8);
    auto on_edge = [](fpga::Visualizer::Point point, fpga::Coord direction) {
        int x = point.x - 2 * fpga::Visualizer::tile_pixels;
        int y = point.y - 2 * fpga::Visualizer::tile_pixels;
        if (direction.x < 0) return x < 32;
        if (direction.x > 0) return x > fpga::Visualizer::tile_pixels - 32;
        if (direction.y < 0) return y < 32;
        return y > fpga::Visualizer::tile_pixels - 32;
    };
    for (auto direction : directions) {
        for (int lane = 0; lane < 4; ++lane) {
            int node = node_id(direction, lane);
            auto src = visualizer.nodePosition({8, 8}, fpga::CB_NODE_SRC, node);
            auto dst = visualizer.nodePosition({8, 8}, fpga::CB_NODE_DST, node);
            require(src && dst, "long jump endpoint omitted");
            require(on_edge(*src, direction), "long SRC drawn on wrong edge");
            require(on_edge(*dst, {-direction.x, -direction.y}),
                    "offscreen incoming DST drawn on outgoing edge: "
                        + std::to_string(node));
        }
    }
}

void runSingleRowEdgesRegression()
{
    fpga::Device& device = fpga::Device::current();
    prepareDevice(device);
    fpga::CBType& cb = device.cb_types.front();
    const std::array<fpga::Coord, 4> directions{{{0, -1}, {1, 0},
                                               {0, 1}, {-1, 0}}};
    // Forty incoming and twenty-four outgoing nodes per edge reproduce the
    // real failure image's overflow without depending on database wire names.
    for (int edge = 0; edge < 4; ++edge) {
        for (int i = 0; i < 40; ++i) {
            int src = 500 + edge * 100 + i % 24;
            int dst = 1500 + edge * 100 + i;
            fpga::CBJumpState destinations;
            addBit(destinations.jump, dst);
            cb.dst_by_src[src].push_back(
                {directions[edge], cb.type_id, destinations, {}, false});
            addBit(cb.dst_local[dst].local, 4);
            addBit(cb.valid_dst_nodes, dst);
            cb.rememberNodeName(fpga::CB_NODE_SRC, src,
                               "EDGE_SOURCE_" + std::to_string(src));
            cb.rememberNodeName(fpga::CB_NODE_DST, dst,
                               "EDGE_TARGET_" + std::to_string(dst));
        }
    }
    fpga::Visualizer visualizer(device);
    visualizer.drawCB(2, 2);
    std::set<std::pair<int, int>> points;
    auto check = [&](int edge, fpga::CBNodeNameType type, int node) {
        auto p = visualizer.nodePosition({2, 2}, type, node);
        require(p.has_value(), "single-row layout omitted an edge node");
        int x = p->x - 2 * fpga::Visualizer::tile_pixels;
        int y = p->y - 2 * fpga::Visualizer::tile_pixels;
        require(x >= 10 && x <= 501 && y >= 10 && y <= 501,
                "single-row edge node escaped its tile");
        require((edge == 0 && y == 10) || (edge == 1 && x == 501)
                    || (edge == 2 && y == 501) || (edge == 3 && x == 10),
                "edge node wrapped into a second row: node=" + std::to_string(node)
                    + " edge=" + std::to_string(edge) + " x=" + std::to_string(x)
                    + " y=" + std::to_string(y));
        require(points.emplace(x, y).second,
                "DST and SRC nodes overlap in the single row");
        return edge % 2 == 0 ? x : y;
    };
    for (int edge = 0; edge < 4; ++edge) {
        int previous_src = 0;
        int previous_dst = 0;
        // Walking clockwise, DST grows forward and SRC grows backward.
        int dst_sign = edge < 2 ? 1 : -1;
        int start = edge == 0 ? 13 : 68;
        int end = edge == 0 ? 498 : 443;
        for (int i = 0; i < 40; ++i) {
            int position = check(edge, fpga::CB_NODE_DST,
                                 1500 + ((edge + 2) % 4) * 100 + i);
            if (i == 0 && edge != 3) { // Existing fixture has one extra west DST.
                require(position == (dst_sign > 0 ? start : end),
                        "DST did not use the edge-specific corner padding");
            }
            if (i) require((position - previous_dst) * dst_sign > 0,
                           "DST order was not preserved");
            previous_dst = position;
        }
        for (int i = 0; i < 24; ++i) {
            int position = check(edge, fpga::CB_NODE_SRC, 500 + edge * 100 + i);
            if (i == 0 && edge != 1) { // Existing fixture has one extra east SRC.
                require(position == (dst_sign > 0 ? end : start),
                        "SRC did not use the edge-specific corner padding");
            }
            if (i) require((position - previous_src) * dst_sign < 0,
                           "SRC order was not preserved");
            previous_src = position;
        }
    }
    require(visualizer.stats().title_anchor_collisions == 0,
            "single-row captions were dropped at duplicate anchors");
}

void runCompactCaptionRegression()
{
    fpga::Device& device = fpga::Device::current();
    prepareDevice(device);
    fpga::CBType& cb = device.cb_types.front();
    cb.node_names.clear();
    const std::string label = "L_H_U_08";
    const std::array<int, 4> sources{{0xf0, 0x100, 0x10, 0xf00}};
    const std::array<fpga::Coord, 4> directions{{{0, -1}, {1, 0},
                                               {0, 1}, {-1, 0}}};
    for (int edge = 0; edge < 4; ++edge) {
        fpga::CBJumpState destinations;
        addBit(destinations.jump, 3);
        cb.dst_by_src[sources[edge]].clear();
        cb.dst_by_src[sources[edge]].push_back(
            {directions[edge], cb.type_id, destinations, {}, false});
        cb.rememberNodeName(fpga::CB_NODE_SRC, sources[edge], label);
    }
    fpga::Visualizer visualizer(device);
    visualizer.drawCB(2, 2);
    // Native 4x6 glyphs: bottom and right strokes must survive in every
    // orientation; counters and inter-character gaps must stay unpainted.
    const std::array<std::array<int, 6>, 8> expected{{
        {{8, 8, 8, 8, 8, 15}}, {{0, 0, 0, 0, 0, 15}},
        {{9, 9, 15, 9, 9, 9}}, {{0, 0, 0, 0, 0, 15}},
        {{9, 9, 9, 9, 9, 6}}, {{0, 0, 0, 0, 0, 15}},
        {{6, 9, 9, 9, 9, 6}}, {{6, 9, 6, 9, 9, 6}}
    }};
    constexpr int origin = 2 * fpga::Visualizer::tile_pixels;
    constexpr int text_width = 7 * 6 + 4;
    for (int edge = 0; edge < 4; ++edge) {
        auto node = visualizer.nodePosition({2, 2}, fpga::CB_NODE_SRC, sources[edge]);
        require(node.has_value(), "caption test source omitted");
        fpga::Visualizer::Point anchor;
        if (edge == 0) anchor = {node->x - 2, origin + 13};
        else if (edge == 1) anchor = {origin + 501 - 3 - text_width, node->y - 2};
        else if (edge == 2) anchor = {node->x + 2, origin + 498};
        else anchor = {origin + 13, node->y - 2};
        for (int ch = 0; ch < 8; ++ch) {
            for (int row = 0; row < 6; ++row) {
                for (int column = 0; column < (ch == 7 ? 4 : 6); ++column) {
                    int x = anchor.x, y = anchor.y;
                    if (edge == 0) { x += 5 - row; y += ch * 6 + column; }
                    else if (edge == 2) { x += row; y -= ch * 6 + column; }
                    else { x += ch * 6 + column; y += row; }
                    bool ink = column < 4 && (expected[ch][row] & (8 >> column));
                    bool black = visualizer.pixel(x, y) == fpga::Visualizer::Color{0, 0, 0, 255};
                    require(black == ink,
                            "compact caption lost a stroke or filled a gap: edge="
                                + std::to_string(edge) + " glyph=" + label[ch]
                                + " row=" + std::to_string(row)
                                + " column=" + std::to_string(column));
                }
            }
        }
    }
}

} // namespace

int main()
{
    try {
        runVisualizerRegression();
        runOffscreenIncomingDirectionRegression();
        runSingleRowEdgesRegression();
        runCompactCaptionRegression();
        std::puts("visualizer tests passed");
        return 0;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "visualizer test failed: %s\n", error.what());
        return 1;
    }
}
