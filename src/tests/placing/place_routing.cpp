#include "PlaceRouting.h"
#include <cmath>
#include <stdexcept>

void routingPressureRegression() {
    auto require = [](bool ok, const char* message) {
        if (!ok) throw std::runtime_error(message);
    };
    auto& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
    device.tile_types.clear();
    device.size_width = 3; device.size_height = 1;
    device.tile_grid.resize(3);
    auto& type = device.cb_types.emplace_back();
    type.name = "PRESSURE_BOX"; type.type_id = 0;
    for (int i=0; i<8; ++i) {
        type.rememberNodeName(fpga::CB_NODE_SRC, i, "EXIT_"+std::to_string(i));
        type.rememberNodeName(fpga::CB_NODE_DST, i, "ENTRY_"+std::to_string(i));
    }
    type.rememberNodeName(fpga::CB_NODE_JOINT, 300, "LINK_A");
    type.rememberNodeName(fpga::CB_NODE_JOINT, 301, "LINK_B");
    type.rememberNodeName(fpga::CB_NODE_SRC, 0, "EXIT_ALIAS");
    for (int x=0; x<3; ++x) {
        auto& tile = device.tile_grid[x];
        tile.coord = {x,0}; tile.cb_coord = tile.coord;
        tile.cb_type = tile.cb.type = &type;
    }
    auto& busy = device.tile_grid[0];
    // Establish the attachment before Device builds its topology lookup.
    device.tile_grid[1].cb_coord = {0,0};
    busy.cb.src.jump.setBit(0);
    pnr::PlaceRouting routing;
    require(routing.pressure({0,0}) == 0, "first-run policy must be inactive");
    routing.enabled = true;
    require(std::abs(routing.pressure({0,0}) - 0.125) < 1e-12,
            "SRC usage must count the actual bank, not the mask bit width");
    busy.cb.dst.jump.setBit(0); busy.cb.dst.jump.setBit(1);
    require(routing.pressure({0,0}) == 0.25, "DST congestion was ignored");
    busy.cb.joint.jump.setBit(300);
    require(routing.pressure({0,0}) == 0.5, "high-ID JOINT bottleneck was hidden");
    // A resource-side view shares CB coordinates with the real route Tile.
    require(routing.pressure({1,0}) == 0.5, "attached CB ownership was ignored");
    require(routing.allowsMove({0,0},{2,0}), "movement to free fabric rejected");
    require(!routing.allowsMove({2,0},{0,0}), "refinement undid congestion avoidance");
    busy.cb.src.jump = {}; busy.cb.dst.jump = {}; busy.cb.joint.jump = {};
    require(routing.pressure({1,0}) == 0, "pressure retained stale routing leases");
    require(routing.allowsMove({2,0},{0,0}), "released fabric still excluded");
}
