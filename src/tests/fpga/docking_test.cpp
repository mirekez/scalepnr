#include "Device.h"
#include "Docking.h"

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

u256 bit(int index)
{
    return u256{0, 1} << index;
}

int encodedJump(int dx, int dy, int num = 0)
{
    auto encode = [](int value) {
        return value & 0xf;
    };
    return (encode(dx) << 6) | (encode(dy) << 2) | (num & 0x3);
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
    constexpr int dst = 0;
    constexpr int pin = 20;
    std::vector<int> srcs{
        encodedJump(0, -1),
        encodedJump(1, 0),
        encodedJump(0, 1),
        encodedJump(-1, 0),
    };

    cb.rememberNodeName(fpga::CB_NODE_DST, dst, "D0");
    cb.rememberNodeName(fpga::CB_NODE_LOCAL, pin, "PIN0");
    for (int src : srcs) {
        cb.rememberNodeName(fpga::CB_NODE_SRC, src, "S" + std::to_string(src));
        cb.dst_src[dst].jump |= bit(src);
        cb.src_dst[src].jump |= bit(dst);
        rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_SRC, src);
    }
    cb.dst_local[dst].local |= bit(pin);
    rememberConn(cb, fpga::CB_NODE_DST, dst, fpga::CB_NODE_LOCAL, pin);
    cb.rebuildOutgoingSrcs();
    return cb;
}

std::vector<fpga::Tile*> resetGrid(int width, int height, fpga::CBType& cb)
{
    fpga::Device& device = fpga::Device::current();
    device.tile_grid.clear();
    device.cb_types.clear();
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
    require(result.success, "dockGrounding did not find the deliberately freed corridor");
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

} // namespace

int main()
{
    try {
        for (unsigned seed = 1; seed <= 20; ++seed) {
            docking_finds_one_random_free_path(seed);
        }
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
