#include "TileType.h"

#include <iostream>
#include <random>
#include <stdexcept>

namespace {
void require(bool condition, const char* message)
{
    if (!condition) throw std::runtime_error(message);
}

void random_queries_match_reference()
{
    std::mt19937 random(9137);
    fpga::TilePinMap maps[2];
    for (auto& map : maps) {
        for (auto type : {fpga::TILE_PIN_INPUT, fpga::TILE_PIN_OUTPUT}) {
            for (int site = 0; site < 2; ++site) {
                for (int pin = 0; pin < 12; ++pin) {
                    int resource = site * 256 + pin;
                    map.rememberResourcePinName(type, resource, "PIN" + std::to_string(pin));
                    map.rememberResourcePinName(type, resource, "ALIAS" + std::to_string(pin % 3));
                    auto& nodes = type == fpga::TILE_PIN_INPUT ? map.input_nodes : map.output_nodes;
                    for (int i = 0; i < 3; ++i) {
                        int local = random() % 4096;
                        nodes[resource] |= NodeMask{0, 1} << local;
                        if (i != 0) map.rememberEndpointRouteRef(type, resource, local,
                            i == 1 ? "FABRIC_A" : "FABRIC_B", {i - 1, site});
                    }
                }
            }
        }
        // Primary-only names and alias precedence exercise both reference lookup paths.
        map.resource_pin_names[{fpga::TILE_PIN_INPUT, 20}] = "PRIMARY";
        map.input_nodes[20] = NodeMask{0, 1} << 4095;
        map.resource_pin_names[{fpga::TILE_PIN_INPUT, 21}] = "ALIAS0";
        map.input_nodes[21] = NodeMask{0, 1} << 4094;
    }
    const std::string pins[] = {"PIN0", "PIN11", "ALIAS0", "ALIAS2", "PRIMARY", "MISSING", ""};
    const std::string routes[] = {"", "FABRIC_A", "FABRIC_B", "MISSING"};
    const fpga::Coord deltas[] = {{0, 0}, {1, 0}, {0, 1}, {-1, 0}};
    fpga::PinLookupCache cache;
    for (int repeat = 0; repeat < 2; ++repeat) {
        for (const auto& map : maps) {
            for (auto type : {fpga::TILE_PIN_INPUT, fpga::TILE_PIN_OUTPUT}) {
                for (const auto& pin : pins) for (int site : {-1, 0, 1, 2}) {
                    for (const auto& route : routes) for (bool strict : {false, true}) {
                        for (int d = -1; d < 4; ++d) {
                            const auto* delta = d < 0 ? nullptr : &deltas[d];
                            // Every cached mask must exactly preserve all query filters.
                            require(map.getNodesForPin(type, pin, site, route, strict, delta)
                                == map.getNodesForPinUncached(type, pin, site, route, strict, delta),
                                "cached pin mask differs from reference");
                        }
                    }
                }
            }
        }
    }
    require(cache.hits == cache.misses && cache.hits > 0, "second query pass was not cached");
    require(cache.size() <= 16384, "cache exceeded its memory bound");
}

void scope_and_capacity_are_safe()
{
    fpga::TilePinMap map;
    map.rememberResourcePinName(fpga::TILE_PIN_INPUT, 0, "P");
    map.input_nodes[0] = NodeMask{0, 1} << 10;
    require(fpga::PinLookupCache::active() == nullptr, "previous test leaked its cache");
    {
        fpga::PinLookupCache outer(2);
        for (int pin = 0; pin < 10; ++pin) {
            auto name = pin == 0 ? "P" : std::to_string(pin);
            require(map.getNodesForPin(fpga::TILE_PIN_INPUT, name)
                == map.getNodesForPinUncached(fpga::TILE_PIN_INPUT, name),
                "full cache changed lookup result");
        }
        require(outer.size() == 2, "cache capacity was not enforced");
        {
            fpga::PinLookupCache inner(0);
            require(map.getNodesForPin(fpga::TILE_PIN_INPUT, "P") == map.input_nodes[0],
                "zero-capacity cache changed result");
            require(inner.size() == 0, "zero-capacity cache retained an entry");
        }
        require(fpga::PinLookupCache::active() == &outer, "nested scope lost caller cache");
        map.getNodesForPin(fpga::TILE_PIN_INPUT, "1");
        require(outer.hits == 1, "empty mask was not cached");
    }
    // Model edits between operations cannot leave stale masks in the next operation.
    map.input_nodes[0] = NodeMask{0, 1} << 22;
    {
        fpga::PinLookupCache next;
        require(map.getNodesForPin(fpga::TILE_PIN_INPUT, "P") == map.input_nodes[0],
            "cache survived its owning operation");
    }
    require(fpga::PinLookupCache::active() == nullptr, "cache scope leaked");
}
}

int main()
{
    try {
        random_queries_match_reference();
        scope_and_capacity_are_safe();
        std::cout << "pin lookup cache tests passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
