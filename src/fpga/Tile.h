#pragma once

#include "Inst.h"
#include "Net.h"
#include "debug.h"
#include "referable.h"
#include "TileType.h"
#include "Crossbar.h"

#include <array>
#include <string>
#include <vector>

namespace fpga {

struct Tile
{
    // must have
    Coord coord;
    Coord name;
    enum {
      TILE_NULL,
      TILE_IO,
      TILE_LUTS,
      TILE_LUTS_RAM,
      TILE_BRAM,
      TILE_LRAM,
      TILE_DSP,
    } type = TILE_NULL;

    int luts6cnt = 0;
    int luts5cnt = 0;
    int luts1cnt = 0;
    int regs_cnt = 0;
    int carry = 0;
    int mux = 0;
//    int memcnt = 4;
//    int memtype = 6;
    // optional
    int clk_a = -1;
    int clk_b = -1;
    int memctl_a = -1;
    int memctl_b = -1;

    CBState cb;
    CBType* cb_type;
    TileType* tile_type = nullptr;
    TilePinState pin_state;
    std::string full_name;
    std::vector<std::string> sites;
    std::vector<std::string> site_types;
    std::vector<Ref<rtl::Net>> routedNets;
    std::array<uint16_t, ELEMENT_TYPE_COUNT> elements_pos{};
    std::array<uint16_t, ELEMENT_TYPE_COUNT> elements_free{};
    std::array<std::array<uint16_t, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT> elements_left{};
    std::array<std::array<uint16_t, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT> elements_right{};
    bool elements_initialized = false;

    const std::string makeName() const
    {
        return std::format("TILE_X{}Y{}", name.x, name.y);
    }

    void assign(rtl::Inst* inst);
    int tryAdd(rtl::Inst* inst);
    int getNodeNum(std::string type, std::string port, int pos);
    // Resolve the resource-side endpoint for a selected local tile-pin node.
    int getResourceNodeNum(const std::string& type, const std::string& port, int pos, TilePinNameType dir, int local) const;
    u256 getPinNodes(const std::string& type, const std::string& port, int pos) const;
    u256 getOutputPinNodes(const std::string& type, const std::string& port, int pos) const;
    bool isPinNodeLeased(int local) const;
    bool leasePinNode(int local);

};

// Insert tile-local passthrough resources when a fabric route starts or ends
// inside a packed element chain instead of at the chain edge.
bool preparePassthroughRouteEndpoints(rtl::Inst*& from, std::string& from_port,
                                      rtl::Inst*& to, std::string& to_port,
                                      rtl::Net*& net, bool allow_new_source_passthrough = true);

}
