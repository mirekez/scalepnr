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
    NodeMask incoming_dst_nodes;  // destination nodes reached by physical jumps into this route tile
    TileType* tile_type = nullptr;
    TilePinState pin_state;
    std::string full_name;
    Coord cb_coord{-1, -1};
    std::string cb_full_name;
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
    // Release one placed element and rebuild compact occupancy on next use.
    bool unassign(rtl::Inst* inst);
    int tryAdd(rtl::Inst* inst);
    int tryAddAt(rtl::Inst* inst, int pos);
    std::vector<int> candidatePositions(rtl::Inst* inst);
    int getNodeNum(std::string type, std::string port, int pos);
    // Resolve the resource-side endpoint for a selected local tile-pin node.
    int getResourceNodeNum(const std::string& type, const std::string& port, int pos, TilePinNameType dir, int local) const;
    NodeMask getPinNodes(const std::string& type, const std::string& port, int pos) const;
    NodeMask getOutputPinNodes(const std::string& type, const std::string& port, int pos) const;
    // Resolve endpoint local nodes that belong to an explicit adjacent route crossbar type.
    NodeMask getPinNodesForRouteType(const std::string& type, const std::string& port, int pos, TilePinNameType dir,
                                 const std::string& route_type) const;
    NodeMask getPinNodesForRouteType(const std::string& type, const std::string& port, int pos, TilePinNameType dir,
                                 const std::string& route_type, Coord route_delta) const;
    bool isPinNodeLeased(int local) const;
    bool leasePinNode(int local);

};

// Return whether an instance belongs to one of the abstract placeable element
// columns represented by TileType::elements.
bool isPlaceableElement(const rtl::Inst& inst);

// Insert tile-local passthrough resources when a fabric route starts or ends
// inside a packed element chain instead of at the chain edge.
bool preparePassthroughRouteEndpoints(rtl::Inst*& from, std::string& from_port,
                                      rtl::Inst*& to, std::string& to_port,
                                      rtl::Net*& net, bool allow_new_source_passthrough = true);

// Repack an existing generated endpoint beside its newly placed connected cell.
bool rehomeGeneratedPassthrough(rtl::Inst& inst, std::string* fail_reason = nullptr);

// Return joints that other packed input endpoints must retain on this route tile.
NodeMask packedInputJointReservations(Tile& route_tile, rtl::Inst* except_inst = nullptr,
                                      const std::string& except_port = {});

}
