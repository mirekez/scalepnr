#pragma once

#include "Inst.h"
#include "Net.h"
#include "debug.h"
#include "referable.h"
#include "TileType.h"
#include "Crossbar.h"

#include <array>
#include <optional>
#include <limits>
#include <string>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fpga {

// Maximum percentage of each resource type used beside a sparse routing crossbar.
inline constexpr unsigned SPARSE_ROUTING_TILE_LOAD_MAX = 50;
static_assert(SPARSE_ROUTING_TILE_LOAD_MAX <= 100);

struct Tile
{
    struct RoutedBinding
    {
        rtl::Net* net = nullptr;
        uint64_t route_id = 0;
    };
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
    CBType* cb_type = nullptr;
    bool sparse = false;  // load-time neighborhood classification of the owning crossbar
    NodeMask incoming_dst_nodes;  // destination nodes reached by physical jumps into this route tile
    TileType* tile_type = nullptr;
    TilePinState pin_state;
    std::string full_name;
    Coord cb_coord{-1, -1};
    std::string cb_full_name;
    std::vector<std::string> sites;
    std::vector<std::string> site_types;
    std::vector<Tile*> attached_resource_tiles;  // resource tiles sharing this tile's physical crossbar
    std::unordered_map<int, rtl::Conn*> input_local_reservations;
    std::vector<std::pair<rtl::Conn*, NodeMask>> input_joint_reservations;
    bool input_joint_reservations_initialized = false;
    std::unordered_map<int, NodeMask> mandatory_input_joints;
    std::vector<Ref<rtl::Net>> routedNets;
    std::vector<RoutedBinding> routed_bindings;
    bool routed_bindings_authoritative = false;
    mutable std::unordered_set<rtl::Net*> routed_net_index;
    mutable size_t routed_net_index_size = std::numeric_limits<size_t>::max();
    mutable const Ref<rtl::Net>* routed_net_index_data = nullptr;
    std::array<uint16_t, ELEMENT_TYPE_COUNT> elements_pos{};
    std::array<uint16_t, ELEMENT_TYPE_COUNT> elements_free{};
    std::array<std::array<uint16_t, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT> elements_left{};
    std::array<std::array<uint16_t, ELEMENT_BITMAP_BITS>, ELEMENT_TYPE_COUNT> elements_right{};
    bool elements_initialized = false;

    // Maintain constant-time membership beside the owning routed-net refs.
    bool hasRoutedNet(rtl::Net* net) const;
    void addRoutedNet(rtl::Net* net);
    void removeRoutedNet(rtl::Net* net);
    void clearRoutedNets();
    // Index physical bindings that touch this tile without scanning every
    // fanout binding stored by a shared logical net.
    void addRoutedBinding(rtl::Net* net, uint64_t route_id);
    void removeRoutedBindings(rtl::Net* net);

    const std::string makeName() const
    {
        return std::format("TILE_X{}Y{}", name.x, name.y);
    }

    void assign(rtl::Inst* inst);
    void invalidatePlacementCaches();
    // Release one placed element and rebuild compact occupancy on next use.
    bool unassign(rtl::Inst* inst);
    bool hasFreeElement(ElementType type);
    // Apply the sparse occupancy quota without removing any physical position bits.
    unsigned packingCapacity(unsigned positions) const;
    unsigned freeElementCount(ElementType type);
    bool hasOccupiedElementNeighbors(rtl::Inst* inst);
    // Return the exact position tryAdd would select without assigning the cell
    // or consuming any Element resource.
    int peekAdd(rtl::Inst* inst, bool enforce_route_capacity = true);
    int tryAdd(rtl::Inst* inst, bool enforce_route_capacity = true);
    int tryAddAt(rtl::Inst* inst, int pos,
                 bool enforce_route_capacity = true);
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
std::optional<ElementType> elementTypeForInst(const rtl::Inst& inst);

// Report whether an occupied input local is already reserved by this exact
// physical driver, allowing compatible packed sinks to share the endpoint.
bool inputLocalReservedByDriver(Tile& route_tile, int local,
                                rtl::Conn* driver);

// Insert tile-local passthrough resources when a fabric route starts or ends
// inside a packed chain; the caller owns atomic route unlease and retargeting.
bool preparePassthroughRouteEndpoints(rtl::Inst*& from, std::string& from_port,
                                      rtl::Inst*& to, std::string& to_port,
                                      rtl::Net*& net, bool allow_new_source_passthrough = true);

// Repack an existing generated endpoint beside its newly placed connected cell.
bool rehomeGeneratedPassthrough(rtl::Inst& inst, std::string* fail_reason = nullptr);

// Return joints that other packed input endpoints must retain on this route tile.
NodeMask packedInputJointReservations(Tile& route_tile, rtl::Inst* except_inst = nullptr,
                                      const std::string& except_port = {});

}
