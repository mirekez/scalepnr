#pragma once
// This is how I see Crossbar, nodes on one side, nodes on other side and intermediate flexible joint nodes.
// Possible connection pairs are read from database for each type of tile
//
//                                                 CHAIN NODES  NODES FROM/TO
//                                                    │││││     UPPER TILE
//                                                    │││││      ││
//       NODES FROM/TO LEFT TILE              ┌───────┼┼┼┼┼──────┼┼────────────────┐      NODES FROM/TO
//    ──────────────────────────────┐         │    ┌──┼┼┼┼┼──────┼┼────────────────┼──────RIGHT TILE
//    ──────────────────────────────┼─────────┼────┼──┼┼┼┼┼──┐   ││    ┌───────────┼──────
//                                  │    ┌────┼────┼──┼┼┼┼┼──┼───┘│    │    ┌─────┐│
//                                  │    │    │    │  │││││  │    │    │    │     ││
//                  ┌───────────────┼────┼────┼────┼──▲▲▲▲▲──▲────▲────▲────▲──┐  ││
//                  │             ┌─▼─┐┌─▼─┐┌─▼─┐┌─▼─┐│││││┌─┴─┐┌─┴─┐┌─┴─┐┌─┴─┐│  ││
//                  │             │MUX││MUX││MUX││MUX│││││││MUX││MUX││MUX││MUX││  ││ NODES
//                  │             └┬┬┬┘└┬┬┬┘└┬┬┬┘└┬┬─┘│││││└▲▲▲┘└─▲▲┘└▲▲▲┘└▲▲▲┘│  ││ FROM/TO
//                  │              │││  │││  │││  │└──┤│││└─┼┼┼─┐ ││  │││┌─┘││ │  ││ BOTTOM
//                  │              │││  │││  ││└──┼───┼┘│└──┼┼┼┐│ ││  ││││┌─┘│ │  ││ TILE
//                  │ Tile         │││  │││  │└───┼───┼─┼───┘││││ ││  ││└┼┼─┐│ │  ││
//                  │              │││  │└┼──┼┬───┼───┼─┼────┘│││ ││  ││ ││ ││ │  ││
//                  │              │└┼──┼─┼─┐││ ┌─┼───┼─┴─────┼┼┼─┼┴──┼┼─┼┼┐││ │  ││
//                  │ Crossbar     │ │  │ │ │││ │ │ ┌─┼──┬──┐ │││ │ ┌─┘│ │││││ │  ││
//                  │              │ └──┼┐│┌▼▼▼─▼─▼─▼┐│  │ ┌┴─┴┴┴─┴─┴┐ │ │││││ │
//                  │              │┌───┘│││MUX JOINT││  │ │MUX JOINT│ │ │││││ │
//                  │              ││    ││└┬┬┬┬┬┬┬─┬┘│  │ └▲▲▲──▲▲─▲┘ │ │││││ │
//                  │              ││┌───┼┼─┘││││││┌┼─┴─┐│  │││  ││┌┼──┼─┘││││ │
//                  │              │││  ┌┼┼──┘││││││└───┼┼──┘││  │││└─┐│  ││││ │
//                  │              │││  │││   ││││└┼────┼┼┬──┘│  │││  ││  ││││ │
//                  │              │││  ││├──┐│││└─┼───┐│││   │  │││  ││┌─┘│││ │
//                  │              │││  │││  │││└──┼──┐││││   │  │││  │││  │││ │
//                  │              │││  │││  ││└──┐│┌─┼┼┼┼┼──┐│  │││  │││  │││ │
//                  │              │││  │││  ││┌──┼┼┼─┼┼┼┼┼─┐││  │││  │││  │││ │
//                  │             ┌▼▼▼┐┌▼▼▼┐┌▼▼▼┐┌▼▼▼┐│││││┌┴┴┴┐┌┴┴┴┐┌┴┴┴┐┌┴┴┴┐│
//                  │             │MUX││MUX││MUX││MUX│││││││MUX││MUX││MUX││MUX││
//                  │             └─┬─┘└─┬─┘└─┬─┘└─┬─┘│││││└─▲─┘└─▲─┘└─▲─┘└─▲─┘│
//                  └───────────────▼────▼────▼────▼-─▼▼▼▼▼──┼────┼────┼────┼──┘
//
//                                   NODES TO LOGIC  CTRL NODES  NODES FROM LOGIC
//
//                                                  ELEMENTS

#include <string>
#include <stdint.h>
#include <map>
#include <set>
#include <unordered_map>
#include <vector>

#include "DeviceFormat.h"
#include "Types.h"
#include "debug.h"
#include "NodeMask.h"

#define CB_MAX_NODES 2048
#define CB_INVALID_TYPE_ID 0xffff

namespace fpga {

typedef std::vector<std::vector<std::vector<std::vector<std::vector<std::string>>>>> TechMap;  // "aaa\nbbb;ccc=ddd,eee:fff" easy substitutions descriptions format

struct CBJumpNode  // this is a jump in mesh
{
    union {
        struct {
            uint16_t num:3;
            uint16_t delta_y:4;
            uint16_t delta_x:4;
        };
        uint16_t jump;
    }__attribute__((packed));
}__attribute__((packed));

struct CBLocalNode  // this is a generic local node to a Tile
{
    uint16_t local;
}__attribute__((packed));

struct CBJointNode  // this is a joint node inside CB
{
    uint16_t joint;
}__attribute__((packed));

struct CBJumpState
{  // [(delta_x << 7) + (delta_y << 3) + num]
    NodeMask jump;
};

struct CBLocalState
{
    NodeMask local;
};

struct CBJointState
{
    NodeMask joint;
};

enum CBNodeNameType : uint8_t
{
    CB_NODE_LOCAL,
    CB_NODE_JOINT,
    CB_NODE_JUMP,
    CB_NODE_SRC,
    CB_NODE_DST,
};

struct CBNodeNameKey
{
    uint16_t type;
    uint16_t value;

    bool operator==(const CBNodeNameKey& other) const
    {
        return type == other.type && value == other.value;
    }
};

struct CBNodeNameKeyHash
{
    std::size_t operator()(const CBNodeNameKey& key) const
    {
        return (static_cast<std::size_t>(key.type) << 16) | key.value;
    }
};

struct CBConnNameKey
{
    uint16_t from_type;
    uint16_t from_value;
    uint16_t to_type;
    uint16_t to_value;

    bool operator==(const CBConnNameKey& other) const
    {
        return from_type == other.from_type && from_value == other.from_value
            && to_type == other.to_type && to_value == other.to_value;
    }
};

struct CBConnName
{
    std::string from;
    std::string to;
};

struct CBConnNameKeyHash
{
    std::size_t operator()(const CBConnNameKey& key) const
    {
        return (static_cast<std::size_t>(key.from_type) << 48)
            | (static_cast<std::size_t>(key.from_value) << 32)
            | (static_cast<std::size_t>(key.to_type) << 16)
            | key.to_value;
    }
};

struct CBType
{
    std::string name;
    uint16_t type_id = CB_INVALID_TYPE_ID;
    uint16_t base_type_id = CB_INVALID_TYPE_ID;

    struct ResolvedJump
    {
        Coord delta;
        uint16_t target_cb_type_id = CB_INVALID_TYPE_ID;
        CBJumpState dsts;
        bool target_tile_coord = false;
    };

    struct ResolvedJumpTable
    {
        std::unordered_map<uint16_t, std::vector<ResolvedJump>> values;

        std::vector<ResolvedJump>& operator[](int src)
        {
            return values[static_cast<uint16_t>(src)];
        }

        const std::vector<ResolvedJump>& operator[](int src) const
        {
            static const std::vector<ResolvedJump> empty;
            auto it = values.find(static_cast<uint16_t>(src));
            return it == values.end() ? empty : it->second;
        }

        void clear()
        {
            values.clear();
        }
    };

    template<typename State>
    struct StateTable
    {
        std::unordered_map<uint16_t, State> values;

        State& operator[](int node)
        {
            return values[static_cast<uint16_t>(node)];
        }

        const State& operator[](int node) const
        {
            static const State empty{};
            auto it = values.find(static_cast<uint16_t>(node));
            return it == values.end() ? empty : it->second;
        }

        void clear()
        {
            values.clear();
        }
    };

    StateTable<CBJumpState> local_src;
    StateTable<CBJointState> local_joint;
    StateTable<CBLocalState> local_local;
    StateTable<CBJointState> src_joint;
    ResolvedJumpTable dst_by_src;  // maps a selected SRC bit to destination bits and target delta
    std::unordered_map<uint16_t, std::vector<Coord>> src_priority_deltas;  // numeric routing-priority deltas loaded for this SRC bit
    StateTable<CBJumpState> priority_srcs_by_delta;  // loaded numeric delta -> SRC bits used for route priority
    StateTable<CBJumpState> joint_src;
    StateTable<CBLocalState> joint_local;
    StateTable<CBJointState> joint_joint;
    StateTable<CBJumpState> dst_src;
    StateTable<CBLocalState> dst_local;
    StateTable<CBJointState> dst_joint;
    StateTable<CBJumpState> joint_reachable_srcs;
    StateTable<CBJointState> src_reachable_joints;
    StateTable<CBJointState> local_reachable_joints;
    StateTable<CBJumpState> dsts_reaching_src;
    StateTable<CBJumpState> dsts_reaching_local;
    NodeMask local_input_nodes;
    NodeMask local_output_nodes;
    NodeMask valid_dst_nodes;

    struct NodeEnum
    {
        int base_id;   // id it starts from
        int cnt;       // nodes count
        int start_num; // start number it gets
    };

    std::map<std::string,NodeEnum> nodes_enum;
    std::unordered_map<CBNodeNameKey, std::string, CBNodeNameKeyHash> node_names;
    std::unordered_map<std::string, uint16_t> local_nodes_by_name;
    std::unordered_map<std::string, uint16_t> src_nodes_by_name;
    std::unordered_map<std::string, uint16_t> dst_nodes_by_name;
    std::unordered_map<std::string, uint16_t> joint_nodes_by_name;
    std::unordered_map<CBConnNameKey, std::vector<CBConnName>, CBConnNameKeyHash> conn_names;
    std::unordered_map<CBNodeNameKey, std::vector<uint16_t>, CBNodeNameKeyHash> outgoing_srcs;
    TechMap annotation_map;
    bool derived_masks_valid = false;

    void preParseNode(std::string name, TechMap& map, bool finish);
    int /*0-3*/ parseNode(std::string name, TechMap& map,
                         CBLocalNode& local_node, CBJumpNode& src_node, CBJumpNode& dst_node, CBJointNode& joint_node,
                         CBLocalState& local_state, CBJumpState& src_state, CBJumpState& dst_state, CBJointState& joint_state);

    void loadFromSpec(const CBTypeSpec& spec, TechMap& map);
    int localNodeNum(const std::string& name) const;
    void rememberNodeName(CBNodeNameType type, int value, const std::string& name);
    const std::string* nodeName(CBNodeNameType type, int value) const;
    int nodeNum(CBNodeNameType type, const std::string& name) const;
    void rememberConnName(CBNodeNameType from_type, int from_value,
                          CBNodeNameType to_type, int to_value,
                          const std::string& from_name, const std::string& to_name);
    const CBConnName* connName(CBNodeNameType from_type, int from_value,
                               CBNodeNameType to_type, int to_value) const;
    const std::vector<CBConnName>* connNames(CBNodeNameType from_type, int from_value,
                                             CBNodeNameType to_type, int to_value) const;
    const std::vector<uint16_t>* srcNodes(CBNodeNameType from_type, int from_value) const;
    void rebuildPrioritySrcsByDelta();
    void rebuildOutgoingSrcs();
    void ensureDerivedMasks();
    NodeMask dstMaskForSrc(int src) const;
    bool sameDstBySrc(const CBType& other) const;
    bool sameRoutingSubtype(const CBType& other) const;

    bool canOut(int local, int src, int orig_curr, int& joint);  // can exit source Tile
    bool canJump(int dst, int src, int orig_curr, int& joint);  // can jump to another Tile
    bool canIn(int dst, int local, int& joint);  // can enter destination Tile
};

struct CBState
{
    CBJumpState src;
    CBLocalState local;
    CBJumpState joint;
    CBJumpState dst;
    CBJumpState src_deadend;
    CBType* type;

    int iterate(bool jump, int pos, const Coord& from, const Coord& to, int curr = 0, bool ignore_deadend = false);  // iterates possible source bits for this Tile node
    bool leaseOut(int pos, int curr, int orig_curr, int joint = -1);  // leases particular bit in exit state
    bool leaseJump(int pos, int curr, int orig_curr, int joint = -1);  // leases particular bit in exit state
    bool leaseIn(int pos, int curr, int joint = -1);  // tries to enter Tile in big recursive loop
    Coord makeJump(const Coord& src, int curr, int orig_curr);  // make jump during big recursion loop

};

}
