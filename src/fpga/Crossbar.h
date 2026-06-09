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
#include <unordered_map>
#include <vector>

#include "DeviceFormat.h"
#include "debug.h"
#include "u256.h"

#define CB_MAX_NODES 256

namespace fpga {

typedef std::vector<std::vector<std::vector<std::vector<std::vector<std::string>>>>> TechMap;  // "aaa\nbbb;ccc=ddd,eee:fff" easy substitutions descriptions format

struct CBJumpNode  // this is a generic jump in mesh
{
    union {
        struct {
            uint8_t num:2;
            uint8_t length:3;
            uint8_t dir:3;  // angle from y axis
        };
        uint8_t jump;
    }__attribute__((packed));
}__attribute__((packed));

struct CBLocalNode  // this is a generic local node to a Tile
{
    uint8_t local;
}__attribute__((packed));

struct CBJointNode  // this is a joint node inside CB
{
    uint8_t joint;
}__attribute__((packed));

struct CBJumpState
{  // [dir][length*4 + num]
    union {
        uint32_t dirs[8];
        u256 jump;
    }__attribute__((packed));
}__attribute__((packed));

struct CBLocalState
{
    u256 local;
};

struct CBJointState
{
    u256 joint;
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
    uint8_t type;
    uint8_t value;

    bool operator==(const CBNodeNameKey& other) const
    {
        return type == other.type && value == other.value;
    }
};

struct CBNodeNameKeyHash
{
    std::size_t operator()(const CBNodeNameKey& key) const
    {
        return (static_cast<std::size_t>(key.type) << 8) | key.value;
    }
};

struct CBConnNameKey
{
    uint8_t from_type;
    uint8_t from_value;
    uint8_t to_type;
    uint8_t to_value;

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
        return (static_cast<std::size_t>(key.from_type) << 24)
            | (static_cast<std::size_t>(key.from_value) << 16)
            | (static_cast<std::size_t>(key.to_type) << 8)
            | key.to_value;
    }
};

struct CBType
{
    std::string name;
    CBJumpState local_src[CB_MAX_NODES];
    CBJointState local_joint[CB_MAX_NODES];
    CBLocalState local_local[CB_MAX_NODES];
    CBJointState src_joint[CB_MAX_NODES];
    CBJumpState src_dst[CB_MAX_NODES];
    CBJumpState joint_src[CB_MAX_NODES];
    CBLocalState joint_local[CB_MAX_NODES];
    CBJointState joint_joint[CB_MAX_NODES];
    CBJumpState dst_src[CB_MAX_NODES];
    CBLocalState dst_local[CB_MAX_NODES];
    CBJointState dst_joint[CB_MAX_NODES];
    u256 local_input_nodes;
    u256 local_output_nodes;

    struct NodeEnum
    {
        int base_id;   // id it starts from
        int cnt;       // nodes count
        int start_num; // start number it gets
    };

    std::map<std::string,NodeEnum> nodes_enum;
    std::unordered_map<CBNodeNameKey, std::string, CBNodeNameKeyHash> node_names;
    std::unordered_map<std::string, uint8_t> local_nodes_by_name;
    std::unordered_map<std::string, uint8_t> src_nodes_by_name;
    std::unordered_map<std::string, uint8_t> dst_nodes_by_name;
    std::unordered_map<std::string, uint8_t> joint_nodes_by_name;
    std::unordered_map<CBConnNameKey, std::vector<CBConnName>, CBConnNameKeyHash> conn_names;
    std::unordered_map<CBNodeNameKey, std::vector<uint8_t>, CBNodeNameKeyHash> outgoing_srcs;
    TechMap annotation_map;

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
    const std::vector<uint8_t>* srcNodes(CBNodeNameType from_type, int from_value) const;
    void rebuildOutgoingSrcs();

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

    int iterate(bool jump, int pos, const Coord& from, const Coord& to, int curr = 0);  // iterates all possible ways to exit Tile
    bool leaseOut(int pos, int curr, int orig_curr, int joint = -1);  // leases particular bit in exit state
    bool leaseJump(int pos, int curr, int orig_curr, int joint = -1);  // leases particular bit in exit state
    bool leaseIn(int pos, int curr, int joint = -1);  // tries to enter Tile in big recursive loop
    Coord makeJump(const Coord& src, int curr, int orig_curr);  // make jump during big recursion loop

};

static constexpr const int search_dirs[8][8] = {
                                         {0, 1, 7, 2, 6, 3, 5, 4},
                                         {5, 1, 2, 0, 3, 7, 4, 6},
                                         {7, 6, 2, 3, 1, 4, 0, 5},
                                         {6, 0, 7, 3, 4, 2, 5, 1},
                                         {2, 7, 1, 0, 4, 5, 3, 6},
                                         {7, 3, 0, 2, 1, 5, 6, 4},
                                         {5, 0, 4, 1, 3, 2, 6, 7},
                                         {0, 6, 1, 5, 2, 4, 3, 7},
                                        }; // these are reordered directions for better search like 0 -1 +1 -2 +2 ...


}
