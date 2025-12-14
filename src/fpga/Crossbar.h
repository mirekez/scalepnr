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
#include <vector>

#include "DeviceFormat.h"
#include "debug.h"
#include "u256.h"

#define CB_MAX_NODES 256

namespace fpga {

typedef std::vector<std::vector<std::vector<std::vector<std::vector<std::string>>>>> TechMap;  // "aaa\nbbb;ccc=ddd,eee:fff" easy descriptions format

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

struct CBType
{
    std::string name;
    CBJumpState local_src[CB_MAX_NODES];
    CBJointState local_joint[CB_MAX_NODES];
    CBJointState src_joint[CB_MAX_NODES];
    CBJumpState src_dst[CB_MAX_NODES];
    CBJumpState joint_src[CB_MAX_NODES];
    CBLocalState joint_local[CB_MAX_NODES];
    CBJumpState dst_src[CB_MAX_NODES];
    CBLocalState dst_local[CB_MAX_NODES];
    CBJointState dst_joint[CB_MAX_NODES];

    struct NodeEnum
    {
        int base_id;   // id it starts from
        int cnt;       // nodes count
        int start_num; // start number it gets
    };

    std::map<std::string,NodeEnum> modes_enum;

    void preParseNode(std::string name, bool finish);
    int /*0-3*/ parseNode(std::string name, TechMap& map,
                         CBLocalNode& local_node, CBJumpNode& src_node, CBJumpNode& dst_node, CBJointNode& joint_node,
                         CBLocalState& local_state, CBJumpState& src_state, CBJumpState& dst_state, CBJointState& joint_state);

    void loadFromSpec(const CBTypeSpec& spec, TechMap& map);

    bool canOut(int local, int src, int& joint);  // can exit source Tile
    bool canJump(int dst, int src, int& joint);  // can jump to another Tile
    bool canIn(int dst, int local, int& joint);  // can enter destination Tile
};

struct CBState
{
    CBJumpState src;
    CBLocalState local;
    CBJumpState joint;
    CBJumpState dst;
    CBType* type;

    static constexpr const int dirs[8] = {0, 1, 7, 2, 6, 3, 5, 4};  // these are reordered directions for better search like 0 -1 +1 -2 +2 ...

    int iterateOut(int pos, const Coord& from, const Coord& to, int curr = 0);  // iterates all possible ways to exit Tile
    void leaseOut(int pos, int curr = 0);  // blocks particular bit in exit state
    bool tryIn(int dst, int local);  // tries to enter Tile in big recursive loop
    Coord makeJump(const Coord& src, int curr);  // make jump during big recursion loop

};

}
