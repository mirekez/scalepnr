#include "Tech.h"
#include "Device.h"
#include "Wire.h"

#include "tcl_pnr.h"

#include <set>
#include <sstream>

using namespace technology;
using namespace fpga;

namespace {

std::string tileNodeName(const Tile* tile, CBNodeNameType type, int value)
{
    if (!tile || !tile->cb_type || value < 0) {
        return {};
    }

    const std::string* node_name = tile->cb_type->nodeName(type, value);
    if (!node_name) {
        return {};
    }

    std::stringstream ss;
    ss << tile->cb_type->name << "_X" << tile->name.x << "Y" << tile->name.y << "." << *node_name;
    return ss.str();
}

std::string tileNodeName(const Tile* tile, const std::string& node_name)
{
    if (!tile || !tile->cb_type || node_name.empty()) {
        return {};
    }

    std::stringstream ss;
    ss << tile->cb_type->name << "_X" << tile->name.x << "Y" << tile->name.y << "." << node_name;
    return ss.str();
}

std::string tileName(const Tile* tile)
{
    if (!tile || !tile->cb_type) {
        return {};
    }
    std::stringstream ss;
    ss << tile->cb_type->name << "_X" << tile->name.x << "Y" << tile->name.y;
    return ss.str();
}

std::string resourceTileName(const Tile* tile)
{
    if (!tile || !tile->tile_type) {
        return {};
    }
    std::stringstream ss;
    ss << tile->tile_type->name << "_X" << tile->name.x << "Y" << tile->name.y;
    return ss.str();
}

std::string nodeNameOnly(const Tile* tile, CBNodeNameType type, int value)
{
    if (!tile || !tile->cb_type || value < 0) {
        return {};
    }
    const std::string* node_name = tile->cb_type->nodeName(type, value);
    return node_name ? *node_name : std::string{};
}

std::string tileConnectionNodeName(const Tile* tile, const std::string& dst, const std::string& src)
{
    std::string name = tileName(tile);
    if (name.empty() || dst.empty() || src.empty()) {
        return {};
    }
    std::stringstream ss;
    ss << name << "." << dst << "." << src;
    return ss.str();
}

std::string tileResourceNodeName(const Tile* tile, TilePinNameType type, int local, int resource_node)
{
    if (!tile || !tile->tile_type || local < 0) {
        return {};
    }
    const std::string* resource_wire = resource_node >= 0
        ? tile->tile_type->pin_map.localResourceName(type, resource_node, local)
        : nullptr;
    const std::string* local_wire = resource_node >= 0
        ? tile->tile_type->pin_map.localWireName(type, resource_node, local)
        : nullptr;
    if (!resource_wire && !local_wire) {
        resource_wire = tile->tile_type->pin_map.localResourceName(type, local);
        local_wire = tile->tile_type->pin_map.localWireName(type, local);
    }
    std::string tile_name = resourceTileName(tile);
    if (tile_name.empty() || !resource_wire || !local_wire) {
        return {};
    }
    std::stringstream ss;
    if (type == TILE_PIN_INPUT) {
        ss << tile_name << "." << *resource_wire << "." << *local_wire;
    }
    else {
        ss << tile_name << "." << *local_wire << "." << *resource_wire;
    }
    return ss.str();
}

std::string nodeText(const std::string& name, const std::string& detail)
{
    std::stringstream ss;
    ss << (name.empty() ? "<unnamed>" : name) << "(" << detail << ")";
    return ss.str();
}

std::string fallbackTileNodeName(const Tile* tile, const std::string& kind, int value)
{
    std::string name = tileName(tile);
    if (name.empty() || value < 0) {
        return {};
    }
    std::stringstream ss;
    ss << name << "." << kind << value;
    return ss.str();
}

std::string localDetail(int local)
{
    std::stringstream ss;
    ss << "node=" << local;
    return ss.str();
}

std::string jointDetail(int joint)
{
    std::stringstream ss;
    ss << "node=" << joint;
    return ss.str();
}

std::string jumpDetail(int jump)
{
    int angle = jump / 32;
    int path = jump % 32;
    int length = path / 4;
    int num = path % 4;
    std::stringstream ss;
    ss << "node=" << jump << ",angle=" << angle << ",length=" << length << ",num=" << num;
    return ss.str();
}

void appendNode(std::stringstream& ss, const std::string& name, const std::string& detail)
{
    if (name.empty()) {
        return;
    }
    if (ss.tellp() > 0) {
        ss << " -> ";
    }
    ss << nodeText(name, detail);
}

std::string bestFromNodeNameOnly(const Tile* tile, const Wire& wire)
{
    if (wire.jump >= 0) {
        std::string local_name = nodeNameOnly(tile, CB_NODE_LOCAL, wire.local);
        if (!local_name.empty()) {
            return local_name;
        }
        return nodeNameOnly(tile, CB_NODE_DST, wire.local);
    }

    std::string local_name = nodeNameOnly(tile, CB_NODE_LOCAL, wire.local);
    if (!local_name.empty()) {
        return local_name;
    }
    return nodeNameOnly(tile, CB_NODE_DST, wire.local);
}

std::string bestFromNodeName(const Tile* tile, const Wire& wire)
{
    if (wire.jump >= 0) {
        std::string local_name = tileNodeName(tile, CB_NODE_LOCAL, wire.local);
        if (!local_name.empty()) {
            return local_name;
        }
        return tileNodeName(tile, CB_NODE_DST, wire.local);
    }

    std::string local_name = tileNodeName(tile, CB_NODE_LOCAL, wire.local);
    if (!local_name.empty()) {
        return local_name;
    }
    return tileNodeName(tile, CB_NODE_DST, wire.local);
}

std::string tilePinResourceNodeName(const Tile* tile, int local, int resource_node, TilePinNameType& found_type)
{
    std::string input_node = tileResourceNodeName(tile, TILE_PIN_INPUT, local, resource_node);
    if (!input_node.empty()) {
        found_type = TILE_PIN_INPUT;
        return input_node;
    }
    found_type = TILE_PIN_OUTPUT;
    return tileResourceNodeName(tile, TILE_PIN_OUTPUT, local, resource_node);
}

std::string formatFragment(const Wire& wire)
{
    std::stringstream ss;
    Device& device = Device::current();
    const Tile* from_tile = device.getTile(wire.from.x, wire.from.y);
    const Tile* to_tile = device.getTile(wire.to.x, wire.to.y);
    const Tile* resource_tile = wire.resource.x >= 0 && wire.resource.y >= 0
        ? device.getTile(wire.resource.x, wire.resource.y)
        : from_tile;

    if (wire.type == Wire::WIRE_TILE_PIN) {
        std::string local_name = tileNodeName(from_tile, CB_NODE_LOCAL, wire.local);
        if (local_name.empty()) {
            local_name = fallbackTileNodeName(from_tile, "LOCAL", wire.local);
        }
        appendNode(ss, local_name, localDetail(wire.local));

        TilePinNameType resource_type = TILE_PIN_INPUT;
        std::string resource_node = tilePinResourceNodeName(resource_tile, wire.local, wire.resource_node, resource_type);
        const std::string* pin = nullptr;
        if (resource_tile && resource_tile->tile_type) {
            pin = wire.resource_node >= 0
                ? resource_tile->tile_type->pin_map.localPinName(resource_type, wire.resource_node, wire.local)
                : nullptr;
            if (!pin) {
                pin = resource_tile->tile_type->pin_map.localPinName(resource_type, wire.local);
            }
        }
        std::stringstream detail;
        detail << localDetail(wire.local);
        if (pin) {
            detail << ",pin=" << *pin;
        }
        else if (!wire.port.empty()) {
            detail << ",port=" << wire.port;
        }
        if (wire.pos >= 0) {
            detail << ",pos=" << wire.pos;
        }
        if (!resource_node.empty()) {
            appendNode(ss, resource_node, detail.str());
        }
    }
    else if (wire.jump >= 0) {
        std::string from_name = bestFromNodeName(from_tile, wire);
        if (from_name.empty()) {
            from_name = fallbackTileNodeName(from_tile, "LOCAL", wire.local);
        }
        std::string jump_name = tileNodeName(from_tile, CB_NODE_SRC, wire.jump);
        if (!wire.src_wire_name.empty()) {
            jump_name = tileNodeName(from_tile, wire.src_wire_name);
        }
        if (jump_name.empty()) {
            jump_name = tileNodeName(from_tile, CB_NODE_JUMP, wire.jump);
        }
        if (jump_name.empty()) {
            jump_name = fallbackTileNodeName(from_tile, "JUMP", wire.jump);
        }
        std::string dst_name = tileNodeName(to_tile, CB_NODE_DST, wire.jump);
        if (!wire.dst_wire_name.empty()) {
            dst_name = tileNodeName(to_tile, wire.dst_wire_name);
        }
        if (dst_name.empty()) {
            dst_name = fallbackTileNodeName(to_tile, "JUMP", wire.jump);
        }
        std::string from_node = bestFromNodeNameOnly(from_tile, wire);
        std::string jump_node = nodeNameOnly(from_tile, CB_NODE_SRC, wire.jump);
        if (!wire.src_wire_name.empty()) {
            jump_node = wire.src_wire_name;
        }
        if (jump_node.empty()) {
            jump_node = nodeNameOnly(from_tile, CB_NODE_JUMP, wire.jump);
        }
        appendNode(ss, from_name, wire.local >= 0 && nodeNameOnly(from_tile, CB_NODE_LOCAL, wire.local).empty() ? jumpDetail(wire.local) : localDetail(wire.local));
        if (wire.joint >= 0) {
            std::string joint_name = tileNodeName(from_tile, CB_NODE_JOINT, wire.joint);
            if (joint_name.empty()) {
                joint_name = fallbackTileNodeName(from_tile, "JOINT", wire.joint);
            }
            std::string joint_node = nodeNameOnly(from_tile, CB_NODE_JOINT, wire.joint);
            appendNode(ss, joint_name, jointDetail(wire.joint));
            appendNode(ss, tileConnectionNodeName(from_tile, joint_node, from_node), "connection");
            appendNode(ss, tileConnectionNodeName(from_tile, jump_node, joint_node), "connection");
        }
        else {
            appendNode(ss, tileConnectionNodeName(from_tile, jump_node, from_node), "connection");
        }
        appendNode(ss, jump_name, jumpDetail(wire.jump));
        appendNode(ss, dst_name, jumpDetail(wire.jump));
    }
    else {
        std::string from_name = bestFromNodeName(from_tile, wire);
        if (from_name.empty()) {
            from_name = fallbackTileNodeName(from_tile, "LOCAL", wire.local);
        }
        appendNode(ss, from_name, localDetail(wire.local));
        if (wire.joint >= 0) {
            std::string joint_name = tileNodeName(from_tile, CB_NODE_JOINT, wire.joint);
            if (joint_name.empty()) {
                joint_name = fallbackTileNodeName(from_tile, "JOINT", wire.joint);
            }
            appendNode(ss, joint_name, jointDetail(wire.joint));
        }
    }

    return ss.str();
}

std::string formatRoute(const std::vector<Wire>& route)
{
    std::stringstream ss;
    ss << route.front().net_name;
    for (const Wire& fragment : route) {
        std::string formatted = formatFragment(fragment);
        if (!formatted.empty()) {
            ss << " " << formatted;
        }
    }
    return ss.str();
}

void collectRoutes(rtl::Inst& inst, const std::set<std::string>& requested_nets, Tcl_Interp *interp, Tcl_Obj *list_obj)
{
    for (const auto& route : inst.wires) {
        if (route.empty() || route.front().net_name.empty()) {
            continue;
        }
        if (!requested_nets.contains(route.front().net_name)) {
            continue;
        }

        std::string formatted = formatRoute(route);
        Tcl_Obj *wordObj = Tcl_NewStringObj(formatted.c_str(), -1);
        Tcl_ListObjAppendElement(interp, list_obj, wordObj);
    }

    for (auto& sub_inst : inst.insts) {
        collectRoutes(sub_inst, requested_nets, interp, list_obj);
    }
}

}

int
get_wires_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    if (objc < 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "");
        return TCL_ERROR;
    }

    std::set<std::string> requested_nets;
    for (int arg = 1; arg < objc; ++arg) {
        int list_objc = 0;
        Tcl_Obj **list_objv = nullptr;
        if (Tcl_ListObjGetElements(interp, objv[arg], &list_objc, &list_objv) != TCL_OK) {
            return TCL_ERROR;
        }
        for (int i = 0; i < list_objc; ++i) {
            requested_nets.insert(Tcl_GetString(list_objv[i]));
        }
    }

    Tcl_Obj *list_obj = Tcl_NewListObj(0, NULL);
    collectRoutes(Tech::current().design.top, requested_nets, interp, list_obj);

    Tcl_SetObjResult(interp, list_obj);
    std::print("\n");
    return TCL_OK;
}
