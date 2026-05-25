#include "Tech.h"
#include "Device.h"
#include "Wire.h"

#include "tcl_pnr.h"

#include <cerrno>
#include <cstring>
#include <fstream>
#include <format>
#include <limits>
#include <set>
#include <sstream>
#include <string>
#include <vector>

using namespace technology;
using namespace fpga;

namespace {

constexpr size_t FULL_NAME_LIMIT = std::numeric_limits<size_t>::max();
constexpr const char* BEL_LETTERS = "ABCD";

std::string fullInstName(rtl::Inst& inst)
{
    return inst.makeName(FULL_NAME_LIMIT);
}

void collectInsts(rtl::Inst& inst, std::vector<rtl::Inst*>& insts)
{
    insts.push_back(&inst);
    for (auto& sub_inst : inst.insts) {
        collectInsts(sub_inst, insts);
    }
}

std::string tileName(const Tile* tile)
{
    if (!tile) {
        return {};
    }
    if (!tile->full_name.empty()) {
        return tile->full_name;
    }
    if (tile->tile_type) {
        return std::format("{}_X{}Y{}", tile->tile_type->name, tile->name.x, tile->name.y);
    }
    if (tile->cb_type) {
        return std::format("{}_X{}Y{}", tile->cb_type->name, tile->name.x, tile->name.y);
    }
    return {};
}

int siteIndex(int pos)
{
    return pos >= 128 ? 1 : 0;
}

int belIndex(int pos)
{
    int local = pos >= 128 ? pos - 128 : pos;
    return (local / 4) % 4;
}

std::string siteName(rtl::Inst& inst)
{
    if (!inst.tile.peer) {
        return {};
    }
    Tile& tile = *inst.tile;
    int index = siteIndex(inst.pos);
    if (!tile.sites.empty()) {
        if (index < static_cast<int>(tile.sites.size())) {
            return tile.sites[index];
        }
        return tile.sites.back();
    }
    return tileName(&tile);
}

std::string siteTypeName(rtl::Inst& inst)
{
    if (!inst.tile.peer) {
        return {};
    }
    Tile& tile = *inst.tile;
    int index = siteIndex(inst.pos);
    if (index < static_cast<int>(tile.site_types.size())) {
        return tile.site_types[index];
    }
    if (!tile.site_types.empty()) {
        return tile.site_types.back();
    }
    return {};
}

std::string belName(rtl::Inst& inst)
{
    if (!inst.cell_ref.peer) {
        return {};
    }
    const std::string& type = inst.cell_ref->type;
    int bel = belIndex(inst.pos);
    if (type.find("FD") == 0) {
        return std::format("{}FF", BEL_LETTERS[bel]);
    }
    if (type.find("LUT") == 0) {
        int size = 6;
        if (type.size() >= 4 && type[3] >= '1' && type[3] <= '6') {
            size = type[3] - '0';
        }
        if (size < 5) {
            size = 6;
        }
        return std::format("{}{}LUT", BEL_LETTERS[bel], size);
    }
    if (type.find("CARRY") == 0) {
        return type;
    }
    if (type.find("MUX") == 0) {
        if (type.find("F8") != std::string::npos) {
            return "F8MUX";
        }
        if (type.find("F7") != std::string::npos) {
            return bel < 2 ? "F7AMUX" : "F7BMUX";
        }
        return type;
    }
    return {};
}

std::string vivadoBelName(rtl::Inst& inst)
{
    std::string bel = belName(inst);
    std::string site_type = siteTypeName(inst);
    if (bel.empty() || site_type.empty()) {
        return bel;
    }
    return site_type + "." + bel;
}

std::string nodeName(const Tile* tile, CBNodeNameType type, int value)
{
    if (!tile || !tile->cb_type || value < 0) {
        return {};
    }
    const std::string* name = tile->cb_type->nodeName(type, value);
    return name ? *name : std::string{};
}

std::string bestFromNodeName(const Tile* tile, const Wire& wire)
{
    if (!tile) {
        return {};
    }
    std::string local = nodeName(tile, CB_NODE_LOCAL, wire.local);
    std::string dst = nodeName(tile, CB_NODE_DST, wire.local);
    if (wire.pos == 1 && !dst.empty()) {
        return dst;
    }
    return local.empty() ? dst : local;
}

std::string tileTypeNameForPip(const Tile* tile)
{
    if (!tile) {
        return {};
    }
    if (tile->cb_type) {
        return tile->cb_type->name;
    }
    if (tile->tile_type) {
        return tile->tile_type->name;
    }
    return {};
}

std::string formatPip(const Tile* tile, const std::string& src, const std::string& dst, bool crossbar)
{
    std::string tile_name = tileName(tile);
    std::string tile_type;
    if (tile) {
        if (crossbar && tile->cb_type) {
            tile_type = tile->cb_type->name;
        }
        else if (!crossbar && tile->tile_type) {
            tile_type = tile->tile_type->name;
        }
    }
    if (tile_type.empty()) {
        tile_type = tileTypeNameForPip(tile);
    }
    if (tile_name.empty() || src.empty() || dst.empty()) {
        return {};
    }
    return std::format("{}/{}.{}{}{}", tile_name, tile_type, src, crossbar ? "->>" : "->", dst);
}

void addPip(std::vector<std::string>& pips, const std::string& pip)
{
    if (!pip.empty()) {
        pips.push_back(pip);
    }
}

std::vector<std::string> routePips(const Wire& wire)
{
    std::vector<std::string> pips;
    Device& device = Device::current();
    const Tile* from = device.getTile(wire.from.x, wire.from.y);

    if (wire.type == Wire::WIRE_TILE_PIN) {
        const Tile* resource = Device::current().getTile(wire.resource.x, wire.resource.y);
        if (!resource) {
            resource = from;
        }
        if (!resource || !resource->tile_type || wire.local < 0) {
            return pips;
        }
        for (TilePinNameType type : {TILE_PIN_INPUT, TILE_PIN_OUTPUT}) {
            if (wire.pin_dir >= 0 && wire.pin_dir != type) {
                continue;
            }
            const std::string* resource_node = wire.resource_node >= 0
                ? resource->tile_type->pin_map.localResourceName(type, wire.resource_node, wire.local)
                : nullptr;
            const std::string* local = wire.resource_node >= 0
                ? resource->tile_type->pin_map.localWireName(type, wire.resource_node, wire.local)
                : nullptr;
            if (!resource_node && !local) {
                resource_node = resource->tile_type->pin_map.localResourceName(type, wire.local);
                local = resource->tile_type->pin_map.localWireName(type, wire.local);
            }
            if (!resource_node || !local) {
                continue;
            }
            addPip(pips, type == TILE_PIN_INPUT
                ? formatPip(resource, *resource_node, *local, false)
                : formatPip(resource, *local, *resource_node, false));
        }
        return pips;
    }

    std::string from_node = bestFromNodeName(from, wire);
    if (wire.jump >= 0) {
        std::string src = wire.src_wire_name.empty() ? nodeName(from, CB_NODE_SRC, wire.jump) : wire.src_wire_name;
        if (src.empty()) {
            src = nodeName(from, CB_NODE_JUMP, wire.jump);
        }
        if (wire.joint >= 0) {
            std::string joint = nodeName(from, CB_NODE_JOINT, wire.joint);
            addPip(pips, formatPip(from, from_node, joint, true));
            addPip(pips, formatPip(from, joint, src, true));
        }
        else {
            addPip(pips, formatPip(from, from_node, src, true));
        }
        return pips;
    }

    if (wire.joint >= 0) {
        std::string joint = nodeName(from, CB_NODE_JOINT, wire.joint);
        addPip(pips, formatPip(from, from_node, joint, true));
    }
    return pips;
}

void exportPlacement(std::ofstream& out, rtl::Inst& top)
{
    std::vector<rtl::Inst*> insts;
    collectInsts(top, insts);
    std::set<std::string> rows;

    out << "==============================\n";
    out << "SECTION: PLACEMENT\n";
    out << "==============================\n";
    out << "cell,ref_name,site,bel,loc\n";

    for (rtl::Inst* inst : insts) {
        if (!inst || !inst->cell_ref.peer || !inst->tile.peer) {
            continue;
        }
        std::string site = siteName(*inst);
        std::ostringstream row;
        row << fullInstName(*inst) << ","
            << inst->cell_ref->type << ","
            << site << ","
            << vivadoBelName(*inst) << ","
            << site;
        rows.insert(row.str());
    }
    for (const std::string& row : rows) {
        out << row << "\n";
    }
}

void exportRouting(std::ofstream& out, rtl::Inst& top)
{
    std::vector<rtl::Inst*> insts;
    collectInsts(top, insts);

    out << "\n";
    out << "==============================\n";
    out << "SECTION: ROUTING_PIPS\n";
    out << "==============================\n";
    out << "net,pip\n";

    std::set<std::pair<std::string, std::string>> seen;
    for (rtl::Inst* inst : insts) {
        if (!inst) {
            continue;
        }
        for (const auto& route : inst->wires) {
            if (route.empty()) {
                continue;
            }
            std::string net = route.front().net_name;
            if (net.empty()) {
                continue;
            }
            for (const Wire& wire : route) {
                for (const std::string& pip : routePips(wire)) {
                    if (seen.insert({net, pip}).second) {
                        out << net << "," << pip << "\n";
                    }
                }
            }
        }
    }
}

}

int
export_pnr_cmd(
    ClientData unused,
    Tcl_Interp *interp,
    int objc,
    Tcl_Obj *const objv[])
{
    (void)unused;
    if (objc != 2) {
        Tcl_WrongNumArgs(interp, 1, objv, "filename");
        return TCL_ERROR;
    }

    const char* filename = Tcl_GetString(objv[1]);
    errno = 0;
    std::ofstream out(filename);
    if (!out) {
        Tcl_SetObjResult(interp, Tcl_NewStringObj(
            std::format("cant open '{}' for writing: errno={} ({})", filename, errno, std::strerror(errno)).c_str(), -1));
        return TCL_ERROR;
    }

    exportPlacement(out, Tech::current().design.top);
    exportRouting(out, Tech::current().design.top);
    return TCL_OK;
}
