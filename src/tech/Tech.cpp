#include "Tech.h"
#include "Device.h"
#include "Timings.h"
#include "RtlFormat.h"
#include "PrintDesign.h"
#include "getInsts.h"
#include "json/json.h"

#include <fstream>
#include <cerrno>
#include <cstring>
#include <memory>
#include <vector>
#include <functional>
#include <limits>

using namespace technology;

namespace {

constexpr size_t FULL_NAME_LIMIT = std::numeric_limits<size_t>::max();

std::string fullInstName(rtl::Inst& inst)
{
    return inst.makeName(FULL_NAME_LIMIT);
}

std::string fullConnNetName(rtl::Conn& conn)
{
    return conn.makeNetName(nullptr, FULL_NAME_LIMIT);
}

void collectInsts(rtl::Inst& inst, std::vector<rtl::Inst*>& insts)
{
    insts.push_back(&inst);
    for (auto& sub_inst : inst.insts) {
        collectInsts(sub_inst, insts);
    }
}

rtl::Inst* findInst(rtl::Inst& inst, const std::string& name)
{
    if (fullInstName(inst) == name) {
        return &inst;
    }
    for (auto& sub_inst : inst.insts) {
        if (rtl::Inst* found = findInst(sub_inst, name)) {
            return found;
        }
    }
    return nullptr;
}

Json::Value coordToJson(const fpga::Coord& coord)
{
    Json::Value value(Json::arrayValue);
    value.append(coord.x);
    value.append(coord.y);
    return value;
}

fpga::Coord coordFromJson(const Json::Value& value)
{
    return fpga::Coord{value[0].asInt(), value[1].asInt()};
}

Json::Value stringMapToJson(const std::map<std::string,std::string>& values)
{
    Json::Value out(Json::objectValue);
    for (const auto& [key, value] : values) {
        out[key] = value;
    }
    return out;
}

Json::Value ioAssignmentsToJson(const std::map<std::string,std::map<std::string,std::string>>& values)
{
    Json::Value out(Json::arrayValue);
    for (const auto& [port, properties] : values) {
        Json::Value item(Json::objectValue);
        item["port"] = port;
        item["properties"] = stringMapToJson(properties);
        if (auto it = properties.find("PACKAGE_PIN"); it != properties.end()) {
            item["package_pin"] = it->second;
        }
        if (auto it = properties.find("IOSTANDARD"); it != properties.end()) {
            item["iostandard"] = it->second;
        }
        out.append(item);
    }
    return out;
}

std::string cbTileName(const fpga::Tile* tile)
{
    if (!tile || !tile->cb_type) {
        return {};
    }
    return std::format("{}_X{}Y{}", tile->cb_type->name, tile->name.x, tile->name.y);
}

std::string resourceTileName(const fpga::Tile* tile)
{
    if (!tile || !tile->tile_type) {
        return {};
    }
    return std::format("{}_X{}Y{}", tile->tile_type->name, tile->name.x, tile->name.y);
}

std::string resourceTileName(const fpga::Tile* tile, const fpga::TileType* type)
{
    if (!tile || !type) {
        return {};
    }
    return std::format("{}_X{}Y{}", type->name, tile->name.x, tile->name.y);
}

Json::Value connectionToJson(rtl::Conn& sink_conn)
{
    Json::Value out(Json::objectValue);
    out["sink_port"] = sink_conn.port_ref.peer ? sink_conn.port_ref->makeName() : "";
    out["net"] = fullConnNetName(sink_conn);

    rtl::Conn* driver = sink_conn.follow();
    if (driver && driver->inst_ref.peer && driver->port_ref.peer) {
        out["driver_inst"] = fullInstName(*driver->inst_ref);
        out["driver_type"] = driver->inst_ref->cell_ref.peer ? driver->inst_ref->cell_ref->type : "";
        out["driver_port"] = driver->port_ref->makeName();
        out["same_tile"] = sink_conn.inst_ref.peer && sink_conn.inst_ref->tile.peer && driver->inst_ref->tile.peer
            && sink_conn.inst_ref->tile.peer == driver->inst_ref->tile.peer;
        if (driver->inst_ref->tile.peer) {
            out["driver_tile"] = resourceTileName(&*driver->inst_ref->tile);
            out["driver_pos"] = driver->inst_ref->pos;
        }
    }
    return out;
}

const std::string* cbNodeName(const fpga::Tile* tile, fpga::CBNodeNameType type, int value)
{
    if (!tile || !tile->cb_type || value < 0) {
        return nullptr;
    }
    return tile->cb_type->nodeName(type, value);
}

void appendString(Json::Value& array, const std::string& value)
{
    if (!value.empty()) {
        array.append(value);
    }
}

Json::Value namedNodeJson(const std::string& kind, const std::string& tile, const std::string& node, int value)
{
    Json::Value out(Json::objectValue);
    out["kind"] = kind;
    out["value"] = value;
    if (!tile.empty()) {
        out["tile"] = tile;
    }
    if (!node.empty()) {
        out["node"] = node;
        out["full_name"] = tile.empty() ? node : tile + "." + node;
    }
    return out;
}

std::string inferredJumpEndName(const std::string* src)
{
    if (!src) {
        return {};
    }
    std::string name = *src;
    size_t pos = name.find("BEG");
    if (pos == std::string::npos) {
        return {};
    }
    name.replace(pos, 3, "END");
    return name;
}

std::string connectionFeature(const std::string& tile, const std::string* dst, const std::string* src)
{
    if (tile.empty() || !dst || !src) {
        return {};
    }
    return tile + "." + *dst + "." + *src;
}

std::string connectionFeature(const std::string& tile, const std::string& dst, const std::string& src)
{
    if (tile.empty() || dst.empty() || src.empty()) {
        return {};
    }
    return tile + "." + dst + "." + src;
}

bool hasJointPath(const fpga::CBJointState& from_joints, const fpga::CBJointState& to_joints)
{
    return (from_joints.joint & to_joints.joint) != u256{};
}

bool hasLocalToSrcPath(const fpga::CBType* type, int local, int src)
{
    if (!type || local < 0 || local >= CB_MAX_NODES || src < 0 || src >= CB_MAX_NODES) {
        return false;
    }
    if ((type->local_src[local].jump & (u256{0,1} << src)) != u256{}) {
        return true;
    }
    return hasJointPath(type->local_joint[local], type->src_joint[src]);
}

bool hasDstToSrcPath(const fpga::CBType* type, int dst, int src)
{
    if (!type || dst < 0 || dst >= CB_MAX_NODES || src < 0 || src >= CB_MAX_NODES) {
        return false;
    }
    if ((type->dst_src[dst].jump & (u256{0,1} << src)) != u256{}) {
        return true;
    }
    return hasJointPath(type->dst_joint[dst], type->src_joint[src]);
}

const fpga::CBConnName* selectConcreteConn(const fpga::CBType* type,
                                           fpga::CBNodeNameType from_type, int from_value,
                                           fpga::CBNodeNameType to_type, int to_value,
                                           const std::string& preferred_from = {},
                                           const std::string& preferred_to = {})
{
    if (!type) {
        return nullptr;
    }
    const std::vector<fpga::CBConnName>* conns = type->connNames(from_type, from_value, to_type, to_value);
    if (!conns || conns->empty()) {
        return nullptr;
    }
    auto matches = [&](const fpga::CBConnName& conn, bool match_from, bool match_to) {
        return (!match_from || conn.from == preferred_from)
            && (!match_to || conn.to == preferred_to);
    };
    if (!preferred_from.empty() && !preferred_to.empty()) {
        for (const fpga::CBConnName& conn : *conns) {
            if (matches(conn, true, true)) {
                return &conn;
            }
        }
        return nullptr;
    }
    if (!preferred_from.empty()) {
        for (const fpga::CBConnName& conn : *conns) {
            if (matches(conn, true, false)) {
                return &conn;
            }
        }
        return nullptr;
    }
    if (!preferred_to.empty()) {
        for (const fpga::CBConnName& conn : *conns) {
            if (matches(conn, false, true)) {
                return &conn;
            }
        }
        return nullptr;
    }
    return &conns->front();
}

Json::Value tilePinAnnotationForType(const fpga::Tile* tile, const fpga::TileType* type, int local, int resource_node)
{
    Json::Value out(Json::objectValue);
    if (!tile || !type || local < 0) {
        return out;
    }

    std::string tile_name = resourceTileName(tile, type);
    out["tile"] = tile_name;
    out["tile_type"] = type->name;

    for (auto dir : {fpga::TILE_PIN_INPUT, fpga::TILE_PIN_OUTPUT}) {
        const std::string* resource = resource_node >= 0 ? type->pin_map.localResourceName(dir, resource_node, local) : nullptr;
        const std::string* wire = resource_node >= 0 ? type->pin_map.localWireName(dir, resource_node, local) : nullptr;
        const std::string* pin = resource_node >= 0 ? type->pin_map.localPinName(dir, resource_node, local) : nullptr;
        if (!resource && !wire && !pin) {
            resource = type->pin_map.localResourceName(dir, local);
            wire = type->pin_map.localWireName(dir, local);
            pin = type->pin_map.localPinName(dir, local);
        }
        if (!resource && !wire && !pin) {
            continue;
        }

        Json::Value dir_json(Json::objectValue);
        if (resource) {
            dir_json["resource_node"] = *resource;
            dir_json["resource_full_name"] = tile_name + "." + *resource;
        }
        if (wire) {
            dir_json["local_wire"] = *wire;
            dir_json["local_wire_full_name"] = tile_name + "." + *wire;
        }
        if (pin) {
            dir_json["pin"] = *pin;
        }
        if (resource && wire) {
            dir_json["fasm_feature"] = dir == fpga::TILE_PIN_INPUT
                ? tile_name + "." + *resource + "." + *wire
                : tile_name + "." + *wire + "." + *resource;
        }

        out[dir == fpga::TILE_PIN_INPUT ? "input" : "output"] = dir_json;
    }

    return out;
}

Json::Value tilePinAnnotation(const fpga::Tile* tile, int local, int resource_node)
{
    Json::Value out(Json::objectValue);
    if (!tile || local < 0) {
        return out;
    }

    if (tile->tile_type) {
        out = tilePinAnnotationForType(tile, tile->tile_type, local, resource_node);
    }

    return out;
}

const fpga::Tile* wireResourceTile(const fpga::Wire& wire)
{
    if (wire.resource.x >= 0 && wire.resource.y >= 0) {
        return fpga::Device::current().getTile(wire.resource.x, wire.resource.y);
    }
    return fpga::Device::current().getTile(wire.from.x, wire.from.y);
}

Json::Value wireAnnotation(const fpga::Wire& wire)
{
    fpga::Device& device = fpga::Device::current();
    const fpga::Tile* from = device.getTile(wire.from.x, wire.from.y);
    const fpga::Tile* to = device.getTile(wire.to.x, wire.to.y);
    const fpga::Tile* resource = wire.type == fpga::Wire::WIRE_TILE_PIN ? wireResourceTile(wire) : from;

    Json::Value out(Json::objectValue);
    out["from_cb_tile"] = cbTileName(from);
    out["to_cb_tile"] = cbTileName(to);
    out["from_resource_tile"] = resourceTileName(resource);
    if (wire.type == fpga::Wire::WIRE_TILE_PIN) {
        out["resource_tile"] = resourceTileName(resource);
        out["resource_coord"] = coordToJson(resource ? resource->coord : fpga::Coord{});
        out["resource_vendor_coord"] = coordToJson(resource ? resource->name : fpga::Coord{});
        out["resource_cell_type"] = wire.cell_type;
        out["resource_port"] = wire.port;
        out["resource_pos"] = wire.pos;
        out["resource_node"] = wire.resource_node;
        out["resource_pin_dir"] = wire.pin_dir == fpga::TILE_PIN_OUTPUT ? "output"
            : (wire.pin_dir == fpga::TILE_PIN_INPUT ? "input" : "");
    }
    Json::Value nodes(Json::arrayValue);
    Json::Value features(Json::arrayValue);

    if (wire.type == fpga::Wire::WIRE_TILE_PIN) {
        const std::string* local = cbNodeName(from, fpga::CB_NODE_LOCAL, wire.local);
        nodes.append(namedNodeJson("crossbar_local", cbTileName(from), local ? *local : "", wire.local));
        out["tile_resource"] = tilePinAnnotation(resource, wire.local, wire.resource_node);
        const Json::Value& pin_info = out["tile_resource"];
        if ((wire.pin_dir < 0 || wire.pin_dir == fpga::TILE_PIN_INPUT)
            && pin_info.isMember("input") && pin_info["input"].isMember("fasm_feature")) {
            appendString(features, pin_info["input"]["fasm_feature"].asString());
        }
        if ((wire.pin_dir < 0 || wire.pin_dir == fpga::TILE_PIN_OUTPUT)
            && pin_info.isMember("output") && pin_info["output"].isMember("fasm_feature")) {
            appendString(features, pin_info["output"]["fasm_feature"].asString());
        }
    }
    else if (wire.jump >= 0) {
        const std::string* src = cbNodeName(from, fpga::CB_NODE_SRC, wire.jump);
        if (!src) {
            src = cbNodeName(from, fpga::CB_NODE_JUMP, wire.jump);
        }
        const std::string* dst = cbNodeName(to, fpga::CB_NODE_DST, wire.jump);
        const std::string* local = cbNodeName(from, fpga::CB_NODE_LOCAL, wire.local);
        const std::string* prev_dst = cbNodeName(from, fpga::CB_NODE_DST, wire.local);
        bool use_prev_dst = wire.pos == 1
            || (prev_dst && hasDstToSrcPath(from ? from->cb_type : nullptr, wire.local, wire.jump)
                && !hasLocalToSrcPath(from ? from->cb_type : nullptr, wire.local, wire.jump));
        const std::string* from_node = use_prev_dst ? prev_dst : (local ? local : prev_dst);
        fpga::CBNodeNameType from_type = use_prev_dst || !local ? fpga::CB_NODE_DST : fpga::CB_NODE_LOCAL;

        std::string from_name = from_node ? *from_node : "";
        std::string src_name = src ? *src : "";
        if (!wire.src_wire_name.empty()) {
            src_name = wire.src_wire_name;
        }
        if (wire.joint >= 0) {
            const std::string* joint = cbNodeName(from, fpga::CB_NODE_JOINT, wire.joint);
            std::string joint_name = joint ? *joint : "";
            if (from && from->cb_type) {
                std::string preferred_from = from_type == fpga::CB_NODE_DST ? wire.dst_wire_name : std::string{};
                if (const fpga::CBConnName* conn = selectConcreteConn(from->cb_type, from_type, wire.local,
                        fpga::CB_NODE_JOINT, wire.joint, preferred_from)) {
                    from_name = conn->from;
                    joint_name = conn->to;
                    appendString(features, connectionFeature(cbTileName(from), conn->to, conn->from));
                }
                else {
                    appendString(features, connectionFeature(cbTileName(from), joint, from_node));
                }
                if (const fpga::CBConnName* conn = selectConcreteConn(from->cb_type, fpga::CB_NODE_JOINT, wire.joint,
                        fpga::CB_NODE_SRC, wire.jump, joint_name, wire.src_wire_name)) {
                    joint_name = conn->from;
                    src_name = conn->to;
                    appendString(features, connectionFeature(cbTileName(from), conn->to, conn->from));
                }
                else {
                    appendString(features, connectionFeature(cbTileName(from), src, joint));
                }
            }
            else {
                appendString(features, connectionFeature(cbTileName(from), joint, from_node));
                appendString(features, connectionFeature(cbTileName(from), src, joint));
            }
            nodes.append(namedNodeJson(use_prev_dst || !local ? "crossbar_dst" : "crossbar_local", cbTileName(from), from_name, wire.local));
            nodes.append(namedNodeJson("crossbar_joint", cbTileName(from), joint_name, wire.joint));
        }
        else {
            if (from && from->cb_type) {
                std::string preferred_from = from_type == fpga::CB_NODE_DST ? wire.dst_wire_name : std::string{};
                if (const fpga::CBConnName* conn = selectConcreteConn(from->cb_type, from_type, wire.local,
                        fpga::CB_NODE_SRC, wire.jump, preferred_from, wire.src_wire_name)) {
                    from_name = conn->from;
                    src_name = conn->to;
                    appendString(features, connectionFeature(cbTileName(from), conn->to, conn->from));
                }
                else {
                    appendString(features, connectionFeature(cbTileName(from), src, from_node));
                }
            }
            else {
                appendString(features, connectionFeature(cbTileName(from), src, from_node));
            }
            nodes.append(namedNodeJson(use_prev_dst || !local ? "crossbar_dst" : "crossbar_local", cbTileName(from), from_name, wire.local));
        }
        if (!wire.src_wire_name.empty()) {
            src_name = wire.src_wire_name;
        }
        nodes.append(namedNodeJson("crossbar_src_jump", cbTileName(from), src_name, wire.jump));
        std::string dst_name = wire.dst_wire_name;
        if (dst_name.empty()) {
            dst_name = inferredJumpEndName(src_name.empty() ? nullptr : &src_name);
        }
        if (dst_name.empty() && dst) {
            dst_name = *dst;
        }
        nodes.append(namedNodeJson("crossbar_dst_jump", cbTileName(to), dst_name, wire.jump));
    }
    else {
        const std::string* local = cbNodeName(from, fpga::CB_NODE_LOCAL, wire.local);
        nodes.append(namedNodeJson("crossbar_local", cbTileName(from), local ? *local : "", wire.local));
        if (wire.joint >= 0) {
            const std::string* joint = cbNodeName(from, fpga::CB_NODE_JOINT, wire.joint);
            std::string local_name = local ? *local : "";
            std::string joint_name = joint ? *joint : "";
            if (from && from->cb_type) {
                if (const fpga::CBConnName* conn = selectConcreteConn(from->cb_type, fpga::CB_NODE_LOCAL, wire.local,
                        fpga::CB_NODE_JOINT, wire.joint)) {
                    local_name = conn->from;
                    joint_name = conn->to;
                    appendString(features, connectionFeature(cbTileName(from), conn->to, conn->from));
                }
                else {
                    appendString(features, connectionFeature(cbTileName(from), joint, local));
                }
            }
            else {
                appendString(features, connectionFeature(cbTileName(from), joint, local));
            }
            nodes[static_cast<Json::ArrayIndex>(nodes.size() - 1)] = namedNodeJson("crossbar_local", cbTileName(from), local_name, wire.local);
            nodes.append(namedNodeJson("crossbar_joint", cbTileName(from), joint_name, wire.joint));
        }
    }

    out["nodes"] = nodes;
    out["fasm_features"] = features;
    return out;
}

Json::Value wireToJson(const fpga::Wire& wire)
{
    Json::Value value(Json::objectValue);
    value["type"] = wire.type == fpga::Wire::WIRE_TILE_PIN ? "tile_pin" : "crossbar";
    value["from"] = coordToJson(wire.from);
    value["to"] = coordToJson(wire.to);
    value["local"] = wire.local;
    value["pos"] = wire.pos;
    value["jump"] = wire.jump;
    value["joint"] = wire.joint;
    value["resource"] = coordToJson(wire.resource);
    value["resource_node"] = wire.resource_node;
    value["pin_dir"] = wire.pin_dir;
    value["cell_type"] = wire.cell_type;
    value["port"] = wire.port;
    value["net"] = wire.net_name;
    value["src_wire"] = wire.src_wire_name;
    value["dst_wire"] = wire.dst_wire_name;
    value["annotation"] = wireAnnotation(wire);
    return value;
}

fpga::Wire wireFromJson(const Json::Value& value)
{
    fpga::Wire wire;
    std::string type = value.get("type", "crossbar").asString();
    wire.type = type == "tile_pin" ? fpga::Wire::WIRE_TILE_PIN : fpga::Wire::WIRE_CROSSBAR;
    wire.from = coordFromJson(value["from"]);
    wire.to = coordFromJson(value["to"]);
    wire.local = value.get("local", -1).asInt();
    wire.pos = value.get("pos", -1).asInt();
    wire.jump = value.get("jump", -1).asInt();
    wire.joint = value.get("joint", -1).asInt();
    if (value.isMember("resource")) {
        wire.resource = coordFromJson(value["resource"]);
    }
    wire.resource_node = value.get("resource_node", -1).asInt();
    wire.pin_dir = value.get("pin_dir", -1).asInt();
    wire.cell_type = value.get("cell_type", "").asString();
    wire.port = value.get("port", "").asString();
    wire.net_name = value.get("net", "").asString();
    wire.src_wire_name = value.get("src_wire", "").asString();
    wire.dst_wire_name = value.get("dst_wire", "").asString();
    return wire;
}

void resetDeviceState(fpga::Device& device)
{
    for (auto& tile_ref : device.tile_grid) {
        std::memset(&tile_ref.cb, 0, sizeof(tile_ref.cb));
        tile_ref.cb.type = tile_ref.cb_type;
        tile_ref.pin_state = {};
    }
}

void clearInstState(rtl::Inst& inst)
{
    inst.tile.clear();
    inst.coord = fpga::Coord{};
    inst.pos = -1;
    inst.wires.clear();
    for (auto& sub_inst : inst.insts) {
        clearInstState(sub_inst);
    }
}

void markBit(u256& bits, int index)
{
    if (index >= 0 && index < CB_MAX_NODES) {
        bits |= u256{0,1} << index;
    }
}

void markJump(fpga::CBJumpState& state, int index)
{
    if (index >= 0 && index < CB_MAX_NODES) {
        state.jump = state.jump | (u256{0,1} << index);
    }
}

void restoreWireState(const fpga::Wire& wire)
{
    fpga::Device& device = fpga::Device::current();
    fpga::Tile* from = device.getTile(wire.from.x, wire.from.y);
    fpga::Tile* to = device.getTile(wire.to.x, wire.to.y);

    if (wire.type == fpga::Wire::WIRE_TILE_PIN) {
        if (from && (wire.pin_dir < 0 || wire.pin_dir == fpga::TILE_PIN_INPUT)) {
            markBit(from->pin_state.leased_nodes, wire.local);
            markBit(from->cb.local.local, wire.local);
        }
        return;
    }

    if (from) {
        markBit(from->cb.local.local, wire.local);
        markJump(from->cb.dst, wire.local);
        markJump(from->cb.src, wire.jump);
        markJump(from->cb.joint, wire.joint);
    }
    if (to && wire.jump >= 0) {
        markJump(to->cb.dst, wire.jump);
    }
}

}

CombDelays Tech::comb_delays;
std::multimap<std::string,std::string> Tech::clocked_ports;
std::multimap<std::string,std::string> Tech::buffers_ports;

Tech& Tech::current()
{
    static bool inited = false;
    static Tech tech;
    if (!inited) {
        tech.init();
        inited = true;
    }
    return tech;
}

void Tech::recursivePrintTimingReport(clk::TimingPath& path, unsigned limit, int level)
{
    std::vector<clk::TimingPath*> paths;
    paths.reserve(path.sub_paths.size());
    for (auto& sub_path : path.sub_paths) {
        if (!sub_path.data_output && !sub_path.precalculated) {
            continue;
        }
        paths.push_back(&sub_path);
    }

    sort(paths.begin(), paths.end(), [](clk::TimingPath* a, clk::TimingPath* b) { return a->max_setup_time > b->max_setup_time; });

    unsigned cnt = 0;
    for (auto& sub_path : paths) {
        if (++cnt == limit) {
            break;
        }

        if (path.sub_paths.size() > 1) {
            std::print("\n");
            for (int i=0; i < level + 1; ++i) {
                std::print("  ");
            }
        }

        if (sub_path->precalculated) {
            if (sub_path->precalculated->max_length < (int)limit) {
                std::print("*<- '{}'({})::: {:.3f}/{:.3f} ns, fanout: {}, fanin: {}", sub_path->precalculated->data_output->inst_ref->makeName(),
                    sub_path->precalculated->data_output->inst_ref->cell_ref->type, sub_path->precalculated->max_setup_time, sub_path->precalculated->min_setup_time,
                    (static_cast<Referable<rtl::Conn>*>(sub_path->precalculated->data_output))->getPeers().size(), sub_path->precalculated->sub_paths.size());
                if (sub_path->precalculated->sub_paths.size() > 1) {
                    std::print(" :");
                }
                recursivePrintTimingReport(*sub_path->precalculated, limit, level + 1);
            }
            else {
                std::print(" <- '{}'({}) ...(depth {}/{} is hidden)::: {:.3f}/{:.3f} ns, fanout: {}, fanin: {}", sub_path->precalculated->data_output->inst_ref->makeName(),
                    sub_path->precalculated->data_output->inst_ref->cell_ref->type, sub_path->precalculated->max_length, sub_path->precalculated->min_length,
                    sub_path->precalculated->max_setup_time, sub_path->precalculated->min_setup_time, (static_cast<Referable<rtl::Conn>*>(sub_path->precalculated->data_output))->getPeers().size(),
                    sub_path->precalculated->sub_paths.size());
            }
        }
        else {
            std::print(" <- '{}'({})::: {:.3f}/{:.3f} ns, fanout: {}, fanin: {}", sub_path->data_output->inst_ref->makeName(), sub_path->data_output->inst_ref->cell_ref->type,
                sub_path->max_setup_time, sub_path->min_setup_time, (static_cast<Referable<rtl::Conn>*>(sub_path->data_output))->getPeers().size(), sub_path->sub_paths.size());
            if (sub_path->sub_paths.size() > 1) {
                std::print(" :");
            }
            recursivePrintTimingReport(*sub_path, limit, level + 1);
        }
    }
    if (paths.size() > limit) {
        std::print("\n");
        for (int i=0; i < level + 1; ++i) {
            std::print("  ");  //??
        }
        std::print(" ...");
    }
}

void Tech::prepareTimingLists()
{
    timings.makeTimingsList(design, clocks);
}

void Tech::estimateTimings(unsigned limit_paths, unsigned limit_rows)
{
    timings.calculateTimings();
    for (auto& conns : timings.clocked_inputs) {
        std::print("\nclock: {}", conns.first->name);

        std::vector<clk::Timings::TimingInfo*> infos;
        infos.reserve(conns.second.size());
        for (auto& info : conns.second) {
            if (!info.path.data_output) {
                continue;
            }
            infos.push_back(&info);
        }

        sort(infos.begin(), infos.end(), [](clk::Timings::TimingInfo* a, clk::Timings::TimingInfo* b) { return a->path.max_setup_time > b->path.max_setup_time; });

        unsigned cnt = 0;
        for (auto& info : infos) {
            if (++cnt == limit_paths) {
                break;
            }
            std::print("\nconn: '{}' ('{}')::: {:.3f}/{:.3f}ns, length: {}/{}", info->data_in->makeName(), info->data_in->inst_ref->cell_ref->type,
                info->path.max_setup_time, info->path.min_setup_time, info->path.max_length, info->path.min_length);
            recursivePrintTimingReport(info->path, limit_rows);
        }
        if (infos.size() > limit_paths) {
            std::print("\n...");
        }
    }
}

void Tech::openDesign()
{
    std::print("\nOpening design...");
    estimate.clocks = &clocks;
    estimate.estimateDesign(design);
    estimate.printBunches();
}

void Tech::placeDesign()
{
    std::print("\nPlacing design...");
    outline.placeIOBs(estimate.data_outs, assignments);
    outline.placeInstIOBs(design.top, assignments);
    outline.optimizeOutline(estimate.data_outs);
    place.placeDesign(estimate.data_outs);
}

void Tech::routeDesign()
{
    std::print("\nRouting design...");
    route.routeDesign(estimate.data_outs);
}

void Tech::printDesign(std::string& inst_name, int limit)
{
    rtl::PrintDesign printer;
    printer.tech = this;
    printer.limit = limit;

    if (inst_name == "*") {
        for (auto& out : estimate.data_outs) {
            printer.print(out.reg);
        }
    }
    else {
        std::vector<rtl::Inst*> insts;
        std::vector<rtl::instFilter> filters;
        filters.emplace_back(rtl::instFilter{});
        filters.back().blackbox = true;
        filters.back().regexp = true;
        filters.back().name = inst_name;

        rtl::getInsts(&insts, filters, &design.top);
        for (auto& inst : insts) {
            printer.print(inst);
            break;
        }
    }
}

void Tech::loadDesign(const std::string& filename, const std::string& top_module)
{
    std::print("\nLoading design from '{}' ('{}')...", filename, top_module);
    rtl::Design& rtl = Tech::current().design;
    RtlFormat rtl_format;
    rtl_format.loadFromJson(filename, &rtl);
    rtl.build(top_module);
    rtl.printReport();
}

void Tech::writeDesignState(const std::string& filename)
{
    Json::Value root(Json::objectValue);
    root["format"] = "scalepnr-design-state";
    root["version"] = 1;

    Json::Value insts(Json::arrayValue);
    std::vector<rtl::Inst*> all_insts;
    collectInsts(design.top, all_insts);
    for (rtl::Inst* inst : all_insts) {
        Json::Value inst_json(Json::objectValue);
        inst_json["name"] = fullInstName(*inst);
        inst_json["cell"] = inst->cell_ref.peer ? inst->cell_ref->name : "";
        inst_json["type"] = inst->cell_ref.peer ? inst->cell_ref->type : "";
        if (inst->cell_ref.peer) {
            inst_json["params"] = stringMapToJson(inst->cell_ref->parameters);
            inst_json["attrs"] = stringMapToJson(inst->cell_ref->attributes);
        }
        inst_json["pos"] = inst->pos;
        inst_json["placed"] = inst->tile.peer != nullptr;
        Json::Value connections(Json::arrayValue);
        for (auto& conn_ref : inst->conns) {
            rtl::Conn& conn = conn_ref;
            if (conn.port_ref.peer && conn.port_ref->type == rtl::Port::PORT_IN) {
                connections.append(connectionToJson(conn));
            }
        }
        inst_json["connections"] = connections;
        if (inst->tile.peer) {
            inst_json["coord"] = coordToJson(inst->tile->coord);
            inst_json["tile_name"] = coordToJson(inst->tile->name);
            inst_json["tile_type"] = inst->tile->tile_type ? inst->tile->tile_type->name : "";
            inst_json["cb_type"] = inst->tile->cb_type ? inst->tile->cb_type->name : "";
            Json::Value annotation(Json::objectValue);
            annotation["cb_tile"] = cbTileName(&*inst->tile);
            annotation["resource_tile"] = resourceTileName(&*inst->tile);
            annotation["grid_coord"] = coordToJson(inst->tile->coord);
            annotation["vendor_coord"] = coordToJson(inst->tile->name);
            annotation["pos"] = inst->pos;
            annotation["connections"] = connections;
            inst_json["annotation"] = annotation;
        }

        Json::Value routes(Json::arrayValue);
        for (const auto& route : inst->wires) {
            Json::Value route_json(Json::arrayValue);
            for (const fpga::Wire& wire : route) {
                route_json.append(wireToJson(wire));
            }
            routes.append(route_json);
        }
        inst_json["routes"] = routes;
        insts.append(inst_json);
    }
    root["insts"] = insts;
    root["io_assignments"] = ioAssignmentsToJson(io_properties);

    errno = 0;
    std::ofstream out(filename);
    PNR_ASSERT(out, "cant open '{}' for writing design state: errno={} ({}) fail={} bad={}",
        filename, errno, std::strerror(errno), out.fail(), out.bad());
    Json::StreamWriterBuilder builder;
    builder["indentation"] = "  ";
    std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
    writer->write(root, &out);
}

void Tech::readDesignState(const std::string& filename)
{
    std::ifstream in(filename);
    PNR_ASSERT(in, "cant open '{}' for reading design state", filename);

    Json::Value root;
    Json::Reader reader;
    PNR_ASSERT(reader.parse(in, root), "cant parse design state '{}'", filename);
    PNR_ASSERT(root.get("format", "").asString() == "scalepnr-design-state", "unknown design state format in '{}'", filename);

    fpga::Device& device = fpga::Device::current();
    clearInstState(design.top);
    resetDeviceState(device);
    assignments.clear();
    io_properties.clear();

    for (const auto& io_json : root["io_assignments"]) {
        std::string port = io_json.get("port", "").asString();
        if (port.empty() || !io_json.isMember("properties")) {
            continue;
        }
        for (const auto& prop : io_json["properties"].getMemberNames()) {
            std::string value = io_json["properties"][prop].asString();
            io_properties[port][prop] = value;
            if (prop == "PACKAGE_PIN") {
                assignments[port] = value;
            }
        }
    }

    for (const auto& inst_json : root["insts"]) {
        std::string name = inst_json["name"].asString();
        rtl::Inst* inst = findInst(design.top, name);
        PNR_ASSERT(inst, "design state references unknown inst '{}'", name);

        inst->pos = inst_json.get("pos", -1).asInt();
        if (inst_json.get("placed", false).asBool()) {
            fpga::Coord coord = coordFromJson(inst_json["coord"]);
            fpga::Tile* tile = device.getTile(coord.x, coord.y);
            PNR_ASSERT(tile, "design state references tile out of range ({},{}) for '{}'", coord.x, coord.y, name);
            inst->tile.set(static_cast<Referable<fpga::Tile>*>(tile));
            inst->coord = coord;
        }

        for (const auto& route_json : inst_json["routes"]) {
            std::vector<fpga::Wire> route;
            for (const auto& wire_json : route_json) {
                fpga::Wire wire = wireFromJson(wire_json);
                restoreWireState(wire);
                route.push_back(std::move(wire));
            }
            inst->wires.push_back(std::move(route));
        }
    }
}

//void Tech::printDesign(std::string& inst_name, int limit)
//{
//}

void Tech::init()
{
//    design.tech = this;
    clocks.tech = this;
    timings.tech = this;
    estimate.tech = this;
    outline.tech = this;
    place.tech = this;
    route.tech = this;
    route.fpga = &fpga::Device::current();

    comb_delays = {{  // this is just for RTL timing estimation testing
        {"INV", {1, {0.05,0.05}}},
        {"LUT2", {2, {0.08,0.08}}},
        {"LUT3", {3, {0.1,0.1,0.1}}},
        {"LUT4", {4, {0.1,0.1,0.1,0.1}}},
        {"LUT5", {5, {0.1,0.1,0.1,0.1,0.1}}},
        {"LUT6", {6, {0.1,0.1,0.1,0.1,0.1,0.1}}},
        {"LUT6_2", {6, {0.1,0.1,0.1,0.1,0.1,0.1, 0.1,0.1,0.1,0.1,0.1,0.1}}},
        {"MUXF7", {3, {0.02,0.02,0.02}}},
        {"MUXF8", {3, {0.02,0.02,0.02}}},
        {"CARRY4", {10, {0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,
                         0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02, 0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,0.02,}}}
    }};

    clocked_ports = {  // yosys RTL ports
    {"FD", "C"},
    {"FDCE", "C"},
    {"FDCE_1", "C"},
    {"FDPE", "C"},
    {"FDPE_1", "C"},
    {"FDRE", "C"},
    {"FDRE_1", "C"},
    {"FDSE", "C"},
    {"FDSE_1", "C"},
    {"FDRSE", "C"},
    {"FDRSE_1", "C"},
    {"FDDRCPE", "C0"},
    {"FDDRCPE", "C1"},
    {"FDDRRSE", "C0"},
    {"FDDRRSE", "C1"},
    {"SRL16E", "CLK"},
    {"SRLC16", "CLK"},
    {"SRLC16E", "CLK"},
    {"SRL32E", "CLK"},
    {"RAM128X1D", "WCLK"},
    {"RAM128X1S", "WCLK"},
    {"RAM128X1S_1", "WCLK"},
    {"RAM16X1D", "WCLK"},
    {"RAM16X1D_1", "WCLK"},
    {"RAM16X1S", "WCLK"},
    {"RAM16X1S_1", "WCLK"},
    {"RAM16X2S", "WCLK"},
    {"RAM16X4S", "WCLK"},
    {"RAM16X8S", "WCLK"},
    {"RAM256X1D", "WCLK"},
    {"RAM256X1S", "WCLK"},
    {"RAM32M", "WCLK"},
    {"RAM32M16", "WCLK"},
    {"RAM32X16DR8", "WCLK"},
    {"RAM32X1D", "WCLK"},
    {"RAM32X1D_1", "WCLK"},
    {"RAM32X1S", "WCLK"},
    {"RAM32X1S_1", "WCLK"},
    {"RAM32X2S", "WCLK"},
    {"RAM32X4S", "WCLK"},
    {"RAM32X8S", "WCLK"},
    {"RAM512X1S", "WCLK"},
    {"RAM64M", "WCLK"},
    {"RAM64M8", "WCLK"},
    {"RAM64X1D", "WCLK"},
    {"RAM64X1D_1", "WCLK"},
    {"RAM64X1S", "WCLK"},
    {"RAM64X1S_1", "WCLK"},
    {"RAM64X2S", "WCLK"},
    {"RAM64X8SW", "WCLK"},
    {"RAMB16", "CLKA"},
    {"RAMB16", "CLKB"},
    {"RAMB16BWER", "CLKA"},
    {"RAMB16BWER", "CLKB"},
    {"RAMB16BWE_S18", "CLK"},
    {"RAMB16BWE_S18_S18", "CLKA"},
    {"RAMB16BWE_S18_S18", "CLKB"},
    {"RAMB16BWE_S18_S9", "CLKA"},
    {"RAMB16BWE_S18_S9", "CLKB"},
    {"RAMB16BWE_S36", "CLK"},
    {"RAMB16BWE_S36_S18", "CLKA"},
    {"RAMB16BWE_S36_S18", "CLKB"},
    {"RAMB16BWE_S36_S36", "CLKA"},
    {"RAMB16BWE_S36_S36", "CLKB"},
    {"RAMB16BWE_S36_S9", "CLKA"},
    {"RAMB16BWE_S36_S9", "CLKB"},
    {"RAMB16_S1", "CLK"},
    {"RAMB16_S18", "CLK"},
    {"RAMB16_S18_S18", "CLKA"},
    {"RAMB16_S18_S18", "CLKB"},
    {"RAMB16_S18_S36", "CLKA"},
    {"RAMB16_S18_S36", "CLKB"},
    {"RAMB16_S1_S1", "CLKA"},
    {"RAMB16_S1_S1", "CLKB"},
    {"RAMB16_S1_S18", "CLKA"},
    {"RAMB16_S1_S18", "CLKB"},
    {"RAMB16_S1_S2", "CLKA"},
    {"RAMB16_S1_S2", "CLKB"},
    {"RAMB16_S1_S36", "CLKA"},
    {"RAMB16_S1_S36", "CLKB"},
    {"RAMB16_S1_S4", "CLKA"},
    {"RAMB16_S1_S4", "CLKB"},
    {"RAMB16_S1_S9", "CLKA"},
    {"RAMB16_S1_S9", "CLKB"},
    {"RAMB16_S2", "CLK"},
    {"RAMB16_S2_S18", "CLKA"},
    {"RAMB16_S2_S18", "CLKB"},
    {"RAMB16_S2_S2", "CLKA"},
    {"RAMB16_S2_S2", "CLKB"},
    {"RAMB16_S2_S36", "CLKA"},
    {"RAMB16_S2_S36", "CLKB"},
    {"RAMB16_S2_S4", "CLKA"},
    {"RAMB16_S2_S4", "CLKB"},
    {"RAMB16_S2_S9", "CLKA"},
    {"RAMB16_S2_S9", "CLKB"},
    {"RAMB16_S36", "CLK"},
    {"RAMB16_S36_S36", "CLKA"},
    {"RAMB16_S36_S36", "CLKB"},
    {"RAMB16_S4", "CLK"},
    {"RAMB16_S4_S18", "CLKA"},
    {"RAMB16_S4_S18", "CLKB"},
    {"RAMB16_S4_S36", "CLKA"},
    {"RAMB16_S4_S36", "CLKB"},
    {"RAMB16_S4_S4", "CLKA"},
    {"RAMB16_S4_S4", "CLKB"},
    {"RAMB16_S4_S9", "CLKA"},
    {"RAMB16_S4_S9", "CLKB"},
    {"RAMB16_S9", "CLK"},
    {"RAMB16_S9_S18", "CLKA"},
    {"RAMB16_S9_S18", "CLKB"},
    {"RAMB16_S9_S36", "CLKA"},
    {"RAMB16_S9_S36", "CLKB"},
    {"RAMB16_S9_S9", "CLKA"},
    {"RAMB16_S9_S9", "CLKB"},
    {"RAMB18", "CLKA"},
    {"RAMB18", "CLKB"},
    {"RAMB18E1", "CLKARDCLK"},
    {"RAMB18E1", "CLKAWRCLK"},
    {"RAMB18E2", "CLKARDCLK"},
    {"RAMB18E2", "CLKAWRCLK"},
    {"RAMB18SDP", "RDCLK"},
    {"RAMB18SDP", "WRCLK"},
    {"RAMB32_S64_ECC", "RDCLK"},
    {"RAMB32_S64_ECC", "WRCLK"},
    {"RAMB36", "CLKA"},
    {"RAMB36", "CLKB"},
    {"RAMB36E1", "CLKARDCLK"},
    {"RAMB36E1", "CLKAWRCLK"},
    {"RAMB36E2", "CLKARDCLK"},
    {"RAMB36E2", "CLKAWRCLK"},
    {"RAMB36SDP", "RDCLK"},
    {"RAMB36SDP", "WRCLK"},
    {"RAMB4_S1", "CLK"},
    {"RAMB4_S16", "CLK"},
    {"RAMB4_S16_S16", "CLKA"},
    {"RAMB4_S16_S16", "CLKB"},
    {"RAMB4_S1_S1", "CLKA"},
    {"RAMB4_S1_S1", "CLKB"},
    {"RAMB4_S1_S16", "CLKA"},
    {"RAMB4_S1_S16", "CLKB"},
    {"RAMB4_S1_S2", "CLKA"},
    {"RAMB4_S1_S2", "CLKB"},
    {"RAMB4_S1_S4", "CLKA"},
    {"RAMB4_S1_S4", "CLKB"},
    {"RAMB4_S1_S8", "CLKA"},
    {"RAMB4_S1_S8", "CLKB"},
    {"RAMB4_S2", "CLK"},
    {"RAMB4_S2_S16", "CLKA"},
    {"RAMB4_S2_S16", "CLKB"},
    {"RAMB4_S2_S2", "CLKA"},
    {"RAMB4_S2_S2", "CLKB"},
    {"RAMB4_S2_S4", "CLKA"},
    {"RAMB4_S2_S4", "CLKB"},
    {"RAMB4_S2_S8", "CLKA"},
    {"RAMB4_S2_S8", "CLKB"},
    {"RAMB4_S4", "CLK"},
    {"RAMB4_S4_S16", "CLKA"},
    {"RAMB4_S4_S16", "CLKB"},
    {"RAMB4_S4_S4", "CLKA"},
    {"RAMB4_S4_S4", "CLKB"},
    {"RAMB4_S4_S8", "CLKA"},
    {"RAMB4_S4_S8", "CLKB"},
    {"RAMB4_S8", "CLK"},
    {"RAMB4_S8_S16", "CLKA"},
    {"RAMB4_S8_S16", "CLKB"},
    {"RAMB4_S8_S8", "CLKA"},
    {"RAMB4_S8_S8", "CLKB"},
    {"RAMB8BWER", "CLKARDCLK"},
    {"RAMB8BWER", "CLKAWRCLK"},
    };

    buffers_ports = {
    {"BUFG", "O"},
    {"IBUF", "O"},
    {"OBUF", "O"},
    };

}

void technology::readTechMap(std::string maptext, TechMap& map)
{
    map.clear();
    std::stringstream ss(maptext);
    std::string line;
    while (std::getline(ss, line, '\n')) {
        map.resize(map.size()+1);
        auto& lineref = map.back();
        std::string expr;
        std::stringstream ss1(line);
        while (std::getline(ss1, expr, ';')) {
            lineref.resize(lineref.size()+1);
            auto& exprref = lineref.back();
            std::string equal;
            std::stringstream ss2(expr);
            while (std::getline(ss2, equal, '=')) {
                exprref.resize(exprref.size()+1);
                auto& equalref = exprref.back();
                std::string token;
                std::stringstream ss3(equal);
                while (std::getline(ss3, token, ':')) {
                    equalref.resize(equalref.size()+1);
                    auto& tokenref = equalref.back();
                    std::string part;
                    std::stringstream ss4(token);
                    while (std::getline(ss4, part, ',')) {
                        tokenref.emplace_back(part);
//                        std::print("\nemplacing {} {} {} {} {}", line, expr, equal, token, part);
                    }
                }
            }
        }
    }
}
