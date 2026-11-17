#pragma once

#include "Device.h"

#include <string>
#include <vector>

namespace pnr
{

// Placement invalidates both reservation maps; a remote LOCAL arrival may
// reach a tile before any DST placement probe has initialized those maps.
inline bool inputLocalReserved(fpga::Tile &tile, int node)
{
    if (!tile.input_joint_reservations_initialized)
        fpga::packedInputJointReservations(tile);
    return tile.input_local_reservations.contains(node);
}

inline bool canUsePhysicalJump(const fpga::CBState &cb, int dst, int src,
                               int joint, int joint2,
                               bool allow_existing_dst = false)
{
    if (cb.src.jump.testBit(src))
    {
        return false;
    }

    if (!allow_existing_dst && cb.dst.jump.testBit(dst))
    {
        return false;
    }

    if ((joint >= 0 && cb.joint.jump.testBit(joint)) ||
        (joint2 >= 0 && cb.joint.jump.testBit(joint2)))
    {
        return false;
    }

    return true;
}

inline bool canLeaseJump(const fpga::CBState &cb, int dst, int src, int joint,
                         int joint2, bool allow_existing_dst = false)
{
    return !cb.src_deadend.jump.testBit(src) &&
           canUsePhysicalJump(cb, dst, src, joint, joint2, allow_existing_dst);
}

inline int selectJointToSrc(const fpga::Tile &tile,
                            fpga::CBNodeNameType from_type, int from_node,
                            int src, int *joint2, bool free_joints_only = false)
{
    if (!tile.cb_type)
    {
        return -2;
    }

    tile.cb_type->ensureDerivedMasks();
    int joint = -1;
    if (joint2)
    {
        *joint2 = -1;
    }

    // Free placement growth must exhaust alternate joints before asking for a
    // transit cut; the topology-only selector can return an occupied first
    // joint.
    if (free_joints_only &&
        (from_type == fpga::CB_NODE_DST || from_type == fpga::CB_NODE_LOCAL))
    {
        const auto &direct = from_type == fpga::CB_NODE_DST
                                 ? tile.cb_type->dst_src.get(from_node).jump
                                 : tile.cb_type->local_src.get(from_node).jump;
        if (direct.testBit(src))
            return -1;
        const NodeMask starts =
            (from_type == fpga::CB_NODE_DST
                 ? tile.cb_type->dst_joint.get(from_node).joint
                 : tile.cb_type->local_joint.get(from_node).joint) &
            ~tile.cb.joint.jump;
        const NodeMask ends =
            tile.cb_type->src_reachable_joints.get(src).joint &
            ~tile.cb.joint.jump;
        joint = (starts & ends).firstSetBit();
        if (joint >= 0)
            return joint;
        starts.for_each_set_bit(
            [&](int first)
            {
                joint = (tile.cb_type->joint_joint.get(first).joint & ends)
                            .firstSetBit();
                if (joint < 0)
                    return false;
                if (joint2)
                    *joint2 = first;
                return true;
            });
        return joint >= 0 ? joint : -2;
    }

    if (from_type == fpga::CB_NODE_DST)
    {
        return tile.cb_type->canJump(from_node, src, src, joint, joint2) ? joint
                                                                         : -2;
    }

    if (from_type == fpga::CB_NODE_LOCAL)
    {
        return tile.cb_type->canOut(from_node, src, src, joint, joint2) ? joint
                                                                        : -2;
    }

    if (tile.cb_type->joint_src.get(from_node).jump.testBit(src))
    {
        return -1;
    }

    NodeMask joints_to_src = tile.cb_type->src_reachable_joints.get(src).joint;
    joint = (tile.cb_type->joint_joint.get(from_node).joint & joints_to_src)
                .firstSetBit();
    return joint >= 0 ? joint : -2;
}

inline fpga::InternedString srcWireName(const fpga::Tile &tile,
                                        fpga::CBNodeNameType from_type,
                                        int from_node, int src, int joint,
                                        const std::string &incoming_wire)
{
    if (!tile.cb_type)
    {
        return {};
    }

    if (joint >= 0)
    {
        if (const fpga::CBConnName *conn = tile.cb_type->connName(
                fpga::CB_NODE_JOINT, joint, fpga::CB_NODE_SRC, src))
        {
            return conn->to;
        }

        if (const fpga::CBConnName *conn = tile.cb_type->connName(
                fpga::CB_NODE_SRC, src, fpga::CB_NODE_JOINT, joint))
        {
            return conn->from;
        }

    }

    if (const fpga::CBConnName *conn = tile.cb_type->connName(
            from_type, from_node, fpga::CB_NODE_SRC, src))
    {
        if (!incoming_wire.empty() && conn->from != incoming_wire)
        {
            return {};
        }

        return conn->to;
    }

    if (!incoming_wire.empty())
    {
        return {};
    }

    if (const std::string *name =
            tile.cb_type->nodeName(fpga::CB_NODE_SRC, src))
    {
        return *name;
    }

    return {};
}

inline fpga::InternedString fromWireName(const fpga::Tile &tile,
                                         fpga::CBNodeNameType from_type,
                                         int from_node, int src, int joint,
                                         const std::string &src_wire)
{
    if (!tile.cb_type)
    {
        return {};
    }

    if (joint >= 0)
    {
        if (const fpga::CBConnName *conn = tile.cb_type->connName(
                from_type, from_node, fpga::CB_NODE_JOINT, joint))
        {
            return conn->from;
        }

    }

    if (const fpga::CBConnName *conn = tile.cb_type->connName(
            from_type, from_node, fpga::CB_NODE_SRC, src))
    {
        if (!src_wire.empty() && conn->to != src_wire)
        {
            return {};
        }

        return conn->from;
    }

    if (const std::string *name = tile.cb_type->nodeName(from_type, from_node))
    {
        return *name;
    }

    return {};
}

inline void markGraphArrivals(std::vector<fpga::Wire> &route)
{
    for (size_t i = 1; i < route.size(); ++i)
    {
        auto &hop = route[i - 1];
        const auto &next = route[i];
        if (hop.type == fpga::Wire::WIRE_CROSSBAR && hop.jump >= 0 &&
            next.type == fpga::Wire::WIRE_ROUTE_EDGE &&
            next.from_node_type == fpga::CB_NODE_DST &&
            next.from_node == hop.dst && next.from == hop.to)
        {
            hop.owns_landing = true;
        }

    }

}

} // namespace pnr
