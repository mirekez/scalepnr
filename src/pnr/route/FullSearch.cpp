#include "FullSearch.h"
#include "Docking.h"
#include <bit>
#include <numeric>

#include <algorithm>

namespace pnr
{
using namespace fpga;
using TerminalEntry = CBType::TerminalEntry;

// Return the first alias for Docking's route fragments and route tests;
// delegates the lookup to FullSearch::connNames.
const CBConnName *FullSearch::connName(const CBType &type,
                                       CBNodeNameType from_type, int from_value,
                                       CBNodeNameType to_type, int to_value)
{
    const std::vector<CBConnName> *conns =
        connNames(type, from_type, from_value, to_type, to_value);
    if (!conns || conns->empty())
    {
        return nullptr;
    }

    return &conns->front();
}

// Return aliases to Tech's selectConcreteConn, RouteDesign's route validation,
// and FullSearch::connName.
const std::vector<CBConnName> *
FullSearch::connNames(const CBType &type, CBNodeNameType from_type,
                      int from_value, CBNodeNameType to_type, int to_value)
{
    if (from_value < 0 || from_value >= CB_MAX_NODES || to_value < 0 ||
        to_value >= CB_MAX_NODES)
    {
        return nullptr;
    }

    CBConnNameKey key{
        static_cast<uint16_t>(from_type), static_cast<uint16_t>(from_value),
        static_cast<uint16_t>(to_type), static_cast<uint16_t>(to_value)};
    auto it = type.conn_names.find(key);
    return it == type.conn_names.end() ? nullptr : &it->second;
}

// Return reachability from rebuildOutgoingSrcs to RouteDesign and Docking;
// FullSearch::orderedSrcNodes also uses it to filter choices.
const std::vector<uint16_t> *FullSearch::srcNodes(const CBType &type,
                                                  CBNodeNameType from_type,
                                                  int from_value)
{
    if (from_value < 0 || from_value >= CB_MAX_NODES)
    {
        return nullptr;
    }

    CBNodeNameKey key{static_cast<uint16_t>(from_type),
                      static_cast<uint16_t>(from_value)};
    auto it = type.outgoing_srcs.find(key);
    return it == type.outgoing_srcs.end() ? nullptr : &it->second;
}

// Enumerate the mask selector's complete order for bounded Docking callers.
const std::vector<uint16_t> &FullSearch::orderedSrcNodes(CBType &type,
                                                         Coord angle)
{
    const uint16_t key = CBType::directionIndex(angle);
    auto known = type.ordered_srcs_by_target.find(key);
    if (known != type.ordered_srcs_by_target.end())
        return known->second;
    type.ensureDerivedMasks();
    NodeMask valid;
    for (const auto &[src, entries] : type.dst_by_src.values)
        if (!entries.empty())
            valid.setBit(src);
    for (const auto &[node, sources] : type.outgoing_srcs)
        for (uint16_t src : sources)
            valid.setBit(src);
    auto &ordered = type.ordered_srcs_by_target[key];
    CBType::FastSearchCursor cursor;
    for (int src; (src = type.fastSearchNextHop(angle, cursor, valid)) >= 0;)
        ordered.push_back(static_cast<uint16_t>(src));
    return ordered;
}

// Filter the complete numeric order for Docking's typed-node enumeration.
const std::vector<uint16_t> &
FullSearch::orderedSrcNodes(CBType &type, CBNodeNameType from_type,
                            int from_value, Coord angle)
{
    static const std::vector<uint16_t> empty;
    if (from_value < 0 || from_value >= CB_MAX_NODES)
        return empty;
    type.ensureDerivedMasks();
    const uint32_t key = (uint32_t(CBType::directionIndex(angle)) << 15) |
                         (uint32_t(from_type) << 12) | uint32_t(from_value);
    auto known = type.ordered_node_srcs_by_target.find(key);
    if (known != type.ordered_node_srcs_by_target.end())
        return known->second;
    auto &filtered = type.ordered_node_srcs_by_target[key];
    const auto *reachable = srcNodes(type, from_type, from_value);
    if (!reachable)
        return filtered;
    NodeMask allowed;
    for (uint16_t src : *reachable)
        allowed.setBit(src);
    for (uint16_t src : orderedSrcNodes(type, angle))
        if (allowed.testBit(src))
            filtered.push_back(src);
    return filtered;
}

// Enumerate direct and short JOINT ingress paths for RouteDesign's
// endpoint grounding and Docking's forward trace.
const std::vector<CBType::TerminalEntry> &
FullSearch::terminalEntries(CBType &type, int local)
{
    static const std::vector<TerminalEntry> empty;
    if (local < 0 || local >= CB_MAX_NODES)
    {
        return empty;
    }

    type.ensureDerivedMasks();
    auto known =
        type.terminal_entries_by_local.find(static_cast<uint16_t>(local));
    if (known != type.terminal_entries_by_local.end() &&
        (!known->second.empty() ||
         type.dsts_reaching_local.get(local).jump == NodeMask{}))
    {
        return known->second;
    }

    std::vector<TerminalEntry> &paths =
        type.terminal_entries_by_local[static_cast<uint16_t>(local)];
    paths.clear();
    auto add = [&](int dst, int joint, int joint2)
    {
        auto same = [&](const TerminalEntry &path)
        {
            return path.dst == dst && path.joint == joint &&
                   path.joint2 == joint2;
        };

        if (std::find_if(paths.begin(), paths.end(), same) == paths.end())
        {
            paths.push_back(TerminalEntry{static_cast<uint16_t>(dst),
                                          static_cast<int16_t>(joint),
                                          static_cast<int16_t>(joint2)});
        }

    };

    NodeMask local_bit = NodeMask{0, 1} << local;
    NodeMask joints_to_local = type.local_reachable_joints.get(local).joint;
    type.dsts_reaching_local.get(local).jump.for_each_set_bit(
        [&](int dst)
        {
            if ((type.dst_local.get(dst).local & local_bit) != NodeMask{})
            {
                add(dst, -1, -1);
            }

            (type.dst_joint.get(dst).joint & joints_to_local)
                .for_each_set_bit(
                    [&](int joint)
                    {
                        add(dst, joint, -1);
                        return false;
                    });
            type.dst_joint.get(dst).joint.for_each_set_bit(
                [&](int first_joint)
                {
                    (type.joint_joint.get(first_joint).joint & joints_to_local)
                        .for_each_set_bit(
                            [&](int second_joint)
                            {
                                add(dst, second_joint, first_joint);
                                return false;
                            });
                    return false;
                });
            return false;
        });
    return paths;
}

// Sparse rank lookup keeps direction groups compact and access constant-time.
const BackwardHopChoices::Range *
BackwardHopChoices::range(uint16_t direction) const
{
    const unsigned page = direction / 16, bit = direction % 16;
    const uint16_t present = directions[page];
    if (!(present & (uint16_t(1) << bit)))
        return nullptr;
    return &ranges[page_offsets[page] +
                   std::popcount(
                       uint16_t(present & ((uint16_t(1) << bit) - 1)))];
}

// Build immutable physical-hop groups once, outside recursive trace calls.
BackwardHopChoices
FullSearch::backwardHopChoices(std::span<const BackwardResolveSource> sources,
                               Coord target)
{
    BackwardHopChoices result;
    result.sources = sources;
    for (uint32_t at = 0; at < sources.size(); ++at)
        if (sources[at].tile && sources[at].tile->cb_type)
            result.source_indices.push_back(at);
    const auto direction = [&](uint32_t at)
    {
        return CBType::hopDirectionIndex(sources[at].tile->coord - target);
    };

    std::stable_sort(result.source_indices.begin(), result.source_indices.end(),
                     [&](uint32_t left, uint32_t right)
                     {
                         return direction(left) < direction(right);
                     });
    for (uint32_t begin = 0; begin < result.source_indices.size();)
    {
        const uint16_t bucket = direction(result.source_indices[begin]);
        uint32_t end = begin + 1;
        while (end < result.source_indices.size() &&
               direction(result.source_indices[end]) == bucket)
            ++end;
        result.directions[bucket / 16] |= uint16_t(1) << (bucket % 16);
        result.ranges.push_back({begin, end});
        begin = end;
    }

    uint32_t offset = 0;
    for (size_t page = 0; page < result.directions.size(); ++page)
    {
        result.page_offsets[page] = offset;
        offset += std::popcount(result.directions[page]);
    }

    return result;
}

// Invert only direct numeric arcs; backward hops do not enumerate CB paths.
BackwardCrossbarMasks FullSearch::backwardMasks(const CBType &type)
{
    BackwardCrossbarMasks result;
    for (const auto &[dst, row] : type.dst_src.values)
        row.jump.for_each_set_bit(
            [&](int src)
            {
                result.source_dsts[src].jump.setBit(dst);
                return false;
            });
    for (const auto &[local, row] : type.local_src.values)
        row.jump.for_each_set_bit(
            [&](int src)
            {
                result.source_locals[src].local.setBit(local);
                return false;
            });
    for (const auto &[joint, row] : type.joint_src.values)
        row.jump.for_each_set_bit(
            [&](int src)
            {
                result.source_joints[src].joint.setBit(joint);
                return false;
            });
    for (const auto &[dst, row] : type.dst_local.values)
        row.local.for_each_set_bit(
            [&](int local)
            {
                result.local_dsts[local].jump.setBit(dst);
                return false;
            });
    for (const auto &[dst, row] : type.dst_joint.values)
        row.joint.for_each_set_bit(
            [&](int joint)
            {
                result.joint_dsts[joint].jump.setBit(dst);
                return false;
            });
    return result;
}

} // namespace pnr
