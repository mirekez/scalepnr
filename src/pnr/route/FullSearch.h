#pragma once

#include "Crossbar.h"
#include <span>

namespace fpga
{
struct Tile;
}

namespace pnr
{

struct BackwardResolveSource;

// Immutable numeric direction groups built with Docking's reverse index.
// A recursive hop borrows this storage and keeps only scalar cursors.
struct BackwardHopChoices
{
    struct Range
    {
        uint32_t begin = 0;
        uint32_t end = 0;
    };

    std::array<uint16_t, 16> directions{};
    std::array<uint32_t, 16> page_offsets{};
    std::vector<Range> ranges;
    std::vector<uint32_t> source_indices;
    std::span<const BackwardResolveSource> sources;

    const Range *range(uint16_t direction) const;
};

// Direct inverse masks for one immutable CB type, built outside tracing.
struct BackwardCrossbarMasks
{
    fpga::CBType::StateTable<fpga::CBJumpState> source_dsts;
    fpga::CBType::StateTable<fpga::CBLocalState> source_locals;
    fpga::CBType::StateTable<fpga::CBJointState> source_joints;
    fpga::CBType::StateTable<fpga::CBJumpState> local_dsts;
    fpga::CBType::StateTable<fpga::CBJumpState> joint_dsts;
};

// Vector enumeration belongs to bounded full search, exposed only by Docking.
// Crossbar loading retains ownership of the immutable connectivity tables.
struct FullSearch
{
    static const fpga::CBConnName *
    connName(const fpga::CBType &type, fpga::CBNodeNameType from_type,
             int from_value, fpga::CBNodeNameType to_type, int to_value);
    static const std::vector<fpga::CBConnName> *
    connNames(const fpga::CBType &type, fpga::CBNodeNameType from_type,
              int from_value, fpga::CBNodeNameType to_type, int to_value);
    static const std::vector<uint16_t> *srcNodes(const fpga::CBType &type,
                                                 fpga::CBNodeNameType from_type,
                                                 int from_value);
    static const std::vector<uint16_t> &orderedSrcNodes(fpga::CBType &type,
                                                        fpga::Coord angle);
    static const std::vector<uint16_t> &
    orderedSrcNodes(fpga::CBType &type, fpga::CBNodeNameType from_type,
                    int from_value, fpga::Coord angle);
    static const std::vector<fpga::CBType::TerminalEntry> &
    terminalEntries(fpga::CBType &type, int local);
    static BackwardHopChoices
    backwardHopChoices(std::span<const BackwardResolveSource> sources,
                       fpga::Coord target);
    static BackwardCrossbarMasks backwardMasks(const fpga::CBType &type);
};

} // namespace pnr
