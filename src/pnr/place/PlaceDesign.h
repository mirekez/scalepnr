#pragma once

#include "Design.h"
#include "Device.h"
#include "RegBunch.h"
#include "TileSet.h"
#include "Inst.h"
#include "Clocks.h"
#include "png_draw.h"

#include <vector>
#include <string>
#include <chrono>
#include <array>

namespace technology
{
    struct Tech;
}

namespace pnr
{

struct PlaceDesign
{
    technology::Tech* tech = nullptr;
    fpga::Device* fpga = nullptr;

    static constexpr const int mesh_width = 10;
    static constexpr const int mesh_height = 10;

    int fpga_width = 0;
    int fpga_height = 0;

    float aspect_x = 0;
    float aspect_y = 0;

    float image_zoom = 4;

    std::vector<Referable<fpga::Tile>>* tile_grid = nullptr;

    uint64_t travers_mark = 0;
    uint64_t place_calls = 0;
    uint64_t place_tile_trials = 0;
    uint64_t place_commits = 0;
    std::chrono::steady_clock::time_point place_started;
    std::chrono::steady_clock::time_point place_next_report;
    static constexpr int place_region_count = mesh_width * mesh_height;
    using CandidateList = std::vector<uint32_t>;
    std::array<std::array<CandidateList, place_region_count>, fpga::ELEMENT_TYPE_COUNT> place_candidates;
    std::array<std::array<size_t, place_region_count>, fpga::ELEMENT_TYPE_COUNT> place_candidate_cursor{};
    void preparePlaceCandidates();
    int tryAddNear(rtl::Inst& inst, fpga::ElementType type, const Coord& origin);
    void recursivePackBunch(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    void placeDesign(std::list<Referable<RegBunch>>& bunch_list);
    void recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int depth = 0);
    png_draw image;
};

}
