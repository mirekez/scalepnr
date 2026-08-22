#include "OutlineDesign.h"
#include "OutlineGrid.h"
#include "Device.h"
#include "Tech.h"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <chrono>
#include <deque>
#include <limits>
#include <math.h>
#include <unordered_set>

using namespace pnr;

namespace {

size_t package_assignment_count = 0;
size_t package_assignment_fallback_count = 0;
std::chrono::steady_clock::time_point package_assignment_start;

bool siteCoordinate(const std::string& name, int& x, int& y)
{
    size_t x_pos = name.rfind('X');
    size_t y_pos = x_pos == std::string::npos ? std::string::npos : name.find('Y', x_pos + 1);
    if (x_pos == std::string::npos || y_pos == std::string::npos) {
        return false;
    }
    const char* begin = name.data();
    const char* end = begin + name.size();
    auto x_result = std::from_chars(begin + x_pos + 1, begin + y_pos, x);
    auto y_result = std::from_chars(begin + y_pos + 1, end, y);
    return x_result.ec == std::errc{} && x_result.ptr == begin + y_pos
        && y_result.ec == std::errc{} && y_result.ptr == end;
}

}

int pnr::packageSitePosition(const fpga::Tile& tile, const std::string& site)
{
    auto physical = std::find(tile.sites.begin(), tile.sites.end(), site);
    if (physical == tile.sites.end()) {
        return -1;
    }
    if (tile.tile_type && !tile.tile_type->sites.empty()) {
        int physical_x = 0;
        int physical_y = 0;
        int physical_min_x = 0;
        int physical_min_y = 0;
        int model_min_x = 0;
        int model_min_y = 0;
        bool have_physical_min = false;
        bool have_model_min = false;
        if (siteCoordinate(site, physical_x, physical_y)) {
            for (const std::string& candidate : tile.sites) {
                int x = 0;
                int y = 0;
                if (!siteCoordinate(candidate, x, y)) {
                    continue;
                }
                if (!have_physical_min || x < physical_min_x) {
                    physical_min_x = x;
                }
                if (!have_physical_min || y < physical_min_y) {
                    physical_min_y = y;
                }
                have_physical_min = true;
            }
            for (const fpga::SiteModel& candidate : tile.tile_type->sites) {
                int x = 0;
                int y = 0;
                if (!siteCoordinate(candidate.name, x, y)) {
                    continue;
                }
                if (!have_model_min || x < model_min_x) {
                    model_min_x = x;
                }
                if (!have_model_min || y < model_min_y) {
                    model_min_y = y;
                }
                have_model_min = true;
            }
            if (have_physical_min && have_model_min) {
                int relative_x = physical_x - physical_min_x;
                int relative_y = physical_y - physical_min_y;
                for (const fpga::SiteModel& candidate : tile.tile_type->sites) {
                    int x = 0;
                    int y = 0;
                    if (siteCoordinate(candidate.name, x, y)
                        && x - model_min_x == relative_x && y - model_min_y == relative_y) {
                        return candidate.pos;
                    }
                }
            }
        }

        size_t index = static_cast<size_t>(physical - tile.sites.begin());
        if (index < tile.tile_type->sites.size()) {
            return tile.tile_type->sites[index].pos;
        }
    }
    return static_cast<int>(physical - tile.sites.begin());
}

namespace {

bool assignToPackagePin(rtl::Inst& inst, const std::string& port_name,
                        std::map<std::string,std::string>& assignments,
                        const std::unordered_map<std::string, fpga::Pin*>& package_pins,
                        const std::unordered_map<std::string, fpga::Tile*>& package_tiles)
{
    auto assignment = assignments.find(port_name);
    if (assignment == assignments.end()) {
        return false;
    }

    auto pin_it = package_pins.find(assignment->second);
    if (pin_it != package_pins.end()) {
        fpga::Pin& pin = *pin_it->second;
        auto tile_it = package_tiles.find(pin.tile);
        fpga::Tile* tile = tile_it == package_tiles.end() ? nullptr : tile_it->second;
        if (!tile) {
            PNR_ASSERT(false, "cant find tile '{}' for assigned pin '{}' on port '{}'", pin.tile, pin.name, port_name);
            return false;
        }

        if (!inst.tile.peer) {
            tile->assign(&inst);
        }
        inst.coord = tile->coord;
        inst.pos = pnr::packageSitePosition(*tile, pin.site);
        PNR_ASSERT(inst.pos >= 0, "package pin '{}' references unknown site '{}' in tile '{}'",
                   pin.name, pin.site, pin.tile);
        PNR_LOG1("OUTL", "placeIOBs, assigned '{}' to pin '{}' tile '{}' grid ({},{}) pos {}",
            inst.makeName(), pin.name, pin.tile, tile->coord.x, tile->coord.y, inst.pos);
        ++package_assignment_count;
        if (package_assignment_count%4096 == 0) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - package_assignment_start).count();
            std::print("\nOUTLINE_IO_PROGRESS assigned={} fallback={} elapsed_s={:.3f}",
                       package_assignment_count, package_assignment_fallback_count,
                       elapsed);
            fflush(stdout);
        }
        return true;
    }

    PNR_ASSERT(false, "cant find assigned package pin '{}' for port '{}'", assignment->second, port_name);
    return false;
}

void fixOutlineAtAssignedTile(rtl::Inst& inst, RegBunch* bunch = nullptr)
{
    if (!inst.tile.peer) {
        return;
    }
    const auto& device = fpga::Device::current();
    float x_extent = static_cast<float>(OutlineDesign::mesh_width) - 0.05F;
    float y_extent = static_cast<float>(OutlineDesign::mesh_height) - 0.05F;
    float x_divisor = static_cast<float>(std::max(1, device.size_width - 1));
    float y_divisor = static_cast<float>(std::max(1, device.size_height - 1));
    inst.outline.x = inst.tile->coord.x*x_extent/x_divisor;
    inst.outline.y = inst.tile->coord.y*y_extent/y_divisor;
    inst.outline.fixed = true;
    if (bunch) {
        bunch->x = inst.outline.x;
        bunch->y = inst.outline.y;
        bunch->fixed = true;
    }
}

std::string portNameFromIopadInstName(const std::string& inst_name, const std::map<std::string,std::string>& assignments)
{
    for (const auto& [port, pin] : assignments) {
        std::string scalar = port;
        std::string indexed = port;
        size_t open = indexed.find('[');
        size_t close = indexed.find(']', open == std::string::npos ? 0 : open);
        if (open != std::string::npos && close != std::string::npos) {
            std::string index = indexed.substr(open + 1, close - open - 1);
            if (index == "0") {
                scalar.erase(open);
            }
            indexed.replace(open, close - open + 1, "_" + index);
        }
        if (inst_name.ends_with("." + scalar) || inst_name.ends_with("." + indexed)) {
            return port;
        }
    }
    return {};
}

std::string portNameFromIopadConnections(
    rtl::Inst& inst, const std::map<std::string,std::string>& assignments)
{
    auto assignedGlobalPort = [&assignments](rtl::Conn* connection) {
        std::string port_name;
        if (!connection || !connection->port_ref.peer
            || !connection->port_ref->is_global) {
            return port_name;
        }
        port_name = connection->port_ref->name;
        if (connection->port_ref->bitnum != -1) {
            port_name += "[" + std::to_string(connection->port_ref->bitnum) + "]";
        }
        return assignments.contains(port_name) ? port_name : std::string{};
    };

    for (auto& connection : inst.conns) {
        if (!connection.port_ref.peer) {
            continue;
        }
        if (std::string port_name = assignedGlobalPort(connection.follow());
            !port_name.empty()) {
            return port_name;
        }
        for (auto* peer_ref : connection.getPeers()) {
            auto& peer = rtl::Conn::fromBase(*peer_ref);
            if (std::string port_name = assignedGlobalPort(&peer);
                !port_name.empty()) {
                return port_name;
            }
        }
    }
    return {};
}

int countReachableCells(rtl::Inst& inst, RegBunch* bunch, uint64_t mark)
{
    if (inst.mark == mark) {
        return 0;
    }
    inst.mark = mark;

    int cells = inst.cell_ref.peer && inst.cell_ref->module_ref.peer && inst.cell_ref->module_ref->is_blackbox ? 1 : 0;

    for (auto& conn : inst.conns) {
        if (!conn.port_ref.peer || conn.port_ref->type != rtl::Port::PORT_IN) {
            continue;
        }
        rtl::Conn* driver_conn = conn.follow();
        if (!driver_conn || driver_conn->port_ref->is_global
            || !driver_conn->inst_ref.peer) {
            continue;
        }
        cells += countReachableCells(*driver_conn->inst_ref, nullptr, mark);
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            if (subbunch.reg) {
                cells += countReachableCells(*subbunch.reg, &subbunch, mark);
            }
        }
    }

    return cells;
}

int countDesignCells(std::list<Referable<RegBunch>>& bunch_list)
{
    uint64_t mark = rtl::Inst::genMark();
    int cells = 0;
    for (auto& bunch : bunch_list) {
        if (bunch.reg) {
            cells += countReachableCells(*bunch.reg, &bunch, mark);
        }
    }
    return cells;
}

bool hasFixedBunch(const RegBunch& bunch)
{
    if (bunch.fixed || (bunch.reg && bunch.reg->outline.fixed)) {
        return true;
    }
    return std::ranges::any_of(
        bunch.sub_bunches, [](const Referable<RegBunch>& subbunch) {
            return hasFixedBunch(subbunch);
        });
}

}

void OutlineDesign::preparePackageLookup()
{
    auto& device = fpga::Device::current();
    package_assignment_count = 0;
    package_assignment_fallback_count = 0;
    package_assignment_start = std::chrono::steady_clock::now();
    package_pins.clear();
    package_pins.reserve(device.pins.size());
    for (fpga::Pin& pin : device.pins) {
        package_pins[pin.name] = &pin;
    }
    package_tiles.clear();
    package_tiles.reserve(device.tile_grid.size());
    for (auto& tile : device.tile_grid) {
        if (!tile.tile_type) {
            continue;
        }
        std::string name = tile.tile_type->name + "_X"
            + std::to_string(tile.name.x) + "Y" + std::to_string(tile.name.y);
        package_tiles[std::move(name)] = &tile;
    }
}

void OutlineDesign::placeIOBs(std::list<Referable<RegBunch>>& bunch_list, std::map<std::string,std::string>& assignments, int depth)
{
    if (depth == 0) {
        preparePackageLookup();
    }
    for (auto& bunch : bunch_list) {
        PNR_LOG3_("OUTL", depth, "placeIOBs, bunch: {} ({})", bunch.reg->makeName(), bunch.reg->cell_ref->type);

        if (bunch.reg->cell_ref->type == "IBUF" || bunch.reg->cell_ref->type == "OBUF") {

            for (auto& conn : std::ranges::views::reverse(bunch.reg->conns)) {
                rtl::Conn* curr = &conn;
                if (bunch.reg->cell_ref->type == "IBUF" && curr->port_ref->type == rtl::Port::PORT_IN) {
                    curr = curr->follow();
                    if (!curr /*|| !curr->inst_ref->cell_ref->module_ref->is_blackbox*/ || curr->port_ref->is_global) {  // after BUFs (can be something?)
                        continue;
                    }

                    auto port_name = curr->port_ref->name + (curr->port_ref->bitnum != -1 ? ("[" + std::to_string(curr->port_ref->bitnum) + "]") : "");

                    PNR_LOG2("OUTL", "placeIOBs, looking for assignments for '{}': '{}'", bunch.reg->makeName(), port_name);

                    if (assignToPackagePin(*bunch.reg, port_name, assignments,
                                           package_pins, package_tiles)) {
                        fixOutlineAtAssignedTile(*bunch.reg, &bunch);
                    }
                }
            }

            if (!bunch.reg->tile.peer) {
                std::string port_name = portNameFromIopadConnections(
                    *bunch.reg, assignments);
                if (port_name.empty()) {
                    ++package_assignment_fallback_count;
                    port_name = portNameFromIopadInstName(
                        bunch.reg->makeName(), assignments);
                }
                if (!port_name.empty()) {
                    PNR_LOG2("OUTL", "placeIOBs, looking for assignments for '{}': '{}'", bunch.reg->makeName(), port_name);
                    assignToPackagePin(*bunch.reg, port_name, assignments,
                                       package_pins, package_tiles);
                }
            }
            fixOutlineAtAssignedTile(*bunch.reg, &bunch);
        }

        placeIOBs(bunch.sub_bunches, assignments, depth + 1);
    }
}

void OutlineDesign::placeInstIOBs(rtl::Inst& inst, std::map<std::string,std::string>& assignments, int depth)
{
    if (depth == 0 && package_pins.empty() && package_tiles.empty()) {
        preparePackageLookup();
    }
    if ((inst.cell_ref->type == "IBUF" || inst.cell_ref->type == "OBUF") && !inst.tile.peer) {
        std::string port_name = portNameFromIopadConnections(inst, assignments);
        if (port_name.empty()) {
            ++package_assignment_fallback_count;
            port_name = portNameFromIopadInstName(inst.makeName(), assignments);
        }
        if (!port_name.empty()) {
            PNR_LOG2_("OUTL", depth, "placeInstIOBs, looking for assignments for '{}': '{}'", inst.makeName(), port_name);
            assignToPackagePin(inst, port_name, assignments,
                               package_pins, package_tiles);
            fixOutlineAtAssignedTile(inst, inst.bunch_ref.peer);
        }
    }

    for (auto& sub_inst : inst.insts) {
        placeInstIOBs(sub_inst, assignments, depth + 1);
    }
}

void OutlineDesign::attractBunch(RegBunch& bunch, int x, int y, int depth,
                                 RegBunch* exclude, bool propagate)
{
    PNR_LOG3_("OUTL", depth, "attractBunch, bunch: {} ({}), bunch.x: {}, bunch.y: {}, x: {}, y: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type, bunch.x, bunch.y, x, y);

    if (propagate) {
        if (bunch.parent && bunch.parent != exclude && ((int)round(bunch.parent->x) != (int)round(bunch.x) || (int)round(bunch.parent->y) != (int)round(bunch.y))) {
            attractBunch(*bunch.parent, x, y, depth+1, &bunch);
        }

        for (auto& subbunch : bunch.sub_bunches) {
//    for (auto& link : bunch.uplinks) {
//        auto& subbunch = *link.conn->inst_ref->bunch_ref.peer;
            if (&subbunch != exclude && ((int)round(subbunch.x) != (int)round(bunch.x) || (int)round(subbunch.y) != (int)round(bunch.y))) {
                attractBunch(subbunch, x, y, depth+1, &bunch);
            }
//    }
        }
    }

    if (bunch.fixed) {
        return;
    }

    float step = travers_mark == 0 ? 0.1 : ( bunch.mark != travers_mark ? 0.05 : 0 );
    if (avg_comb_in_bunch != 0) step = 0.01;

    int rx = round(x);
    int ry = round(y);
    int rbx = round(bunch.x);
    int rby = round(bunch.y);

    if (rbx < rx - 1 || rbx > rx + 1 ) {
        bunch.x += (x > bunch.x ? step : -step);
        PNR_LOG3("OUTL", " bunch => x: {}", bunch.x);
    }

    if (rby < ry - 1 || rby > ry + 1 ) {
        bunch.y += (y > bunch.y ? step : -step);
        PNR_LOG3("OUTL", " bunch => y: {}", bunch.y);
    }

    bunch.mark = travers_mark;
}

uint64_t OutlineDesign::recurseSecondaryLinks(RegBunch& bunch, int depth)
{
    uint64_t diffs = 0;
    int timing_uplinks = 0;
    int timing_uplinks_placed = 0;
    uint64_t sum_distance = 0;

    for (auto& subbunch : bunch.sub_bunches) {
        sum_distance += recurseSecondaryLinks(subbunch, depth + 1);
    }

    if (avg_comb_in_bunch != 0 && bunch.size_comb_own > avg_comb_in_bunch/2) {
        attractBunch(bunch, mesh_width/2, mesh_height/2, 0, &bunch);
    }

    for (auto& link : bunch.uplinks) {
        if (link.conn && link.conn->inst_ref.peer
            && link.conn->inst_ref->bunch_ref.peer) {
            ++timing_uplinks;
            RegBunch& linked = *link.conn->inst_ref->bunch_ref.peer;
            if (linked.x != -1) {
                ++timing_uplinks_placed;
                int x_dist = bunch.x - link.conn->inst_ref->bunch_ref->x;
                int y_dist = bunch.y - link.conn->inst_ref->bunch_ref->y;
                if ((x_dist >= 0 ? x_dist : -x_dist) + (y_dist >= 0 ? y_dist : -y_dist) > 1) {
                    int strength = 1;
                    if (bunch.clk_ref.peer && bunch.clk_ref->period_ns > 0) {
                        double ratio = link.delay/bunch.clk_ref->period_ns;
                        if (ratio >= 0.75) ++strength;
                        if (ratio >= 0.95 || link.deficit >= 0) ++strength;
                    }
                    for (int pull = 0; pull < strength; ++pull) {
                        bool propagate = link.secondary;
                        attractBunch(linked, bunch.x, bunch.y, 0, &bunch,
                                     propagate);
                        attractBunch(bunch, linked.x, linked.y, 0, &linked,
                                     propagate);
                        ++diffs;
                    }
                }

                x_dist = bunch.x - linked.x;
                y_dist = bunch.y - linked.y;
                uint64_t distance = (x_dist>=0?x_dist:-x_dist)+(y_dist>=0?y_dist:-y_dist);
                sum_distance += distance > 1 ? distance : 0;
            }
        }
    }

    PNR_LOG2_("OUTL", depth, "recurseSecondaryLinks, bunch: {} ({}), sum_distance: {}, uplinks: {}, timing: {}, placed: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type,
        sum_distance, bunch.uplinks.size(), timing_uplinks, timing_uplinks_placed);
    return sum_distance;
}

void OutlineDesign::recurseRadialAllocation(RegBunch& bunch, int x, int y, int depth)
{
    PNR_LOG2_("OUTL", depth, "recurseRadialAllocation, bunch: {} ({}), x: {}, y: {}, size: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type, x, y, bunch.size_comb);

    if (!bunch.fixed && uniform_unanchored_allocation) {
        constexpr size_t region_count = mesh_width*mesh_height;
        size_t selected = allocation_cursor%region_count;
        bool found = false;
        for (size_t offset = 0; offset < region_count; ++offset) {
            size_t candidate = (allocation_cursor + offset)%region_count;
            if (allocated_registers[candidate] + bunch.size_regs_own
                    <= allocation_register_target
                && allocated_combs[candidate] + bunch.size_comb_own
                    <= allocation_comb_target) {
                selected = candidate;
                found = true;
                break;
            }
        }
        if (!found) {
            auto overflow = [&](size_t candidate) {
                return std::max(0,
                    allocated_registers[candidate] + bunch.size_regs_own
                        - allocation_register_target)
                    + std::max(0,
                        allocated_combs[candidate] + bunch.size_comb_own
                            - allocation_comb_target);
            };
            for (size_t candidate = 0; candidate < region_count; ++candidate) {
                if (overflow(candidate) < overflow(selected)) {
                    selected = candidate;
                }
            }
        }
        int row = static_cast<int>(selected/mesh_width);
        int column = static_cast<int>(selected%mesh_width);
        if (row%2 != 0) {
            column = mesh_width - 1 - column;
        }
        bunch.x = static_cast<float>(column) + 0.5F;
        bunch.y = static_cast<float>(row) + 0.5F;
        allocated_registers[selected] += bunch.size_regs_own;
        allocated_combs[selected] += bunch.size_comb_own;
        allocation_cursor = selected;
        if (allocated_registers[selected] >= allocation_register_target
            || allocated_combs[selected] >= allocation_comb_target) {
            allocation_cursor = (selected + 1)%region_count;
        }
    }
    else if (!bunch.fixed) {
        bunch.x = (float)x + 0.5;
        bunch.y = (float)y + 0.5;

        if (x == 0 && y != mesh_height-1) {
            ++y;
        }
        else
        if (y == mesh_height-1 && x != mesh_width-1) {
            ++x;
        }
        else
        if (x == mesh_width-1 && y != 0) {
            --y;
        }
        else
        if (y == 0 && x != 0) {
            --x;
        }
    }

    for (auto& subbunch : bunch.sub_bunches) {
        recurseRadialAllocation(subbunch, x, y, depth + 1);
    }
}

void OutlineDesign::recurseStatsDesign(RegBunch& bunch, int depth)
{
    int box_y = outlineMeshIndex(bunch.y, mesh_height);
    int box_x = outlineMeshIndex(bunch.x, mesh_width);
    boxes[box_y][box_x].size_regs += bunch.size_regs_own;
    boxes[box_y][box_x].size_luts += bunch.size_comb_own;
    boxes[box_y][box_x].bunches.push_back(&bunch);

    for (auto& link : bunch.uplinks) {
        if (link.secondary) {
        }
        else {
        }
    }

    for (auto& subbunch : bunch.sub_bunches) {
        recurseStatsDesign(subbunch, depth + 1);
    }
}

void OutlineDesign::recurseDrawOutline(std::list<Referable<RegBunch>>& bunch_list, int i, int depth)
{
    if (depth == 0) {
        image.init(mesh_width*100, mesh_height*100);
        image.clear();
    }

    for (auto& bunch : bunch_list) {

        image.draw_space(bunch.x*100, bunch.y*100, 0, 0, 255, 255, 50);
        image.draw_space(bunch.x*100, bunch.y*100, 0, 255, 0, 255, bunch.size_comb_own);

        for (auto& link : bunch.uplinks) {
            if (link.conn->inst_ref->outline.fixed || bunch.reg->outline.fixed) {
                image.draw_line(link.conn->inst_ref->bunch_ref->x*100, link.conn->inst_ref->bunch_ref->y*100, bunch.x*100, bunch.y*100, 200, 200, 200, 255);
            }
            else
            if (link.secondary) {
                image.draw_line(link.conn->inst_ref->bunch_ref->x*100, link.conn->inst_ref->bunch_ref->y*100, bunch.x*100, bunch.y*100, 255, 0, 0, 255);
            }
            else {
                image.draw_line(link.conn->inst_ref->bunch_ref->x*100, link.conn->inst_ref->bunch_ref->y*100, bunch.x*100, bunch.y*100, 0, 200, 200, 255);
            }
        }

        PNR_LOG2_("OUTL", depth, "recurseDrawOutline, bunch: {} ({}), x: {}, y: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type, bunch.x, bunch.y);

        recurseDrawOutline(bunch.sub_bunches, i, depth + 1);
    }


    if (depth == 0) {
        image.write(std::string("outline_output-") + std::to_string(i) + ".png");
    }
}

void OutlineDesign::optimizeOutline(std::list<Referable<RegBunch>>& bunch_list)
{
//    set_pixel(image_data, width, 100, 100, 255, 0, 0, 255);
//    draw_line(image_data, width, 50, 50, 450, 50, 0, 0, 255, 255);   // Horizontal blue line

    auto& fpga = fpga::Device::current();

    int total_bunches = 0;
    int total_regs = 0;
    int total_comb = 0;

    for (auto& bunch : bunch_list) {
        total_bunches += bunch.size;
        total_regs += bunch.size_regs;
        total_comb += bunch.size_comb;  // need size of CARRY, MUX, SRL?   // then think about BRAM, LRAM, DSP
    }
    int design_cells = countDesignCells(bunch_list);
    if (design_cells <= 0) {
        design_cells = std::max(total_bunches, total_regs + total_comb);
    }
    iteration_limit = outlineBunchIterationLimit(design_cells);
    combs_per_box = /*total_comb*/(float)fpga.cnt_luts / (mesh_width*mesh_height);

    uniform_unanchored_allocation = !std::ranges::any_of(
        bunch_list, [](const Referable<RegBunch>& bunch) {
            return hasFixedBunch(bunch);
        });
    allocation_cursor = 0;
    allocated_registers.fill(0);
    allocated_combs.fill(0);
    allocation_register_target = std::max(
        1, (total_regs + mesh_width*mesh_height - 1)
            /(mesh_width*mesh_height));
    allocation_comb_target = std::max(
        1, (total_comb + mesh_width*mesh_height - 1)
            /(mesh_width*mesh_height));

    fpga_width = fpga.size_width*2;
    fpga_height = fpga.size_height*2;
    aspect_x = (float)fpga_width/mesh_width;
    aspect_y = (float)fpga_height/mesh_height;
    step_x = (float)mesh_width/fpga_width;
    step_y = (float)mesh_height/fpga_height;

    boxes1 = new int[fpga_width*fpga_height];

    PNR_LOG1("OUTL", "optimizeOutline, fpga_width: {}, fpga_height: {}, aspect_x: {:.3f}, aspect_y: {:.3f}, step_x: {:.3f}, step_y: {:.3f}, total_regs: {}, total_comb: {}, total_bunches: {}, cells: {}, iteration_limit: {}, combs_per_box: {}",
        fpga_width, fpga_height, aspect_x, aspect_y, step_x, step_y, total_regs, total_comb, total_bunches, design_cells, iteration_limit, combs_per_box);

    for (auto& bunch : bunch_list) {
        recurseRadialAllocation(bunch, 0, 0);
    }

/*        for (auto& bunch : bunch_list) {
    for (int i=0; i < 10; ++i) {
            uint64_t sum_distance = recurseSecondaryLinks(bunch);
        std::print(std::cerr, "i: {}, sum_distance: {}\n", i, sum_distance);
    }
        }
*/
travers_mark = 0;
avg_comb_in_bunch = 0;
    int bunch_iteration_limit = uniform_unanchored_allocation
        ? 0 : iteration_limit;
    auto bunch_phase_start = std::chrono::steady_clock::now();
    for (int i=0; i < bunch_iteration_limit; ++i) {
//std::print("---- {}\n", i);
//        recurseDrawOutline(bunch_list, i);

        if (i > 100) {
            for (size_t y=0; y < mesh_height; ++y) {
                for (size_t x=0; x < mesh_width; ++x) {
                    boxes[y][x].size_regs = 0;
                    boxes[y][x].size_luts = 0;
                    boxes[y][x].bunches.clear();
                }
            }

            for (auto& bunch : bunch_list) {
                recurseStatsDesign(bunch);
            }

                if (i == 101 || (i + 1) % 100 == 0
                    || i + 1 == bunch_iteration_limit) {
                std::print("\n{}\n", combs_per_box);
                for (size_t y=0; y < mesh_height; ++y) {
                    for (size_t x=0; x < mesh_width; ++x) {
                        std::print("{:5d}", boxes[y][x].size_luts);
                    }
                    std::print("\n");
                }
                std::print("\n");
            }

            size_t min_x;
            size_t min_y;
            int min_luts = 1000000000;
            for (int y=mesh_height/2; y >= 0; --y) {
                for (int x=mesh_width/2; x >= 0; --x) {
                    if (boxes[y][x].size_luts < min_luts) {
                        min_luts = boxes[y][x].size_luts;
                        min_y = y;
                        min_x = x;
                    }
                }
            }

            for (size_t y=0; y < mesh_height; ++y) {
                for (size_t x=0; x < mesh_width; ++x) {
                    if (boxes[y][x].size_luts > combs_per_box) {
                        int num = boxes[y][x].size_luts - combs_per_box;
                        for (auto* bunch : boxes[y][x].bunches) {
                            attractBunch(*bunch, min_x, min_y, 0, bunch);
                            num -= bunch->size_comb_own;
                            if (num < 0) break;
                        }
                    }
                }
            }
        }


        uint64_t sum_distance = 0;
        for (auto& bunch : bunch_list) {
if (i > 50) {
travers_mark = rtl::Inst::genMark();
}
if (i > 100) {
avg_comb_in_bunch = total_comb / total_bunches;
travers_mark = 0;
}
if (i > 150) {
travers_mark = 0;
avg_comb_in_bunch = 0;
}
            sum_distance += recurseSecondaryLinks(bunch);

            PNR_LOG2("OUTL", "fixing bunch: {} ({}), sum_distance: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type, sum_distance);
        }
        if ((i + 1) % 25 == 0 || i + 1 == bunch_iteration_limit) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - bunch_phase_start).count();
            std::print("\nOUTLINE_PROGRESS phase=bunch iteration={}/{} distance={} elapsed_s={:.3f}",
                i + 1, iteration_limit, sum_distance, elapsed);
            fflush(stdout);
        }
//        std::print(std::cerr, "i: {}, sum_distance: {}\n", i, sum_distance);
    }
    double bunch_phase_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - bunch_phase_start).count();

    ////////////////////////////////////////// design

    travers_mark = rtl::Inst::genMark();
    for (auto& bunch : bunch_list) {
        recurseInstAllocation(*bunch.reg, &bunch);
    }

    travers_mark = rtl::Inst::genMark();
    optimization_peers.clear();
    optimization_peers.reserve(static_cast<size_t>(design_cells));
    optimization_sinks.clear();
    optimization_sinks.reserve(static_cast<size_t>(design_cells));
    optimization_drivers.clear();
    optimization_drivers.reserve(static_cast<size_t>(design_cells));
    optimization_edges.clear();
    optimization_edges.reserve(static_cast<size_t>(design_cells));
    for (auto& bunch : bunch_list) {
        recurseInstPrepare(*bunch.reg, &bunch);
    }

    int instance_iteration_limit = outlineInstanceIterationLimit(
        iteration_limit, fpga_width, fpga_height);
    auto instance_phase_start = std::chrono::steady_clock::now();
    for (int i=0; i < instance_iteration_limit; ++i) {
//        image.init(mesh_width*aspect_x*image_zoom, mesh_height*aspect_y*image_zoom);
//        image.clear();
//        travers_mark = rtl::Inst::genMark();
//        for (auto& bunch : bunch_list) {
//            recurseDrawDesign(*bunch.reg, &bunch, 1);
//        }
//        travers_mark = rtl::Inst::genMark();
//        for (auto& bunch : bunch_list) {
//            recurseDrawDesign(*bunch.reg, &bunch, 0);
//        }
//        image.write(std::string("design_output-") + std::to_string(i) + ".png");

//        travers_mark = rtl::Inst::genMark();
//        FILE* out = fopen((std::string("design_output-") + std::to_string(i) + ".txt").c_str(), "w");
//        for (auto& bunch : bunch_list) {
//            recurseDumpDesign(*bunch.reg, &bunch, out);
//        }
//        fclose(out);

        memset(boxes1, 0, fpga_width*fpga_height*sizeof(int));
        travers_mark = rtl::Inst::genMark();
        for (auto& bunch : bunch_list) {
//std::print("{} --- {} ({})\n", i, bunch.reg->makeName(), bunch.reg->cell_ref->type);fflush(stdout);
            recurseOptimizeInsts(*bunch.reg, &bunch, i);
        }
        if ((i + 1) % 25 == 0 || i + 1 == instance_iteration_limit) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - instance_phase_start).count();
            std::print("\nOUTLINE_PROGRESS phase=instance iteration={}/{} elapsed_s={:.3f}",
                i + 1, instance_iteration_limit, elapsed);
            fflush(stdout);
        }
    }

    // Fine-grid spreading can leave short combinational runs locally folded
    // even after their register bunches have been stretched between fixed I/Os.
    // Treat timing connections as undirected springs for one final relaxation.
    // Fixed instances are boundary conditions, so chains interpolate between
    // their anchors instead of collapsing to one point.
    double instance_phase_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - instance_phase_start).count();
    struct TensionComponent {
        std::vector<rtl::Inst*> cells;
        std::array<float, 2> direction;
    };
    std::vector<TensionComponent> tension_components;
    std::unordered_map<rtl::Inst*, std::array<float, 2>> tension_directions;
    tension_directions.reserve(optimization_peers.size());
    std::unordered_set<rtl::Inst*> component_visited;
    component_visited.reserve(optimization_peers.size());
    for (const auto& [seed, seed_peers] : optimization_peers) {
        if (!seed || component_visited.contains(seed)) {
            continue;
        }
        std::vector<rtl::Inst*> component;
        std::vector<rtl::Inst*> stack{seed};
        component_visited.insert(seed);
        float source_x = 0;
        float source_y = 0;
        float sink_x = 0;
        float sink_y = 0;
        int source_count = 0;
        int sink_count = 0;
        while (!stack.empty()) {
            rtl::Inst* inst = stack.back();
            stack.pop_back();
            component.push_back(inst);
            bool has_drivers = optimization_drivers.contains(inst)
                && !optimization_drivers[inst].empty();
            bool has_sinks = optimization_sinks.contains(inst)
                && !optimization_sinks[inst].empty();
            if (inst->outline.fixed && !has_drivers) {
                source_x += inst->outline.x;
                source_y += inst->outline.y;
                ++source_count;
            }
            if (inst->outline.fixed && !has_sinks) {
                sink_x += inst->outline.x;
                sink_y += inst->outline.y;
                ++sink_count;
            }
            for (rtl::Inst* peer : optimization_peers[inst]) {
                if (peer && component_visited.insert(peer).second) {
                    stack.push_back(peer);
                }
            }
        }
        if (source_count == 0 || sink_count == 0) {
            continue;
        }
        source_x /= source_count;
        source_y /= source_count;
        sink_x /= sink_count;
        sink_y /= sink_count;
        float dx = sink_x - source_x;
        float dy = sink_y - source_y;
        float abs_x = std::abs(dx);
        float abs_y = std::abs(dy);
        // Preserve diagonal tension when both axes materially contribute;
        // otherwise use the dominant cardinal direction. This prevents a
        // small imbalance in randomly distributed boundary sources from
        // tilting a nominally horizontal or vertical tree.
        if (abs_x > 0.35F*abs_y && abs_y > 0.35F*abs_x) {
            dx = dx < 0 ? -1.0F : 1.0F;
            dy = dy < 0 ? -1.0F : 1.0F;
        }
        else if (abs_x >= abs_y) {
            dx = dx < 0 ? -1.0F : 1.0F;
            dy = 0;
        }
        else {
            dx = 0;
            dy = dy < 0 ? -1.0F : 1.0F;
        }
        float length = std::sqrt(dx*dx + dy*dy);
        if (length == 0) {
            continue;
        }
        std::array<float, 2> direction{dx/length, dy/length};
        for (rtl::Inst* inst : component) {
            tension_directions.emplace(inst, direction);
        }
        tension_components.push_back(
            TensionComponent{std::move(component), direction});
    }

    constexpr int constellation_iterations = 300;
    constexpr float constellation_relaxation = 1.6F;
    constexpr float minimum_forward_step = 0.001F;
    auto constellation_phase_start = std::chrono::steady_clock::now();
    uint64_t directed_tension_corrections = 0;
    for (int iteration = 0; iteration < constellation_iterations; ++iteration) {
        for (auto& [inst, peers] : optimization_peers) {
            // An unanchored spring component has no absolute solution: repeated
            // neighbor averaging collapses the whole component to its centroid.
            // Only components with fixed source and sink boundary conditions were
            // entered in tension_directions above and can be relaxed safely.
            if (!inst || inst->outline.fixed || peers.empty()
                || !tension_directions.contains(inst)) {
                continue;
            }
            float x = 0;
            float y = 0;
            for (rtl::Inst* peer : peers) {
                x += peer->outline.x;
                y += peer->outline.y;
            }
            float divisor = static_cast<float>(peers.size());
            float target_x = x/divisor;
            float target_y = y/divisor;
            inst->outline.x = std::clamp(
                inst->outline.x + constellation_relaxation
                    *(target_x - inst->outline.x),
                0.0F, 9.95F);
            inst->outline.y = std::clamp(
                inst->outline.y + constellation_relaxation
                    *(target_y - inst->outline.y),
                0.0F, 9.95F);
        }
    }
    // A spring average alone can fold a high-fanout tree: a junction may be
    // pulled behind its most advanced child by two less advanced children.
    // Once the transverse spring placement has settled, project every timing
    // edge into the component's source-to-sink tension cone. Alternating the
    // edge order propagates corrections efficiently in both directions.
    auto project_edge = [&](rtl::Inst* driver, rtl::Inst* sink) {
        auto direction = tension_directions.find(driver);
        if (direction == tension_directions.end()) {
            return false;
        }
        float dx = direction->second[0];
        float dy = direction->second[1];
        float forward = (sink->outline.x - driver->outline.x)*dx
            + (sink->outline.y - driver->outline.y)*dy;
        if (forward >= minimum_forward_step) {
            return false;
        }
        float correction = minimum_forward_step - forward;
        bool move_driver = !driver->outline.fixed;
        bool move_sink = !sink->outline.fixed;
        if (!move_driver && !move_sink) {
            return false;
        }
        float driver_share = move_driver ? (move_sink ? 0.5F : 1.0F) : 0;
        float sink_share = move_sink ? (move_driver ? 0.5F : 1.0F) : 0;
        driver->outline.x = std::clamp(
            driver->outline.x - correction*driver_share*dx, 0.0F, 9.95F);
        driver->outline.y = std::clamp(
            driver->outline.y - correction*driver_share*dy, 0.0F, 9.95F);
        sink->outline.x = std::clamp(
            sink->outline.x + correction*sink_share*dx, 0.0F, 9.95F);
        sink->outline.y = std::clamp(
            sink->outline.y + correction*sink_share*dy, 0.0F, 9.95F);
        ++directed_tension_corrections;
        return true;
    };

    // First solve acyclic timing constellations directly. A reverse pass
    // derives each cell's latest feasible progress from fixed sinks; a
    // forward pass then raises junctions enough to follow every fixed source.
    // This is the directed analogue of pulling a branched rope taut and
    // avoids the slow diffusion of corrections along long chains.
    std::unordered_map<rtl::Inst*, float> tension_upper;
    std::unordered_map<rtl::Inst*, float> tension_value;
    tension_upper.reserve(optimization_peers.size());
    tension_value.reserve(optimization_peers.size());
    uint64_t direct_tension_adjustments = 0;
    for (const TensionComponent& component : tension_components) {
        std::unordered_map<rtl::Inst*, int> indegree;
        indegree.reserve(component.cells.size());
        std::vector<rtl::Inst*> ready;
        ready.reserve(component.cells.size());
        for (rtl::Inst* inst : component.cells) {
            int drivers = optimization_drivers.contains(inst)
                ? static_cast<int>(optimization_drivers[inst].size()) : 0;
            indegree.emplace(inst, drivers);
            if (drivers == 0) {
                ready.push_back(inst);
            }
        }
        std::vector<rtl::Inst*> order;
        order.reserve(component.cells.size());
        while (!ready.empty()) {
            rtl::Inst* inst = ready.back();
            ready.pop_back();
            order.push_back(inst);
            auto sinks = optimization_sinks.find(inst);
            if (sinks == optimization_sinks.end()) {
                continue;
            }
            for (rtl::Inst* sink : sinks->second) {
                auto degree = indegree.find(sink);
                if (degree != indegree.end() && --degree->second == 0) {
                    ready.push_back(sink);
                }
            }
        }
        if (order.size() != component.cells.size()) {
            continue;
        }
        float dx = component.direction[0];
        float dy = component.direction[1];
        for (rtl::Inst* inst : component.cells) {
            tension_upper[inst] = inst->outline.fixed
                ? inst->outline.x*dx + inst->outline.y*dy
                : std::numeric_limits<float>::infinity();
        }
        for (auto node = order.rbegin(); node != order.rend(); ++node) {
            auto sinks = optimization_sinks.find(*node);
            if (sinks == optimization_sinks.end()) {
                continue;
            }
            for (rtl::Inst* sink : sinks->second) {
                float sink_upper = tension_upper[sink];
                if (std::isfinite(sink_upper)) {
                    tension_upper[*node] = std::min(
                        tension_upper[*node],
                        sink_upper - minimum_forward_step);
                }
            }
        }
        for (rtl::Inst* inst : order) {
            float current = inst->outline.x*dx + inst->outline.y*dy;
            float value = std::min(current, tension_upper[inst]);
            if (inst->outline.fixed) {
                value = current;
            }
            else {
                auto drivers = optimization_drivers.find(inst);
                if (drivers != optimization_drivers.end()) {
                    for (rtl::Inst* driver : drivers->second) {
                        value = std::max(
                            value, tension_value[driver]
                                + minimum_forward_step);
                    }
                }
                value = std::min(value, tension_upper[inst]);
            }
            tension_value[inst] = value;
        }
        for (rtl::Inst* inst : order) {
            if (inst->outline.fixed) {
                continue;
            }
            float current = inst->outline.x*dx + inst->outline.y*dy;
            float adjustment = tension_value[inst] - current;
            if (std::abs(adjustment) <= 0.000001F) {
                continue;
            }
            inst->outline.x = std::clamp(
                inst->outline.x + adjustment*dx, 0.0F, 9.95F);
            inst->outline.y = std::clamp(
                inst->outline.y + adjustment*dy, 0.0F, 9.95F);
            ++direct_tension_adjustments;
        }
    }
    int tension_projection_iterations = 0;
    constexpr int tension_projection_limit = 50;
    for (; tension_projection_iterations < tension_projection_limit;
         ++tension_projection_iterations) {
        size_t corrections_before = directed_tension_corrections;
        for (auto edge = optimization_edges.rbegin();
             edge != optimization_edges.rend(); ++edge) {
            project_edge(edge->first, edge->second);
        }
        for (const auto& [driver, sink] : optimization_edges) {
            project_edge(driver, sink);
        }
        if (directed_tension_corrections == corrections_before) {
            ++tension_projection_iterations;
            break;
        }
    }
    legalizeOutlineCapacity();
    double constellation_phase_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - constellation_phase_start).count();
    std::print("\nOUTLINE_SUMMARY cells={} bunch_iterations={} instance_iterations={} constellation_iterations={} directed_edges={} tension_adjustments={} tension_iterations={} tension_corrections={} bunch_s={:.3f} instance_s={:.3f} constellation_s={:.3f}",
        design_cells, bunch_iteration_limit, instance_iteration_limit,
        constellation_iterations, optimization_edges.size(),
        direct_tension_adjustments, tension_projection_iterations,
        directed_tension_corrections,
        bunch_phase_seconds,
        instance_phase_seconds, constellation_phase_seconds);
    fflush(stdout);

//    std::print("\n");
//    for (int y=0; y < fpga_height; ++y) {
//        for (int x=0; x < fpga_width; ++x) {
//            std::print("{} ", boxes1[y*fpga_width + x]);
//        }
//        std::print("\n");
//    }
//    std::print("\n");
}

void OutlineDesign::legalizeOutlineCapacity()
{
    auto& device = fpga::Device::current();
    int width = device.size_width;
    int height = device.size_height;
    if (width <= 0 || height <= 0 || device.tile_grid.empty()) {
        return;
    }

    constexpr int type_count = fpga::ELEMENT_TYPE_COUNT;
    using TypeCounts = std::array<uint16_t, type_count>;
    size_t tile_count = static_cast<size_t>(width*height);
    std::vector<TypeCounts> capacity(tile_count);
    std::vector<TypeCounts> occupancy(tile_count);
    for (const fpga::Tile& tile : device.tile_grid) {
        if (!tile.tile_type || tile.coord.x < 0 || tile.coord.y < 0
            || tile.coord.x >= width || tile.coord.y >= height) {
            continue;
        }
        std::array<uint16_t, type_count> masks{};
        for (const fpga::Element& element : tile.tile_type->elements) {
            if (element.bitmap_pos < fpga::ELEMENT_BITMAP_BITS) {
                masks[element.type] |= static_cast<uint16_t>(
                    1U << element.bitmap_pos);
            }
        }
        size_t index = static_cast<size_t>(tile.coord.y*width + tile.coord.x);
        for (int type = 0; type < type_count; ++type) {
            capacity[index][type] = static_cast<uint16_t>(
                std::popcount(static_cast<unsigned>(masks[type])));
        }
    }

    std::vector<rtl::Inst*> cells;
    cells.reserve(optimization_peers.size());
    std::unordered_set<rtl::Inst*> collected;
    collected.reserve(optimization_peers.size());
    auto collect = [&](rtl::Inst* inst) {
        if (inst && fpga::isPlaceableElement(*inst)
            && collected.insert(inst).second) {
            cells.push_back(inst);
        }
    };
    for (const auto& [driver, sink] : optimization_edges) {
        collect(driver);
        collect(sink);
    }
    for (const auto& [inst, peers] : optimization_peers) {
        (void)peers;
        collect(inst);
    }

    std::unordered_map<rtl::Inst*, Coord> assigned;
    assigned.reserve(cells.size());
    auto reserve = [&](rtl::Inst& inst, Coord coord,
                       fpga::ElementType type) {
        size_t index = static_cast<size_t>(coord.y*width + coord.x);
        ++occupancy[index][type];
        assigned[&inst] = coord;
        float physical_aspect_x = static_cast<float>(width)/mesh_width;
        float physical_aspect_y = static_cast<float>(height)/mesh_height;
        inst.outline.x = (coord.x + 0.5F)/physical_aspect_x;
        inst.outline.y = (coord.y + 0.5F)/physical_aspect_y;
    };

    for (rtl::Inst* inst : cells) {
        if (!inst || !inst->outline.fixed || !inst->tile.peer) continue;
        std::optional<fpga::ElementType> type = fpga::elementTypeForInst(*inst);
        if (!type) continue;
        Coord coord = inst->tile->coord;
        size_t index = static_cast<size_t>(coord.y*width + coord.x);
        if (index < tile_count && occupancy[index][*type] < capacity[index][*type]) {
            reserve(*inst, coord, *type);
        }
    }

    if (uniform_unanchored_allocation && !cells.empty()) {
        std::vector<rtl::Inst*> breadth_first;
        breadth_first.reserve(cells.size());
        std::unordered_set<rtl::Inst*> visited;
        visited.reserve(cells.size());
        for (rtl::Inst* seed : cells) {
            if (!seed || !visited.insert(seed).second) continue;
            std::deque<rtl::Inst*> pending{seed};
            while (!pending.empty()) {
                rtl::Inst* inst = pending.front();
                pending.pop_front();
                breadth_first.push_back(inst);
                auto peers = optimization_peers.find(inst);
                if (peers == optimization_peers.end()) continue;
                for (rtl::Inst* peer : peers->second) {
                    if (peer && fpga::isPlaceableElement(*peer)
                        && visited.insert(peer).second) {
                        pending.push_back(peer);
                    }
                }
            }
        }
        cells = std::move(breadth_first);
    }

    size_t overflow_cells = 0;
    size_t moved_cells = 0;
    int maximum_move = 0;
    float physical_aspect_x = static_cast<float>(width)/mesh_width;
    float physical_aspect_y = static_cast<float>(height)/mesh_height;

    std::vector<TypeCounts> requested_occupancy(tile_count);
    for (rtl::Inst* inst : cells) {
        if (!inst || inst->outline.fixed) continue;
        std::optional<fpga::ElementType> type = fpga::elementTypeForInst(*inst);
        if (!type) continue;
        Coord preferred{
            std::clamp(static_cast<int>(inst->outline.x*physical_aspect_x),
                       0, width - 1),
            std::clamp(static_cast<int>(inst->outline.y*physical_aspect_y),
                       0, height - 1),
        };
        size_t index = static_cast<size_t>(preferred.y*width + preferred.x);
        if (requested_occupancy[index][*type] >= capacity[index][*type]) {
            ++overflow_cells;
        }
        else {
            ++requested_occupancy[index][*type];
        }
    }

    for (rtl::Inst* inst : cells) {
        if (!inst || assigned.contains(inst) || inst->outline.fixed) continue;
        std::optional<fpga::ElementType> type = fpga::elementTypeForInst(*inst);
        if (!type) continue;
        Coord preferred{
            std::clamp(static_cast<int>(inst->outline.x*physical_aspect_x),
                       0, width - 1),
            std::clamp(static_cast<int>(inst->outline.y*physical_aspect_y),
                       0, height - 1),
        };
        Coord selected = preferred;
        bool found = false;
        double best_score = std::numeric_limits<double>::infinity();
        auto peers = optimization_peers.find(inst);
        size_t preferred_index = static_cast<size_t>(
            preferred.y*width + preferred.x);
        if (!uniform_unanchored_allocation
            && occupancy[preferred_index][*type]
                < capacity[preferred_index][*type]) {
            found = true;
            best_score = 0;
        }
        for (const fpga::Tile& tile : device.tile_grid) {
            if (found && !uniform_unanchored_allocation) break;
            if (!tile.tile_type || tile.coord.x < 0 || tile.coord.y < 0
                || tile.coord.x >= width || tile.coord.y >= height) {
                continue;
            }
            size_t index = static_cast<size_t>(
                tile.coord.y*width + tile.coord.x);
            if (occupancy[index][*type] >= capacity[index][*type]) {
                continue;
            }
            int target_distance = std::abs(tile.coord.x - preferred.x)
                + std::abs(tile.coord.y - preferred.y);
            int peer_distance = 0;
            int assigned_peers = 0;
            if (peers != optimization_peers.end()) {
                for (rtl::Inst* peer : peers->second) {
                    auto placed_peer = assigned.find(peer);
                    if (placed_peer == assigned.end()) continue;
                    peer_distance += std::abs(
                        tile.coord.x - placed_peer->second.x)
                        + std::abs(
                            tile.coord.y - placed_peer->second.y);
                    ++assigned_peers;
                }
            }
            double average_peer_distance = assigned_peers == 0 ? 0
                : static_cast<double>(peer_distance)/assigned_peers;
            unsigned total_occupancy = 0;
            for (uint16_t count : occupancy[index]) {
                total_occupancy += count;
            }
            double score = target_distance
                + 6.0*average_peer_distance
                + 0.02*total_occupancy;
            if (score < best_score) {
                best_score = score;
                selected = tile.coord;
                found = true;
            }
        }
        PNR_ASSERT(found,
            "Outline capacity legalization found no '{}' element for '{}'",
            fpga::elementTypeName(*type), inst->makeName());
        int movement = std::abs(selected.x - preferred.x)
            + std::abs(selected.y - preferred.y);
        moved_cells += movement != 0;
        maximum_move = std::max(maximum_move, movement);
        reserve(*inst, selected, *type);
    }

    size_t relaxation_passes = 0;
    size_t relaxation_moves = 0;
    if (uniform_unanchored_allocation) {
        constexpr size_t max_relaxation_passes = 50;
        for (size_t pass = 0; pass < max_relaxation_passes; ++pass) {
            size_t pass_moves = 0;
            auto relax = [&](rtl::Inst* inst) {
                if (!inst || inst->outline.fixed) return;
                auto current_it = assigned.find(inst);
                auto peers_it = optimization_peers.find(inst);
                std::optional<fpga::ElementType> type =
                    fpga::elementTypeForInst(*inst);
                if (current_it == assigned.end()
                    || peers_it == optimization_peers.end() || !type) {
                    return;
                }
                std::vector<int> peer_x;
                std::vector<int> peer_y;
                peer_x.reserve(peers_it->second.size());
                peer_y.reserve(peers_it->second.size());
                int current_cost = 0;
                int peer_x_sum = 0;
                int peer_y_sum = 0;
                for (rtl::Inst* peer : peers_it->second) {
                    auto peer_it = assigned.find(peer);
                    if (peer_it == assigned.end()) continue;
                    peer_x.push_back(peer_it->second.x);
                    peer_y.push_back(peer_it->second.y);
                    peer_x_sum += peer_it->second.x;
                    peer_y_sum += peer_it->second.y;
                    int distance = std::abs(
                        current_it->second.x - peer_it->second.x)
                        + std::abs(
                            current_it->second.y - peer_it->second.y);
                    current_cost += distance*distance;
                }
                if (peer_x.empty()) return;
                Coord center{
                    static_cast<int>(std::lround(
                        static_cast<double>(peer_x_sum)/peer_x.size())),
                    static_cast<int>(std::lround(
                        static_cast<double>(peer_y_sum)/peer_y.size())),
                };

                Coord current = current_it->second;
                size_t current_index = static_cast<size_t>(
                    current.y*width + current.x);
                --occupancy[current_index][*type];
                Coord selected = current;
                int selected_cost = current_cost;
                bool found = false;
                for (int radius = 0; radius < width + height && !found;
                     ++radius) {
                    for (int dy = -radius; dy <= radius; ++dy) {
                        int dx = radius - std::abs(dy);
                        for (int sign : {-1, 1}) {
                            if (dx == 0 && sign == 1) continue;
                            Coord candidate{center.x + sign*dx,
                                            center.y + dy};
                            if (candidate.x < 0 || candidate.x >= width
                                || candidate.y < 0 || candidate.y >= height) {
                                continue;
                            }
                            size_t index = static_cast<size_t>(
                                candidate.y*width + candidate.x);
                            if (occupancy[index][*type]
                                >= capacity[index][*type]) {
                                continue;
                            }
                            int cost = 0;
                            for (size_t peer = 0; peer < peer_x.size(); ++peer) {
                                int distance = std::abs(
                                    candidate.x - peer_x[peer])
                                    + std::abs(candidate.y - peer_y[peer]);
                                cost += distance*distance;
                            }
                            if (!found || cost < selected_cost) {
                                selected = candidate;
                                selected_cost = cost;
                            }
                            found = true;
                        }
                    }
                }
                if (!found || selected_cost >= current_cost) {
                    ++occupancy[current_index][*type];
                    return;
                }
                reserve(*inst, selected, *type);
                ++pass_moves;
            };
            if (pass%2 == 0) {
                for (rtl::Inst* inst : cells) relax(inst);
            }
            else {
                for (auto inst = cells.rbegin(); inst != cells.rend(); ++inst) {
                    relax(*inst);
                }
            }
            relaxation_moves += pass_moves;
            ++relaxation_passes;
            if (pass_moves == 0) break;
        }
    }
    std::print(
        "\nOUTLINE_CAPACITY cells={} overflow={} moved={} max_move={} "
        "relaxation_passes={} relaxation_moves={} tiles={}",
        assigned.size(), overflow_cells, moved_cells, maximum_move,
        relaxation_passes, relaxation_moves, tile_count);
}

void OutlineDesign::recurseInstAllocation(rtl::Inst& inst, RegBunch* bunch, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    PNR_LOG3_("OUTL", depth, "recurseInstAllocation, inst: {} ({}), x: {}, y: {}", inst.makeName(), inst.cell_ref->type, inst.bunch_ref->x + 0.5, inst.bunch_ref->y + 0.5);
    if (!inst.outline.fixed) {
        // Bunch coordinates already denote the center of an Outline region.
        // Rounding and adding another half-step shifted every member into the
        // next region and outside the spreading window around its own bunch.
        inst.outline.x = inst.bunch_ref->x;
        inst.outline.y = inst.bunch_ref->y;
    }

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || curr->port_ref->is_global || !curr->inst_ref.peer
                || !curr->inst_ref->cell_ref->module_ref->is_blackbox) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;
            if (peer->bunch_ref.peer == inst.bunch_ref.peer) {
                if (peer->mark != travers_mark) {
                    recurseInstAllocation(*curr->inst_ref.peer, nullptr, depth + 1);
                }
            }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseInstAllocation(*subbunch.reg, &subbunch, depth + 1);
        }
    }
}

void OutlineDesign::recurseInstPrepare(rtl::Inst& inst, RegBunch* bunch, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || curr->port_ref->is_global || !curr->inst_ref.peer
                || !curr->inst_ref->cell_ref->module_ref->is_blackbox) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;
            optimization_peers[&inst].push_back(peer);
            optimization_peers[peer].push_back(&inst);
            optimization_sinks[peer].push_back(&inst);
            optimization_drivers[&inst].push_back(peer);
            optimization_edges.emplace_back(peer, &inst);
            if (peer->bunch_ref.peer != inst.bunch_ref.peer) {
                if (peer->outline.x > inst.outline.x + 0.5 && peer->outline.y > inst.outline.y + 0.5) {
                    inst.outline.x += 0.49;
                    inst.outline.y += 0.49;
                }
                else
                if (peer->outline.x > inst.outline.x + 0.5 && peer->outline.y < inst.outline.y - 0.5) {
                    inst.outline.x += 0.49;
                    inst.outline.y -= 0.49;
                }
                else
                if (peer->outline.x < inst.outline.x - 0.5 && peer->outline.y < inst.outline.y - 0.5) {
                    inst.outline.x -= 0.49;
                    inst.outline.y -= 0.49;
                }
                else
                if (peer->outline.x < inst.outline.x - 0.5 && peer->outline.y > inst.outline.y + 0.5) {
                    inst.outline.x -= 0.49;
                    inst.outline.y += 0.49;
                }
                else
                if (peer->outline.x > inst.outline.x + 0.5) {
                    inst.outline.x += 0.49;
                }
                else
                if (peer->outline.y > inst.outline.y + 0.5) {
                    inst.outline.y += 0.49;
                }
                else
                if (peer->outline.x < inst.outline.x - 0.5) {
                    inst.outline.x -= 0.49;
                }
                else
                if (peer->outline.y < inst.outline.y - 0.5) {
                    inst.outline.y -= 0.49;
                }
                PNR_LOG3_("OUTL", depth, "recurseInstPrepare, inst: {} ({}), x: {}, y: {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y);
            }
            else {
                if (peer->mark != travers_mark) {
                    recurseInstPrepare(*peer, nullptr, depth + 1);
                }
            }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseInstPrepare(*subbunch.reg, &subbunch, depth + 1);
        }
    }
}

uint32_t eee = 0;

void OutlineDesign::recurseOptimizeInsts(rtl::Inst& inst, RegBunch* bunch, int i, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    if (inst.outline.x < 0) {
        inst.outline.x = 0;
    }
    if (inst.outline.x > 9.99) {
        inst.outline.x = 9.95;
    }
    if (inst.outline.y < 0) {
        inst.outline.y = 0;
    }
    if (inst.outline.y > 9.99) {
        inst.outline.y = 9.95;
    }
//if (i<100 || (i>150 && i<200)) {
    int m = boxes1[(int)(inst.outline.y*aspect_y)*fpga_width + (int)(inst.outline.x*aspect_x)];
    ++eee;
    if (m > 1) {
        if ((eee%8==0 || eee%8==1) && inst.outline.x + step_x < inst.bunch_ref->x + 0.5 && boxes1[(int)(inst.outline.y*aspect_y)*fpga_width + (int)(inst.outline.x*aspect_x+1)] < m) {
            inst.outline.x += step_x;
        }
        if ((eee%8==2 || eee%8==3) && inst.outline.y + step_y < inst.bunch_ref->y + 0.5 && boxes1[(int)(inst.outline.y*aspect_y+1)*fpga_width + (int)(inst.outline.x*aspect_x)] < m) {
            inst.outline.y += step_y;
        }
        if ((eee%8==4 || eee%8==5) && inst.outline.x - step_x > inst.bunch_ref->x - 0.5 && boxes1[(int)(inst.outline.y*aspect_y)*fpga_width + (int)(inst.outline.x*aspect_x-1)] < m) {
            inst.outline.x -= step_x;
        }
        if ((eee%8==6 || eee%8==7) && inst.outline.y - step_y > inst.bunch_ref->y - 0.5 && boxes1[(int)(inst.outline.y*aspect_y-1)*fpga_width + (int)(inst.outline.x*aspect_x)] < m) {
            inst.outline.y -= step_y;
        }
    }
    if (inst.outline.x < 0) {
        inst.outline.x = 0;
    }
    if (inst.outline.x > 9.99) {
        inst.outline.x = 9.95;
    }
    if (inst.outline.y < 0) {
        inst.outline.y = 0;
    }
    if (inst.outline.y > 9.99) {
        inst.outline.y = 9.95;
    }
//}
    ++boxes1[(int)(inst.outline.y*aspect_y)*fpga_width + (int)(inst.outline.x*aspect_x)];

    auto peers = optimization_peers.find(&inst);
    if (peers != optimization_peers.end()) {
        for (rtl::Inst* peer : peers->second) {
            if (peer->mark != travers_mark) {
////    inst.mark = travers_mark;

            if (peer->bunch_ref.peer != inst.bunch_ref.peer) {
//if ((i > 100 && i < 150) || i > 200) {
                attractInst(inst, bunch, step_x, peer->outline.x, peer->outline.y, i, peer, depth + 1);
////                attractInst(*peer, bunch, step_x, inst.outline.x, inst.outline.y, i, &inst, depth + 1);
//}
            }
            else {
//if ((i > 100 && i < 150) || i > 200) {
                attractInst(inst, bunch, step_x, peer->outline.x, peer->outline.y, i, peer, depth + 1);
////                attractInst(*peer, bunch, step_x, inst.outline.x, inst.outline.y, i, &inst, depth + 1);
//}
////                    peer->mark = travers_mark;
                    recurseOptimizeInsts(*peer, nullptr, depth + 1);
            }
            }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseOptimizeInsts(*subbunch.reg, &subbunch, i, depth + 1);
        }
    }
}

void OutlineDesign::attractInst(rtl::Inst& inst, RegBunch* bunch, float step, float x, float y, int i, rtl::Inst* exclude, int depth)
{
    PNR_LOG3_("OUTL", depth, "attractInst, inst: {} ({}), outline.x: {}, outline.y: {}, x: {}, y: {}, step: {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y, step);

//if (inst.makeName() == "$abc$712025$auto$blifparse.cc:535:parse_blif$718286") {
//    std::print("\n!!!!!!!!!!!!!!!!!! attractInst, inst: {} ({}), outline.x: {}, outline.y: {}, x: {}, y: {}, step: {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y, step);
//}
    PNR_ASSERT(inst.bunch_ref.peer != nullptr, "bunch ref of inst '{}' is zero", inst.makeName());

    if (!inst.outline.fixed)
    if ((i > 50 && boxes1[(int)(inst.outline.x + (x > inst.outline.x ? step : -step))*fpga_width + (int)(inst.outline.y + (y > inst.outline.y ? step : -step))] == 0)
        || (inst.outline.x + (x > inst.outline.x ? step : -step) < inst.bunch_ref->x + 0.5 && inst.outline.x + (x + inst.outline.x ? step : -step) > inst.bunch_ref->x - 0.5
        && inst.outline.y + (y > inst.outline.y ? step : -step) < inst.bunch_ref->y + 0.5 && inst.outline.y + (y + inst.outline.y ? step : -step) > inst.bunch_ref->y - 0.5)) {

        inst.outline.x += (x-step_x > inst.outline.x ? step : (x+step_x < inst.outline.x ? -step : 0));
        inst.outline.y += (y-step_y > inst.outline.y ? step : (y+step_y < inst.outline.y ? -step : 0));

//if (inst.makeName() == "$abc$712025$auto$blifparse.cc:535:parse_blif$718286") {
//    std::print("\n!!!!!!!!!!!!!!!!!! attractInst, x: {}, y: {}", inst.outline.x, inst.outline.y);
//}

        auto peers = optimization_peers.find(&inst);
        if (peers != optimization_peers.end()) {
            for (rtl::Inst* peer : peers->second) {
                // Recursive constellation motion is useful along a chain, but
                // expands geometrically at high-degree fork nodes. Forks are
                // handled by the final spring relaxation instead.
                if (peers->second.size() <= 2
                    && step > step_x/5 && peer != exclude) {
                    attractInst(*peer, bunch, step/2, x, y, i, exclude, depth + 1);
                }
            }
        }
    }
}

void OutlineDesign::recurseDrawDesign(rtl::Inst& inst, RegBunch* bunch, int mode, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

if (mode == 0) {
    if (inst.cell_ref->type.find("LUT") != std::string::npos) {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 255, 0, 255);
    }
    else {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 0, 255, 255);
    }
}

//if (inst.outline.x < 0.3 && inst.outline.y < 0.3) std::print("\n----------------- {} ({}) {} {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y);


    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || curr->port_ref->is_global || !curr->inst_ref.peer
                || !curr->inst_ref->cell_ref->module_ref->is_blackbox) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;

            if (peer->outline.fixed || curr->inst_ref->outline.fixed) {
                image.draw_line(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, peer->outline.x*aspect_x*image_zoom, peer->outline.y*aspect_y*image_zoom, 200, 200, 200, 100);
            }
            else
            if (peer->bunch_ref.peer != inst.bunch_ref.peer) {
if (mode == 1) {
                image.draw_line(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, peer->outline.x*aspect_x*image_zoom, peer->outline.y*aspect_y*image_zoom, 255, 0, 0, 100);
}
            }
            else {
if (mode == 1) {
                image.draw_line(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, peer->outline.x*aspect_x*image_zoom, peer->outline.y*aspect_y*image_zoom, 0, 200, 200, 100);
}
            }

            if (peer->mark != travers_mark) {
//                peer->mark = travers_mark;
                recurseDrawDesign(*peer, nullptr, mode, depth + 1);
            }
        }
    }


    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseDrawDesign(*subbunch.reg, &subbunch, mode, depth + 1);
        }
    }
}

void OutlineDesign::recurseDumpDesign(rtl::Inst& inst, RegBunch* bunch, FILE* out, int depth)
{
    if (inst.mark == travers_mark /*&& bunch == nullptr*/) {
        return;
    }
    inst.mark = travers_mark;

    std::print(out, "{} ({}), coords: {},{}\n", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y);

    if (inst.cell_ref->type.find("LUT") != std::string::npos) {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 255, 0, 255);
    }
    else {
        image.set_pixel(inst.outline.x*aspect_x*image_zoom, inst.outline.y*aspect_y*image_zoom, 0, 0, 255, 255);
    }

    for (auto& conn : std::ranges::views::reverse(inst.conns)) {
        rtl::Conn* curr = &conn;
        if (curr->port_ref->type == rtl::Port::PORT_IN) {
            if (tech->check_clocked(curr->inst_ref->cell_ref->type, curr->port_ref->name)) {  // excluding clock ports
                continue;
            }

            curr = curr->follow();
            if (!curr || curr->port_ref->is_global || !curr->inst_ref.peer
                || !curr->inst_ref->cell_ref->module_ref->is_blackbox) {  // after BUFs (can be something?)
                continue;
            }

            rtl::Inst* peer = curr->inst_ref.peer;

            if (peer->mark != travers_mark) {
//                peer->mark = travers_mark;
                recurseDumpDesign(*peer, nullptr, out, depth + 1);
            }
        }
    }


    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseDumpDesign(*subbunch.reg, &subbunch, out, depth + 1);
        }
    }
}
