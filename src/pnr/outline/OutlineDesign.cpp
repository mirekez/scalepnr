#include "OutlineDesign.h"
#include "OutlineGrid.h"
#include "Device.h"
#include "Tech.h"

#include <algorithm>
#include <array>
#include <bit>
#include <charconv>
#include <chrono>
#include <cstdlib>
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

size_t OutlineDesign::prepareSharedCombLinks(std::list<Referable<RegBunch>>& bunch_list)
{
    shared_comb_links.clear();
    std::unordered_set<rtl::Inst*> visited;
    size_t count = 0;
    auto visit = [&](auto&& self, rtl::Inst& inst) -> void {
        if (!visited.insert(&inst).second) return;
        for (auto& input : inst.conns) {
            if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN
                || input.port_ref->is_global
                || tech->check_clocked(inst.cell_ref->type, input.port_ref->name)) continue;
            auto* output = input.follow();
            auto* driver = output ? output->inst_ref.peer : nullptr;
            if (!driver || !output->port_ref.peer || output->port_ref->is_global
                || !driver->cell_ref.peer || !driver->cell_ref->module_ref.peer
                || !driver->cell_ref->module_ref->is_blackbox) continue;
            auto* sink_bunch = inst.bunch_ref.peer;
            auto* source_bunch = driver->bunch_ref.peer;
            if (sink_bunch && source_bunch && sink_bunch != source_bunch
                && !isGravityAttractor(*driver)) {
                auto& links = shared_comb_links[sink_bunch];
                bool represented = std::ranges::any_of(sink_bunch->uplinks,
                    [&](const BunchLink& link) {
                        return link.conn && link.conn->inst_ref.peer
                            && link.conn->inst_ref->bunch_ref.peer == source_bunch;
                    });
                if (!represented && std::ranges::find(links, source_bunch) == links.end()) {
                    links.push_back(source_bunch);
                    ++count;
                }
            }
            self(self, *driver);
        }
    };
    auto visit_bunch = [&](auto&& self, RegBunch& bunch) -> void {
        if (bunch.reg) visit(visit, *bunch.reg);
        for (auto& child : bunch.sub_bunches) self(self, child);
    };
    for (auto& bunch : bunch_list) visit_bunch(visit_bunch, bunch);
    return count;
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

    if (auto found = shared_comb_links.find(&bunch); found != shared_comb_links.end()) {
        for (auto* linked : found->second) {
            // A shared LUT has one owner but every consumer needs proximity
            // to it. Repair the planning graph, not Estimate's ownership tree.
            // This direct connection does not pull an entire register subtree.
            attractBunch(*linked, bunch.x, bunch.y, 0, &bunch, false);
            attractBunch(bunch, linked->x, linked->y, 0, linked, false);
            int distance = std::abs(static_cast<int>(bunch.x-linked->x))
                + std::abs(static_cast<int>(bunch.y-linked->y));
            if (distance > 1) sum_distance += distance;
        }
    }

    PNR_LOG2_("OUTL", depth, "recurseSecondaryLinks, bunch: {} ({}), sum_distance: {}, uplinks: {}, timing: {}, placed: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type,
        sum_distance, bunch.uplinks.size(), timing_uplinks, timing_uplinks_placed);
    return sum_distance;
}

RadialAnchorGuide OutlineDesign::prepareRadialAnchorGuides(RegBunch& bunch)
{
    RadialAnchorGuide guide;
    if (bunch.fixed) {
        guide.x_sum = bunch.x;
        guide.y_sum = bunch.y;
        guide.count = 1;
    }
    for (auto& child : bunch.sub_bunches) {
        RadialAnchorGuide child_guide = prepareRadialAnchorGuides(child);
        guide.x_sum += child_guide.x_sum;
        guide.y_sum += child_guide.y_sum;
        guide.depth_sum += child_guide.depth_sum + child_guide.count;
        guide.count += child_guide.count;
    }
    radial_anchor_guides[&bunch] = guide;
    return guide;
}

void OutlineDesign::recurseRadialAllocation(
    RegBunch& bunch, float x, float y, int depth)
{
    PNR_LOG2_("OUTL", depth, "recurseRadialAllocation, bunch: {} ({}), x: {}, y: {}, size: {}", bunch.reg->makeName(), bunch.reg->cell_ref->type, x, y, bunch.size_comb);

    float child_x = x;
    float child_y = y;
    if (bunch.fixed) {
        // A fixed root already owns the correct package-edge position. Seed
        // its descendants from that position; using the top-level caller's
        // (0, 0) made every right/bottom anchored tree begin at top-left.
        child_x = bunch.x;
        child_y = bunch.y;
    }
    else if (uniform_unanchored_allocation) {
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
        child_x = bunch.x;
        child_y = bunch.y;
    }
    else {
        auto fixed_targets = radial_anchor_guides.find(&bunch);
        if (fixed_targets != radial_anchor_guides.end()
            && fixed_targets->second.count != 0) {
            const RadialAnchorGuide& guide = fixed_targets->second;
            double target_x = guide.x_sum/guide.count;
            double target_y = guide.y_sum/guide.count;
            double remaining_edges = 1.0 + guide.depth_sum/guide.count;
            bunch.x = x + static_cast<float>((target_x - x)/remaining_edges);
            bunch.y = y + static_cast<float>((target_y - y)/remaining_edges);
            child_x = bunch.x;
            child_y = bunch.y;
        }
        else {
            int perimeter_x = std::clamp(
                static_cast<int>(std::floor(x)), 0, mesh_width - 1);
            int perimeter_y = std::clamp(
                static_cast<int>(std::floor(y)), 0, mesh_height - 1);
            bunch.x = static_cast<float>(perimeter_x) + 0.5F;
            bunch.y = static_cast<float>(perimeter_y) + 0.5F;

            if (perimeter_x == 0 && perimeter_y != mesh_height - 1) {
                ++perimeter_y;
            }
            else if (perimeter_y == mesh_height - 1
                     && perimeter_x != mesh_width - 1) {
                ++perimeter_x;
            }
            else if (perimeter_x == mesh_width - 1 && perimeter_y != 0) {
                --perimeter_y;
            }
            else if (perimeter_y == 0 && perimeter_x != 0) {
                --perimeter_x;
            }
            child_x = static_cast<float>(perimeter_x);
            child_y = static_cast<float>(perimeter_y);
        }
    }

    for (auto& subbunch : bunch.sub_bunches) {
        recurseRadialAllocation(subbunch, child_x, child_y, depth + 1);
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

    boxes1.assign(static_cast<size_t>(fpga_width)*fpga_height, 0);

    PNR_LOG1("OUTL", "optimizeOutline, fpga_width: {}, fpga_height: {}, aspect_x: {:.3f}, aspect_y: {:.3f}, step_x: {:.3f}, step_y: {:.3f}, total_regs: {}, total_comb: {}, total_bunches: {}, cells: {}, iteration_limit: {}, combs_per_box: {}",
        fpga_width, fpga_height, aspect_x, aspect_y, step_x, step_y, total_regs, total_comb, total_bunches, design_cells, iteration_limit, combs_per_box);

    shared_comb_links.clear();
    if (!std::getenv("SCALEPNR_OUTLINE_REFERENCE_MISSING_SHARED_COMBS")) {
        size_t links = prepareSharedCombLinks(bunch_list);
        std::print("\nOUTLINE_SHARED_COMB_LINKS added={} consumer_bunches={}\n",
            links, shared_comb_links.size());
    }

    radial_anchor_guides.clear();
    radial_anchor_guides.reserve(static_cast<size_t>(total_bunches));
    for (auto& bunch : bunch_list) {
        prepareRadialAnchorGuides(bunch);
    }
    for (auto& bunch : bunch_list) {
        recurseRadialAllocation(bunch, 0, 0);
    }
    if (debug_snapshot) debug_snapshot("outline_bunch_initial");

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
            if (debug_snapshot) {
                debug_snapshot(std::format(
                    "outline_bunch_{:03d}", i + 1));
            }
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
    if (debug_snapshot) debug_snapshot("outline_instance_initial");

    travers_mark = rtl::Inst::genMark();
    optimization_peers.clear();
    optimization_peers.reserve(static_cast<size_t>(design_cells));
    optimization_sinks.clear();
    optimization_sinks.reserve(static_cast<size_t>(design_cells));
    optimization_drivers.clear();
    optimization_drivers.reserve(static_cast<size_t>(design_cells));
    optimization_order.clear();
    optimization_order.reserve(static_cast<size_t>(design_cells));
    optimization_edges.clear();
    optimization_edges.reserve(static_cast<size_t>(design_cells));
    for (auto& bunch : bunch_list) {
        recurseInstPrepare(*bunch.reg, &bunch);
    }
    std::vector<std::pair<std::string, rtl::Inst*>> named_order;
    named_order.reserve(optimization_order.size());
    for (rtl::Inst* inst : optimization_order) {
        named_order.emplace_back(inst->makeName(
            std::numeric_limits<size_t>::max()), inst);
    }
    std::ranges::sort(named_order, {},
                      &std::pair<std::string, rtl::Inst*>::first);
    optimization_order.clear();
    optimization_order.reserve(named_order.size());
    std::unordered_map<rtl::Inst*, size_t> optimization_rank;
    optimization_rank.reserve(named_order.size());
    for (const auto& [name, inst] : named_order) {
        (void)name;
        optimization_rank.emplace(inst, optimization_order.size());
        optimization_order.push_back(inst);
    }
    for (auto& [inst, peers] : optimization_peers) {
        (void)inst;
        std::ranges::stable_sort(peers, {}, [&](rtl::Inst* peer) {
            auto found = optimization_rank.find(peer);
            return found == optimization_rank.end()
                ? std::numeric_limits<size_t>::max() : found->second;
        });
    }
    struct FixedEdgeStats {
        size_t anchors = 0;
        size_t peers = 0;
        size_t combinational_peers = 0;
        size_t attractor_peers = 0;
    };
    std::array<FixedEdgeStats, 4> fixed_edge_stats{};
    const auto& physical_device = fpga::Device::current();
    for (const auto& [inst, peers] : optimization_peers) {
        if (!inst || !inst->outline.fixed || !inst->tile.peer) continue;
        const fpga::Coord coordinate = inst->tile->coord;
        std::array<bool, 4> sides{
            coordinate.y == 0,
            coordinate.x == physical_device.size_width - 1,
            coordinate.y == physical_device.size_height - 1,
            coordinate.x == 0,
        };
        for (size_t side = 0; side < sides.size(); ++side) {
            if (!sides[side]) continue;
            ++fixed_edge_stats[side].anchors;
            for (rtl::Inst* peer : peers) {
                if (!peer) continue;
                ++fixed_edge_stats[side].peers;
                if (isGravityAttractor(*peer)) {
                    ++fixed_edge_stats[side].attractor_peers;
                }
                else {
                    ++fixed_edge_stats[side].combinational_peers;
                }
            }
        }
    }
    std::print(
        "\nOUTLINE_FIXED_GRAPH top={}/{}/{}/{} right={}/{}/{}/{} "
        "bottom={}/{}/{}/{} left={}/{}/{}/{}",
        fixed_edge_stats[0].anchors, fixed_edge_stats[0].peers,
        fixed_edge_stats[0].combinational_peers,
        fixed_edge_stats[0].attractor_peers,
        fixed_edge_stats[1].anchors, fixed_edge_stats[1].peers,
        fixed_edge_stats[1].combinational_peers,
        fixed_edge_stats[1].attractor_peers,
        fixed_edge_stats[2].anchors, fixed_edge_stats[2].peers,
        fixed_edge_stats[2].combinational_peers,
        fixed_edge_stats[2].attractor_peers,
        fixed_edge_stats[3].anchors, fixed_edge_stats[3].peers,
        fixed_edge_stats[3].combinational_peers,
        fixed_edge_stats[3].attractor_peers);

    size_t component_count = 0;
    size_t unanchored_components = 0;
    size_t single_side_components = 0;
    size_t opposing_side_components = 0;
    size_t largest_component = 0;
    size_t largest_opposing_component = 0;
    std::unordered_set<rtl::Inst*> component_seen;
    component_seen.reserve(optimization_peers.size());
    for (const auto& [seed, unused] : optimization_peers) {
        (void)unused;
        if (!seed || !component_seen.insert(seed).second) continue;
        ++component_count;
        size_t component_cells = 0;
        unsigned side_mask = 0;
        std::vector<rtl::Inst*> stack{seed};
        while (!stack.empty()) {
            rtl::Inst* inst = stack.back();
            stack.pop_back();
            ++component_cells;
            if (inst->outline.fixed && inst->tile.peer) {
                const fpga::Coord coordinate = inst->tile->coord;
                side_mask |= coordinate.y == 0 ? 1U : 0U;
                side_mask |= coordinate.x == physical_device.size_width - 1
                    ? 2U : 0U;
                side_mask |= coordinate.y == physical_device.size_height - 1
                    ? 4U : 0U;
                side_mask |= coordinate.x == 0 ? 8U : 0U;
            }
            auto found = optimization_peers.find(inst);
            if (found == optimization_peers.end()) continue;
            for (rtl::Inst* peer : found->second) {
                if (peer && component_seen.insert(peer).second) {
                    stack.push_back(peer);
                }
            }
        }
        largest_component = std::max(largest_component, component_cells);
        if (side_mask == 0) ++unanchored_components;
        if (std::popcount(side_mask) == 1) ++single_side_components;
        bool opposing = (side_mask & 0x5U) == 0x5U
            || (side_mask & 0xAU) == 0xAU;
        if (opposing) {
            ++opposing_side_components;
            largest_opposing_component = std::max(
                largest_opposing_component, component_cells);
        }
    }
    std::print(
        "\nOUTLINE_COMPONENTS count={} unanchored={} single_side={} "
        "opposing_sides={} largest={} largest_opposing={}",
        component_count, unanchored_components, single_side_components,
        opposing_side_components, largest_component,
        largest_opposing_component);
    fflush(stdout);

    int instance_iteration_limit = outlineInstanceIterationLimit(
        iteration_limit, fpga_width, fpga_height);
    tech->place.place_timing.tech = tech;
    tech->place.place_timing.preparePlacementGuide(tech->timings);
    struct FixedWeightStats {
        double total = 0;
        double maximum = 0;
        size_t edges = 0;
    };
    std::array<FixedWeightStats, 4> fixed_weight_stats{};
    for (const auto& [inst, peers] : optimization_peers) {
        if (!inst || !inst->outline.fixed || !inst->tile.peer) continue;
        const fpga::Coord coordinate = inst->tile->coord;
        std::array<bool, 4> sides{
            coordinate.y == 0,
            coordinate.x == physical_device.size_width - 1,
            coordinate.y == physical_device.size_height - 1,
            coordinate.x == 0,
        };
        for (rtl::Inst* peer : peers) {
            if (!peer) continue;
            double weight = tech->place.place_timing
                .placementNetWeight(*inst, *peer);
            for (size_t side = 0; side < sides.size(); ++side) {
                if (!sides[side]) continue;
                fixed_weight_stats[side].total += weight;
                fixed_weight_stats[side].maximum = std::max(
                    fixed_weight_stats[side].maximum, weight);
                ++fixed_weight_stats[side].edges;
            }
        }
    }
    auto average_fixed_weight = [&](size_t side) {
        return fixed_weight_stats[side].edges == 0 ? 0.0
            : fixed_weight_stats[side].total
                / fixed_weight_stats[side].edges;
    };
    std::print(
        "\nOUTLINE_FIXED_WEIGHT top={:.3f}/{:.3f} "
        "right={:.3f}/{:.3f} bottom={:.3f}/{:.3f} left={:.3f}/{:.3f}",
        average_fixed_weight(0), fixed_weight_stats[0].maximum,
        average_fixed_weight(1), fixed_weight_stats[1].maximum,
        average_fixed_weight(2), fixed_weight_stats[2].maximum,
        average_fixed_weight(3), fixed_weight_stats[3].maximum);
    fflush(stdout);
    timing_attraction_roots = 0;
    timing_attraction_zero_force_roots = 0;
    timing_attraction_moved_cells = 0;
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

        std::fill(boxes1.begin(), boxes1.end(), 0);
        travers_mark = rtl::Inst::genMark();
        for (auto& bunch : bunch_list) {
//std::print("{} --- {} ({})\n", i, bunch.reg->makeName(), bunch.reg->cell_ref->type);fflush(stdout);
            recurseOptimizeInsts(*bunch.reg, &bunch, i);
        }
        moveTimingAttractorsSimultaneously();
        if ((i + 1) % 25 == 0 || i + 1 == instance_iteration_limit) {
            double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - instance_phase_start).count();
            std::print("\nOUTLINE_PROGRESS phase=instance iteration={}/{} elapsed_s={:.3f}",
                i + 1, instance_iteration_limit, elapsed);
            fflush(stdout);
            if (debug_snapshot) {
                debug_snapshot(std::format(
                    "outline_instance_{:03d}", i + 1));
            }
        }
    }

    // The instance phase above performs the complete timing attraction. Keep
    // it local: each clocked cell decides from its own timing connections and
    // carries that movement only through a bounded combinational neighborhood.

    if (legalize_capacity_in_outline) {
        legalizeOutlineCapacity();
    }
    if (debug_snapshot) debug_snapshot("outline_final");
    double instance_phase_seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - instance_phase_start).count();
    std::print("\nOUTLINE_SUMMARY cells={} bunch_iterations={} instance_iterations={} timing_attraction_roots={} timing_attraction_zero_force_roots={} timing_attraction_moved_cells={} timing_attraction_rejected=0 directed_edges={} bunch_s={:.3f} instance_s={:.3f}",
        design_cells, bunch_iteration_limit, instance_iteration_limit,
        timing_attraction_roots, timing_attraction_zero_force_roots,
        timing_attraction_moved_cells, optimization_edges.size(),
        bunch_phase_seconds, instance_phase_seconds);
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
    capacity_history.clear();
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
        bool used_preferred_directly = false;
        double best_score = std::numeric_limits<double>::infinity();
        double selected_peer_distance = 0;
        int selected_assigned_peers = 0;
        auto peers = optimization_peers.find(inst);
        size_t preferred_index = static_cast<size_t>(
            preferred.y*width + preferred.x);
        int preferred_peer_distance_sum = 0;
        int preferred_assigned_peers = 0;
        if (peers != optimization_peers.end()) {
            for (rtl::Inst* peer : peers->second) {
                auto placed_peer = assigned.find(peer);
                if (placed_peer == assigned.end()) continue;
                preferred_peer_distance_sum += std::abs(
                    preferred.x - placed_peer->second.x)
                    + std::abs(preferred.y - placed_peer->second.y);
                ++preferred_assigned_peers;
            }
        }
        double preferred_peer_distance = preferred_assigned_peers == 0 ? 0
            : static_cast<double>(preferred_peer_distance_sum)
                / preferred_assigned_peers;
        bool preferred_available = occupancy[preferred_index][*type]
            < capacity[preferred_index][*type];
        auto considerCandidate = [&](Coord candidate) {
            if (candidate.x < 0 || candidate.x >= width
                || candidate.y < 0 || candidate.y >= height) {
                return;
            }
            size_t index = static_cast<size_t>(
                candidate.y*width + candidate.x);
            if (index >= device.tile_grid.size()) {
                return;
            }
            const fpga::Tile& tile = device.tile_grid[index];
            if (!tile.tile_type || tile.coord.x != candidate.x
                || tile.coord.y != candidate.y) {
                return;
            }
            if (occupancy[index][*type] >= capacity[index][*type]) {
                return;
            }
            int target_distance = std::abs(candidate.x - preferred.x)
                + std::abs(candidate.y - preferred.y);
            int peer_distance = 0;
            int assigned_peers = 0;
            if (peers != optimization_peers.end()) {
                for (rtl::Inst* peer : peers->second) {
                    auto placed_peer = assigned.find(peer);
                    if (placed_peer == assigned.end()) continue;
                    peer_distance += std::abs(
                        candidate.x - placed_peer->second.x)
                        + std::abs(
                            candidate.y - placed_peer->second.y);
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
                selected = candidate;
                found = true;
                selected_peer_distance = average_peer_distance;
                selected_assigned_peers = assigned_peers;
            }
        };
        // Smear an overfull preferred Tile into the nearest physical ring.
        // Searching the device's row-major Tile array made the result depend on
        // array order and could send a cell from the middle of the device to an
        // edge.  Complete one Manhattan ring before considering a farther one;
        // the timing score only chooses between equally near legal positions.
        for (int radius = 0; radius < width + height && !found; ++radius) {
            for (int dx = -radius; dx <= radius; ++dx) {
                int dy = radius - std::abs(dx);
                considerCandidate({preferred.x + dx, preferred.y + dy});
                if (dy != 0) {
                    considerCandidate({preferred.x + dx, preferred.y - dy});
                }
            }
            used_preferred_directly = radius == 0 && found;
        }
        PNR_ASSERT(found,
            "Outline capacity legalization found no '{}' element for '{}'",
            fpga::elementTypeName(*type), inst->makeName());
        int movement = std::abs(selected.x - preferred.x)
            + std::abs(selected.y - preferred.y);
        moved_cells += movement != 0;
        maximum_move = std::max(maximum_move, movement);
        if (record_capacity_history) {
            capacity_history.push_back(OutlineCapacityTrace{
                .inst = inst,
                .preferred = preferred,
                .selected = selected,
                .assignment_order = assigned.size(),
                .preferred_occupancy = occupancy[preferred_index][*type],
                .preferred_capacity = capacity[preferred_index][*type],
                .assigned_peers = selected_assigned_peers,
                .preferred_peer_distance = preferred_peer_distance,
                .selected_peer_distance = selected_peer_distance,
                .selected_score = best_score,
                .preferred_available = preferred_available,
                .used_preferred_directly = used_preferred_directly,
            });
        }
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
            auto add_peer = [&](rtl::Inst* owner, rtl::Inst* linked) {
                auto [found, inserted] = optimization_peers.try_emplace(owner);
                if (inserted) optimization_order.push_back(owner);
                found->second.push_back(linked);
            };
            add_peer(&inst, peer);
            add_peer(peer, &inst);
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
            if (peer->mark == travers_mark) {
                continue;
            }
            if (peer->bunch_ref.peer == inst.bunch_ref.peer) {
                recurseOptimizeInsts(*peer, nullptr, depth + 1);
            }
        }
    }

    if (bunch) {
        for (auto& subbunch : bunch->sub_bunches) {
            recurseOptimizeInsts(*subbunch.reg, &subbunch, i, depth + 1);
        }
    }
}

size_t OutlineDesign::moveTimingAttractorsSimultaneously()
{
    struct Position {
        double x = 0;
        double y = 0;
    };
    struct Movement {
        double x = 0;
        double y = 0;
        // Registers have one resolved target; followers combine several roots.
        bool resolved = false;
        float target_x = 0;
        float target_y = 0;
        size_t follower_roots = 0;
    };
    struct Propagation {
        rtl::Inst* inst = nullptr;
        int hop = 0;
        double weight = 1;
    };

    constexpr double attraction_step = 0.10;
    constexpr double maximum_step = 0.75;
    constexpr double propagation_fade = 0.50;
    constexpr int propagation_hops = 4;

    if (!tech || aspect_x <= 0 || aspect_y <= 0
        || fpga_width <= 0 || fpga_height <= 0) {
        return 0;
    }

    // All decisions in this pass use one immutable picture. In particular, a
    // register processed later cannot see where an earlier register moved.
    std::unordered_map<rtl::Inst*, Position> positions;
    positions.reserve(optimization_peers.size());
    for (rtl::Inst* inst : optimization_order) {
        const auto& peers = optimization_peers.at(inst);
        if (inst) {
            positions.try_emplace(inst, Position{
                inst->outline.x*aspect_x, inst->outline.y*aspect_y});
        }
        for (rtl::Inst* peer : peers) {
            if (peer) {
                positions.try_emplace(peer, Position{
                    peer->outline.x*aspect_x, peer->outline.y*aspect_y});
            }
        }
    }

    std::unordered_map<rtl::Inst*, Movement> movements;
    movements.reserve(positions.size());
    // Opt-in diagnostic: attribute a follower's displacement to its roots,
    // including how far those roots actually moved after boundary clamping.
    rtl::Inst* traced = nullptr;
    static size_t trace_pass = 0;
    if (const char* name = std::getenv("SCALEPNR_OUTLINE_TRACE_CELL")) {
        ++trace_pass;
        const char* first = std::getenv("SCALEPNR_OUTLINE_TRACE_FIRST_PASS");
        const char* last = std::getenv("SCALEPNR_OUTLINE_TRACE_LAST_PASS");
        if ((!first || trace_pass >= std::strtoul(first,nullptr,10))
            && (!last || trace_pass <= std::strtoul(last,nullptr,10)))
            for (auto* inst : optimization_order)
                if (inst && inst->cell_ref.peer && inst->cell_ref->name == name) {
                    traced=inst; break;
                }
    }
    struct Contribution { rtl::Inst* root; Movement delta; };
    std::vector<Contribution> contributions;
    rtl::Inst* movement_root = nullptr;
    auto bounded = [](Movement movement, double limit) {
        double length = std::hypot(movement.x, movement.y);
        if (length > limit && length > 0) {
            movement.x *= limit/length;
            movement.y *= limit/length;
        }
        return movement;
    };
    auto feasibleTarget = [&](rtl::Inst* inst, Movement movement) {
        movement = bounded(movement, maximum_step);
        const Position position = positions.at(inst);
        double minimum_x = 0;
        double maximum_x = std::max(0.0, static_cast<double>(fpga_width)-0.001);
        double minimum_y = 0;
        double maximum_y = std::max(0.0, static_cast<double>(fpga_height)-0.001);
        if (inst->bunch_ref.peer && !inst->bunch_ref->fixed) {
            minimum_x = std::max(minimum_x, (inst->bunch_ref->x-0.5)*aspect_x);
            maximum_x = std::min(maximum_x, (inst->bunch_ref->x+0.5)*aspect_x);
            minimum_y = std::max(minimum_y, (inst->bunch_ref->y-0.5)*aspect_y);
            maximum_y = std::min(maximum_y, (inst->bunch_ref->y+0.5)*aspect_y);
            if (minimum_x > maximum_x) minimum_x = maximum_x;
            if (minimum_y > maximum_y) minimum_y = maximum_y;
        }
        // Resolve the actual float Outline coordinates before propagating.
        return std::pair<float,float>{
            std::clamp(position.x+movement.x,minimum_x,maximum_x)/aspect_x,
            std::clamp(position.y+movement.y,minimum_y,maximum_y)/aspect_y};
    };
    // Same-start diagnostic control only; normal placement uses feasible motion.
    const bool reference_propagation = std::getenv(
        "SCALEPNR_OUTLINE_REFERENCE_UNCLIPPED_PROPAGATION") != nullptr;
    auto add_movement = [&](rtl::Inst* inst, Movement movement) {
        if (!inst || inst->outline.fixed) {
            return;
        }
        movements[inst].x += movement.x;
        movements[inst].y += movement.y;
        if (inst == traced) contributions.push_back({movement_root,movement});
    };

    // Relax each follower relative to its connected root's feasible position,
    // not just by copying the root's translation. Copying translations leaves
    // an existing COMB detour intact, and supplies no correction at all when
    // the register cannot move. Roots still originate every force; a COMB
    // without a register/I/O in this bounded neighborhood never moves itself.
    // Another register is a new force origin, not a follower: it must decide
    // from its own connections in the immutable picture above.  Otherwise a
    // register can be dragged repeatedly by several surrounding roots and a
    // whole region moves as one rigid cloud.
    auto propagate = [&](rtl::Inst* source, Position target, int first_hop) {
        std::unordered_set<rtl::Inst*> visited;
        visited.reserve(32);
        visited.insert(source);
        std::deque<Propagation> pending;
        auto source_peers = optimization_peers.find(source);
        if (source_peers != optimization_peers.end()) {
            for (rtl::Inst* peer : source_peers->second) {
                if (peer && !peer->outline.fixed
                    && visited.insert(peer).second) {
                    pending.push_back({peer, first_hop, tech->place.place_timing
                        .placementNetWeight(*source, *peer)});
                }
            }
        }
        while (!pending.empty()) {
            Propagation current = pending.front();
            pending.pop_front();
            if (isGravityAttractor(*current.inst)) {
                continue;
            }
            double fade = std::pow(propagation_fade, current.hop);
            const auto position = positions.at(current.inst);
            add_movement(current.inst, {
                attraction_step*current.weight*fade*(target.x-position.x),
                attraction_step*current.weight*fade*(target.y-position.y)});
            ++movements.at(current.inst).follower_roots;
            if (current.hop >= propagation_hops) {
                continue;
            }
            auto peers = optimization_peers.find(current.inst);
            if (peers == optimization_peers.end()) {
                continue;
            }
            for (rtl::Inst* peer : peers->second) {
                if (peer && !peer->outline.fixed
                    && visited.insert(peer).second) {
                    pending.push_back({peer, current.hop + 1, current.weight});
                }
            }
        }
    };

    for (rtl::Inst* attractor : optimization_order) {
        movement_root = attractor;
        const auto& peers = optimization_peers.at(attractor);
        if (!attractor || !isGravityAttractor(*attractor) || peers.empty()) {
            continue;
        }
        ++timing_attraction_roots;
        const Position origin = positions.at(attractor);
        if (attractor->outline.fixed) {
            // Fixed I/O retains full strength at its immediate follower;
            // subsequent connections fade. Never propagate through a REG.
            propagate(attractor, origin, 0);
            continue;
        }

        Movement personal;
        size_t connected_peers = 0;
        for (rtl::Inst* peer : peers) {
            if (!peer) {
                continue;
            }
            const Position target = positions.at(peer);
            double dx = target.x - origin.x;
            double dy = target.y - origin.y;
            double distance = std::hypot(dx, dy);
            if (distance <= 0.000001) {
                continue;
            }
            double weight = tech->place.place_timing
                .placementNetWeight(*attractor, *peer);
            // This is a spring vector, not a compass direction.  Retaining
            // distance is what lets an uneven register chain pull itself taut:
            // equal weights at unequal distances must not cancel.
            personal.x += weight*dx;
            personal.y += weight*dy;
            ++connected_peers;
        }
        if (connected_peers != 0) {
            personal.x *= attraction_step/connected_peers;
            personal.y *= attraction_step/connected_peers;
        }
        personal = bounded(personal, maximum_step);
        const auto target = feasibleTarget(attractor, personal);
        if (!reference_propagation) {
            personal.x = target.first*aspect_x-origin.x;
            personal.y = target.second*aspect_y-origin.y;
        }
        if (std::hypot(personal.x,personal.y) <= 0.000001)
            ++timing_attraction_zero_force_roots;
        add_movement(attractor, personal);
        propagate(attractor, {origin.x+personal.x, origin.y+personal.y}, 1);
        auto& movement = movements.at(attractor);
        movement.resolved = true;
        movement.target_x = target.first;
        movement.target_y = target.second;
    }

    size_t moved = 0;
    for (auto& [inst, movement] : movements) {
        // Several connected roots define one relative-position correction.
        // Do not amplify its speed merely because the follower has fanout.
        if (movement.follower_roots) {
            movement.x /= movement.follower_roots;
            movement.y /= movement.follower_roots;
        }
        if (std::hypot(movement.x, movement.y) <= 0.000001) {
            continue;
        }
        auto target = movement.resolved
            ? std::pair{movement.target_x,movement.target_y} : feasibleTarget(inst,movement);
        if (inst->outline.x == target.first && inst->outline.y == target.second) continue;
        inst->outline.x = target.first;
        inst->outline.y = target.second;
        ++moved;
        ++timing_attraction_moved_cells;
    }
    if (traced) {
        // The attraction grid has twice the physical Tile resolution.
        const auto from=positions.at(traced);
        std::print("\nOUTLINE_FORCE_CELL pass={} cell={} from=({:.6f},{:.6f}) to=({:.6f},{:.6f})\n",
            trace_pass,traced->makeName(200),from.x/2,from.y/2,
            traced->outline.x*aspect_x/2,traced->outline.y*aspect_y/2);
        for (const auto& contribution : contributions) {
            auto* root=contribution.root;
            const auto origin=positions.at(root);
            std::print("OUTLINE_FORCE_ROOT pass={} root={} fixed={} from=({:.6f},{:.6f}) to=({:.6f},{:.6f}) follower_delta=({:.6f},{:.6f})\n",
                trace_pass,root->makeName(200),root->outline.fixed,
                origin.x/2,origin.y/2,root->outline.x*aspect_x/2,root->outline.y*aspect_y/2,
                contribution.delta.x/(2*std::max(size_t{1},movements.at(traced).follower_roots)),
                contribution.delta.y/(2*std::max(size_t{1},movements.at(traced).follower_roots)));
        }
    }
    return moved;
}

bool OutlineDesign::isGravityAttractor(const rtl::Inst& inst) const
{
    if (inst.outline.fixed) {
        return true;
    }
    if (!tech || !inst.cell_ref.peer) {
        return false;
    }
    const std::string& type = inst.cell_ref.peer->type;
    return tech->clocked_ports.find(type) != tech->clocked_ports.end()
        || tech->buffers_ports.find(type) != tech->buffers_ports.end();
}

void OutlineDesign::attractInst(rtl::Inst& inst, RegBunch* bunch, float step, float x, float y, int i, rtl::Inst* exclude, int depth)
{
    PNR_LOG3_("OUTL", depth, "attractInst, inst: {} ({}), outline.x: {}, outline.y: {}, x: {}, y: {}, step: {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y, step);

//if (inst.makeName() == "$abc$712025$auto$blifparse.cc:535:parse_blif$718286") {
//    std::print("\n!!!!!!!!!!!!!!!!!! attractInst, inst: {} ({}), outline.x: {}, outline.y: {}, x: {}, y: {}, step: {}", inst.makeName(), inst.cell_ref->type, inst.outline.x, inst.outline.y, x, y, step);
//}
    PNR_ASSERT(inst.bunch_ref.peer != nullptr, "bunch ref of inst '{}' is zero", inst.makeName());

    float candidate_x = inst.outline.x
        + (x > inst.outline.x ? step : -step);
    float candidate_y = inst.outline.y
        + (y > inst.outline.y ? step : -step);
    int physical_x = std::clamp(
        static_cast<int>(candidate_x*aspect_x), 0, fpga_width - 1);
    int physical_y = std::clamp(
        static_cast<int>(candidate_y*aspect_y), 0, fpga_height - 1);
    bool within_bunch = candidate_x < inst.bunch_ref->x + 0.5F
        && candidate_x > inst.bunch_ref->x - 0.5F
        && candidate_y < inst.bunch_ref->y + 0.5F
        && candidate_y > inst.bunch_ref->y - 0.5F;

    if (!inst.outline.fixed
        && ((i > 50 && boxes1[physical_y*fpga_width + physical_x] == 0)
            || within_bunch)) {

        inst.outline.x += (x-step_x > inst.outline.x ? step : (x+step_x < inst.outline.x ? -step : 0));
        inst.outline.y += (y-step_y > inst.outline.y ? step : (y+step_y < inst.outline.y ? -step : 0));

//if (inst.makeName() == "$abc$712025$auto$blifparse.cc:535:parse_blif$718286") {
//    std::print("\n!!!!!!!!!!!!!!!!!! attractInst, x: {}, y: {}", inst.outline.x, inst.outline.y);
//}

        auto peers = optimization_peers.find(&inst);
        if (peers != optimization_peers.end()) {
            for (rtl::Inst* peer : peers->second) {
                if (depth < 4 && step > step_x/5 && peer != exclude
                    && !isGravityAttractor(*peer)) {
                    attractInst(*peer, peer->bunch_ref.peer, step/2, x, y,
                                i, &inst, depth + 1);
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
