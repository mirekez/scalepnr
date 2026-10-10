#include "pnr.h"
#include "Tech.h"
#include "Device.h"
#include "Element.h"
#include "Wire.h"
#include "on_return.h"

#include <algorithm>
#include <chrono>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace pnr {
namespace {
void collect(rtl::Inst& inst, std::vector<rtl::Inst*>& cells) {
    if (inst.cell_ref.peer && inst.cell_ref->module_ref.peer &&
        inst.cell_ref->module_ref->is_blackbox &&
        inst.cell_ref->type != "VCC" && inst.cell_ref->type != "GND" &&
        !inst.cell_ref->attributes.contains("scalepnr_constant")) cells.push_back(&inst);
    for (auto& child : inst.insts) collect(child, cells);
}
double seconds(std::chrono::steady_clock::time_point begin) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now()-begin).count();
}
bool complete(const rtl::NetRouteBinding& binding) {
    if (!binding.owner || binding.route_index >= binding.owner->wires.size()) return false;
    const auto& route = binding.owner->wires[binding.route_index];
    return fpga::isRouteComplete(route);
}

void validateConnections(technology::Tech& tech, const std::vector<rtl::Inst*>& cells) {
    // Index by sink, not logical driver: constants and local-chain helpers can
    // replace a logical source with a physical one during this run.
    std::unordered_map<rtl::Inst*, std::unordered_set<std::string>> routed;
    std::unordered_map<rtl::Module*, std::unordered_set<int>> direct;
    for (auto& module : tech.design.modules) for (auto& net : module.nets) {
        for (int id : net.designators) if (net.designatorIsVoid(id)) direct[&module].insert(id);
        for (const auto& binding : net.routes)
            if (binding.to && complete(binding)) routed[binding.to].insert(binding.to_port);
    }
    for (auto* sink : cells) {
        if (sink->pnr_pending) continue;
        for (auto& input : sink->conns) {
            if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) continue;
            auto* driver = input.follow();
            if (!driver || !driver->inst_ref.peer || driver->inst_ref->pnr_pending ||
                !driver->inst_ref->cell_ref.peer ||
                !driver->inst_ref->cell_ref->module_ref->is_blackbox || driver->port_ref->is_global) continue;
            auto* module = sink->cell_ref->module_ref->parent_ref.peer;
            if (direct[module].contains(input.port_ref->designator)) continue;
            if (!routed[sink].contains(input.port_ref->makeName()))
                throw std::runtime_error("unrouted admitted input: "+sink->makeName()+"/"+input.port_ref->makeName());
        }
    }
}
}

void Pnr::plan() {
    if (!run_cells) throw std::invalid_argument("PnR run cell target must be positive");
    runs.clear();
    failure.clear();
    std::vector<rtl::Inst*> cells;
    collect(tech_.design.top, cells);
    std::unordered_map<rtl::Inst*, size_t> index;
    std::vector<size_t> parent(cells.size());
    std::iota(parent.begin(), parent.end(), 0);
    auto root = [&](size_t id) {
        while (parent[id] != id) { parent[id] = parent[parent[id]]; id = parent[id]; }
        return id;
    };
    auto join = [&](size_t a, size_t b) { parent[root(b)] = root(a); };
    std::unordered_map<RegBunch*, size_t> bunches;
    for (size_t i=0; i<cells.size(); ++i) {
        index.emplace(cells[i], i);
        if (auto* bunch = cells[i]->bunch_ref.peer) {
            auto [it, inserted] = bunches.emplace(bunch, i);
            if (!inserted) join(i, it->second);
        }
    }
    for (size_t i=0; i<cells.size(); ++i) {
        if (auto* bunch = cells[i]->bunch_ref.peer; bunch && index.contains(bunch->reg))
            join(i, index.at(bunch->reg));
    }
    // Keep existing strict local mux/carry chains indivisible across bunches.
    // Ordinary LUT-to-register data connections are not mandatory local chains.
    std::unordered_map<rtl::Inst*, size_t> clock_consumers;
    for (size_t i=0; i<cells.size(); ++i) {
        auto sink_type = fpga::elementTypeForInst(*cells[i]);
        for (auto& input : cells[i]->conns) {
            if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) continue;
            auto* driver = input.follow();
            if (!driver || !driver->inst_ref.peer || !index.contains(driver->inst_ref.peer)) continue;
            auto* from = driver->inst_ref.peer;
            auto source_type = fpga::elementTypeForInst(*from);
            if (source_type && sink_type &&
                ((*sink_type == fpga::ELEMENT_MUXF7 && *source_type == fpga::ELEMENT_LUT5) ||
                 (*sink_type == fpga::ELEMENT_MUXF8 && *source_type == fpga::ELEMENT_MUXF7) ||
                 (*sink_type == fpga::ELEMENT_CARRY && *source_type == fpga::ELEMENT_CARRY)))
                join(i, index.at(from));
            if (tech_.check_clocked(cells[i]->cell_ref->type, input.port_ref->name)) {
                auto [it, inserted] = clock_consumers.emplace(from, i);
                auto rank = [&](size_t id) {
                    auto* ranked = cells[id]->bunch_ref.peer ? cells[id]->bunch_ref->reg : cells[id];
                    return std::pair{ranked->stats.max_deficit, ranked->stats.top_max_delay};
                };
                if (!inserted && rank(i) > rank(it->second)) it->second = i;
            }
        }
    }
    // Admit each clock cone with its highest-ranked consumer, not alone and
    // not with every consumer (which would collapse a whole domain into a run).
    // In particular an unused later domain's output buffer must not be admitted
    // before any sink exists to anchor its physical placement.
    for (const auto& [source, consumer] : clock_consumers) {
        std::vector<rtl::Inst*> pending{source};
        std::unordered_set<rtl::Inst*> seen;
        while (!pending.empty()) {
            auto* cell = pending.back(); pending.pop_back();
            if (!seen.insert(cell).second) continue;
            join(consumer, index.at(cell));
            for (auto& input : cell->conns) {
                if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) continue;
                auto* upstream = input.follow();
                if (upstream && upstream->inst_ref.peer && index.contains(upstream->inst_ref.peer))
                    pending.push_back(upstream->inst_ref.peer);
            }
        }
    }
    struct Group { std::vector<rtl::Inst*> cells; double deficit=-std::numeric_limits<double>::infinity(); double delay=0; std::string name; };
    std::unordered_map<size_t, Group> grouped;
    for (size_t i=0; i<cells.size(); ++i) {
        auto& group = grouped[root(i)];
        group.cells.push_back(cells[i]);
        const auto* ranked = cells[i]->bunch_ref.peer ? cells[i]->bunch_ref->reg : cells[i];
        group.deficit = std::max(group.deficit, ranked->stats.max_deficit);
        group.delay = std::max(group.delay, ranked->stats.top_max_delay);
        auto name = cells[i]->makeName(std::numeric_limits<size_t>::max());
        if (group.name.empty() || name < group.name) group.name = std::move(name);
    }
    std::vector<Group*> ordered;
    for (auto& [id, group] : grouped) ordered.push_back(&group);
    std::sort(ordered.begin(), ordered.end(), [](auto* a, auto* b) {
        if (a->deficit != b->deficit) return a->deficit > b->deficit;
        if (a->delay != b->delay) return a->delay > b->delay;
        return a->name < b->name;
    });
    for (auto* group : ordered) {
        if (runs.empty() || (!runs.back().cells.empty() &&
                            runs.back().cells.size()+group->cells.size() > run_cells)) runs.emplace_back();
        auto& run = runs.back();
        if (run.cells.empty()) run.deficit = group->deficit;
        else run.deficit = std::max(run.deficit, group->deficit);
        std::sort(group->cells.begin(), group->cells.end(), [](auto* a, auto* b) {
            return a->makeName(std::numeric_limits<size_t>::max()) < b->makeName(std::numeric_limits<size_t>::max());
        });
        run.cells.insert(run.cells.end(), group->cells.begin(), group->cells.end());
    }
    std::print("\nPNR_PLAN cells={} groups={} runs={} target_cells={}", cells.size(), ordered.size(), runs.size(), run_cells);
}

bool Pnr::run() {
    const int previous_verbosity = debug_level;
    debug_level = std::max(debug_level, 1);
    on_return restore_verbosity([&] { debug_level = previous_verbosity; });
    if (!tech_.design.top.cell_ref.peer) {
        failure = "load a design before pnr_design";
        return false;
    }
    if (tech_.estimate.data_outs.empty()) {
        tech_.estimate.tech = &tech_;
        tech_.estimate.clocks = &tech_.clocks;
        tech_.estimate.estimateDesign(tech_.design);
    }
    try { plan(); } catch (const std::exception& error) { failure=error.what(); return false; }
    std::vector<rtl::Inst*> all;
    collect(tech_.design.top, all);
    // Routers and Tiles retain Net pointers. Reserve helper space once, before
    // any run leases resources; reserving a growing prefix each run relocates
    // already referenced Net objects.
    for (auto& module : tech_.design.modules) {
        if (std::ranges::any_of(module.nets, [](const auto& net) { return !net.routes.empty(); })) {
            failure = "pnr_design requires fresh routing state";
            return false;
        }
    }
    std::unordered_map<rtl::Module*, size_t> module_cells;
    for (auto* cell : all) ++module_cells[cell->cell_ref->module_ref->parent_ref.peer];
    for (auto& module : tech_.design.modules)
        if (module_cells.contains(&module) || !module.nets.empty())
            module.nets.reserve(module.nets.size() + std::max(size_t{1024}, module_cells[&module]*8));
    for (auto* cell : all) cell->pnr_pending = true;
    const bool old_preserve = tech_.route.preserve_routes;
    tech_.route.preserve_routes = true;
    on_return restore_mode([&] { tech_.route.preserve_routes = old_preserve; });
    size_t completed_cells = 0;
    for (size_t number=0; number<runs.size(); ++number) {
        auto& run = runs[number];
        for (auto* cell : run.cells) cell->pnr_pending = false;
        std::print("\nPNR_RUN begin run={}/{} new_cells={} completed_cells={} pending_cells={} target={} oversized={} deficit={:.3f}",
            number+1, runs.size(), run.cells.size(), completed_cells,
            all.size()-completed_cells-run.cells.size(), run_cells, run.cells.size()>run_cells, run.deficit);
        std::fflush(stdout);
        // Stage-local flat bunch view: retain real membership while preserving
        // Estimate's original tree and stable references outside this run.
        std::list<Referable<RegBunch>> original;
        original.splice(original.end(), tech_.estimate.data_outs);
        std::unordered_map<rtl::Inst*, Referable<RegBunch>*> previous;
        std::unordered_map<RegBunch*, Referable<RegBunch>*> shadows;
        for (auto* cell : run.cells) {
            if (!cell->bunch_ref.peer) {
                auto& orphan = original.emplace_back(RegBunch{});
                orphan.reg = cell; orphan.size = 1;
                orphan.x = orphan.y = 5;
                cell->bunch_ref.set(&orphan);
            }
            auto* old = cell->bunch_ref.peer;
            previous[cell] = old;
            Referable<RegBunch>* shadow = nullptr;
            if (old && shadows.contains(old)) shadow = shadows.at(old);
            if (!shadow) {
                auto& created = tech_.estimate.data_outs.emplace_back(RegBunch{});
                shadow = &created;
                created.reg = old ? old->reg : cell;
                if (old) {
                    if (old->clk_ref.peer) created.clk_ref.set(old->clk_ref.peer);
                    created.uplinks = old->uplinks;
                    created.x = old->x; created.y = old->y;
                    shadows[old] = shadow;
                }
                created.fixed = created.reg->outline.fixed;
                created.size = 1;
            }
            cell->bunch_ref.set(shadow);
            if (cell->cnt_clocks > 0) ++shadow->size_regs_own;
            else ++shadow->size_comb_own;
            shadow->size_regs = shadow->size_regs_own;
            shadow->size_comb = shadow->size_comb_own;
        }
        on_return restore_bunches([&] {
            for (auto& [old, shadow] : shadows) {
                old->x=shadow->x; old->y=shadow->y;
                if (run.success && old->reg && old->reg->tile.peer) {
                    old->x = old->reg->outline.x;
                    old->y = old->reg->outline.y;
                }
                if (run.success) old->fixed=true;
            }
            for (auto& [cell, old] : previous) {
                cell->bunch_ref.clear();
                if (old) cell->bunch_ref.set(old);
            }
            tech_.estimate.data_outs.clear();
            tech_.estimate.data_outs.splice(tech_.estimate.data_outs.end(), original);
        });
        try {
            auto start = std::chrono::steady_clock::now();
            {
                const bool previous = tech_.place.routing.enabled;
                on_return restore_routing_preference([&] { tech_.place.routing.enabled = previous; });
                tech_.place.routing.enabled = completed_cells != 0;
                tech_.place.routing.reset();
                std::print("\nPNR_RUN placement run={} routing_aware={} frozen_cells={}",
                    number+1, tech_.place.routing.enabled, completed_cells);
                tech_.placeDesign();
            }
            run.placement_seconds = seconds(start);
            for (auto* cell : run.cells) {
                if (fpga::isPlaceableElement(*cell) && !cell->tile.peer)
                    throw std::runtime_error("unplaced cell: "+cell->makeName());
            }
            for (auto* sink : all) for (auto& input : sink->conns) {
                if (!input.port_ref.peer || input.port_ref->type != rtl::Port::PORT_IN) continue;
                auto* driver = input.follow();
                if (!driver || !driver->inst_ref.peer || driver->port_ref->is_global) continue;
                if (sink->pnr_pending || driver->inst_ref->pnr_pending) ++run.deferred_connections;
                else ++run.eligible_connections;
            }
            std::print("\nPNR_RUN placed run={} seconds={:.3f} eligible_connections={} deferred_connections={}",
                number+1, run.placement_seconds, run.eligible_connections, run.deferred_connections);
            start = std::chrono::steady_clock::now();
            tech_.routeDesign();
            run.routing_seconds = seconds(start);
            // Freeze the actual admitted state, including generated helpers.
            std::vector<rtl::Inst*> current;
            collect(tech_.design.top, current);
            validateConnections(tech_, current);
            auto timing = tech_.place.place_timing.analyze(tech_.timings);
            run.estimated_wns_ns = timing.worst_slack_ns;
            run.unplaced_timing_edges = timing.unplaced_edges;
            std::print("\nPNR_RUN timing run={} estimated_wns_ns={:.3f} endpoints={} unplaced_edges={}",
                number+1, run.estimated_wns_ns, timing.endpoints, run.unplaced_timing_edges);
            for (auto* cell : current) if (!cell->pnr_pending) {
                if (!cell->tile.peer) throw std::runtime_error("unplaced admitted cell: "+cell->makeName());
                // Routing may insert physical endpoint helpers. Keep their
                // bunch reference alive when the stage-local view is removed.
                if (!cell->pnr_completed && !previous.contains(cell)) for (const auto& [old, shadow] : shadows)
                    if (cell->bunch_ref.peer == shadow) {
                        previous[cell] = static_cast<Referable<RegBunch>*>(old);
                        break;
                    }
                cell->outline.fixed = true;
                cell->pnr_completed = true;
                cell->outline.x = cell->coord.x/tech_.place.aspect_x;
                cell->outline.y = cell->coord.y/tech_.place.aspect_y;
            }
            for (auto& module : tech_.design.modules) for (auto& net : module.nets)
                for (const auto& binding : net.routes)
                    if (complete(binding)) { net.pnr_preserved=true; break; }
            completed_cells += run.cells.size();
            run.completed_cells = completed_cells;
            run.success = true;
            if (run_completed) run_completed(run);
            std::print("\nPNR_RUN complete run={}/{} cells={} cumulative={} placement_s={:.3f} routing_s={:.3f}",
                number+1, runs.size(), run.cells.size(), completed_cells, run.placement_seconds, run.routing_seconds);
            std::fflush(stdout);
        } catch (const std::exception& error) {
            failure = error.what();
            std::print("\nPNR_RUN failed run={}/{} completed_cells={} reason='{}'\n", number+1, runs.size(), completed_cells, failure);
            return false;
        }
    }
    std::print("\nPNR_COMPLETE runs={} cells={}\n", runs.size(), completed_cells);
    return true;
}
} // namespace pnr
