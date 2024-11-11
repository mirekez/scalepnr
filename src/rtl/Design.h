#pragma once

#include "Module.h"
#include "Cell.h"
#include "referable.h"

#include <list>
#include <unordered_map>

namespace rtl
{

struct Design
{
    std::list<Referable<Module>> modules;
    Referable<CellInst> top;
    Referable<Cell> top_cell;
    std::unordered_map<std::string,Referable<Module>&> modules_map;

    bool build(const std::string& top_module)
    {
        PNR_LOG("RTL ", "building hierarchy...");
        for (auto& module : modules) {
            modules_map.emplace(module.name, module);
        }
        for (auto& module : modules) {
            PNR_LOG1("RTL ", "checking module '{}'...", module.name);
            for (auto& cell : module.cells) {
                auto submod_it = modules_map.find(cell.type);
                if (submod_it == modules_map.end()) {
                    PNR_ERROR("Cant find module with name '{}' for cell '{}'\n", cell.name, cell.type);
                    return false;
                }
                bool found = false;
                for (auto& submod : module.submodules_ref) {
                    if (submod->name == cell.type) {
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    module.submodules_ref.emplace_back(Ref<Module>());
                    module.submodules_ref.back().set(&submod_it->second);
                    PNR_LOG2("RTL ", "adding submodule '{}' to {}", submod_it->second.name, module.name);
                }
            }
        }

        auto topmod_it = modules_map.find(top_module);
        if (topmod_it == modules_map.end()) {
            PNR_ERROR("Cant find module with name '{}' for being a top module\n", top_module);
            return false;
        }
        PNR_LOG("RTL ", "found top module: '{}'", topmod_it->second.name);
        top_cell.name = "top";
        top_cell.type = topmod_it->second.name;
        top_cell.ports.reserve(topmod_it->second.ports.size());
        for (auto& port : topmod_it->second.ports) {
            top_cell.ports.emplace_back(rtl::Port());
            top_cell.ports.back().name = port.name;
            top_cell.ports.back().type = port.type;
            top_cell.ports.back().designator = port.designator;
        }

        if (build_hier(&top, topmod_it->second, top_cell) == 0) {
            return false;
        }
        if (!connect_hier(top)) {
            return false;
        }
        return true;
    }

    int build_hier(Referable<CellInst>* inst, Referable<Module>& mod, Referable<Cell>& cell, int level = 0, std::string hier_name = "")
    {
//        PNR_LOG1("RTL ", "instantiating module: '{}'...", mod.name);
        inst->depth = level++;
        inst->cell_ref.set(&cell);
        hier_name = level != 0 ? (hier_name != "" ? hier_name + "|" : "") + cell.name : hier_name;
        inst->module_ref.set(&mod);

        inst->conns.reserve(cell.ports.size());
        int i = -1;
        for (auto& port : cell.ports) {  // repeat connections in CellInst after Cell
            ++i;
            PNR_LOG1("RTL ", "creating inst '{}' connection {} for port '{}' designator {}", inst->cell_ref->name, i, port.name, port.designator);
            auto* conn = &inst->conns.emplace_back(rtl::Conn{});
            conn->port_ref.set(&port);
            conn->inst_ref.set(inst);
            conn->designator = port.designator;
        }

        int max_height = 0;
        for (auto& sub_cell : mod.cells) {
            auto* sub_inst = &inst->insts.emplace_back(/*std::make_unique<Referable<CellInst>>(*/CellInst{}/*)*/);
            auto mod_it = modules_map.find(sub_cell.type);
            if (mod_it == modules_map.end()) {
                PNR_ERROR("Cant find module with name '{}' for cell '{}' in inst '{}'\n", sub_cell.type, sub_cell.name, hier_name == "" ? "top" : hier_name);
                return 0;
            }

            PNR_LOG2("RTL ", "instantiating cell: '{}' ({}) of module: {}, level: {}", hier_name != "" ? hier_name + "|" + sub_cell.name : sub_cell.name,
                sub_cell.type, mod.name, level);

            int height = 0;
            if ((height = build_hier(sub_inst, mod_it->second, sub_cell, level, hier_name)) == 0) {
                return 0;
            }
            if (height > max_height) {
                max_height = height;
            }
            sub_inst->parent.set(inst);

        }
        inst->height = max_height;
        return max_height + 1;
    }

    bool connect_hier(Referable<CellInst>& inst, int level = 0)
    {
        std::unordered_map<int,Referable<Conn>*> conns_map;
        int i = -1;
        if (inst.module_ref->ports.size() != inst.conns.size()) {
            PNR_ERROR("internal error: ports number of the module '{}' ({}) is not same as connections number of its cell '{}' ({})\n", inst.module_ref->name,
                inst.module_ref->ports.size(), inst.cell_ref->name, inst.conns.size());
            return false;
        }

        PNR_LOG1("RTL ", "connecting cell: '{}' ({}) of module: {}, level: {}... ", inst.cell_ref->name, inst.cell_ref->type, inst.module_ref->name, level);
        level++;

        // making a map of designators on current level

        i = -1;
        for (auto& port : inst.module_ref->ports) {
            ++i;
            if (i >= (int)inst.conns.size()) {
                PNR_ERROR("internal error: no instance connection {} was leased for cell '{}' port '{}' designator '{}'\n", i, inst.cell_ref->name,
                    port.name, port.designator);
                return false;
            }
            conns_map[port.designator] = &inst.conns[i];
        }

        for (auto& sub_inst : inst.insts) {
            i = -1;
            for (auto& port : sub_inst.cell_ref->ports) {
                ++i;
                if (i >= (int)sub_inst.conns.size()) {
                    PNR_ERROR("internal error: no instance connection {} was leased for cell '{}' port '{}' designator '{}'\n", i, sub_inst.cell_ref->name,
                        port.name, port.designator);
                    return false;
                }
                conns_map[port.designator] = &sub_inst.conns[i];
            }
        }

        // linking connections
        i = -1;
        for (auto& port : inst.module_ref->ports) {
            ++i;
            if (port.type == rtl::Port::PORT_IN) {
                auto it = conns_map.find(port.designator);
                while (it != conns_map.end() && it->first == port.designator) {  // while?
                    if (it->second->port_ref->type == rtl::Port::PORT_IN) {
                        PNR_LOG2("RTL ", "port '{}' to cell '{}'({}) port '{}' ({},{})", port.name, it->second->inst_ref->cell_ref->name,
                            it->second->inst_ref->cell_ref->type, it->second->port_ref->name, i, port.designator);
                        inst.conns[i].output_ref.set(it->second);
                    }
                    ++it;
                }
            }
        }

        return true;
    }
};





}
